"""Live video over NDNSF's high-level predictive stream API (prototype).

This is the A/B alternative to today's video path — a fresh
``publish_segmented`` producer per frame on a latest-wins name
(``run_drone_agent.video_loop``) that the dashboard *polls*
(``run_dashboard._video_relay_loop``, an aggregate ~8 fps cap across all
feeds, silent drop on any fetch race). That design is the direct cause of
the bursty framerate and iuas-01's total stall (per-frame segmented
producer churn is the same path that poisons the SVS node).

The stream API replaces it with one long-lived session per camera:
NDNSF owns sequencing, adaptive prefetch, reordering, and bounded FEC
recovery, and the consumer subscribes once instead of polling.

Requires NDNSF >= origin/main (matianxing1992) — the streaming API. The
June fleet build does NOT have it; deploying this needs the NDNSF flake
input bumped to the streaming rev.

Design notes
------------
* The high-level ``StreamPublisher.push`` takes ONE app-signed NDN Data
  packet per call, so each frame must fit a single signed Data under the
  stream's ``signed_wire_cap`` (default 8800). We push one frame per
  sample and size frames below ``FRAME_BUDGET``. Full-resolution frames
  that exceed one packet are the production follow-up: the lower-level
  ``LiveStreamPublisher.publish_sample(reservation, opaque_sources)``
  takes a list of opaque chunks as ONE sample and signs internally, which
  is the correct multi-segment-per-frame path. Kept out of the prototype
  to stay faithful to the documented simple API.
* The producer's ``descriptor`` must reach the consumer out-of-band to
  subscribe. It serializes via ``to_dict()``; we ship it as JSON over the
  existing small-payload data plane (reliable — only large/segmented
  payloads were ever affected by the poisoning bug).
"""

from __future__ import annotations

import json
from typing import Callable, Optional

# One pushed frame must encode into a single signed Data under the stream's
# signed-wire cap. Leave headroom for the Name + SignatureInfo/Value over the
# JPEG content; a frame larger than this must be downscaled by the caller.
SIGNED_WIRE_CAP = 8800
FRAME_BUDGET = 7000


def make_app_signed_data(
    name: str, payload: bytes, signing_identity: str = ""
) -> bytes:
    """Return the signed NDN Data wire for one frame, app-named + app-signed.

    Uses NDNSF's single-shot segmented signer with a segment size at the wire
    cap so a within-budget frame yields exactly one packet — the wire we push.
    Raises if the frame would segment (caller must downscale below
    ``FRAME_BUDGET``), because the high-level ``push`` is one-Data-per-call.
    """
    from ndnsf import make_segmented_data_packets

    packets = make_segmented_data_packets(
        name,
        payload,
        signing_identity=signing_identity,
        max_segment_size=SIGNED_WIRE_CAP,
        freshness_ms=300,
    )
    if len(packets) != 1:
        raise ValueError(
            f"frame {len(payload)}B did not fit one Data "
            f"({len(packets)} segments); downscale below {FRAME_BUDGET}B"
        )
    return packets[0].wire


def default_video_stream_config(
    stream_id: str,
    data_prefix: str,
    *,
    fps: float,
    fec_source_items: int = 4,
    fec_max_source_bytes: int = 1400,
    fec_recovery_budget_ms: int = 200,
    mapping_ahead_blocks: int = 8,
    retained_items: int = 1200,
):
    """Video-tuned :class:`StreamConfig` (one sample class, GF(256) FEC)."""
    from ndnsf import (
        LiveStreamFecOptions,
        SampleClassProfile,
        StreamAdvancedOptions,
        StreamConfig,
    )

    return StreamConfig(
        stream_id=stream_id,
        data_prefix=data_prefix,
        sample_period_ms=1000.0 / max(fps, 1.0),
        # One opaque "video" class; seed 1 source item/sample, allow growth
        # to 4 so a slightly-larger frame still fits without a config change.
        sample_classes=(SampleClassProfile("video", 1, 4),),
        fec=LiveStreamFecOptions.gf256_two_repair(
            source_items=fec_source_items,
            max_source_bytes=fec_max_source_bytes,
            recovery_budget_ms=fec_recovery_budget_ms,
        ),
        advanced=StreamAdvancedOptions(
            mapping_ahead_blocks=mapping_ahead_blocks,
            retained_items=retained_items,
        ),
    )


class VideoStreamProducer:
    """One long-lived predictive stream per camera; push one JPEG per frame."""

    def __init__(
        self,
        provider,
        stream_id: str,
        data_prefix: str,
        *,
        fps: float = 15.0,
        signing_identity: str = "",
    ) -> None:
        self._config = default_video_stream_config(
            stream_id, data_prefix, fps=fps
        )
        self._stream = provider.create_stream(self._config)
        self._descriptor = self._stream.start()
        self._definition = self._descriptor.definition
        self._signing_identity = signing_identity
        self._seq = 0

    @property
    def descriptor_dict(self) -> dict:
        """Serializable descriptor the consumer needs to subscribe."""
        return self._descriptor.to_dict()

    @property
    def descriptor_json(self) -> bytes:
        return json.dumps(self.descriptor_dict).encode()

    @property
    def seq(self) -> int:
        return self._seq

    def publish_frame(self, jpeg: bytes) -> bool:
        """Push one frame; returns False (skipped) if it exceeds the budget.

        Names each frame under the stream's mapping root so the consumer's
        adaptive fetcher can predict and prefetch the next sample.
        """
        if len(jpeg) > FRAME_BUDGET:
            return False
        name = (
            f"{self._definition.mapping_root}/v/"
            f"{self._definition.mapping_version}/seq={self._seq}"
        )
        self._stream.push(
            make_app_signed_data(name, jpeg, self._signing_identity)
        )
        # Emit FEC parity for the pending source segment so a single lost
        # frame can be recovered without a retransmit round-trip.
        self._stream.flush()
        self._seq += 1
        return True

    def stop(self) -> None:
        try:
            self._stream.stop()
        except Exception:
            pass


class VideoStreamConsumer:
    """Subscribe once to a producer's descriptor; deliver frames to a sink.

    ``on_frame(cursor, jpeg)`` fires on framework threads as items are
    verified and admitted (in cursor order once reordering settles). Counters
    are plain ints updated only from the callback thread; read them after the
    measurement window or guard with your own lock if you read concurrently.
    """

    def __init__(
        self,
        user,
        descriptor: "dict | bytes | str",
        on_frame: Callable[[int, bytes], None],
        *,
        require_full_delivery: bool = False,
        prefetch_policy: str = "adaptive-sample-atomic",
        on_status: Optional[Callable[[object], None]] = None,
    ) -> None:
        from ndnsf import (
            LiveStreamItemAdmission,
            PredictiveStreamDescriptor,
            StreamSubscriptionOptions,
        )

        if isinstance(descriptor, (bytes, bytearray)):
            descriptor = json.loads(bytes(descriptor).decode())
        elif isinstance(descriptor, str):
            descriptor = json.loads(descriptor)
        desc = PredictiveStreamDescriptor.from_dict(descriptor)

        self.delivered = 0
        self.recovered = 0  # frames reconstructed by FEC rather than fetched

        def _on_item(item):
            self.delivered += 1
            # provenance distinguishes a directly-fetched item from one the
            # bounded FEC recovery rebuilt — the metric that proves the
            # recovery path is actually saving frames the old path dropped.
            if "recover" in (item.provenance or ""):
                self.recovered += 1
            try:
                on_frame(item.cursor, item.content)
            except Exception:
                pass
            return LiveStreamItemAdmission.accept_item()

        self._subscriber = user.subscribe_stream(
            desc,
            StreamSubscriptionOptions(
                on_item=_on_item,
                start="latest",
                prefetch_policy=prefetch_policy,
                aggregate_interest_limit=64,
                enable_fec_recovery=True,
                require_full_delivery=require_full_delivery,
                interest_lifetime_ms=500,
                on_status=on_status,
            ),
        )

    def stop(self) -> None:
        try:
            self._subscriber.stop()
        except Exception:
            pass
