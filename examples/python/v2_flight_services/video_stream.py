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

Configuration
-------------
The defaults here follow the NDNSF author's recommended starting points
for a live camera (matianxing1992, streaming-options guidance):

* ``StreamAdvancedOptions`` is left at the wrapper defaults
  (``mapping_ahead_blocks=4``, ``retained_items=600``,
  ``max_pending_interests=256``, ``signed_wire_cap=8800``,
  ``startup_timeout_ms=1000``). Raise ``mapping_ahead_blocks`` only if
  measurements show pipeline starvation; raise ``retained_items`` only if a
  longer late-join / recovery history is needed.
* Subscribers use ``start="latest"`` (live), automatic prefetch selection
  (``prefetch_policy=None`` — pin a policy only for reproducible benches),
  ``aggregate_interest_limit=64``, ``enable_fec_recovery=True``, and
  ``require_full_delivery=False`` (skip an unrecoverable frame and continue,
  the right choice for live video — set it True only for recording/telemetry
  where any gap is unacceptable).
* Each pushed frame is exactly one signed Data (see below), so the sample
  class is ``("video", 1, 1)`` — a sample always holds one packet.

FEC and the flush() contract
----------------------------
``flush()`` CLOSES an FEC group: parity protects exactly the frames pushed
since the last flush. So the FEC group size and the flush cadence are one
knob, ``fec_group_frames``:

* ``fec_group_frames=1`` (default, lowest latency): flush after every frame.
  A group of one source packet suits XOR one-repair (or no FEC) — NOT GF(256)
  two-repair, which needs a multi-packet group to mean anything. Parity for a
  frame is available one frame later, so a single lost frame is recoverable
  without a retransmit round-trip, at the cost of ~2x video bandwidth.
* ``fec_group_frames=4``: push four frames, then one flush → GF(256)
  two-repair over four source packets recovers up to two losses per group at
  ~1.5x bandwidth, but a loss is not recoverable until the group closes.

The old prototype configured GF(256) two-repair over four source items yet
called ``flush()`` after every ``push()`` — so every group held a single
packet and the two-repair coding was inert. That mismatch is fixed here by
deriving the scheme from ``fec_group_frames``.

Notes
-----
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
* ``on_frame`` runs on framework delivery threads. Keep it light — do heavy
  decode/inference on a worker queue, not in the callback (the dashboard
  already relays the frame and returns).
"""

from __future__ import annotations

import json
import threading
from typing import Callable, Optional

# One pushed frame must encode into a single signed Data under the stream's
# signed-wire cap. Leave headroom for the Name + SignatureInfo/Value over the
# JPEG content; a frame larger than this must be downscaled by the caller.
SIGNED_WIRE_CAP = 8800
FRAME_BUDGET = 7000

# FEC "max source bytes" must cover the COMPLETE signed Data wire (name +
# signature + content), not just the JPEG payload, or parity can't reconstruct
# a full packet. A within-budget frame's signed Data is bounded by the
# signed-wire cap, so size the FEC source symbol to the cap.
FEC_MAX_SOURCE_BYTES = SIGNED_WIRE_CAP
FEC_RECOVERY_BUDGET_MS = 200  # reasonable for real-time local Wi-Fi


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


def _build_fec(scheme: str, group_frames: int, max_source_bytes: int,
               recovery_budget_ms: int):
    """FEC options coherent with the flush cadence (``group_frames``).

    ``scheme`` is "auto" (derive from group size), "none", "xor", or "gf256".
    "auto": a 1-frame group → XOR one-repair; a multi-frame group → GF(256)
    two-repair over the group. XOR/GF(256) ``source_items`` == the group size
    so the coding matches the packets a single ``flush()`` will bind together.
    """
    from ndnsf import LiveStreamFecOptions

    if scheme == "auto":
        scheme = "xor" if group_frames <= 1 else "gf256"
    if scheme == "none":
        return LiveStreamFecOptions.none()
    if scheme == "xor":
        return LiveStreamFecOptions.xor_one_repair(
            source_items=max(1, group_frames),
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    if scheme == "gf256":
        # GF(256) two-repair needs at least a 2-packet group to be meaningful.
        return LiveStreamFecOptions.gf256_two_repair(
            source_items=max(2, group_frames),
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    raise ValueError(f"unknown FEC scheme: {scheme!r}")


def default_video_stream_config(
    stream_id: str,
    data_prefix: str,
    *,
    fps: float,
    fec_group_frames: int = 1,
    fec_scheme: str = "auto",
    fec_max_source_bytes: int = FEC_MAX_SOURCE_BYTES,
    fec_recovery_budget_ms: int = FEC_RECOVERY_BUDGET_MS,
    advanced=None,
):
    """Video-tuned :class:`StreamConfig` (one sample class, group-coherent FEC).

    ``session_epoch`` is left unset so NDNSF assigns a fresh epoch. ``advanced``
    defaults to the wrapper's recommended starting values; pass a
    :class:`StreamAdvancedOptions` only to override them.
    """
    from ndnsf import (
        SampleClassProfile,
        StreamAdvancedOptions,
        StreamConfig,
    )

    return StreamConfig(
        stream_id=stream_id,
        data_prefix=data_prefix,
        sample_period_ms=1000.0 / max(fps, 1.0),
        # Each pushed frame is exactly one signed Data (make_app_signed_data
        # raises otherwise), so a sample always holds one packet: (1, 1).
        sample_classes=(SampleClassProfile("video", 1, 1),),
        fec=_build_fec(
            fec_scheme, fec_group_frames,
            fec_max_source_bytes, fec_recovery_budget_ms,
        ),
        # Wrapper defaults = the recommended starting point (mapping_ahead=4,
        # retained_items=600, …). Only override if measurements call for it.
        advanced=advanced if advanced is not None else StreamAdvancedOptions(),
    )


class VideoStreamProducer:
    """One long-lived predictive stream per camera; push one JPEG per frame.

    ``fec_group_frames`` sets both the FEC group size and the flush cadence:
    a frame is pushed on every ``publish_frame`` and a group is flushed once
    ``fec_group_frames`` frames have accumulated (1 = flush every frame).
    """

    def __init__(
        self,
        provider,
        stream_id: str,
        data_prefix: str,
        *,
        fps: float = 15.0,
        signing_identity: str = "",
        fec_group_frames: int = 1,
        fec_scheme: str = "auto",
    ) -> None:
        self._fec_group_frames = max(1, int(fec_group_frames))
        self._config = default_video_stream_config(
            stream_id, data_prefix, fps=fps,
            fec_group_frames=self._fec_group_frames, fec_scheme=fec_scheme,
        )
        self._stream = provider.create_stream(self._config)
        self._descriptor = self._stream.start()
        self._definition = self._descriptor.definition
        self._signing_identity = signing_identity
        self._seq = 0
        # Frames pushed but not yet bound into a flushed FEC group.
        self._pending = 0
        self._lock = threading.Lock()

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
        adaptive fetcher can predict and prefetch the next sample. A flush
        (which closes the current FEC group and emits its parity) fires once
        ``fec_group_frames`` frames have accumulated.
        """
        if len(jpeg) > FRAME_BUDGET:
            return False
        with self._lock:
            name = (
                f"{self._definition.mapping_root}/v/"
                f"{self._definition.mapping_version}/seq={self._seq}"
            )
            self._stream.push(
                make_app_signed_data(name, jpeg, self._signing_identity)
            )
            self._seq += 1
            self._pending += 1
            if self._pending >= self._fec_group_frames:
                # Close the group: publish FEC parity for the frames pushed
                # since the last flush so they can be recovered without a
                # retransmit round-trip.
                self._stream.flush()
                self._pending = 0
        return True

    def stop(self) -> None:
        try:
            with self._lock:
                # Flush a partial trailing group so its frames still get parity
                # (and are not held waiting for a group that will never fill).
                if self._pending:
                    self._stream.flush()
                    self._pending = 0
        except Exception:
            pass
        try:
            self._stream.stop()
        except Exception:
            pass


class VideoStreamConsumer:
    """Subscribe once to a producer's descriptor; deliver frames to a sink.

    ``on_frame(cursor, jpeg)`` fires on framework threads as items are
    verified and admitted (in cursor order once reordering settles). Keep it
    light — offload heavy decode/inference to a worker queue. Counters are
    plain ints updated only from the callback thread; read them after the
    measurement window or guard with your own lock if you read concurrently.

    ``prefetch_policy`` defaults to None so NDNSF auto-selects the policy; pin
    a policy (e.g. "adaptive-sample-atomic") only when a bench must be
    reproducible.
    """

    def __init__(
        self,
        user,
        descriptor: "dict | bytes | str",
        on_frame: Callable[[int, bytes], None],
        *,
        require_full_delivery: bool = False,
        prefetch_policy: Optional[str] = None,
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
