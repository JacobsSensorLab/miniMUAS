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
import time
from collections import deque
from typing import Callable, Optional

# One pushed frame must encode into a single signed Data under the stream's
# signed-wire cap. Leave headroom for the Name + SignatureInfo/Value over the
# JPEG content; a frame larger than this must be downscaled by the caller.
SIGNED_WIRE_CAP = 8800
FRAME_BUDGET = 7000

# Bytes of JPEG carried per published Data. One Data can hold at most
# `SIGNED_WIRE_CAP` (8800) INCLUDING name + SignatureInfo/Value, so the payload
# budget is FRAME_BUDGET; take a further `CHUNK_HEADER_BYTES` for the
# reassembly header and leave slack.
#
# A frame larger than one packet is split across several. 8800 is the cap on a
# single Data, NOT on a frame — real NDN video (NDN-RTC and friends) segments
# every frame this way. Before this, `publish_frame` pushed one Data per frame
# and the agent silently re-encoded anything over budget at 256px/q30, so
# asking for HIGHER quality produced LOWER resolution and the bitrate was
# pinned near 250-500 kbps regardless of the requested settings.
# 6800, not 5800: the chunk count is what the publish path costs, since every
# chunk is a separate signed Data with its own push. Measured on the fleet, a
# 1280x800 q75 frame is ~86 KB = 15 chunks at 5800 but 13 at 6800, and the
# fitted frame cost was ~21 ms + ~3.8 ms per chunk. The ceiling is
# FRAME_BUDGET - CHUNK_HEADER_BYTES (6990); 6800 keeps slack under it.
#
# This is NOT the ndn-svs MAX_DATA_SIZE=6000 limit that poisons segmented
# SVSPubSub *service responses* -- that applies to the SVSPubSub publish path,
# while these chunks are individual signed Data pushed into the predictive
# stream, bounded only by SIGNED_WIRE_CAP.
FRAME_CHUNK_BYTES = 6800
CHUNK_HEADER_BYTES = 10
_CHUNK_MAGIC = b"V1"

# Guard against a runaway encode; 2 MB is ~340 chunks.
MAX_FRAME_BYTES = 2 * 1024 * 1024

# Data packets one frame may occupy, and therefore the source-item capacity the
# stream must declare. A flush closes the FEC group on a frame boundary, so the
# group holds this frame's CHUNKS -- not one item per frame.
#
# Getting this wrong is silent and total: NDNSF rejects the push with "flush
# group exceeds configured FEC source capacity" and the stream delivers
# nothing. It is scene-dependent, so it looks like an intermittent per-vehicle
# fault rather than a configuration error -- a busy scene makes a bigger JPEG,
# more chunks, and a group that overflows, while a flat scene stays under.
#
# 64 chunks is ~371 KB, comfortably above a 1280x800 q70 JPEG (~150-250 KB).
FRAME_MAX_CHUNKS = 64


def _chunk_header(frame_seq: int, idx: int, count: int) -> bytes:
    """`V1` + frame_seq(4) + idx(2) + count(2) — 10 bytes, big-endian.

    Self-describing so the consumer can reassemble from per-item delivery
    without depending on NDNSF sample-grouping metadata (the delivered item
    exposes `cursor`/`content`/`provenance`, not a sample id).
    """
    return (
        _CHUNK_MAGIC
        + int(frame_seq).to_bytes(4, "big")
        + int(idx).to_bytes(2, "big")
        + int(count).to_bytes(2, "big")
    )


def parse_chunk_header(buf: bytes):
    """-> (frame_seq, idx, count, payload) or None when not a V1 chunk."""
    if len(buf) < CHUNK_HEADER_BYTES or buf[:2] != _CHUNK_MAGIC:
        return None
    return (
        int.from_bytes(buf[2:6], "big"),
        int.from_bytes(buf[6:8], "big"),
        int.from_bytes(buf[8:10], "big"),
        buf[CHUNK_HEADER_BYTES:],
    )

# FEC "max source bytes" must cover the COMPLETE signed Data wire (name +
# signature + content), not just the JPEG payload, or parity can't reconstruct
# a full packet. A within-budget frame's signed Data is bounded by the
# signed-wire cap, so size the FEC source symbol to the cap.
FEC_MAX_SOURCE_BYTES = SIGNED_WIRE_CAP
FEC_RECOVERY_BUDGET_MS = 200  # reasonable for real-time local Wi-Fi

# A live subscriber prefetches FUTURE cursors — Interests for frames the camera
# has not taken yet — which the producer parks until it produces them. So the
# Interest lifetime has to outlast the wait, and the prefetch depth must not
# reach further ahead than the lifetime covers:
#
#     prefetch_depth / fps  <=  interest_lifetime
#
# The generic defaults (limit 64, lifetime 500 ms) violate this badly for live
# video: at 10 fps, 64 cursors is 6.4 s ahead while each Interest dies after
# 0.5 s, so everything past ~5 frames ahead expires before its frame exists.
# Measured on the fleet: 34 timeouts, retry AND recovery exhausted, then
# `terminal-gap:timeout` — delivery stopped dead after ~11-20 frames while the
# producer was happily pushing 10 fps. Depth 16 at 5-15 fps is 1.1-3.2 s ahead,
# comfortably inside a 4 s lifetime, and 16 deep is ample pipelining on a link
# whose RTT is ~1 ms.
LIVE_INTEREST_LIFETIME_MS = 4000
# Prefetch window, in ITEMS. Once a frame is segmented an item is a CHUNK, not
# a frame, so 16 items is under one frame in flight at 1280px (~20 chunks) —
# and delivered fps tracked chunks-per-frame almost inversely on BOTH stacks
# with 100% fragment completion, i.e. the window was the binding constraint,
# not loss:
#     640px  ~6.8 chunks/frame -> 6.3-11.6 fps
#     960px  ~14.5            -> 3.0-4.2 fps
#     1280px ~19.6            -> 2.2-2.3 fps
# 128 items covers ~6 frames even at 1280px. The depth/lifetime rule still
# holds with room to spare: 128 items at 20 chunks and 10 fps is ~0.64 s of
# lookahead against a 4 s Interest lifetime (the rule that matters is
# frames_ahead / fps <= lifetime, and frames_ahead = limit / chunks_per_frame).
LIVE_INTEREST_LIMIT = 128

# Mapping blocks must fit ONE link fragment.
#
# A Name-Map block is published roughly once per frame (measured: 452 blocks
# for ~437 frames), so `mapping_block_capacity` sets each block's SIZE, not how
# many blocks there are — shrinking it costs nothing in block count.
#
# At the NDNSF default (16 items) a block is ~8.7 KB, which the forwarder must
# split into 6 link fragments; an N-fragment packet is lost if ANY fragment is
# lost, so a 5% frame loss becomes ~26% block loss. Every lost block stalls the
# consumer, which re-requests — measured on the fleet as 26 blocks/s of churn
# against 10 blocks/s of real work, saturating the link and causing the very
# loss it was reacting to.
#
# ~545 B/item measured, so 2 items + name + SignatureInfo/Value lands near
# 1.4 KB — inside one fragment on this fleet's 1452-byte datagram MTU. Keep
# the ITEM lookahead roughly unchanged (2 x 16 = 32 items ~ 3.2 s at 10 fps,
# comfortably inside LIVE_INTEREST_LIFETIME_MS) by raising the block count as
# the capacity falls.
#
# NOTE: do NOT lower `signed_wire_cap` to force this — that cap also bounds a
# pushed FRAME (~7 KB under FRAME_BUDGET), and an over-budget push is rejected.
MAPPING_BLOCK_CAPACITY = 2
MAPPING_AHEAD_BLOCKS = 16


def predictive_data_name(mapping_root: str, mapping_version: int, seq: int) -> str:
    """Canonical predictive-stream Data name for one pushed sample.

    Delegates to NDNSF's ``make_predictive_data_name`` (mirrors the C++
    ``nsf::makePredictiveDataName``): ``mappingRoot + "v" + Number(version) +
    SequenceNumber(seq)``. The version is a nonNegativeInteger component, not
    ASCII digits, so a hand-built f-string name is rejected by the Core as
    "non-canonical predictive Data name".
    """
    from ndnsf import make_predictive_data_name

    return make_predictive_data_name(mapping_root, int(mapping_version), int(seq))


def make_app_signed_data(
    name: str, payload: bytes, signing_identity: str = ""
) -> bytes:
    """Return the signed NDN Data wire for one frame, app-named + app-signed.

    Uses NDNSF's exact-name signer (``make_signed_data``) so the Data carries the
    verbatim predictive name — the high-level ``push`` is one-Data-per-call and
    rejects any name with an appended version/segment. The caller must keep the
    frame a single packet by downscaling below ``FRAME_BUDGET`` (the Core also
    enforces ``signed_wire_cap`` and rejects an over-budget push).
    """
    from ndnsf import make_signed_data

    return make_signed_data(
        name,
        payload,
        signing_identity=signing_identity,
        freshness_ms=300,
    )


def _build_fec(scheme: str, group_frames: int, max_source_bytes: int,
               recovery_budget_ms: int):
    """FEC options coherent with the flush cadence (``group_frames``).

    ``scheme`` is "auto" (derive from group size), "none", "xor", or "gf256".
    "auto": a 1-frame group → XOR one-repair; a multi-frame group → GF(256)
    two-repair over the group.

    ``source_items`` counts the DATA PACKETS a single ``flush()`` binds, which
    since frame segmentation is ``group_frames * FRAME_MAX_CHUNKS`` -- not
    ``group_frames``. Sizing it per FRAME made every multi-chunk frame fail to
    push with "flush group exceeds configured FEC source capacity". A group
    smaller than the declared capacity is fine; only overflow is an error.
    """
    group_items = max(1, group_frames) * FRAME_MAX_CHUNKS
    from ndnsf import LiveStreamFecOptions

    if scheme == "auto":
        scheme = "xor" if group_frames <= 1 else "gf256"
    if scheme == "none":
        return LiveStreamFecOptions.none()
    if scheme == "xor":
        return LiveStreamFecOptions.xor_one_repair(
            source_items=group_items,
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    if scheme == "gf256":
        # GF(256) two-repair needs at least a 2-packet group to be meaningful.
        return LiveStreamFecOptions.gf256_two_repair(
            source_items=max(2, group_items),
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    raise ValueError(f"unknown FEC scheme: {scheme!r}")


def _mapping_options(StreamAdvancedOptions, block_capacity: int,
                     ahead_blocks: int):
    """Advanced options with the Name-Map block geometry capped to one fragment.

    ``StreamAdvancedOptions`` is a FROZEN dataclass — set the fields through the
    constructor; assigning to them raises ``FrozenInstanceError``. Every other
    field keeps its wrapper default.
    """
    return StreamAdvancedOptions(
        mapping_block_capacity=int(block_capacity),
        mapping_ahead_blocks=int(ahead_blocks),
    )


def default_video_stream_config(
    stream_id: str,
    data_prefix: str,
    *,
    fps: float,
    fec_group_frames: int = 1,
    fec_scheme: str = "auto",
    fec_max_source_bytes: int = FEC_MAX_SOURCE_BYTES,
    fec_recovery_budget_ms: int = FEC_RECOVERY_BUDGET_MS,
    mapping_block_capacity: int = MAPPING_BLOCK_CAPACITY,
    mapping_ahead_blocks: int = MAPPING_AHEAD_BLOCKS,
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
        # A sample is one FRAME, which segmentation spreads over up to
        # FRAME_MAX_CHUNKS Data packets. The seed stays 1 (a flat scene really
        # does fit in one packet); the hard max is what a busy scene needs.
        # This was left at (1, 1) when frames became multi-packet.
        sample_classes=(
            SampleClassProfile("video", 1, FRAME_MAX_CHUNKS),
        ),
        fec=_build_fec(
            fec_scheme, fec_group_frames,
            fec_max_source_bytes, fec_recovery_budget_ms,
        ),
        # Wrapper defaults are the recommended starting point, EXCEPT the
        # Name-Map block geometry: the default 16-item block is ~8.7 KB, which
        # fragments 6 ways on a Wi-Fi datagram face and is then all-or-nothing.
        # See MAPPING_BLOCK_CAPACITY.
        advanced=advanced if advanced is not None else _mapping_options(
            StreamAdvancedOptions, mapping_block_capacity, mapping_ahead_blocks,
        ),
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
        self._frame_seq = 0
        # Frames pushed but not yet bound into a flushed FEC group.
        self._pending = 0
        # Frames dropped for needing more than FRAME_MAX_CHUNKS packets. Should
        # stay 0; a rising count means the encoder is outrunning the declared
        # stream geometry and FRAME_MAX_CHUNKS needs raising (in step with the
        # FEC source capacity, which is derived from it).
        self.frames_over_chunk_budget = 0
        # Rolling publish-path timings. `push` is per DATA PACKET (the handoff
        # of one signed Data to the local forwarder); `flush` is per FRAME (it
        # closes the FEC group and emits the group's control packets). These
        # are the two calls that can block on the local face, so if one
        # forwarder is slower to drain the app's socket than another it shows
        # up here and nowhere else -- the video publish loop is self-paced
        # (`delay = 1/fps - cycle`), so a slower handoff lowers the achieved
        # frame rate directly rather than queueing.
        self._push_us = deque(maxlen=4096)
        self._flush_us = deque(maxlen=1024)
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
        """Publish one frame, split across as many Data packets as it needs.

        A frame larger than one packet is segmented: each chunk is its own
        signed Data carrying a `parse_chunk_header` prefix, and the consumer
        reassembles by frame sequence. One Data can hold at most
        `SIGNED_WIRE_CAP` including name and signature, but a FRAME has no such
        limit — that is how NDN video is normally carried.

        The flush after the last chunk closes the FEC group on a frame
        boundary, so parity protects exactly one frame's chunks. That is
        strictly better than the previous group-of-one-frame, where a
        single-packet loss was unrecoverable.

        Returns False for an implausibly large frame (`MAX_FRAME_BYTES`), which
        indicates an encoder problem rather than a transport limit, and for one
        needing more than `FRAME_MAX_CHUNKS` packets -- dropping that frame is
        far better than pushing it, because an over-capacity flush group fails
        the push and takes the whole stream down with it.
        """
        if not jpeg or len(jpeg) > MAX_FRAME_BYTES:
            return False
        chunks = [
            jpeg[i : i + FRAME_CHUNK_BYTES]
            for i in range(0, len(jpeg), FRAME_CHUNK_BYTES)
        ]
        count = len(chunks)
        if count > FRAME_MAX_CHUNKS:
            self.frames_over_chunk_budget += 1
            return False
        with self._lock:
            frame_seq = self._frame_seq
            for idx, chunk in enumerate(chunks):
                name = predictive_data_name(
                    self._definition.mapping_root,
                    self._definition.mapping_version,
                    self._seq,
                )
                wire = make_app_signed_data(
                    name,
                    _chunk_header(frame_seq, idx, count) + chunk,
                    self._signing_identity,
                )
                # Time ONLY the handoff, not the signing: signing is
                # stack-independent, the handoff is what differs per forwarder.
                _t0 = time.perf_counter()
                self._stream.push(wire)
                self._push_us.append((time.perf_counter() - _t0) * 1e6)
                self._seq += 1
            self._frame_seq += 1
            self._pending += 1
            if self._pending >= self._fec_group_frames:
                _t0 = time.perf_counter()
                self._stream.flush()
                self._flush_us.append((time.perf_counter() - _t0) * 1e6)
                self._pending = 0
        return True

    def timing_stats(self) -> dict:
        """Percentiles of the publish-path handoff, in microseconds.

        `push_*` is per Data packet, `flush_*` per frame. Both measure only the
        call into NDNSF/the local face -- signing is excluded, since it is the
        same work whichever forwarder is underneath.
        """
        def pct(samples, q):
            if not samples:
                return 0.0
            ordered = sorted(samples)
            idx = min(len(ordered) - 1, int(len(ordered) * q))
            return round(ordered[idx], 1)

        with self._lock:
            push = list(self._push_us)
            flush = list(self._flush_us)
        return {
            "push_n": len(push),
            "push_p50_us": pct(push, 0.50),
            "push_p90_us": pct(push, 0.90),
            "push_p99_us": pct(push, 0.99),
            "push_max_us": round(max(push), 1) if push else 0.0,
            "flush_n": len(flush),
            "flush_p50_us": pct(flush, 0.50),
            "flush_p90_us": pct(flush, 0.90),
            "flush_p99_us": pct(flush, 0.99),
            "flush_max_us": round(max(flush), 1) if flush else 0.0,
        }

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
        interest_lifetime_ms: int = LIVE_INTEREST_LIFETIME_MS,
        aggregate_interest_limit: int = LIVE_INTEREST_LIMIT,
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

        # Reassembly of segmented frames, keyed by frame sequence. A frame
        # larger than one Data arrives as several items; `on_frame` must only
        # fire once the whole JPEG is back. Bounded: a frame still incomplete
        # once `_REASM_KEEP` newer frames have completed is abandoned, so a
        # permanently-lost chunk cannot pin memory.
        pending_frames: dict = {}
        _REASM_KEEP = 4
        self.partial_frames_dropped = 0

        def _reassemble(raw, cursor):
            parsed = parse_chunk_header(raw)
            if parsed is None:
                # Un-segmented publisher (or a foreign item): pass it straight
                # through, so a consumer keeps working against an old producer.
                return cursor, raw
            frame_seq, idx, count, payload = parsed
            if count <= 1:
                return frame_seq, payload
            slot = pending_frames.setdefault(frame_seq, {})
            slot[idx] = payload
            if len(slot) < count:
                return None
            jpeg = b"".join(slot[i] for i in range(count))
            del pending_frames[frame_seq]
            for stale in [k for k in pending_frames if k < frame_seq - _REASM_KEEP]:
                del pending_frames[stale]
                self.partial_frames_dropped += 1
            return frame_seq, jpeg

        def _on_item(item):
            self.delivered += 1
            # provenance distinguishes a directly-fetched item from one the
            # bounded FEC recovery rebuilt — the metric that proves the
            # recovery path is actually saving frames the old path dropped.
            if "recover" in (item.provenance or ""):
                self.recovered += 1
            try:
                done = _reassemble(item.content, item.cursor)
                if done is not None:
                    on_frame(done[0], done[1])
            except Exception:
                pass
            return LiveStreamItemAdmission.accept_item()

        self._subscriber = user.subscribe_stream(
            desc,
            StreamSubscriptionOptions(
                on_item=_on_item,
                start="latest",
                prefetch_policy=prefetch_policy,
                aggregate_interest_limit=aggregate_interest_limit,
                enable_fec_recovery=True,
                require_full_delivery=require_full_delivery,
                interest_lifetime_ms=interest_lifetime_ms,
                on_status=on_status,
            ),
        )

    def stop(self) -> None:
        try:
            self._subscriber.stop()
        except Exception:
            pass
