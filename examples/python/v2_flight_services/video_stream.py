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
* A frame is split into 5800-byte chunks, one signed Data each, reassembled
  by the consumer from a small chunk header (``parse_chunk_header``).

FEC and the flush() contract
----------------------------
``flush()`` CLOSES an FEC group: parity protects exactly the Data pushed since
the last flush. A frame's chunks are published as ceil(n / FEC_GROUP_MAX_CHUNKS)
groups, each flushed as soon as it is pushed, so a lost chunk is recoverable
from its group's XOR repair without waiting for later frames. The group size is
bounded by the repair's own wire size, not chosen for coding strength (see
FEC_GROUP_MAX_CHUNKS).

Notes
-----
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

# The most one signed Data (Name + SignatureInfo/Value + content) may occupy:
# NDN's packet limit, enforced by NDNSF on every pushed and control packet.
SIGNED_WIRE_CAP = 8800

# Bytes of JPEG carried per published Data, leaving room under SIGNED_WIRE_CAP
# for the Name, SignatureInfo/Value and the `CHUNK_HEADER_BYTES` header.
#
# A frame larger than one packet is split across several. 8800 is the cap on a
# single Data, NOT on a frame — real NDN video (NDN-RTC and friends) segments
# every frame this way. Before this, `publish_frame` pushed one Data per frame
# and the agent silently re-encoded anything over budget at 256px/q30, so
# asking for HIGHER quality produced LOWER resolution and the bitrate was
# pinned near 250-500 kbps regardless of the requested settings.
# 5800. DO NOT raise this without re-checking FEC_GROUP_MAX_CHUNKS -- 6800 was tried on the fleet (2026-09-21) and BROKE high-quality
# 1280x800 outright: StreamPublisher::flush() threw "predictive group commit
# failed" on every frame (1780 failures), so the stream delivered nothing.
#
# It is chunk SIZE, not chunk count: 1280x800 q75 worked at 15 chunks of 5800
# and failed at 13 chunks of 6800. The failure is frame-size dependent at fixed
# chunk size too -- q40 (7 chunks) and q55 (8) ran fine at 6800 while q70 (13)
# failed. That limit is now characterised: it is the FEC repair's control Data
# (FEC_GROUP_MAX_CHUNKS), whose size grows with BOTH chunk size and chunk count.
FRAME_CHUNK_BYTES = 5800
CHUNK_HEADER_BYTES = 18
_CHUNK_MAGIC = b"V2"

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


def _chunk_header(frame_seq: int, idx: int, count: int, pub_ms: int) -> bytes:
    """`V2` + frame_seq(4) + idx(2) + count(2) + pub_ms(8) — 18 bytes, big-endian.

    Self-describing so the consumer can reassemble from per-item delivery
    without depending on NDNSF sample-grouping metadata (the delivered item
    exposes `cursor`/`content`/`provenance`, not a sample id). `pub_ms` is the
    producer's MONOTONIC clock when the frame was published: the consumer only
    ever differences it against itself, so it measures how far behind live the
    stream has fallen without depending on clock sync.
    """
    return (
        _CHUNK_MAGIC
        + int(frame_seq).to_bytes(4, "big")
        + int(idx).to_bytes(2, "big")
        + int(count).to_bytes(2, "big")
        + int(pub_ms).to_bytes(8, "big")
    )


def parse_chunk_header(buf: bytes):
    """-> (frame_seq, idx, count, pub_ms, payload) or None when not a V2 chunk."""
    if len(buf) < CHUNK_HEADER_BYTES or buf[:2] != _CHUNK_MAGIC:
        return None
    return (
        int.from_bytes(buf[2:6], "big"),
        int.from_bytes(buf[6:8], "big"),
        int.from_bytes(buf[8:10], "big"),
        int.from_bytes(buf[10:18], "big"),
        buf[CHUNK_HEADER_BYTES:],
    )

# FEC "max source bytes" must cover the COMPLETE signed Data wire (name +
# signature + content), not just the JPEG payload, or parity can't reconstruct
# a full packet. A within-budget frame's signed Data is bounded by the
# signed-wire cap, so size the FEC source symbol to the cap.
FEC_MAX_SOURCE_BYTES = SIGNED_WIRE_CAP
FEC_RECOVERY_BUDGET_MS = 200  # reasonable for real-time local Wi-Fi

# Chunks per FEC group. The XOR repair travels as ONE signed control Data that
# carries, for every source, its full name, cursor, length and 32-byte digest,
# plus the widest source itself, all under SIGNED_WIRE_CAP. Run against NDNSF's
# own encoder (aarch64, fleet vehicle ids): 19 sources of 5800-byte chunks leave
# 32-45 B spare, each further source adds ~121 B, and at 20 flush() throws
# "signed predictive control Data exceeds wire budget", which fails the whole
# stream. A 1280x800 q85 frame averaged ~98 KB (17 chunks) on the fleet and a
# busy scene goes well past 19, so a frame is split across ceil(n/18) groups;
# 18 leaves room for ECDSA length jitter and 8-character vehicle ids.
FEC_GROUP_MAX_CHUNKS = 18

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
# 24, measured. This is a BANDWIDTH-DELAY PRODUCT bound, not a "how far ahead
# would we like to read" bound, and getting that wrong cost ~3.6x throughput.
#
# ndn-iperf over the GCS<->iuas-01 link, 5800 B chunks, video disabled so the
# path was otherwise idle:
#     window    4     8    16    24    32    48   128   256
#     Mbps   26.9  28.8  36.5  44.2  30.7  27.2  12.2  24.7
# The true path RTT is ~2.8 ms (min), so the BDP is only ~2 chunks; 128 is
# ~60x that and drives the path into congestion collapse -- queueing inflates
# RTT p50 to 19.5 ms and p99 to 220 ms, Interests then expire in the queue, and
# delivery stalls for SECONDS. That is the video stutter, and it is
# self-inflicted: 12.2 Mbps out of an achievable 44.2.
#
# The floor matters too, which is why this is not smaller. A window below one
# frame's chunk count cannot get a frame in flight, and that is the starvation
# the previous note recorded when this was 16 (a 1280px frame is ~20 chunks).
# At ~10 chunks/frame for 960x600, 24 is ~2.4 frames in flight AND the measured
# throughput peak.
#
# Re-measure with ndn-iperf if the radio, MTU or chunk size changes -- the
# right value tracks the BDP, not the frame rate.
LIVE_INTEREST_LIMIT = 24

# The subscriber prefetches at most half its window ahead of the newest
# produced cursor (NDNSF future horizon = min(lookahead, limit) / 2), and the
# slowest a stream produces items is one chunk per frame, so a future Interest
# waits at most (LIVE_INTEREST_LIMIT / 2) / fps for its item to exist. Twice that
# covers RTT and camera jitter. The fixed 4 s ceiling is what one LOST item
# costs: the ordered drain waits out the whole first-attempt lifetime, measured
# on the fleet as stalls of exactly 4.06-4.24 s, each one `timeouts` +1.
# At 30 fps this is 0.8 s; at 6 fps and below it is the old 4 s.
def live_interest_lifetime_ms(fps: float) -> int:
    worst_wait_s = (LIVE_INTEREST_LIMIT / 2) / max(1.0, float(fps))
    return int(min(LIVE_INTEREST_LIFETIME_MS, 2 * worst_wait_s * 1000))

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
# NOTE: do NOT lower `signed_wire_cap` to force this — that cap also bounds
# every pushed chunk and FEC control packet, and an over-budget push is rejected.
MAPPING_BLOCK_CAPACITY = 2
MAPPING_AHEAD_BLOCKS = 16


def _build_fec(scheme: str, max_source_bytes: int, recovery_budget_ms: int):
    """FEC options for groups of up to FEC_GROUP_MAX_CHUNKS source packets.

    ``scheme`` is "xor" (one repair per group), "gf256" (two repairs per group)
    or "none".
    """
    from ndnsf import LiveStreamFecOptions

    if scheme == "none":
        return LiveStreamFecOptions.none()
    if scheme == "xor":
        return LiveStreamFecOptions.xor_one_repair(
            source_items=FEC_GROUP_MAX_CHUNKS,
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    if scheme == "gf256":
        return LiveStreamFecOptions.gf256_two_repair(
            source_items=FEC_GROUP_MAX_CHUNKS,
            max_source_bytes=max_source_bytes,
            recovery_budget_ms=recovery_budget_ms,
        )
    raise ValueError(f"unknown FEC scheme: {scheme!r}")


# Seconds of published history a producer must keep.
#
# Retention is configured in ITEMS, but what actually matters is how long an
# item stays fetchable, and that is items / item_rate. The wrapper default of
# 600 items is fine for a small stream and a trap for a big one: measured on
# the fleet, iuas-01 at 960x600 q75 pushes ~10 chunks x ~15.5 fps = ~155
# items/s, so 600 items is only ~3.9 SECONDS of history -- shorter than the
# 4 s Interest lifetime it may spend retrying a single lost chunk.
#
# That is a one-way trap, not a slowdown. A lost chunk stalls the ordered
# drain; while the consumer retries, the producer keeps publishing and the
# retention window slides PAST the cursor being retried; the Interest can then
# never be satisfied, so the consumer falls further behind, and so on. Measured
# consequence on iuas-01: 406-1691 timeouts, p99 inter-frame gap 8.9 s and
# 1.5-3.7 fps delivered, while wuas-01 and iuas-02 on the same medium at the
# same moment ran 20-27 fps with 0-3 timeouts. It tracks frame SIZE, which is
# why the airframe with the real camera was the one that broke.
#
# Size it in time instead, against the worst-case recovery latency.
RETENTION_SECONDS = 12.0
# Ceiling so a pathological rate cannot pin unbounded memory. At ~6 KB/item
# this is ~24 MB of published history per stream.
MAX_RETAINED_ITEMS = 4096


def retained_items_for(fps: float) -> int:
    """Items to retain so history spans `RETENTION_SECONDS` at this rate.

    Sized from the WORST case (`FRAME_MAX_CHUNKS` per frame) rather than the
    typical one: under-retaining is the trap described above, while
    over-retaining only costs memory that `MAX_RETAINED_ITEMS` bounds.
    """
    rate = max(1.0, float(fps)) * FRAME_MAX_CHUNKS
    return int(min(MAX_RETAINED_ITEMS, max(600, rate * RETENTION_SECONDS)))


def _mapping_options(StreamAdvancedOptions, block_capacity: int,
                     ahead_blocks: int, fps: float):
    """Advanced options with the Name-Map block geometry capped to one fragment.

    ``StreamAdvancedOptions`` is a FROZEN dataclass — set the fields through the
    constructor; assigning to them raises ``FrozenInstanceError``. Every other
    field keeps its wrapper default.
    """
    return StreamAdvancedOptions(
        mapping_block_capacity=int(block_capacity),
        mapping_ahead_blocks=int(ahead_blocks),
        retained_items=retained_items_for(fps),
    )


def default_video_stream_config(
    stream_id: str,
    data_prefix: str,
    *,
    fps: float,
    fec_scheme: str = "xor",
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
        # The predictive path treats each cursor as its own sample; NDNSF
        # only checks a class's hard max against the FEC source capacity, so
        # the class spans one FEC group (a frame may span several).
        sample_classes=(
            SampleClassProfile("video", 1, FEC_GROUP_MAX_CHUNKS),
        ),
        fec=_build_fec(fec_scheme, fec_max_source_bytes, fec_recovery_budget_ms),
        # Wrapper defaults are the recommended starting point, EXCEPT the
        # Name-Map block geometry: the default 16-item block is ~8.7 KB, which
        # fragments 6 ways on a Wi-Fi datagram face and is then all-or-nothing.
        # See MAPPING_BLOCK_CAPACITY.
        advanced=advanced if advanced is not None else _mapping_options(
            StreamAdvancedOptions, mapping_block_capacity, mapping_ahead_blocks, fps,
        ),
    )


class VideoStreamProducer:
    """One long-lived predictive stream per camera; push one JPEG per frame.

    Each frame is split into chunks and published as one or more FEC groups
    (see FEC_GROUP_MAX_CHUNKS), each named, signed, pushed and flushed by ONE
    NDNSF call with the GIL released.
    """

    def __init__(
        self,
        provider,
        stream_id: str,
        data_prefix: str,
        *,
        fps: float = 15.0,
        signing_identity: str = "",
        fec_scheme: str = "xor",
    ) -> None:
        self._config = default_video_stream_config(
            stream_id, data_prefix, fps=fps, fec_scheme=fec_scheme,
        )
        self._stream = provider.create_stream(self._config)
        self._descriptor = self._stream.start()
        self._signing_identity = signing_identity
        self._seq = 0
        self._frame_seq = 0
        # Frames dropped for needing more than FRAME_MAX_CHUNKS packets. Should
        # stay 0; a rising count means the encoder is producing runaway frames.
        self.frames_over_chunk_budget = 0
        # Rolling per-frame publish cost (name + sign + push + flush of every
        # group). The video loop is self-paced (`delay = 1/fps - cycle`), so
        # this bounds the achieved frame rate directly rather than queueing.
        self._publish_us = deque(maxlen=1024)
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

        Each chunk is its own signed Data carrying a `parse_chunk_header`
        prefix, and the consumer reassembles by frame sequence. The chunks go
        out in ceil(n / FEC_GROUP_MAX_CHUNKS) near-equal FEC groups, each one
        `push_signed_batch` call that names, signs, pushes and flushes with the
        GIL released. The per-chunk Python loop it replaces (make_signed_data +
        push, 17x per 1280x800 frame, then flush with the GIL held) re-queued
        for the GIL on every chunk: 96 ms per frame beside one busy Python
        thread against 8.8 ms batched (aarch64 bench), and the fleet producer
        capped at ~11.5 fps.

        Returns False for an implausibly large frame (`MAX_FRAME_BYTES`), which
        indicates an encoder problem rather than a transport limit, and for one
        needing more than `FRAME_MAX_CHUNKS` packets.
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
        groups = -(-count // FEC_GROUP_MAX_CHUNKS)
        with self._lock:
            frame_seq = self._frame_seq
            pub_ms = time.monotonic_ns() // 1_000_000
            contents = [
                _chunk_header(frame_seq, idx, count, pub_ms) + chunk
                for idx, chunk in enumerate(chunks)
            ]
            t0 = time.perf_counter()
            for g in range(groups):
                part = contents[g * count // groups : (g + 1) * count // groups]
                self._stream.push_signed_batch(
                    self._seq, part, self._signing_identity, 300, True,
                )
                self._seq += len(part)
            self._publish_us.append((time.perf_counter() - t0) * 1e6)
            self._frame_seq += 1
        return True

    def timing_stats(self) -> dict:
        """Percentiles of the per-frame publish cost, in microseconds."""
        with self._lock:
            ordered = sorted(self._publish_us)
        if not ordered:
            return {"publish_n": 0}

        def pct(q):
            return round(ordered[min(len(ordered) - 1, int(len(ordered) * q))], 1)

        return {
            "publish_n": len(ordered),
            "publish_p50_us": pct(0.50),
            "publish_p90_us": pct(0.90),
            "publish_p99_us": pct(0.99),
            "publish_max_us": round(ordered[-1], 1),
        }

    def stop(self) -> None:
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

    ``lag_ms`` is how far behind live the stream has fallen since its best
    delivered frame: producer publish time and local receive time are both
    monotonic, so their difference carries a constant unknown offset that the
    running minimum cancels. NDNSF cannot see this lag itself -- it learns the
    producer's position only from the items it delivers, which are the stale
    ones -- so the caller must act on it (resubscribe at the live edge).

    Poll ``status()`` rather than asking NDNSF for a status callback: the
    callback fires on every drained item and takes the GIL each time.

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
        interest_lifetime_ms: Optional[int] = None,
        aggregate_interest_limit: Optional[int] = None,
        fps: Optional[float] = None,
    ) -> None:
        from ndnsf import (
            LiveStreamItemAdmission,
            PredictiveStreamDescriptor,
            StreamSubscriptionOptions,
        )

        # Prefetch depth must satisfy  depth / item_rate <= interest_lifetime,
        # and the depth that matters is in ITEMS while the constant is fixed.
        # At a LOW item rate the fixed 128 over-reaches badly: measured on the
        # fleet at 320x240 q40 5 fps (~2 chunks/frame, ~10 items/s), 128 items
        # is ~12.8 s of lookahead against a 4 s lifetime, so prefetch Interests
        # expire before the frames they ask for are even produced -- 1534-2181
        # timeouts per 150 s run and p99 inter-frame gaps of 4-8 s on ALL THREE
        # airframes. At 960x600 q75 30 fps the same 128 is ~0.6 s of lookahead
        # and the identical streams ran 20-27 fps with 0-3 timeouts.
        #
        # So derive it from the rate when the caller knows it. chunks/frame is
        # not known before frames arrive, so assume the small-frame case (2);
        # under-estimating only costs pipelining, while over-estimating
        # reproduces the stall above.
        if interest_lifetime_ms is None:
            interest_lifetime_ms = (
                live_interest_lifetime_ms(fps) if fps else LIVE_INTEREST_LIFETIME_MS
            )
        if aggregate_interest_limit is None:
            if fps:
                budget = max(1.0, float(fps)) * 2.0 * (interest_lifetime_ms / 1000.0 / 2.0)
                aggregate_interest_limit = int(min(LIVE_INTEREST_LIMIT, max(8, budget)))
            else:
                aggregate_interest_limit = LIVE_INTEREST_LIMIT

        if isinstance(descriptor, (bytes, bytearray)):
            descriptor = json.loads(bytes(descriptor).decode())
        elif isinstance(descriptor, str):
            descriptor = json.loads(descriptor)
        desc = PredictiveStreamDescriptor.from_dict(descriptor)

        self.delivered = 0
        self.recovered = 0  # frames reconstructed by FEC rather than fetched
        self.lag_ms = 0
        lag_floor = {"ms": None}

        # Reassembly of segmented frames, keyed by frame sequence. A frame
        # larger than one Data arrives as several items; `on_frame` must only
        # fire once the whole JPEG is back. Bounded: a frame still incomplete
        # once `_REASM_KEEP` newer frames have completed is abandoned, so a
        # permanently-lost chunk cannot pin memory.
        pending_frames: dict = {}
        _REASM_KEEP = 4
        self.partial_frames_dropped = 0

        def _reassemble(raw, cursor):
            """-> (frame_seq, jpeg, pub_ms) once a frame is whole, else None."""
            parsed = parse_chunk_header(raw)
            if parsed is None:
                # Un-segmented publisher (or a foreign item): pass it straight
                # through, so a consumer keeps working against an old producer.
                return cursor, raw, None
            frame_seq, idx, count, pub_ms, payload = parsed
            if count <= 1:
                return frame_seq, payload, pub_ms
            slot = pending_frames.setdefault(frame_seq, {})
            slot[idx] = payload
            if len(slot) < count:
                return None
            jpeg = b"".join(slot[i] for i in range(count))
            del pending_frames[frame_seq]
            for stale in [k for k in pending_frames if k < frame_seq - _REASM_KEEP]:
                del pending_frames[stale]
                self.partial_frames_dropped += 1
            return frame_seq, jpeg, pub_ms

        def _note_lag(pub_ms):
            transit = time.monotonic_ns() // 1_000_000 - pub_ms
            if lag_floor["ms"] is None or transit < lag_floor["ms"]:
                lag_floor["ms"] = transit
            self.lag_ms = transit - lag_floor["ms"]

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
                    if done[2] is not None:
                        _note_lag(done[2])
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
            ),
        )

    def status(self):
        """NDNSF's LiveStreamStatus for this subscription (state, reason, counters)."""
        return self._subscriber.status()

    def stop(self) -> None:
        try:
            self._subscriber.stop()
        except Exception:
            pass
