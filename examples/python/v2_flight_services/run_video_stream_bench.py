#!/usr/bin/env python3
"""A/B bench: live video over the NDNSF predictive stream vs the segmented-poll path.

Runs a producer on one node and a consumer on another (or loopback) and
reports skew-immune transport quality — delivery ratio, effective fps,
inter-arrival jitter, out-of-order, and (stream only) FEC recoveries — so we
can decide whether the streaming API fixes the bursty framerate / iuas-01
stall before wiring it into the real video path.

Two transports, same synthetic frames and same metrics:
  --transport stream     : one long-lived create_stream / subscribe_stream
  --transport segmented  : today's per-frame publish_segmented latest-wins name,
                           consumer polls it (single feed, BEST case — the real
                           dashboard shares an ~8 fps cap across all vehicles).

Absolute one-way latency is reported only as a cross-check; trust it solely on
loopback. Cross-node clocks are not synchronized (same lesson as the request
bench), so the go/no-go signals are the skew-immune ones above.

Requires NDNSF >= origin/main (streaming API) for --transport stream.

Examples
--------
  # node A (producer, wuas):
  run_video_stream_bench.py --role producer --transport stream \
      --fps 30 --frame-bytes 6000 --seconds 60

  # node B (consumer, gcs):
  run_video_stream_bench.py --role consumer --transport stream \
      --seconds 60 --out /var/lib/minimuas/bench/video_stream.jsonl
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    provider_kwargs,
    user_kwargs,
)

# Fixed, well-known names for the bench so producer/consumer rendezvous with
# no configuration beyond a shared --stream-id.
BENCH_ROOT = "/muas/v2/bench/video"
HEADER_BYTES = 16  # seq(8) + send_monotonic_ns(8)


def _frame_prefix(stream_id: str) -> str:
    return f"{BENCH_ROOT}/{stream_id}"


def _descriptor_name(stream_id: str) -> str:
    return f"{_frame_prefix(stream_id)}/descriptor"


def _segmented_live_name(stream_id: str) -> str:
    return f"{_frame_prefix(stream_id)}/live"


def _make_frame(seq: int, frame_bytes: int) -> bytes:
    body = seq.to_bytes(8, "big") + time.monotonic_ns().to_bytes(8, "big")
    pad = max(0, frame_bytes - len(body))
    # Deterministic non-constant padding (varies with seq) so a codec-like
    # payload can't be trivially deduped by the transport.
    return body + bytes((seq + i) & 0xFF for i in range(pad))


def _parse_frame(payload: bytes) -> tuple[int, int]:
    seq = int.from_bytes(payload[:8], "big")
    send_ns = int.from_bytes(payload[8:16], "big")
    return seq, send_ns


# --------------------------------------------------------------------------
# Producer
# --------------------------------------------------------------------------

def run_producer(args) -> int:
    from ndnsf import ServiceProvider

    from dataplane import publish_segmented

    provider = ServiceProvider(
        **provider_kwargs(args, _frame_prefix(args.stream_id), "")
    )
    # Descriptor publish rides the provider's own runtime Face (avoid a second
    # ndn::Face in-process; same crash class fixed for the dashboard).
    from dataplane import set_runtime as _set_runtime
    _set_runtime(provider)
    period = 1.0 / max(args.fps, 1.0)
    deadline = time.monotonic() + args.seconds
    published = 0
    keepalive: list = []

    if args.transport == "stream":
        from video_stream import VideoStreamProducer

        prod = VideoStreamProducer(
            provider, args.stream_id, _frame_prefix(args.stream_id),
            fps=args.fps,
            # Frames must be signed by the stream's provider identity — the Core
            # rejects any push whose signer != definition.provider ("outside
            # provider authority"). Here the provider prefix IS that identity.
            signing_identity=_frame_prefix(args.stream_id),
            fec_scheme=args.fec_scheme,
        )
        # Advertise the descriptor on the small-payload plane so the consumer
        # can subscribe; keep it alive for late joiners for the whole run.
        keepalive.append(
            publish_segmented(
                _descriptor_name(args.stream_id),
                prod.descriptor_json,
                freshness_ms=1000,
            )
        )
        print(json.dumps({
            "event": "producer.stream.started",
            "stream_id": args.stream_id,
            "descriptor_name": _descriptor_name(args.stream_id),
        }), flush=True)
        seq = 0
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            if prod.publish_frame(_make_frame(seq, args.frame_bytes)):
                published += 1
            seq += 1
            _sleep_to(t0 + period)
        prod.stop()
    else:
        # Baseline: today's per-frame latest-wins publish_segmented.
        prev = None
        curr = None
        seq = 0
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            frame = _make_frame(seq, args.frame_bytes)
            try:
                producer = publish_segmented(
                    _segmented_live_name(args.stream_id),
                    frame,
                    freshness_ms=300,
                )
                if prev is not None:
                    try:
                        prev.stop()
                    except Exception:
                        pass
                prev, curr = curr, producer
                published += 1
            except Exception as exc:
                print(json.dumps({
                    "event": "producer.segmented.publish_failed",
                    "seq": seq, "error": str(exc),
                }), flush=True)
            seq += 1
            _sleep_to(t0 + period)

    print(json.dumps({
        "event": "producer.finished",
        "transport": args.transport,
        "published": published,
        "target_fps": args.fps,
        "seconds": args.seconds,
    }), flush=True)
    return 0


def _sleep_to(target_mono: float) -> None:
    delay = target_mono - time.monotonic()
    if delay > 0:
        time.sleep(delay)


# --------------------------------------------------------------------------
# Consumer
# --------------------------------------------------------------------------

class _Recorder:
    """Per-frame arrival record, updated from the delivery thread."""

    def __init__(self) -> None:
        self.arrivals: list[tuple[int, float, int]] = []  # (seq, recv_mono, send_ns)
        self.first_mono: float | None = None

    def record(self, payload: bytes) -> None:
        seq, send_ns = _parse_frame(payload)
        now = time.monotonic()
        if self.first_mono is None:
            self.first_mono = now
        self.arrivals.append((seq, now, send_ns))


def run_consumer(args) -> int:
    rec = _Recorder()

    if args.transport == "stream":
        from ndnsf import ServiceUser

        from dataplane import fetch_segmented, set_runtime
        from video_stream import VideoStreamConsumer

        user = ServiceUser(**user_kwargs(args, args.user))
        # Route the descriptor fetch through the user's OWN runtime Face. Without
        # this, fetch_segmented falls back to a standalone ndn::Face; two Faces in
        # one process race ndn-cxx global state and SIGSEGV under the streaming
        # runtime (same crash class fixed for the dashboard).
        set_runtime(user)
        descriptor_json = _await_descriptor(fetch_segmented, args)
        if descriptor_json is None:
            print(json.dumps({"event": "consumer.no_descriptor"}), flush=True)
            return 1

        def on_frame(_cursor: int, content: bytes) -> None:
            rec.record(content)

        consumer = VideoStreamConsumer(
            user, descriptor_json, on_frame,
            require_full_delivery=args.require_full,
        )
        print(json.dumps({"event": "consumer.stream.subscribed"}), flush=True)
        time.sleep(args.seconds)
        recovered = consumer.recovered
        consumer.stop()
        summary = _summarize(rec, args, recovered=recovered)
    else:
        from dataplane import fetch_segmented

        # Poll the latest-wins name single-feed (best case for the old path).
        name = _segmented_live_name(args.stream_id)
        period = 1.0 / max(args.fps, 1.0)
        deadline = time.monotonic() + args.seconds
        last_seq = -1
        print(json.dumps({"event": "consumer.segmented.polling"}), flush=True)
        while time.monotonic() < deadline:
            t0 = time.monotonic()
            try:
                payload = fetch_segmented(name, timeout_ms=700)
                seq, _ = _parse_frame(payload)
                if seq != last_seq:
                    last_seq = seq
                    rec.record(payload)
            except Exception:
                pass  # stream gap; next success is the live frame
            _sleep_to(t0 + period)
        summary = _summarize(rec, args, recovered=0)

    print(json.dumps(summary, indent=2), flush=True)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("a") as fh:
            fh.write(json.dumps(summary) + "\n")
    return 0


def _await_descriptor(fetch_segmented, args):
    name = _descriptor_name(args.stream_id)
    deadline = time.monotonic() + args.descriptor_timeout
    while time.monotonic() < deadline:
        try:
            return fetch_segmented(name, timeout_ms=1000)
        except Exception:
            time.sleep(0.5)
    return None


def _summarize(rec: _Recorder, args, *, recovered: int) -> dict:
    arrivals = rec.arrivals
    n = len(arrivals)
    if n == 0:
        return {
            "event": "summary", "transport": args.transport,
            "delivered": 0, "note": "no frames received",
        }
    seqs = [a[0] for a in arrivals]
    recv = [a[1] for a in arrivals]
    span_seq = max(seqs) - min(seqs) + 1
    window_s = recv[-1] - recv[0] if n > 1 else args.seconds

    # Inter-arrival gaps in arrival order — the skew-immune jitter signal.
    gaps_ms = [(recv[i] - recv[i - 1]) * 1000.0 for i in range(1, n)]
    # Out-of-order: an arrival whose seq is below the running max.
    ooo = 0
    running_max = seqs[0]
    for s in seqs[1:]:
        if s < running_max:
            ooo += 1
        else:
            running_max = s

    # One-way latency (loopback only — cross-node clocks are unsynced).
    # recv is time.monotonic() seconds, send is time.monotonic_ns(); same
    # host shares the clock origin, so recv*1e9 - send is valid nanoseconds.
    lat_ms = (
        [(r * 1e9 - s) / 1e6 for (_seq, r, s) in arrivals]
        if args.loopback_latency else []
    )

    def pct(values, q):
        if not values:
            return None
        idx = min(len(values) - 1, int(round(q * (len(values) - 1))))
        return round(sorted(values)[idx], 2)

    return {
        "event": "summary",
        "transport": args.transport,
        "stream_id": args.stream_id,
        "target_fps": args.fps,
        "fec_scheme": args.fec_scheme if args.transport == "stream" else None,
        "delivered": n,
        "unique_seqs": len(set(seqs)),
        "seq_span": span_seq,
        "delivery_ratio": round(len(set(seqs)) / span_seq, 4),
        "effective_fps": round(n / window_s, 2) if window_s > 0 else None,
        "gap_ms_p50": pct(gaps_ms, 0.50),
        "gap_ms_p95": pct(gaps_ms, 0.95),
        "gap_ms_max": round(max(gaps_ms), 2) if gaps_ms else None,
        "gap_ms_stdev": round(statistics.pstdev(gaps_ms), 2) if len(gaps_ms) > 1 else None,
        "out_of_order": ooo,
        "fec_recovered": recovered,
        "loopback_latency_ms_p50": pct(lat_ms, 0.50) if lat_ms else None,
        "loopback_latency_ms_p95": pct(lat_ms, 0.95) if lat_ms else None,
    }


# --------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    add_common_arguments(p)
    p.add_argument("--role", required=True, choices=["producer", "consumer"])
    p.add_argument("--transport", default="stream",
                   choices=["stream", "segmented"])
    p.add_argument("--user", default="/muas/v2/gcs",
                   help="consumer ServiceUser prefix")
    p.add_argument("--stream-id", default="wuas-01")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--frame-bytes", type=int, default=6000,
                   help="synthetic frame size (bytes); frames are chunked, so any "
                        "size up to video_stream.MAX_FRAME_BYTES is valid")
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--descriptor-timeout", type=float, default=30.0,
                   help="consumer: seconds to wait for the producer descriptor")
    p.add_argument("--require-full", action="store_true",
                   help="stream consumer: require_full_delivery")
    p.add_argument("--fec-scheme", default="xor",
                   choices=["none", "xor", "gf256"],
                   help="stream producer: FEC scheme per chunk group "
                        "(xor = one repair, gf256 = two repairs)")
    p.add_argument("--loopback-latency", action="store_true",
                   help="also report one-way latency (valid only same-host)")
    p.add_argument("--out", type=Path, default=None,
                   help="append the summary JSON to this JSONL file")
    args = p.parse_args()

    add_ndnsf_path(args.ndnsf_root)
    if args.role == "producer":
        return run_producer(args)
    return run_consumer(args)


if __name__ == "__main__":
    sys.exit(main())
