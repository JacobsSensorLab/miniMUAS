#!/usr/bin/env python3
"""NDNSF request benchmark harness (a ServiceUser driving the bench/echo service).

Sweeps the independent variables — request mode (two-phase handshake vs
targeted), response payload size, concurrency, and tokens on/off — against a
provider's synthetic bench/echo service, and records per-request latency,
provider-stamped phase breakdown (when the native runtime supplies it),
success/timeout, response bytes, and NFD packet/byte counter deltas.

Writes raw per-request JSONL (--out) plus a per-cell aggregate summary
(--summary). Report generation (CDFs, box plots, tables) is done off-node from
the raw JSONL, so this stays dependency-light.

Run on the GCS against a drone provider for the 1-hop case, e.g.:
  muas-v2-bench --vehicle-id iuas-01 --provider /muas/v2/iuas-01 \
    --out /var/lib/minimuas/bench/raw.jsonl --summary /var/lib/minimuas/bench/summary.json
Stop muas-v2-dashboard first so the --user identity (/muas/v2/gcs) isn't shared.
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
import statistics
import subprocess
import threading
import time
from pathlib import Path

from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    print_json,
    user_kwargs,
)


# ---- NFD counter snapshot (packet + byte overhead) -------------------------
def nfd_snapshot() -> dict:
    """Node-wide NFD counters: forwarder generalStatus scalars + summed face
    byte counters. Node-wide (includes background telemetry/SVS traffic), so
    per-request deltas are approximate — but two-phase vs targeted still shows
    the extra discovery/ACK interests clearly."""
    if shutil.which("nfdc") is None:
        return {}
    try:
        out = subprocess.run(
            ["nfdc", "status", "report", "json"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        report = json.loads(out.stdout)
    except Exception:
        return {}
    snap: dict = {}
    gs = report.get("nfdStatus", report).get("generalStatus", {})
    for k in ("nInInterests", "nOutInterests", "nInData", "nOutData",
              "nSatisfiedInterests", "nUnsatisfiedInterests"):
        if isinstance(gs.get(k), (int, float)):
            snap[k] = gs[k]
    # sum face-level byte counters
    in_b = out_b = 0
    faces = report.get("nfdStatus", report).get("faces", [])
    if isinstance(faces, list):
        for f in faces:
            c = (f or {}).get("counters", {})
            if isinstance(c.get("nInBytes"), (int, float)):
                in_b += c["nInBytes"]
            if isinstance(c.get("nOutBytes"), (int, float)):
                out_b += c["nOutBytes"]
    snap["nInBytes"] = in_b
    snap["nOutBytes"] = out_b
    return snap


def snap_delta(before: dict, after: dict) -> dict:
    return {k: after.get(k, 0) - before.get(k, 0)
            for k in set(before) | set(after)}


# ---- one benchmark cell ----------------------------------------------------
def make_payload(resp_size: int, delay_ms: int, req_pad: int) -> bytes:
    header = resp_size.to_bytes(4, "big") + delay_ms.to_bytes(4, "big")
    return header + (b"\x00" * req_pad)


def run_cell(user, cell: dict, service: str, provider: str, writer) -> dict:
    """Issue cell['n'] requests at cell['concurrency'], return an aggregate."""
    n = cell["n"]
    mode = cell["mode"]
    payload = make_payload(cell["resp_size"], cell["delay_ms"], cell.get("req_pad", 0))
    timeout_ms = cell.get("timeout_ms", 5000)

    if hasattr(user, "set_use_tokens"):
        user.set_use_tokens(cell["tokens"])

    sem = threading.BoundedSemaphore(cell["concurrency"])
    lock = threading.Lock()
    done = threading.Event()
    records: list = []
    counters = {"done": 0}

    def finish(sent_ns: int, resp, ok: bool):
        recv_ns = time.time_ns()
        rec = {
            "cell": cell["name"], "mode": mode, "resp_size": cell["resp_size"],
            "concurrency": cell["concurrency"], "tokens": cell["tokens"],
            "delay_ms": cell["delay_ms"],
            "total_ms": (recv_ns - sent_ns) / 1e6,
            "ok": bool(ok),
        }
        if ok and resp is not None:
            rec["status"] = bool(resp.status)
            rec["resp_bytes"] = len(resp.payload) if resp.payload else 0
            rec["error"] = resp.error or ""
            timing = getattr(resp, "timing", {}) or {}
            if timing:
                # cross-node breakdown; meaningful only to fleet-clock-sync
                # accuracy (~ms), so treat as best-effort for the 1-hop case.
                rr = timing.get("request_received_ns", 0)
                rs = timing.get("response_sent_ns", 0)
                if rr and rs:
                    rec["out_ms"] = (rr - sent_ns) / 1e6
                    rec["proc_ms"] = (rs - rr) / 1e6
                    rec["back_ms"] = (recv_ns - rs) / 1e6
        with lock:
            records.append(rec)
            writer.write(json.dumps(rec, sort_keys=True) + "\n")
            counters["done"] += 1
            if counters["done"] >= n:
                done.set()
        sem.release()

    def on_resp(sent_ns):
        return lambda resp: finish(sent_ns, resp, True)

    def on_to(sent_ns):
        return lambda _rid: finish(sent_ns, None, False)

    before = nfd_snapshot()
    t0 = time.time_ns()
    for _ in range(n):
        sem.acquire()
        sent_ns = time.time_ns()
        if mode == "targeted":
            user.request_service_targeted_async(
                provider, service, payload,
                on_response=on_resp(sent_ns), on_timeout=on_to(sent_ns),
                timeout_ms=timeout_ms,
            )
        else:
            user.request_service_async(
                service, payload,
                on_response=on_resp(sent_ns), on_timeout=on_to(sent_ns),
                timeout_ms=timeout_ms,
            )
    done.wait(timeout=max(30.0, n * timeout_ms / 1000.0))
    wall_s = (time.time_ns() - t0) / 1e9
    after = nfd_snapshot()

    return aggregate(cell, records, wall_s, snap_delta(before, after))


def pct(xs: list, p: float) -> float:
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = (len(xs) - 1) * p
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def aggregate(cell: dict, records: list, wall_s: float, nfd: dict) -> dict:
    oks = [r for r in records if r["ok"] and r.get("status")]
    lat = [r["total_ms"] for r in oks]
    n = len(records)
    n_ok = len(oks)
    agg = {
        **{k: cell[k] for k in ("name", "mode", "resp_size", "concurrency",
                                "tokens", "delay_ms")},
        "requests": n,
        "success": n_ok,
        "success_rate": (n_ok / n) if n else 0.0,
        "wall_s": round(wall_s, 3),
        "throughput_rps": round(n_ok / wall_s, 2) if wall_s > 0 else 0.0,
    }
    if lat:
        agg["latency_ms"] = {
            "min": round(min(lat), 3), "mean": round(statistics.mean(lat), 3),
            "p50": round(pct(lat, 0.50), 3), "p90": round(pct(lat, 0.90), 3),
            "p95": round(pct(lat, 0.95), 3), "p99": round(pct(lat, 0.99), 3),
            "p999": round(pct(lat, 0.999), 3), "max": round(max(lat), 3),
            "stddev": round(statistics.pstdev(lat), 3) if len(lat) > 1 else 0.0,
        }
        if n_ok and cell["resp_size"] and wall_s > 0:
            agg["goodput_mbps"] = round(
                n_ok * cell["resp_size"] * 8 / 1e6 / wall_s, 3)
        oi = nfd.get("nOutInterests")
        if oi is not None and n_ok:
            agg["out_interests_per_req"] = round(oi / n_ok, 2)
        ob = nfd.get("nOutBytes")
        if ob is not None and n_ok:
            agg["out_bytes_per_req"] = round(ob / n_ok, 1)
    agg["nfd_delta"] = nfd
    return agg


# ---- sweep matrix ----------------------------------------------------------
def default_matrix() -> list:
    """~2400 requests across mode x {payload, concurrency, tokens}, 1-hop."""
    cells = []

    def add(name, mode, resp_size, concurrency, tokens, n, delay_ms=0):
        cells.append({"name": name, "mode": mode, "resp_size": resp_size,
                      "concurrency": concurrency, "tokens": tokens, "n": n,
                      "delay_ms": delay_ms})

    for mode in ("two-phase", "targeted"):
        # A: payload sweep (concurrency 1, tokens off)
        for size in (64, 4096, 65536):
            add(f"A_payload_{size}_{mode}", mode, size, 1, False, 120)
        # B: concurrency sweep (256 B, tokens off)
        for c in (1, 4, 16):
            add(f"B_conc_{c}_{mode}", mode, 256, c, False, 200)
        # C: tokens sweep (256 B, concurrency 1)
        for tok in (False, True):
            add(f"C_tokens_{'on' if tok else 'off'}_{mode}", mode, 256, 1, tok, 120)
    return cells


def main() -> int:
    p = argparse.ArgumentParser(description="NDNSF request benchmark harness")
    add_common_arguments(p)
    p.add_argument("--user", default="/muas/v2/gcs")
    p.add_argument("--vehicle-id", default="iuas-01",
                   help="Provider vehicle id (builds the bench service name).")
    p.add_argument("--provider", default=None,
                   help="Provider prefix for targeted requests "
                        "(default /muas/v2/<vehicle-id>).")
    p.add_argument("--out", type=Path, default=Path("/var/lib/minimuas/bench/raw.jsonl"))
    p.add_argument("--summary", type=Path,
                   default=Path("/var/lib/minimuas/bench/summary.json"))
    p.add_argument("--time-budget-s", type=float, default=840.0)
    p.add_argument("--warmup", type=int, default=15)
    p.add_argument("--settle-ms", type=int, default=300,
                   help="Idle gap between cells so background traffic settles.")
    args = p.parse_args()

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import ServiceUser  # noqa: E402
    from contracts import vehicle_bench_service  # noqa: E402

    service = vehicle_bench_service(args.vehicle_id)
    provider = args.provider or f"/muas/v2/{args.vehicle_id}"
    args.out.parent.mkdir(parents=True, exist_ok=True)

    print_json("bench.start", user=args.user, service=service, provider=provider,
               controller=args.controller, group=args.group)
    user = ServiceUser(**user_kwargs(args, args.user))
    user.start()
    time.sleep(1.0)

    # warm-up (discarded): brings ABE keys / faces / discovery up to steady state
    if args.warmup > 0:
        wu_done = threading.Event()
        wu = {"n": 0}
        wu_lock = threading.Lock()

        def _wu_fin(*_a):
            with wu_lock:
                wu["n"] += 1
                if wu["n"] >= args.warmup:
                    wu_done.set()

        payload = make_payload(64, 0, 0)
        for _ in range(args.warmup):
            user.request_service_async(service, payload,
                                       on_response=lambda r: _wu_fin(),
                                       on_timeout=lambda rid: _wu_fin(),
                                       timeout_ms=5000)
        wu_done.wait(timeout=60.0)
        print_json("bench.warmup.done", n=wu["n"])

    matrix = default_matrix()
    random.Random(1).shuffle(matrix)  # deterministic de-biasing of cell order

    summaries = []
    t_start = time.monotonic()
    with open(args.out, "w") as writer:
        for i, cell in enumerate(matrix):
            if time.monotonic() - t_start > args.time_budget_s:
                print_json("bench.budget_exceeded", ran=i, total=len(matrix),
                           skipped=[c["name"] for c in matrix[i:]])
                break
            print_json("bench.cell.start", **{k: cell[k] for k in
                       ("name", "mode", "resp_size", "concurrency", "tokens", "n")})
            agg = run_cell(user, cell, service, provider, writer)
            summaries.append(agg)
            print_json("bench.cell.done", name=cell["name"],
                       success_rate=agg["success_rate"],
                       throughput_rps=agg["throughput_rps"],
                       p50=agg.get("latency_ms", {}).get("p50"),
                       p99=agg.get("latency_ms", {}).get("p99"))
            time.sleep(args.settle_ms / 1000.0)

    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(
        {"service": service, "provider": provider, "user": args.user,
         "cells": summaries}, indent=2, sort_keys=True))
    print_json("bench.done", cells=len(summaries), out=str(args.out),
               summary=str(args.summary))
    try:
        user.stop()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
