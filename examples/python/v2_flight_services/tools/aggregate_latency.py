#!/usr/bin/env python3
"""Offline aggregator for miniMUAS v2 `metric.*` latency journals.

Runs OFF-NODE on the collected `*.jsonl` metrics journals (plain stdlib;
matplotlib optional for plots). It replays every `metric.latency` event into
per-stage/per-name statistics in the SAME shape as the v1 C++
`src/metrics.hpp` (min / avg / p95 / p99 / success-rate, plus p50 / count) and
writes a CSV so a v1 vs v2 comparison is a direct diff for the NDNSF
maintainer.

Percentiles use the exact `metrics.hpp` estimator (index = floor(p * n),
clamped to n-1, on the sorted sample) so numbers line up with the C++ export.
No NDN imports; this never touches the fabric.

metric.latency schema (emitted by metrics.py):
    {"event":"metric.latency","stage":"service|fetch|crypto|nfd",
     "name":<label>,"total_ms":float,
     "status":bool?,           # success flag (absent -> treated as success)
     "out_ms":float?,"proc_ms":float?,"back_ms":float?,   # provider legs
     ...ctx (service, vehicle, target, timeout)}

Usage:
    python3 tools/aggregate_latency.py results/v2_ndnsf/*.jsonl
    python3 tools/aggregate_latency.py <dir-or-globs> --csv out.csv
    python3 tools/aggregate_latency.py <...> --raw-csv raw.csv   # per-entry
    python3 tools/aggregate_latency.py <...> --plot plots/       # histograms
    python3 tools/aggregate_latency.py <...> --include-rtt       # fold legacy
    python3 tools/aggregate_latency.py --selftest                # unit check
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

# The latency legs metrics.py may attach; also the metric we aggregate on.
_TOTAL_KEY = "total_ms"


def percentile(sorted_values, p):
    """metrics.hpp percentile: sorted[min(floor(p*n), n-1)]."""
    n = len(sorted_values)
    if n == 0:
        return 0.0
    index = int(p * n)
    if index > n - 1:
        index = n - 1
    return float(sorted_values[index])


def summarize(values, successes, fails):
    """Return the metrics.hpp Stats dict for one (stage, name) bucket."""
    count = len(values)
    if count == 0:
        return {
            "count": 0, "success": successes, "fail": fails,
            "success_rate": 0.0, "min_ms": 0.0, "max_ms": 0.0,
            "avg_ms": 0.0, "p50_ms": 0.0, "p95_ms": 0.0, "p99_ms": 0.0,
        }
    ordered = sorted(values)
    total = successes + fails
    return {
        "count": count,
        "success": successes,
        "fail": fails,
        "success_rate": (successes / total) if total else 0.0,
        "min_ms": float(ordered[0]),
        "max_ms": float(ordered[-1]),
        "avg_ms": sum(ordered) / count,
        "p50_ms": percentile(ordered, 0.50),
        "p95_ms": percentile(ordered, 0.95),
        "p99_ms": percentile(ordered, 0.99),
    }


def iter_metric_events(paths):
    """Yield parsed `metric.*` JSON objects from the given files/globs."""
    for spec in paths:
        if os.path.isdir(spec):
            matches = glob.glob(os.path.join(spec, "*.jsonl"))
        else:
            matches = glob.glob(spec)
        for path in sorted(matches):
            if os.path.isdir(path):
                continue
            try:
                with open(path, "r", encoding="utf-8") as handle:
                    for line in handle:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            obj = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        event = obj.get("event", "")
                        if isinstance(event, str) and event.startswith("metric."):
                            yield obj
            except OSError as exc:
                print(f"warning: cannot read {path}: {exc}", file=sys.stderr)


def collect(events, *, include_rtt=False):
    """Fold events into per-(stage,name) samples plus nfd-counter last values.

    Returns (buckets, raw_rows, nfd_last) where:
      buckets[(stage,name)] = {"values":[ms...], "success":n, "fail":n}
      raw_rows = [(stage, name, total_ms, success_int), ...]
      nfd_last = {counter: last_value}
    """
    buckets = defaultdict(lambda: {"values": [], "success": 0, "fail": 0})
    raw_rows = []
    nfd_last = {}

    def add(stage, name, total_ms, status):
        b = buckets[(stage, name)]
        b["values"].append(float(total_ms))
        # A record with no explicit status completed (returned) -> success.
        ok = status is not False
        if ok:
            b["success"] += 1
        else:
            b["fail"] += 1
        raw_rows.append((stage, name, float(total_ms), 1 if ok else 0))

    for obj in events:
        event = obj["event"]
        if event == "metric.latency":
            total = obj.get(_TOTAL_KEY)
            if total is None:
                continue
            add(obj.get("stage", "service"), obj.get("name", "?"), total,
                obj.get("status"))
        elif event == "metric.service_rtt" and include_rtt:
            # Legacy same-node RTT (run_wuas_user). Folded into stage=service
            # only on request, to avoid double-counting the metric.latency the
            # same call now also emits.
            rtt = obj.get("rtt_ms")
            if rtt is not None:
                add("service", obj.get("service", "?"), rtt, obj.get("status"))
        elif event == "metric.nfd_counters":
            for key, value in obj.items():
                if key in ("event", "ts"):
                    continue
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    nfd_last[key] = value

    return buckets, raw_rows, nfd_last


_STAGE_ORDER = {"service": 0, "fetch": 1, "crypto": 2, "nfd": 3}


def rows_from_buckets(buckets):
    rows = []
    for (stage, name), b in buckets.items():
        stats = summarize(b["values"], b["success"], b["fail"])
        rows.append({"stage": stage, "name": name, **stats})
    rows.sort(key=lambda r: (_STAGE_ORDER.get(r["stage"], 9), r["stage"], r["name"]))
    return rows


_CSV_FIELDS = [
    "stage", "name", "count", "success", "fail", "success_rate",
    "min_ms", "avg_ms", "p50_ms", "p95_ms", "p99_ms", "max_ms",
]


def write_csv(rows, path):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(_CSV_FIELDS)
        for r in rows:
            writer.writerow([
                r["stage"], r["name"], r["count"], r["success"], r["fail"],
                f"{r['success_rate']:.4f}",
                f"{r['min_ms']:.3f}", f"{r['avg_ms']:.3f}", f"{r['p50_ms']:.3f}",
                f"{r['p95_ms']:.3f}", f"{r['p99_ms']:.3f}", f"{r['max_ms']:.3f}",
            ])


def write_raw_csv(raw_rows, path):
    """Per-entry export mirroring metrics.hpp exportCSV (latency_ms,success)
    with stage/name columns so the buckets stay distinguishable."""
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["stage", "name", "latency_ms", "success"])
        for stage, name, total_ms, ok in raw_rows:
            writer.writerow([stage, name, f"{total_ms:.3f}", ok])


def print_table(rows, nfd_last):
    if not rows:
        print("(no metric.latency events found)")
    else:
        header = (
            f"{'stage':<8} {'name':<18} {'n':>5} {'ok%':>6} "
            f"{'min':>8} {'avg':>8} {'p50':>8} {'p95':>8} {'p99':>8} {'max':>8}"
        )
        print("\n--- NDNSF Python-boundary latency (ms) ---")
        print(header)
        print("-" * len(header))
        for r in rows:
            print(
                f"{r['stage']:<8} {r['name']:<18} {r['count']:>5} "
                f"{r['success_rate'] * 100:>6.1f} "
                f"{r['min_ms']:>8.2f} {r['avg_ms']:>8.2f} {r['p50_ms']:>8.2f} "
                f"{r['p95_ms']:>8.2f} {r['p99_ms']:>8.2f} {r['max_ms']:>8.2f}"
            )
    if nfd_last:
        print("\n--- NFD counters (last observed) ---")
        for key in sorted(nfd_last):
            print(f"  {key:<28} {nfd_last[key]}")


def make_plots(buckets, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"warning: matplotlib unavailable, skipping plots ({exc})",
              file=sys.stderr)
        return
    os.makedirs(out_dir, exist_ok=True)
    for (stage, name), b in buckets.items():
        if not b["values"]:
            continue
        fig, ax = plt.subplots()
        bins = max(10, int(math.sqrt(len(b["values"]))))
        ax.hist(b["values"], bins=bins)
        ax.set_title(f"{stage}/{name} latency (n={len(b['values'])})")
        ax.set_xlabel("latency (ms)")
        ax.set_ylabel("count")
        safe = f"{stage}_{name}".replace("/", "_").replace(":", "_")
        fig.savefig(os.path.join(out_dir, f"{safe}.png"), dpi=100,
                    bbox_inches="tight")
        plt.close(fig)
    print(f"wrote plots to {out_dir}/")


def selftest():
    """Feed synthetic metric.latency events through the pipeline and assert
    the CSV stats reproduce the metrics.hpp math."""
    # service/detect: latencies 10..100 ms, one failure at the top.
    events = []
    for i in range(1, 101):
        events.append({
            "event": "metric.latency", "stage": "service", "name": "detect",
            "total_ms": float(i * 10), "status": i != 100,
        })
    # fetch/frame_fetch: a couple of samples, all success (one no status).
    events.append({"event": "metric.latency", "stage": "fetch",
                   "name": "frame_fetch", "total_ms": 5.0, "status": True})
    events.append({"event": "metric.latency", "stage": "fetch",
                   "name": "frame_fetch", "total_ms": 15.0})  # no status
    events.append({"event": "metric.nfd_counters", "nInInterests": 42,
                   "nOutData": 7, "ts": 123.0})

    buckets, raw_rows, nfd_last = collect(events)
    rows = rows_from_buckets(buckets)
    by = {(r["stage"], r["name"]): r for r in rows}

    detect = by[("service", "detect")]
    assert detect["count"] == 100, detect
    assert detect["min_ms"] == 10.0, detect
    assert detect["max_ms"] == 1000.0, detect
    assert abs(detect["avg_ms"] - 505.0) < 1e-9, detect
    # metrics.hpp percentile: floor(p*n) index into sorted (1-based values
    # 10..1000). p50 -> index 50 -> 510; p95 -> index 95 -> 960;
    # p99 -> index 99 -> 1000.
    assert detect["p50_ms"] == 510.0, detect
    assert detect["p95_ms"] == 960.0, detect
    assert detect["p99_ms"] == 1000.0, detect
    assert detect["success"] == 99 and detect["fail"] == 1, detect
    assert abs(detect["success_rate"] - 0.99) < 1e-9, detect

    frame = by[("fetch", "frame_fetch")]
    assert frame["count"] == 2, frame
    assert frame["success"] == 2 and frame["fail"] == 0, frame  # no-status = ok
    assert frame["min_ms"] == 5.0 and frame["max_ms"] == 15.0, frame

    assert nfd_last == {"nInInterests": 42, "nOutData": 7}, nfd_last
    assert len(raw_rows) == 102, len(raw_rows)

    # ordering: service before fetch
    assert [r["stage"] for r in rows][:2] == ["service", "fetch"], rows

    # percentile estimator edge cases
    assert percentile([], 0.95) == 0.0
    assert percentile([1.0], 0.99) == 1.0
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.95) == 4.0  # floor(3.8)=3

    print("selftest OK")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="*",
                        help="*.jsonl files, globs, or directories to scan")
    parser.add_argument("--csv", help="write per-stage summary CSV to this path")
    parser.add_argument("--raw-csv",
                        help="write per-entry CSV (stage,name,latency_ms,success)")
    parser.add_argument("--plot", metavar="DIR",
                        help="write per-bucket latency histograms (needs matplotlib)")
    parser.add_argument("--include-rtt", action="store_true",
                        help="also fold legacy metric.service_rtt into stage=service")
    parser.add_argument("--selftest", action="store_true",
                        help="run built-in synthetic-data unit checks and exit")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.paths:
        build_parser().error("no input paths (or use --selftest)")

    events = list(iter_metric_events(args.paths))
    buckets, raw_rows, nfd_last = collect(events, include_rtt=args.include_rtt)
    rows = rows_from_buckets(buckets)

    print_table(rows, nfd_last)
    if args.csv:
        write_csv(rows, args.csv)
        print(f"\nwrote summary CSV to {args.csv}")
    if args.raw_csv:
        write_raw_csv(raw_rows, args.raw_csv)
        print(f"wrote raw CSV ({len(raw_rows)} entries) to {args.raw_csv}")
    if args.plot:
        make_plots(buckets, args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
