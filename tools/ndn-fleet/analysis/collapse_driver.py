#!/usr/bin/env python3
"""Replay the operator sequence that collapsed every stream on `nfd wifi` (2026-10-05), through
the dashboard's WebSocket, and record what the operator would have seen.

    collapse_driver.py <out.jsonl> [--host minidronesys-03.uom.memphis.edu] [--high iuas-01]

Runs for the length of the `nfd-wifi-collapse` spec's idle window; start it when ndn-fleet logs
"idle: 480 s" (the window opens; the collectors are already running). Every command sent and every video_stats / telemetry /
telemetry_stale / command event received is one JSON line with its unix time.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from flightcheck import WS  # noqa: E402

VEHICLES = ["iuas-01", "iuas-02", "wuas-01"]
SMOOTH = {"width": 640, "height": 400, "fps": 20, "quality": 55}  # dashboard preset 0
DETAIL = {"width": 1280, "height": 800, "fps": 15, "quality": 75}  # dashboard preset 2


def plan(high: str):
    """(seconds after start, label, [(vehicle, enable, preset)])."""
    allv = lambda on, p: [(v, on, p) for v in VEHICLES]
    return [
        (5, "all smooth 640p20", allv(True, SMOOTH)),
        (65, f"{high} -> detail 1280p15", [(high, True, DETAIL)]),
        (80, f"{high} -> smooth 640p20", [(high, True, SMOOTH)]),
        (170, "all off", allv(False, SMOOTH)),
        (230, "all on again at smooth 640p20", allv(True, SMOOTH)),
        (350, "all off", allv(False, SMOOTH)),
        (475, "end", []),
    ]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--host", default="minidronesys-03.uom.memphis.edu")
    ap.add_argument("--high", default="iuas-01")
    args = ap.parse_args()
    ws = WS(args.host, 8080)
    ws.s.settimeout(0.2)
    out = open(args.out, "a")

    def rec(**kw):
        out.write(json.dumps({"t": round(time.time(), 3), **kw}) + "\n")
        out.flush()

    t0 = time.time()
    steps = plan(args.high)
    rec(kind="driver.start", plan=[(s, l) for s, l, _ in steps])
    i = 0
    while i < len(steps):
        at, label, cmds = steps[i]
        if time.time() - t0 >= at:
            rec(kind="driver.step", label=label)
            for vid, on, p in cmds:
                params = {"enable": on, "transport": "stream", **p}
                ws.send_json({"cmd": "video", "vehicle": vid, "params": params})
                rec(kind="driver.sent", vehicle=vid, params=params)
            i += 1
            continue
        try:
            op, pay = ws.recv()
        except socket.timeout:
            continue
        if op == 0x2:
            continue  # a video frame (binary)
        try:
            m = json.loads(pay)
        except Exception:
            continue
        k = m.get("kind") or m.get("type")
        if m.get("type") == "event" or k in ("video_stats", "telemetry_stale", "video_settings"):
            rec(kind=k, **{x: m[x] for x in m if x not in ("type", "kind", "t", "sample")})
        elif k == "telemetry":
            rec(kind="telemetry", vehicle=m.get("vehicle"), age_s=m.get("age_s"),
                sample_age_ms=m.get("sample_age_ms"))
    rec(kind="driver.end")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
