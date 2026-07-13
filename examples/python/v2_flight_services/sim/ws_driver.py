#!/usr/bin/env python3
"""Drive run_dashboard.py over its /ws WebSocket, exactly as the browser UI does.

run_dashboard.py exposes NO POST endpoints; every operator action (start a
raster search, task an audio capture, send a flight command) is a JSON message
on ws://<host>:8080/ws. This is the headless equivalent of clicking the UI, used
both by an operator from a terminal and by sim/smoke.py.

Examples:
    # raster search on wuas-01 -> detection -> auto-dispatched investigation
    python3 ws_driver.py mission

    # audio interrogation routed to the mic airframe iuas-02
    python3 ws_driver.py audio --vehicle iuas-02

    # get vehicles airborne (helps trip cooperative avoidance during a mission)
    python3 ws_driver.py takeoff --vehicle wuas-01
    python3 ws_driver.py takeoff --all

    # fleet return-to-launch (exercises slot-layered smart RTL)
    python3 ws_driver.py rtl --all

    # raw passthrough
    python3 ws_driver.py raw --json '{"cmd":"flight","vehicle":"iuas-01","command":"hold"}'
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys

import aiohttp

CENTER_LAT = 35.1208
CENTER_LON = -89.9347


def mission_params() -> dict:
    return {
        "area": {
            "mode": "center",
            "center_lat": CENTER_LAT,
            "center_lon": CENTER_LON,
            "width_m": 24.0,
            "height_m": 18.0,
        },
        "agl_m": 6.0,
        "leg_spacing_m": 6.0,
        "speed_m_s": 3.0,
        "capture_every_m": 4.0,
        "object_query": "tennis racket",
        "min_confidence": 0.1,
        "max_duration_s": 300.0,
        "investigate_sensors": ["camera", "audio"],
    }


def build_messages(args) -> list[dict]:
    cmd = args.command
    if cmd == "mission":
        return [{"cmd": "start_mission", "params": mission_params()}]
    if cmd == "audio":
        return [{
            "cmd": "sensor",
            "vehicle": args.vehicle,
            "params": {
                "sensor": "audio",
                "mode": "now",
                "duration_s": args.duration_s,
                "target": {"lat": CENTER_LAT + 0.0001, "lon": CENTER_LON},
                "radius_m": 80.0,
                "note": "sim audio interrogation",
            },
        }]
    if cmd in ("takeoff", "hold", "land", "rtl"):
        if args.all:
            if cmd == "takeoff":
                return [{"cmd": "flight", "vehicle": v, "command": "takeoff"}
                        for v in ("wuas-01", "iuas-01", "iuas-02")]
            return [{"cmd": "all", "command": cmd}]
        return [{"cmd": "flight", "vehicle": args.vehicle, "command": cmd}]
    if cmd == "raw":
        return [json.loads(args.json)]
    raise SystemExit(f"unknown command {cmd!r}")


async def drive(args) -> int:
    base = args.url.rstrip("/")
    ws_url = base.replace("http://", "ws://").replace("https://", "wss://") + "/ws"
    messages = build_messages(args)
    timeout = aiohttp.ClientTimeout(total=None)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        async with session.ws_connect(ws_url, heartbeat=15) as ws:
            for m in messages:
                await ws.send_str(json.dumps(m))
                print(json.dumps({"sent": m}), flush=True)

            if args.listen <= 0:
                return 0

            deadline = asyncio.get_event_loop().time() + args.listen
            wanted = set(args.show.split(",")) if args.show else None
            while asyncio.get_event_loop().time() < deadline:
                remaining = deadline - asyncio.get_event_loop().time()
                try:
                    msg = await asyncio.wait_for(ws.receive(), timeout=remaining)
                except asyncio.TimeoutError:
                    break
                if msg.type != aiohttp.WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(msg.data)
                except Exception:
                    continue
                mtype = payload.get("type")
                if wanted is not None and mtype not in wanted:
                    continue
                # keep console readable: telemetry is high-rate
                if mtype == "telemetry":
                    s = payload.get("sample", {})
                    print(json.dumps({"type": "telemetry",
                                      "vehicle": payload.get("vehicle"),
                                      "agl_m": s.get("agl_m"),
                                      "avoid_bias_m": s.get("avoid_bias_m")}), flush=True)
                else:
                    print(json.dumps(payload)[:600], flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Drive the miniMUAS v2 dashboard over /ws")
    p.add_argument("command",
                   choices=["mission", "audio", "takeoff", "hold", "land", "rtl", "raw"])
    p.add_argument("--url", default="http://127.0.0.1:8080")
    p.add_argument("--vehicle", default="iuas-02")
    p.add_argument("--all", action="store_true", help="apply flight command to whole fleet")
    p.add_argument("--duration-s", type=float, default=4.0)
    p.add_argument("--json", default="{}", help="raw command JSON (with 'raw')")
    p.add_argument("--listen", type=float, default=0.0,
                   help="seconds to stream broadcast events after sending")
    p.add_argument("--show", default="",
                   help="comma-separated message types to print (default: all)")
    return p


def main() -> int:
    args = build_parser().parse_args()
    try:
        return asyncio.run(drive(args))
    except aiohttp.ClientError as exc:
        print(f"ws_driver: connection error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
