#!/usr/bin/env python3
"""Diagnostic: can this endpoint deliver GLOBAL_POSITION_INT on request?

Standalone (pymavlink only, no relay imports), single-threaded — no GCS
heartbeat thread, so any failure here is the transport or the autopilot,
not concurrency. Connects, waits for the FC heartbeat, asks for telemetry
two ways (REQUEST_DATA_STREAM and MAV_CMD_SET_MESSAGE_INTERVAL for
GLOBAL_POSITION_INT), then prints a histogram of everything received.

Reading the result:
  * COMMAND_ACK present          -> outbound write path works
  * GLOBAL_POSITION_INT present  -> streams work; bug is elsewhere
  * HEARTBEAT only, no ACK       -> outbound writes are not arriving
                                    (suspect the network path)
  * ACK present, no position     -> autopilot ignores stream requests on
                                    this serial (suspect SITL/params)

Usage:
    python3 probe_mavlink_stream.py tcp:127.0.0.1:5762            # host
    python3 probe_mavlink_stream.py tcp:host.docker.internal:5762 # container
"""

from __future__ import annotations

import json
import sys
import time


def main() -> int:
    endpoint = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5762"
    listen_s = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(endpoint, source_system=254)
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print(json.dumps({"endpoint": endpoint, "error": "no heartbeat in 10s"}))
        return 2
    sysid = hb.get_srcSystem()
    print(
        json.dumps(
            {
                "endpoint": endpoint,
                "heartbeat": True,
                "sysid": sysid,
                "comp": hb.get_srcComponent(),
            }
        ),
        flush=True,
    )

    # Mechanism 1: legacy stream-rate request (what health_check sends).
    conn.mav.request_data_stream_send(sysid, 0, 1, 4, 1)  # ALL @ 4 Hz, start
    # Mechanism 2: modern per-message interval; produces a COMMAND_ACK.
    conn.mav.command_long_send(
        sysid, 0,
        511,        # MAV_CMD_SET_MESSAGE_INTERVAL
        0,
        33.0,       # GLOBAL_POSITION_INT
        250000.0,   # 4 Hz, in microseconds
        0.0, 0.0, 0.0, 0.0, 0.0,
    )

    seen: dict[str, int] = {}
    acks: list[dict[str, int]] = []
    deadline = time.monotonic() + listen_s
    while time.monotonic() < deadline:
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "BAD_DATA":
            t = "BAD_DATA(parse)"
        seen[t] = seen.get(t, 0) + 1
        if t == "COMMAND_ACK":
            acks.append(
                {"command": int(msg.command), "result": int(msg.result)}
            )

    verdict = (
        "streams OK"
        if seen.get("GLOBAL_POSITION_INT")
        else (
            "outbound reaches FC but no position stream"
            if acks
            else "outbound writes likely not arriving"
        )
    )
    print(
        json.dumps(
            {
                "endpoint": endpoint,
                "listened_s": listen_s,
                "message_histogram": dict(sorted(seen.items())),
                "command_acks": acks,
                "global_position_int": bool(seen.get("GLOBAL_POSITION_INT")),
                "verdict": verdict,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if seen.get("GLOBAL_POSITION_INT") else 1


if __name__ == "__main__":
    raise SystemExit(main())
