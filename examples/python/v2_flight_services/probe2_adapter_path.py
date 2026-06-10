#!/usr/bin/env python3
"""Diagnostic 2: replicate MavlinkDroneLink's exact connect sequence in stages.

probe_mavlink_stream.py proved the transport delivers GLOBAL_POSITION_INT on
request. The adapter still times out, so something in MavlinkDroneLink's
sequence breaks it. This probe mimics that sequence and isolates the three
differences one stage at a time:

  stage connect : source_system=255, proactive GCS heartbeat BEFORE any
                  receive, strict comp==1 heartbeat filter (all as
                  MavlinkDroneLink.connect does)
  stage A       : 1Hz GCS heartbeat thread RUNNING (as after connect),
                  REQUEST_DATA_STREAM, then 5s of blocking=False drains
                  (the _drain pattern) counting GLOBAL_POSITION_INT
  stage B       : heartbeat thread STOPPED, same request, same
                  blocking=False drain
  stage C       : thread stopped, same request, blocking=True reads

Reading the result:
  A=0 B>0        -> concurrent heartbeat-thread writes corrupt the TCP
                    stream (UDP datagrams are atomic, TCP is not)
  A=0 B=0 C>0    -> blocking=False recv is broken on this pymavlink/tcp
                    combination
  A=0 B=0 C=0    -> the proactive heartbeat / sysid 255 path upsets the
                    autopilot link
  A>0            -> sequence is fine here; the bug is elsewhere in the
                    adapter

Usage:
    python3 probe2_adapter_path.py tcp:host.docker.internal:5762   # container
    python3 probe2_adapter_path.py tcp:127.0.0.1:5762              # host
"""

from __future__ import annotations

import json
import sys
import threading
import time


def out(event: str, **fields: object) -> None:
    print(json.dumps({"event": event, **fields}, sort_keys=True), flush=True)


def drain_count(conn, sysid: int, seconds: float, *, blocking: bool) -> dict:
    seen: dict[str, int] = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if blocking:
            msg = conn.recv_match(blocking=True, timeout=0.5)
        else:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.01)
                continue
        if msg is None:
            continue
        if msg.get_srcSystem() != sysid:
            continue
        t = msg.get_type()
        seen[t] = seen.get(t, 0) + 1
    return seen


def main() -> int:
    endpoint = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5762"

    from pymavlink import mavutil

    out("probe2.start", endpoint=endpoint, pymavlink=getattr(mavutil.mavlink, "WIRE_PROTOCOL_VERSION", "?"))

    conn = mavutil.mavlink_connection(endpoint, source_system=255, source_component=0)

    # MavlinkDroneLink.connect: proactive GCS heartbeat before any receive.
    conn.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_GCS,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    # Strict comp==AUTOPILOT1 heartbeat filter, then pin target ids.
    AUTOPILOT_COMP = int(mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    hb = None
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        candidate = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if candidate is None:
            continue
        if int(candidate.get_srcComponent()) != AUTOPILOT_COMP:
            continue
        hb = candidate
        break
    if hb is None:
        out("probe2.connect_failed", reason="no FC heartbeat in 10s")
        return 2
    sysid = int(hb.get_srcSystem())
    conn.target_system = sysid
    conn.target_component = int(hb.get_srcComponent())
    out("probe2.connected", sysid=sysid)

    # 1Hz GCS heartbeat thread, same as MavlinkDroneLink._heartbeat_loop.
    stop = threading.Event()

    def hb_loop() -> None:
        while not stop.is_set():
            try:
                conn.mav.heartbeat_send(
                    type=6, autopilot=8, base_mode=0, custom_mode=0,
                    system_status=4,
                )
            except Exception:
                pass
            if stop.wait(1.0):
                return

    thread = threading.Thread(target=hb_loop, daemon=True)
    thread.start()

    def request_streams() -> None:
        conn.mav.request_data_stream_send(sysid, 0, 1, 4, 1)

    # Stage A: thread running, nonblocking drain (the adapter's situation).
    request_streams()
    seen_a = drain_count(conn, sysid, 5.0, blocking=False)
    out(
        "probe2.stage_a.thread_running.nonblocking",
        global_position_int=seen_a.get("GLOBAL_POSITION_INT", 0),
        histogram=dict(sorted(seen_a.items())),
    )

    # Stage B: thread stopped, nonblocking drain.
    stop.set()
    thread.join(timeout=2.0)
    request_streams()
    seen_b = drain_count(conn, sysid, 5.0, blocking=False)
    out(
        "probe2.stage_b.no_thread.nonblocking",
        global_position_int=seen_b.get("GLOBAL_POSITION_INT", 0),
        histogram=dict(sorted(seen_b.items())),
    )

    # Stage C: thread stopped, blocking reads.
    request_streams()
    seen_c = drain_count(conn, sysid, 5.0, blocking=True)
    out(
        "probe2.stage_c.no_thread.blocking",
        global_position_int=seen_c.get("GLOBAL_POSITION_INT", 0),
        histogram=dict(sorted(seen_c.items())),
    )

    a = seen_a.get("GLOBAL_POSITION_INT", 0)
    b = seen_b.get("GLOBAL_POSITION_INT", 0)
    c = seen_c.get("GLOBAL_POSITION_INT", 0)
    if a:
        verdict = "adapter sequence fine here; bug elsewhere"
    elif b:
        verdict = "heartbeat-thread writes break the TCP stream"
    elif c:
        verdict = "nonblocking recv broken on this pymavlink/tcp combo"
    else:
        verdict = "proactive-heartbeat / sysid-255 path upsets the link"
    out("probe2.verdict", a=a, b=b, c=c, verdict=verdict)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
