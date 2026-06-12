#!/usr/bin/env python3
"""HITL probe: drive the real flight-link surface against a bench FC.

Exercises every MAVLink emission the v2 investigate path makes — through
the SAME code path the mission uses (connect_flight_link +
LoggingFlightLink over relay.drone.mavlink) — against a powered flight
controller, and records the autopilot's verdict on each. A grounded FC
cannot fly the mission (indoor prearm checks, no altitude), but it CAN
adjudicate every command: mode changes and COMMAND_LONGs are ACKed by
the real firmware, which is exactly the evidence needed to decide
whether the implementation's MAVLink emissions are erroneous.

Stages (all continue on failure; the report is the point):
  connect    one-connection link bring-up + stream-rate nudge + first fix
  telemetry  GLOBAL_POSITION_INT + armed state readback
  guided     GUIDED mode switch (ACKed by FC; valid while disarmed)
  speed      DO_CHANGE_SPEED (FC verdict recorded either way)
  takeoff    MAV_CMD_NAV_TAKEOFF while DISARMED — expected DENIED; the
             denial is the FC processing the command, which is the test
  goto/yaw   SET_POSITION_TARGET_GLOBAL_INT emissions (no ACK by design;
             ignored by a grounded FC — emission correctness is verified
             by the FC not raising and by dataflash GUID entries in
             flight/sim-on-hardware runs)
  rtl        RTL mode switch and back to GUIDED
  arm        ONLY with --allow-arm and props off: real prearm verdict,
             then immediate disarm. Never combined with takeoff.

PROPS OFF for any --allow-arm use. This probe never sends takeoff to an
armed vehicle.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from mavlink_flight import connect_flight_link
from ndnsf_runtime import print_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="miniMUAS v2 HITL probe")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5771")
    parser.add_argument("--vehicle-id", default="iuas-01")
    parser.add_argument("--position-timeout-s", type=float, default=30.0)
    parser.add_argument("--goto-offset-m", type=float, default=30.0)
    parser.add_argument("--target-agl-m", type=float, default=20.0)
    parser.add_argument(
        "--allow-arm",
        action="store_true",
        help="Attempt a real arm/disarm cycle. PROPS OFF. Records the "
        "FC's prearm verdict; never followed by takeoff.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    results: list[dict[str, object]] = []

    def stage(name: str, **fields: object) -> None:
        results.append({"stage": name, **fields})
        print_json(f"hitl.{name}", **fields)

    # -- connect -------------------------------------------------------
    try:
        link, vehicle, home_alt_m = connect_flight_link(
            args.endpoint,
            vehicle_id=args.vehicle_id,
            position_timeout_s=args.position_timeout_s,
        )
        stage("connect", ok=True, endpoint=args.endpoint, home_alt_m=home_alt_m)
    except Exception as exc:
        stage("connect", ok=False, error=str(exc))
        print_json("hitl.report", verdict="connect-failed", results=results)
        return 2

    try:
        # -- telemetry ---------------------------------------------------
        position = vehicle.position
        stage(
            "telemetry",
            ok=position is not None,
            lat=getattr(position, "lat", None),
            lon=getattr(position, "lon", None),
            alt=getattr(position, "alt", None),
            armed=vehicle.armed,
        )

        # -- guided mode ---------------------------------------------------
        stage("guided", ok=link.set_mode_guided())

        # -- cruise speed ----------------------------------------------------
        stage("speed", ok=link.set_cruise_speed_m_s(5.0))

        # -- takeoff while disarmed: the FC's denial IS the processing proof
        if vehicle.armed:
            stage("takeoff_disarmed", ok=None, skipped="vehicle already armed")
        else:
            stage(
                "takeoff_disarmed",
                ok=link.takeoff(args.target_agl_m),
                expected="denied (disarmed); a denial proves FC processing",
            )

        # -- guided position targets (fire-and-forget by protocol design)
        if position is not None:
            north = args.goto_offset_m / 111_111.0
            try:
                link.goto(position.lat + north, position.lon,
                          home_alt_m + args.target_agl_m)
                stage("goto", ok=True, note="no ACK by protocol; emission only")
            except Exception as exc:
                stage("goto", ok=False, error=str(exc))
            try:
                link.goto(position.lat + north, position.lon,
                          home_alt_m + args.target_agl_m, yaw_deg=90.0)
                stage("goto_yaw", ok=True, note="yaw fields populated")
            except Exception as exc:
                stage("goto_yaw", ok=False, error=str(exc))

        # -- RTL and back ----------------------------------------------------
        stage("rtl", ok=link.rtl())
        stage("guided_after_rtl", ok=link.set_mode_guided())

        # -- optional arm/disarm cycle (props off!) ---------------------------
        if args.allow_arm:
            armed = link.arm()
            stage("arm", ok=armed, note="real prearm verdict from the FC")
            if armed:
                disarm = getattr(link._inner, "disarm", None)
                if disarm is not None:
                    stage("disarm", ok=bool(disarm()))
                else:
                    stage("disarm", ok=None,
                          skipped="link exposes no disarm; disarm manually")
        else:
            stage("arm", ok=None, skipped="pass --allow-arm (props off) to test")

        failed = [r["stage"] for r in results
                  if r.get("ok") is False and r["stage"] != "takeoff_disarmed"]
        verdict = "clean" if not failed else "anomalies"
        print_json(
            "hitl.report",
            verdict=verdict,
            anomalies=failed,
            command_log=[
                {"command": name, **fields} for name, fields in link.command_log
            ],
            results=results,
        )
        return 0 if verdict == "clean" else 1
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
