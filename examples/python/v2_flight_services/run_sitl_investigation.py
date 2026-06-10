#!/usr/bin/env python3
"""Fly the v2 investigate plan against ArduPilot SITL over MAVLink.

This is the hardware-path validation of the slice: the exact plan the
NDNSF IUAS provider compiles (climb -> approach -> orbit ladder -> capture)
executes against a real autopilot, with no NDN involved. Plan compilation
is untouched; only the link, the clock, and the capability profile differ.

Launch SITL first (UAS-IPBRC's chain script, one vehicle):

    cd ~/Documents/Dev/UAS-IPBRC
    scripts/launch_sitl_chain.sh 0

then, once EKF has converged (~20s):

    python run_sitl_investigation.py                       # udp:127.0.0.1:14550
    python run_sitl_investigation.py --endpoint tcp:127.0.0.1:5762

The target point is synthesized a short hop north of wherever the vehicle
sits, at the detected ground ASL, so the same command works at any SITL
home. Expected execution mode is guided-yaw-path: the MAVLink link exposes
yaw control but no native orbit, and the capability ladder reports what
actually ran.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from contracts import ConstraintSet, GeoPoint, InvestigatePointRequest
from camera import frame_source_from_spec
from investigate_plan import add_flight_path, execute_investigation
from mavlink_flight import (
    connect_flight_link,
    ensure_airborne,
    mavlink_capability_profile,
)


def print_json(event: str, **fields: object) -> None:
    print(json.dumps({"event": event, **fields}, sort_keys=True), flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fly the v2 investigate plan on ArduPilot SITL",
    )
    parser.add_argument("--endpoint", default="udp:127.0.0.1:14550")
    parser.add_argument("--vehicle-id", default="iuas-01")
    parser.add_argument("--uas-ipbrc-root", type=Path, default=None)
    parser.add_argument("--home-alt-m", type=float, default=None,
                        help="Ground ASL; auto-detected from telemetry when omitted")
    parser.add_argument("--approach-agl-m", type=float, default=15.0)
    parser.add_argument("--target-north-m", type=float, default=40.0,
                        help="Synthesized target offset north of the vehicle")
    parser.add_argument("--standoff-m", type=float, default=20.0)
    parser.add_argument("--radius-m", type=float, default=8.0)
    parser.add_argument("--turns", type=float, default=1.0)
    parser.add_argument("--speed-mps", type=float, default=4.0)
    parser.add_argument("--min-clearance-m", type=float, default=5.0)
    parser.add_argument("--tick-dt-s", type=float, default=0.5)
    parser.add_argument("--max-ticks", type=int, default=720,
                        help="Wall-time budget = max_ticks * tick_dt_s (default 360s)")
    parser.add_argument("--connect-timeout-s", type=float, default=15.0)
    parser.add_argument("--takeoff-timeout-s", type=float, default=120.0)
    parser.add_argument("--finish", choices=("rtl", "land", "hold"), default="rtl")
    parser.add_argument(
        "--camera",
        default="synthetic",
        help="Capture frame source: synthetic, file:<path>, or opencv:<index|url>",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    add_flight_path(args.uas_ipbrc_root)
    from relay.core.geo import EARTH_M_PER_DEG_LAT

    frame_source = frame_source_from_spec(args.camera)
    print_json("sitl.camera.ready", **frame_source.describe())

    link, vehicle, home_alt_m = connect_flight_link(
        args.endpoint,
        vehicle_id=args.vehicle_id,
        home_alt_m=args.home_alt_m,
        connect_timeout_s=args.connect_timeout_s,
        uas_ipbrc_root=args.uas_ipbrc_root,
    )
    start = vehicle.position
    print_json(
        "sitl.connected",
        endpoint=args.endpoint,
        lat=round(start.lat, 7),
        lon=round(start.lon, 7),
        alt_asl_m=round(start.alt, 2),
        home_alt_m=round(home_alt_m, 2),
    )

    if not ensure_airborne(
        link,
        vehicle,
        target_agl_m=args.approach_agl_m,
        home_alt_m=home_alt_m,
        timeout_s=args.takeoff_timeout_s,
    ):
        print_json("sitl.preflight_failed", reason="arm/takeoff did not complete")
        return 2
    airborne = vehicle.position
    print_json(
        "sitl.airborne",
        agl_m=round(airborne.alt - home_alt_m, 2),
    )

    # Anchor the mission at the vehicle: target a point north of it at
    # ground level, plan altitudes in absolute ASL on the detected ground.
    request = InvestigatePointRequest(
        mission_id="sitl-001",
        source_detection_id="sitl-target",
        target=GeoPoint(
            lat_deg=start.lat + args.target_north_m / EARTH_M_PER_DEG_LAT,
            lon_deg=start.lon,
            alt_m=home_alt_m,
        ),
        approach_alt_m=home_alt_m + args.approach_agl_m,
        standoff_m=args.standoff_m,
        circle_radius_m=args.radius_m,
        circle_count=args.turns,
        sensor_plan=["capture-still"],
        constraints=ConstraintSet(
            max_speed_mps=args.speed_mps,
            min_clearance_m=args.min_clearance_m,
            avoidance_mode="advisory",
        ),
    )
    print_json(
        "sitl.investigation.dispatched",
        target_lat=round(request.target.lat_deg, 7),
        target_lon=round(request.target.lon_deg, 7),
        approach_alt_asl_m=round(request.approach_alt_m, 2),
    )

    outcome = execute_investigation(
        request,
        vehicle_id=args.vehicle_id,
        sensor_id="front",
        tick_dt_s=args.tick_dt_s,
        max_ticks=args.max_ticks,
        uas_ipbrc_root=args.uas_ipbrc_root,
        link=link,
        vehicle=vehicle,
        profile=mavlink_capability_profile(),
        realtime=True,
        frame_source=frame_source,
    )

    final = vehicle.position
    print_json(
        "sitl.investigation.completed",
        status=outcome.result.status,
        mode=outcome.mode,
        notes=outcome.result.notes,
        artifacts=len(outcome.result.artifacts),
        link_commands=[name for name, _ in outcome.command_log],
        final_agl_m=round(final.alt - home_alt_m, 2),
    )

    if args.finish == "rtl":
        link.rtl()
        print_json("sitl.finish", action="rtl")
    elif args.finish == "land":
        link.land()
        print_json("sitl.finish", action="land")
    else:
        print_json("sitl.finish", action="hold")

    print(
        json.dumps(
            {
                "result": asdict(outcome.result),
                "mode": outcome.mode,
                "events": list(outcome.event_names),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if outcome.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
