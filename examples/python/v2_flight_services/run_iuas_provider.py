#!/usr/bin/env python3
"""Run the IUAS investigate-point provider over the real NDNSF Python API.

With a UAS-IPBRC checkout available (`UAS_IPBRC_ROOT` or `--uas-ipbrc-root`),
each request is compiled into `relay.flight` primitives via the orbit
capability ladder and executed to a terminal status before the response is
returned; the reported execution mode (`circle-mode`, `guided-yaw-path`,
`guided-position-only`) reflects what actually ran, and requests the vehicle
cannot satisfy are rejected at the ack stage with the reason. Without a
checkout (or with `--no-execute-plan`) the provider falls back to the
fabricated v0 response so the NDNSF wiring alone can still be demonstrated.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from contracts import (
    CapabilityProfile,
    FlightTaskResult,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    SensorArtifact,
    default_camera_meta,
    gps_time_ns,
    mission_sensor_name,
    vehicle_flight_service,
    vehicle_telemetry_state_name,
)
from camera import frame_source_from_spec
from dataplane import publish_segmented
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    flush_json_log,
    optional_local_nfd,
    print_json,
    provider_kwargs,
    start_journal_publisher,
    start_nfd_counter_scrape,
    start_role_journal,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 IUAS provider")
    add_common_arguments(parser)
    parser.add_argument("--vehicle-id", default="iuas-01")
    parser.add_argument("--provider-prefix", default="/muas/v2/iuas-01")
    parser.add_argument("--provider-id", default="")
    parser.add_argument(
        "--uas-ipbrc-root",
        default=None,
        help=(
            "Path to a UAS-IPBRC checkout providing relay.flight "
            "(default: $UAS_IPBRC_ROOT or ~/Documents/Dev/UAS-IPBRC)"
        ),
    )
    parser.add_argument(
        "--no-execute-plan",
        action="store_true",
        help="Skip primitive execution and return the fabricated v0 response",
    )
    parser.add_argument(
        "--native-orbit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Advertise native circle-mode capability for the simulated "
            "vehicle (--no-native-orbit exercises the guided fallback path)"
        ),
    )
    parser.add_argument(
        "--execution-mode",
        default="simulated-circle-mode",
        help="Notes value reported by the fabricated fallback response",
    )
    parser.add_argument(
        "--mavlink-endpoint",
        default=None,
        help=(
            "Fly investigations on a real autopilot/SITL via MAVLink "
            "(e.g. udp:127.0.0.1:14550, tcp:host.docker.internal:5762). "
            "Request altitudes are interpreted as AGL and rebased onto the "
            "detected ground ASL. Requires relay.flight and pymavlink."
        ),
    )
    parser.add_argument("--mavlink-home-alt-m", type=float, default=None)
    parser.add_argument("--mavlink-tick-dt-s", type=float, default=0.5)
    parser.add_argument(
        "--mavlink-max-ticks",
        type=int,
        default=1200,
        help="Wall-time flight budget = max_ticks * tick_dt_s (default 600s)",
    )
    parser.add_argument(
        "--camera",
        default="synthetic",
        help=(
            "Capture-artifact frame source: synthetic, file:<path>, or "
            "opencv:<index|url> (see camera.py)"
        ),
    )
    parser.add_argument(
        "--log-dir",
        default="/var/lib/minimuas/log",
        help="Directory for the fsync-per-line metrics/event journal "
        "(empty string disables).",
    )
    return parser


def _fabricated_result(
    request: InvestigatePointRequest,
    *,
    vehicle_id: str,
    execution_mode: str,
    frame_source,
) -> tuple[FlightTaskResult, list[bytes]]:
    """v0 behavior: report success without flying anything."""

    started = gps_time_ns()
    artifact_time = gps_time_ns()
    artifact = SensorArtifact(
        data_name=mission_sensor_name(
            request.mission_id,
            vehicle_id,
            "front",
            "frame",
            artifact_time,
            1,
        ),
        kind="image/jpeg",
        gps_time_ns=artifact_time,
        pose=Pose(
            position=GeoPoint(
                lat_deg=request.target.lat_deg,
                lon_deg=request.target.lon_deg,
                alt_m=request.approach_alt_m,
            ),
            yaw_deg=180.0,
        ),
        metadata={"target_id": request.source_detection_id},
    )
    payload = frame_source.capture(
        mission_id=request.mission_id,
        vehicle_id=vehicle_id,
        sensor_id="front",
        gps_time_ns=artifact_time,
        metadata={"target_id": request.source_detection_id},
    )
    return (
        FlightTaskResult(
            task_id=f"{vehicle_id}-investigate-{request.source_detection_id}",
            status="completed",
            started_at_gps_ns=started,
            completed_at_gps_ns=gps_time_ns(),
            artifacts=[artifact],
            notes=execution_mode,
        ),
        [payload],
    )


def main() -> int:
    args = build_parser().parse_args()
    service = vehicle_flight_service(args.vehicle_id, "investigate")
    uas_root = (
        Path(args.uas_ipbrc_root).expanduser() if args.uas_ipbrc_root else None
    )

    investigate_mod = None
    if not args.no_execute_plan:
        try:
            import investigate_plan as _investigate_plan

            _investigate_plan.add_flight_path(uas_root)
            investigate_mod = _investigate_plan
        except Exception as exc:
            print_json(
                "iuas.flight_lib.unavailable",
                error=str(exc),
                fallback="fabricated response",
                hint="set UAS_IPBRC_ROOT or pass --uas-ipbrc-root",
            )

    if args.dry_run:
        print_json(
            "iuas.provider.dry_run",
            service=service,
            provider_prefix=args.provider_prefix,
            plan_execution=(
                "relay.flight" if investigate_mod is not None else "fabricated"
            ),
            native_orbit=args.native_orbit,
        )
        return 0

    start_role_journal(f"{args.vehicle_id}-provider", args.log_dir)
    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

    # Frame source for capture artifacts (synthetic by default).
    try:
        frame_source = frame_source_from_spec(args.camera)
    except Exception as exc:
        print_json("iuas.camera.unavailable", camera=args.camera, error=str(exc))
        return 2
    print_json("iuas.camera.ready", **frame_source.describe())

    # Optional MAVLink-backed flight: same compiled plans, real autopilot.
    mav = None  # (link, vehicle, home_alt_m)
    if args.mavlink_endpoint:
        if investigate_mod is None:
            print_json(
                "iuas.mavlink.unavailable",
                reason="relay.flight is required for MAVLink execution",
            )
            return 2
        import mavlink_flight

        try:
            mav_link, mav_vehicle, mav_home = mavlink_flight.connect_flight_link(
                args.mavlink_endpoint,
                vehicle_id=args.vehicle_id,
                home_alt_m=args.mavlink_home_alt_m,
                uas_ipbrc_root=uas_root,
            )
        except Exception as exc:
            print_json(
                "iuas.mavlink.connect_failed",
                endpoint=args.mavlink_endpoint,
                error=str(exc),
            )
            return 2
        mav = (mav_link, mav_vehicle, mav_home)
        pos = mav_vehicle.position
        print_json(
            "iuas.mavlink.connected",
            endpoint=args.mavlink_endpoint,
            lat=round(pos.lat, 7),
            lon=round(pos.lon, 7),
            home_alt_m=round(mav_home, 2),
        )

    add_ndnsf_path(args.ndnsf_root)
    # Serve this provider's journal over NDN for the dashboard bundle sweep.
    start_journal_publisher(f"{args.vehicle_id}-perception", args.session)
    from ndnsf import AckDecision, ServiceProvider

    provider = ServiceProvider(
        **provider_kwargs(args, args.provider_prefix, args.provider_id)
    )

    # Live segmented-object producers for published sensor artifacts. They
    # must stay referenced (and running) so WUAS/GCS can fetch the objects
    # after the service response returns.
    artifact_producers: list[object] = []

    def publish_artifacts(
        artifacts: list[SensorArtifact],
        payloads: list[bytes],
    ) -> None:
        for artifact, payload in zip(artifacts, payloads):
            try:
                producer = publish_segmented(artifact.data_name, payload)
            except Exception as exc:
                print_json(
                    "iuas.artifact.publish_failed",
                    artifact=artifact.data_name,
                    error=str(exc),
                )
                continue
            artifact_producers.append(producer)
            print_json(
                "iuas.artifact.published",
                artifact=artifact.data_name,
                bytes=len(payload),
                segments=producer.segment_count,
            )

    def active_flight_profile():
        """relay.flight capability profile for the vehicle actually flying."""

        if mav is not None:
            import mavlink_flight

            return mavlink_flight.mavlink_capability_profile()
        return investigate_mod.default_capability_profile(
            native_orbit=args.native_orbit,
        )

    # The investigation IUAS points a forward camera at the target during its
    # orbits; advertise it (with D/R/I bands) so the dashboard's coverage layer
    # draws the FoV cone. Rendered only from these advertised facts.
    invest_sensor_meta = {
        "camera": default_camera_meta(
            60.0, 320, 240, facing="forward", dri_m=[60.0, 30.0, 12.0]
        )
    }

    def build_capability_profile() -> CapabilityProfile:
        if investigate_mod is not None:
            native = active_flight_profile()
            return CapabilityProfile(
                vehicle_id=args.vehicle_id,
                gps_time_ns=gps_time_ns(),
                position=native.position,
                velocity=native.velocity,
                yaw_control=native.yaw_control,
                mode_control=native.mode_control,
                gimbal=native.gimbal,
                obstacle_map=native.obstacle_map,
                signal_sensor=native.signal_sensor,
                extras=sorted(native.extras),
                sensor_meta=invest_sensor_meta,
            )
        # Fabricated fallback mirrors the simulated vehicle's profile.
        return CapabilityProfile(
            vehicle_id=args.vehicle_id,
            gps_time_ns=gps_time_ns(),
            position=True,
            yaw_control=True,
            mode_control=True,
            extras=["orbit"] if args.native_orbit else [],
            sensor_meta=invest_sensor_meta,
        )

    @provider.ack_handler(service)
    def acknowledge(payload: bytes) -> AckDecision:
        request = InvestigatePointRequest.from_bytes(payload)
        if request.circle_radius_m <= 0 or request.approach_alt_m <= 0:
            return AckDecision(status=False, message="invalid request geometry")
        if investigate_mod is None:
            return AckDecision(status=True, message=args.execution_mode)
        compiled = investigate_mod.compile_investigation(
            request,
            vehicle_id=args.vehicle_id,
            profile=active_flight_profile(),
        )
        if compiled.rejected:
            return AckDecision(
                status=False,
                message=compiled.reason or compiled.mode,
            )
        return AckDecision(status=True, message=compiled.mode)

    @provider.handler(service)
    def investigate_point(payload: bytes) -> bytes:
        request = InvestigatePointRequest.from_bytes(payload)
        if investigate_mod is None:
            result, payloads = _fabricated_result(
                request,
                vehicle_id=args.vehicle_id,
                execution_mode=args.execution_mode,
                frame_source=frame_source,
            )
            publish_artifacts(result.artifacts, payloads)
            print_json(
                "iuas.investigation.completed",
                task_id=result.task_id,
                status=result.status,
                execution=result.notes,
                plan="fabricated",
            )
            return result.to_bytes()

        if mav is not None:
            import mavlink_flight

            mav_link, mav_vehicle, mav_home = mav
            if not mavlink_flight.ensure_airborne(
                mav_link,
                mav_vehicle,
                target_agl_m=request.approach_alt_m,
                home_alt_m=mav_home,
            ):
                now = gps_time_ns()
                result = FlightTaskResult(
                    task_id=(
                        f"{args.vehicle_id}-investigate-"
                        f"{request.source_detection_id}"
                    ),
                    status="failed",
                    started_at_gps_ns=now,
                    completed_at_gps_ns=gps_time_ns(),
                    artifacts=[],
                    notes="mavlink preflight failed",
                )
                print_json(
                    "iuas.investigation.completed",
                    task_id=result.task_id,
                    status=result.status,
                    execution="mavlink-preflight-failed",
                )
                return result.to_bytes()
            # Over MAVLink, request altitudes are AGL; rebase onto ground ASL.
            flown = InvestigatePointRequest(
                mission_id=request.mission_id,
                source_detection_id=request.source_detection_id,
                target=GeoPoint(
                    lat_deg=request.target.lat_deg,
                    lon_deg=request.target.lon_deg,
                    alt_m=mav_home + (request.target.alt_m or 0.0),
                ),
                approach_alt_m=mav_home + request.approach_alt_m,
                standoff_m=request.standoff_m,
                circle_radius_m=request.circle_radius_m,
                circle_count=request.circle_count,
                sensor_plan=list(request.sensor_plan),
                constraints=request.constraints,
            )
            outcome = investigate_mod.execute_investigation(
                flown,
                vehicle_id=args.vehicle_id,
                uas_ipbrc_root=uas_root,
                link=mav_link,
                vehicle=mav_vehicle,
                profile=mavlink_flight.mavlink_capability_profile(),
                realtime=True,
                tick_dt_s=args.mavlink_tick_dt_s,
                max_ticks=args.mavlink_max_ticks,
                frame_source=frame_source,
            )
        else:
            outcome = investigate_mod.execute_investigation(
                request,
                vehicle_id=args.vehicle_id,
                native_orbit=args.native_orbit,
                uas_ipbrc_root=uas_root,
                frame_source=frame_source,
            )
        publish_artifacts(
            outcome.result.artifacts,
            list(outcome.artifact_payloads),
        )
        print_json(
            "iuas.investigation.completed",
            task_id=outcome.result.task_id,
            status=outcome.result.status,
            execution=outcome.mode,
            artifacts=len(outcome.result.artifacts),
            link_commands=len(outcome.command_log),
            notes=outcome.result.notes,
        )
        return outcome.result.to_bytes()

    with optional_local_nfd(args.start_local_nfd):
        capability_profile = build_capability_profile()
        telemetry_name = vehicle_telemetry_state_name(args.vehicle_id)
        try:
            capability_producer = publish_segmented(
                telemetry_name,
                capability_profile.to_bytes(),
            )
            artifact_producers.append(capability_producer)
            print_json(
                "iuas.capability.published",
                name=telemetry_name,
                extras=capability_profile.extras,
                yaw_control=capability_profile.yaw_control,
            )
        except Exception as exc:
            print_json(
                "iuas.capability.publish_failed",
                name=telemetry_name,
                error=str(exc),
            )
        print_json("iuas.provider.starting", service=service)
        try:
            return provider.run(service)
        finally:
            flush_json_log()


if __name__ == "__main__":
    raise SystemExit(main())
