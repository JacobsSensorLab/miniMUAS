#!/usr/bin/env python3
"""Run the WUAS v2 mission user over the real NDNSF Python API."""

from __future__ import annotations

import argparse
import atexit
import time

from contracts import (
    CapabilityProfile,
    ConstraintSet,
    DetectionRequest,
    DetectionResponse,
    FlightTaskResult,
    FrameRef,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    expected_orbit_mode,
    gcs_detection_service,
    gps_time_ns,
    mission_frame_name,
    vehicle_flight_service,
    vehicle_telemetry_state_name,
)
from camera import frame_source_from_spec
from dataplane import (
    FRAME_CONTENT_TYPE,
    fetch_segmented,
    parse_frame,
    publish_segmented,
    sha256_hex,
)
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    flush_json_log,
    optional_local_nfd,
    print_json,
    require_success,
    start_journal_publisher,
    start_nfd_counter_scrape,
    start_role_journal,
    user_kwargs,
)
import metrics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 WUAS user")
    add_common_arguments(parser)
    parser.add_argument("--user", default="/muas/v2/wuas-01")
    parser.add_argument("--wuas-id", default="wuas-01")
    parser.add_argument("--iuas-id", default="iuas-01")
    parser.add_argument("--mission-id", default="mission-001")
    parser.add_argument("--ack-timeout-ms", type=int, default=300)
    # Detect must cover: GCS fetching the frame over the radio (segmented,
    # ~120KB) + YOLO inference on the C4 (first forward pass is slowest).
    parser.add_argument("--timeout-ms", type=int, default=30000)
    parser.add_argument("--investigate-timeout-ms", type=int, default=90000)
    parser.add_argument("--artifact-fetch-timeout-ms", type=int, default=15000)
    parser.add_argument("--capability-fetch-timeout-ms", type=int, default=3000)
    parser.add_argument(
        "--camera",
        default="synthetic",
        help=(
            "Published-frame source: synthetic, file:<path>, or "
            "opencv:<index|url> (see camera.py)"
        ),
    )
    parser.add_argument("--list-services", action="store_true")
    parser.add_argument(
        "--log-dir",
        default="/var/lib/minimuas/log",
        help="Directory for the fsync-per-line metrics/event journal "
        "(empty string disables).",
    )
    return parser


def request_with_metric(user, service: str, payload: bytes, **kwargs) -> bytes:
    # Same-node monotonic RTT for the legacy metric.service_rtt event (kept for
    # back-compat with anything already reading it) ...
    mono0 = time.monotonic_ns()
    sent_ns = metrics.stamp()
    response = user.request_service(service, payload, **kwargs)
    rtt_ms = round((time.monotonic_ns() - mono0) / 1_000_000.0, 3)
    print_json(
        "metric.service_rtt",
        service=service,
        rtt_ms=rtt_ms,
        status=bool(response.status),
    )
    # ... and the unified metric.latency schema (with the provider four-point
    # breakdown when the wrapper attaches response.timing).
    metrics.record_service_result(service, sent_ns, response, service=service)
    return require_success(response, service)


def main() -> int:
    args = build_parser().parse_args()
    detection_service = gcs_detection_service()
    investigate_service = vehicle_flight_service(args.iuas_id, "investigate")
    if args.dry_run:
        print_json(
            "wuas.user.dry_run",
            user=args.user,
            detection_service=detection_service,
            investigate_service=investigate_service,
        )
        return 0

    start_role_journal(f"{args.wuas_id}-user", args.log_dir)
    atexit.register(flush_json_log)
    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

    add_ndnsf_path(args.ndnsf_root)
    # Serve this user role's journal over NDN for the dashboard bundle sweep.
    start_journal_publisher(f"{args.wuas_id}-user", args.session)
    from ndnsf import ServiceUser

    with optional_local_nfd(args.start_local_nfd):
        try:
            frame_source = frame_source_from_spec(args.camera)
        except Exception as exc:
            print_json(
                "wuas.camera.unavailable", camera=args.camera, error=str(exc)
            )
            return 2
        print_json("wuas.camera.ready", **frame_source.describe())
        user = ServiceUser(**user_kwargs(args, args.user))
        if args.list_services:
            for entry in user.get_allowed_services():
                print_json(
                    "wuas.allowed_service",
                    service=entry.service,
                    provider_service=entry.provider_service,
                )
            return 0

        frame_time = gps_time_ns()
        frame = FrameRef(
            data_name=mission_frame_name(
                args.mission_id,
                args.wuas_id,
                "front",
                frame_time,
                1,
            ),
            gps_time_ns=frame_time,
            seq=1,
            camera_id="front",
            pose=Pose(
                position=GeoPoint(lat_deg=35.1208, lon_deg=-89.9347, alt_m=40.0),
                yaw_deg=90.0,
            ),
            content_type=FRAME_CONTENT_TYPE,
        )
        frame_payload = frame_source.capture(
            mission_id=args.mission_id,
            vehicle_id=args.wuas_id,
            sensor_id="front",
            gps_time_ns=frame_time,
            metadata={"yaw_deg": "90.0"},
        )
        frame_producer = publish_segmented(frame.data_name, frame_payload)
        print_json(
            "wuas.frame.published",
            frame=frame.data_name,
            bytes=len(frame_payload),
            segments=frame_producer.segment_count,
            sha256=sha256_hex(frame_payload),
        )

        detection_payload = DetectionRequest(
            mission_id=args.mission_id,
            frame=frame,
            object_query="test-object",
        ).to_bytes()
        detection = DetectionResponse.from_bytes(
            request_with_metric(
                user,
                detection_service,
                detection_payload,
                ack_timeout_ms=args.ack_timeout_ms,
                timeout_ms=args.timeout_ms,
            )
        )
        print_json(
            "wuas.detection.received",
            object_id=detection.object_id,
            confidence=detection.confidence,
            evidence=detection.evidence_ref,
        )

        # Capability-aware dispatch: consult the IUAS's advertised flight
        # capabilities and predict the execution mode before sending the
        # investigation, instead of dispatching blind.
        expected_mode = None
        try:
            profile = CapabilityProfile.from_bytes(
                fetch_segmented(
                    vehicle_telemetry_state_name(args.iuas_id),
                    timeout_ms=args.capability_fetch_timeout_ms,
                    metric_name="capability_fetch",
                )
            )
            expected_mode = expected_orbit_mode(profile)
            print_json(
                "wuas.capability.fetched",
                vehicle=profile.vehicle_id,
                extras=profile.extras,
                yaw_control=profile.yaw_control,
                expected_mode=expected_mode,
            )
        except Exception as exc:
            print_json(
                "wuas.capability.unavailable",
                vehicle=args.iuas_id,
                error=str(exc),
            )

        investigation = InvestigatePointRequest(
            mission_id=args.mission_id,
            source_detection_id=detection.object_id,
            target=detection.estimate,
            approach_alt_m=25.0,
            standoff_m=8.0,
            circle_radius_m=6.0,
            circle_count=1.5,
            sensor_plan=["capture-still", "publish-frame"],
            constraints=ConstraintSet(
                max_speed_mps=4.0,
                min_clearance_m=3.0,
                avoidance_mode="advisory",
            ),
        )
        result = FlightTaskResult.from_bytes(
            request_with_metric(
                user,
                investigate_service,
                investigation.to_bytes(),
                ack_timeout_ms=args.ack_timeout_ms,
                timeout_ms=args.investigate_timeout_ms,
            )
        )
        print_json(
            "mission.completed",
            task_id=result.task_id,
            status=result.status,
            execution=result.notes,
            expected_mode=expected_mode,
            mode_as_predicted=(
                result.notes.startswith(expected_mode)
                if expected_mode
                else None
            ),
            artifacts=[artifact.data_name for artifact in result.artifacts],
        )

        # Close the loop on the data plane: fetch the close-range sensor
        # artifacts the IUAS published, exactly as the GCS would.
        for artifact in result.artifacts:
            try:
                payload = fetch_segmented(
                    artifact.data_name,
                    timeout_ms=args.artifact_fetch_timeout_ms,
                    metric_name="artifact_fetch",
                )
                header = parse_frame(payload)
                print_json(
                    "wuas.artifact.fetched",
                    artifact=artifact.data_name,
                    bytes=len(payload),
                    sha256=header["sha256"],
                    sensor_id=header.get("sensor_id"),
                )
            except Exception as exc:
                print_json(
                    "wuas.artifact.fetch_failed",
                    artifact=artifact.data_name,
                    error=str(exc),
                )
                return 1

        frame_producer.stop()
        # A delivered-but-failed task (e.g. the IUAS could not get the
        # vehicle airborne) is a failed mission for exit-code purposes,
        # even though the NDN exchange itself succeeded.
        return 0 if result.status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
