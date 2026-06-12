#!/usr/bin/env python3
"""Run the GCS object-detection provider over the real NDNSF Python API."""

from __future__ import annotations

import argparse

from contracts import (
    DetectionRequest,
    DetectionResponse,
    GeoPoint,
    gcs_detection_service,
    gps_time_ns,
    mission_evidence_name,
)
from dataplane import fetch_segmented, frame_body, parse_frame
from detector import (
    decode_image,
    detector_from_spec,
    offset_latlon,
    project_nadir,
)
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    optional_local_nfd,
    print_json,
    provider_kwargs,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 GCS provider")
    add_common_arguments(parser)
    parser.add_argument("--provider-prefix", default="/muas/v2/gcs")
    parser.add_argument("--provider-id", default="")
    parser.add_argument("--service", default=gcs_detection_service())
    parser.add_argument(
        "--detector",
        default="stub",
        help="stub (offset-based fake) or "
        "yolo:<model.onnx>[?conf=0.35&classes=tennis racket]",
    )
    parser.add_argument(
        "--hfov-deg",
        type=float,
        default=70.0,
        help="capture camera horizontal FOV for nadir geo-projection",
    )
    parser.add_argument("--lat-offset-deg", type=float, default=0.00008)
    parser.add_argument("--lon-offset-deg", type=float, default=0.00006)
    parser.add_argument("--frame-fetch-timeout-ms", type=int, default=5000)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.dry_run:
        print_json(
            "gcs.provider.dry_run",
            service=args.service,
            provider_prefix=args.provider_prefix,
        )
        return 0

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import AckDecision, ServiceProvider, ServiceResponse

    detector = detector_from_spec(args.detector)
    if detector is not None:
        print_json("gcs.detector.ready", **detector.describe())

    provider = ServiceProvider(
        **provider_kwargs(args, args.provider_prefix, args.provider_id)
    )

    @provider.ack_handler(args.service)
    def acknowledge(payload: bytes) -> AckDecision:
        request = DetectionRequest.from_bytes(payload)
        return AckDecision(
            status=bool(request.frame.data_name),
            message="gcs-detection-ready",
        )

    @provider.handler(args.service)
    def detect_object(payload: bytes) -> bytes | ServiceResponse:
        request = DetectionRequest.from_bytes(payload)

        # Detection now consumes the actual published frame object instead
        # of trusting the name reference.
        try:
            frame_payload = fetch_segmented(
                request.frame.data_name,
                timeout_ms=args.frame_fetch_timeout_ms,
            )
            header = parse_frame(frame_payload)
        except Exception as exc:
            print_json(
                "gcs.frame.fetch_failed",
                frame=request.frame.data_name,
                error=str(exc),
            )
            return ServiceResponse(
                status=False,
                error=f"frame fetch failed: {exc}",
            )
        print_json(
            "gcs.frame.fetched",
            frame=request.frame.data_name,
            bytes=len(frame_payload),
            sha256=header["sha256"],
            width=header.get("width"),
            height=header.get("height"),
        )

        timestamp_ns = gps_time_ns()

        if detector is not None:
            image = decode_image(frame_body(frame_payload))
            if image is None:
                return ServiceResponse(
                    status=False, error="frame body is not a decodable image"
                )
            detections = detector.detect(image)
            print_json(
                "gcs.detection.inference",
                frame=request.frame.data_name,
                detections=[d.as_dict() for d in detections],
                all_classes=[d.as_dict() for d in detector.last_all_detections],
            )
            # debugging/dashboard breadcrumb: the exact frame as analyzed,
            # with every above-threshold box drawn (target class in green)
            try:
                import cv2
                annotated = image.copy()
                for det in detector.last_all_detections:
                    x, y, w, h = det.box_xywh
                    hit = det in detections
                    color = (0, 255, 0) if hit else (0, 165, 255)
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(
                        annotated,
                        f"{det.label} {det.confidence:.2f}",
                        (x, max(y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                cv2.imwrite("/tmp/muas-last-detect.jpg", annotated)
            except Exception:
                pass
            if not detections:
                return ServiceResponse(status=False, error="no-detection")
            best = detections[0]
            pose = request.frame.pose
            height_px, width_px = image.shape[:2]
            north_m, east_m = project_nadir(
                best.center_px,
                (width_px, height_px),
                agl_m=max(float(pose.position.alt_m), 0.0),
                hfov_deg=args.hfov_deg,
                heading_deg=getattr(pose, "yaw_deg", None),
            )
            lat_deg, lon_deg = offset_latlon(
                pose.position.lat_deg, pose.position.lon_deg, north_m, east_m
            )
            response = DetectionResponse(
                mission_id=request.mission_id,
                object_id=best.label.replace(" ", "-"),
                confidence=best.confidence,
                estimate=GeoPoint(lat_deg=lat_deg, lon_deg=lon_deg, alt_m=0.0),
                evidence_ref=mission_evidence_name(
                    request.mission_id,
                    best.label.replace(" ", "-"),
                    timestamp_ns,
                ),
            )
            print_json(
                "gcs.detection.completed",
                frame=request.frame.data_name,
                detector="yolo",
                label=best.label,
                confidence=round(best.confidence, 4),
                box_xywh=list(best.box_xywh),
                offset_m={"north": round(north_m, 2), "east": round(east_m, 2)},
                estimate={"lat": lat_deg, "lon": lon_deg},
                evidence=response.evidence_ref,
            )
            return response.to_bytes()

        response = DetectionResponse(
            mission_id=request.mission_id,
            object_id="target-001",
            confidence=0.91,
            estimate=GeoPoint(
                lat_deg=request.frame.pose.position.lat_deg + args.lat_offset_deg,
                lon_deg=request.frame.pose.position.lon_deg + args.lon_offset_deg,
                alt_m=0.0,
            ),
            evidence_ref=mission_evidence_name(
                request.mission_id,
                "target-001",
                timestamp_ns,
            ),
        )
        print_json(
            "gcs.detection.completed",
            frame=request.frame.data_name,
            detector="stub",
            frame_bytes=len(frame_payload),
            evidence=response.evidence_ref,
            confidence=response.confidence,
        )
        return response.to_bytes()

    with optional_local_nfd(args.start_local_nfd):
        print_json("gcs.provider.starting", service=args.service)
        return provider.run(args.service)


if __name__ == "__main__":
    raise SystemExit(main())
