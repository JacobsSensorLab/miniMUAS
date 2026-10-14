#!/usr/bin/env python3
"""Run the GCS object-detection provider over the real NDNSF Python API."""

from __future__ import annotations

import argparse
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeoutError

from contracts import (
    DetectionRequest,
    DetectionResponse,
    GeoPoint,
    gcs_detection_service,
    gps_time_ns,
    mission_evidence_name,
)
from dataplane import fetch_segmented, frame_body, parse_frame, set_runtime
from detector import (
    decode_image,
    detector_from_spec,
    offset_latlon,
    project_ground,
    simulated_target_in_frame,
)
from commands import CommandDeduplicator
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
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 GCS provider")
    add_common_arguments(parser)
    parser.add_argument("--provider-prefix", default="/muas/v2/gcs")
    parser.add_argument("--provider-id", default="")
    parser.add_argument("--service", default=gcs_detection_service())
    parser.add_argument(
        "--detector",
        default="stub",
        help="stub (offset-based fake) or "
        "yolo:<model.onnx>[?conf=0.3&classes=tennis racket|kite] "
        "(see detector.detector_from_spec)",
    )
    parser.add_argument(
        "--hfov-deg",
        type=float,
        default=70.0,
        help="capture camera horizontal FOV for nadir geo-projection",
    )
    parser.add_argument(
        "--cam-yaw-offset-deg",
        type=float,
        default=0.0,
        help="camera mounting rotation about nadir, degrees clockwise from "
        "'image top = vehicle nose'. Added to the capture heading before "
        "projection — the field calibration knob when detections land "
        "consistently rotated around the capture position.",
    )
    parser.add_argument("--lat-offset-deg", type=float, default=0.00008)
    parser.add_argument("--lon-offset-deg", type=float, default=0.00006)
    parser.add_argument("--frame-fetch-timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--detect-admit", type=int, default=2,
        help="Detection requests admitted at once: one runs on the single "
        "worker, the rest wait their turn in order. Excess is fast-rejected "
        "'detector busy' (the dashboard re-queues the frame). A slot frees "
        "when its WORK finishes, not when its handler stops waiting.",
    )
    parser.add_argument(
        "--infer-timeout-s", type=float, default=10.0,
        help="Budget for one frame's decode + inference + annotation. With "
        "the frame fetch timeout it bounds how long a handler waits: "
        "detect_admit x (fetch + infer), the worst case of waiting behind "
        "every other admitted frame.",
    )
    parser.add_argument(
        "--frames-dir",
        default="/var/lib/minimuas/frames",
        help="Save every analyzed frame (annotated) + a JSON sidecar here, "
        "grouped by mission, for post-test analysis (empty string disables).",
    )
    parser.add_argument(
        "--frames-keep",
        type=int,
        default=4000,
        help="Cap on saved frames per mission dir; oldest are pruned.",
    )
    parser.add_argument(
        "--log-dir",
        default="/var/lib/minimuas/log",
        help="Directory for the fsync-per-line metrics/event journal "
        "(empty string disables).",
    )
    return parser


def _save_analyzed_frame(
    frames_dir, keep, mission_id, frame_name, annotated, sidecar, cv2,
    raw_jpeg: bytes = b"",
) -> None:
    """Persist the analyzed (annotated) frame + a JSON sidecar for post-test
    analysis, plus the frame exactly as received (<base>.raw.jpeg): the
    annotated copy has boxes and labels drawn over the object, so the
    2026-10-13 hit frames could not be re-scored afterwards (yolov8n finds
    nothing on them even at conf 0.05). Grouped per mission, filename encodes
    seq + top class + conf, and each mission dir is capped at `keep` frames
    (oldest pruned). Best-effort: never let a save error touch the
    detection response."""
    import json as _json
    import os as _os

    if not frames_dir:
        return
    try:
        seq = (frame_name or "").rsplit("/", 1)[-1] or "0"
        mdir = _os.path.join(frames_dir, (mission_id or "adhoc").replace("/", "_"))
        _os.makedirs(mdir, exist_ok=True)
        top = sidecar.get("all_classes") or []
        sim = sidecar.get("simulated")
        if sim and not sidecar.get("detections"):
            # the response was the simulated target: say so in the name
            label = "SIM-" + str(sim.get("signature") or "target")
            conf = "sim"
        else:
            label = top[0]["label"] if top else "none"
            conf = f"{top[0]['confidence']:.2f}" if top else "0.00"
        label = label.replace(" ", "-").replace("/", "-")
        base = f"{int(seq):06d}_{label}_{conf}" if seq.isdigit() else f"{seq}_{label}_{conf}"
        cv2.imwrite(_os.path.join(mdir, base + ".jpg"), annotated)
        if raw_jpeg:
            with open(_os.path.join(mdir, base + ".raw.jpeg"), "wb") as fh:
                fh.write(raw_jpeg)
        with open(_os.path.join(mdir, base + ".json"), "w") as fh:
            _json.dump(sidecar, fh)
        jpgs = sorted(p for p in _os.listdir(mdir) if p.endswith(".jpg"))
        for old in jpgs[: max(0, len(jpgs) - keep)]:
            for ext in (".jpg", ".json", ".raw.jpeg"):
                try:
                    _os.remove(_os.path.join(mdir, old[:-4] + ext))
                except OSError:
                    pass
    except Exception:
        pass


def submit_admitted(pool, gate, fn, *args):
    """Run fn(*args) on `pool` if `gate` (a semaphore) has a free slot, else
    return None. The slot stays held until fn RETURNS, not until a waiter
    gives up on the future: freeing it on a handler timeout admitted the next
    frame behind a worker still stuck in a 15 s fetch, and every later frame
    inherited the delay (2026-10-13, 16:49 onwards)."""
    if not gate.acquire(blocking=False):
        return None
    try:
        future = pool.submit(fn, *args)
    except BaseException:
        gate.release()
        raise
    future.add_done_callback(lambda _f: gate.release())
    return future


def main() -> int:
    args = build_parser().parse_args()
    if args.dry_run:
        print_json(
            "gcs.provider.dry_run",
            service=args.service,
            provider_prefix=args.provider_prefix,
        )
        return 0

    start_role_journal("gcs-provider", args.log_dir)
    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import AckDecision, ServiceProvider, ServiceResponse

    detector = detector_from_spec(args.detector)
    if detector is not None:
        print_json("gcs.detector.ready", **detector.describe())

    provider = ServiceProvider(
        **provider_kwargs(args, args.provider_prefix, args.provider_id)
    )
    # Route dataplane fetches (frame fetch) through this provider's Face.
    set_runtime(provider)
    # Tokens-off to match the tokens-off users (dashboard, bench); otherwise the
    # provider silently drops token-less requests and they time out.
    if hasattr(provider, "set_use_tokens"):
        provider.set_use_tokens(False)

    @provider.ack_handler(args.service)
    def acknowledge(payload: bytes) -> AckDecision:
        request = DetectionRequest.from_bytes(payload)
        return AckDecision(
            status=bool(request.frame.data_name),
            message="gcs-detection-ready",
        )

    # Heavy detection (fetch + decode + YOLO) runs on ONE dedicated worker so
    # concurrent inferences never thrash the GIL against each other, and a
    # bounded admission gate fast-rejects overflow instead of piling NDNSF
    # handler threads on a busy detector. This keeps the framework's own
    # crypto/sync threads free — the field failure where request_decrypt
    # ballooned to seconds was YOLO monopolising the single interpreter. The
    # native-heavy sections (cv2 decode/forward) release the GIL, so the
    # framework makes progress while the worker runs.
    _infer_pool = ThreadPoolExecutor(
        max_workers=1, thread_name_prefix="gcs-detect"
    )
    admit = max(1, args.detect_admit)
    _detect_gate = threading.BoundedSemaphore(admit)
    # A handler waits for its own frame AND every admitted frame ahead of it
    # on the single worker, each bounded by fetch + inference. The old fixed
    # 10 s ceiling sat below the 15 s fetch timeout the GCS runs with, so on
    # 2026-10-13 a handler gave up (and freed its slot) while the worker was
    # still fetching: "gcs.detection.timeout" at 16:49:03/06 for frames whose
    # fetch had not even started, the cascade behind every failed fetch.
    wait_ceiling_s = admit * (
        args.frame_fetch_timeout_ms / 1000.0 + args.infer_timeout_s
    )
    detector_name = detector.describe()["detector"] if detector else "stub"

    def _ms(seconds: float) -> float:
        return round(seconds * 1000.0, 1)

    def _draw_sim_marker(annotated, sim, cv2) -> None:
        """Magenta crosshair + 'SIMULATED' where the placed target projects:
        a saved frame can never pass for a real sighting."""
        anomaly, (px, py), _offset = sim
        x, y = int(round(px)), int(round(py))
        magenta = (255, 0, 255)
        cv2.drawMarker(annotated, (x, y), magenta, cv2.MARKER_CROSS, 40, 2)
        cv2.circle(annotated, (x, y), 24, magenta, 2)
        cv2.putText(
            annotated,
            f"SIMULATED {anomaly.get('signature') or 'target'}",
            (max(x - 60, 0), max(y - 30, 14)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, magenta, 2,
        )

    def _detect_impl(payload: bytes, admitted_at: float) -> bytes | ServiceResponse:
        started = time.monotonic()
        queue_ms = _ms(started - admitted_at)
        request = DetectionRequest.from_bytes(payload)

        # Detection consumes the actual published frame object instead of
        # trusting the name reference.
        try:
            frame_payload = fetch_segmented(
                request.frame.data_name,
                timeout_ms=args.frame_fetch_timeout_ms,
                metric_name="frame_fetch",
            )
            header = parse_frame(frame_payload)
        except Exception as exc:
            print_json(
                "gcs.frame.fetch_failed",
                frame=request.frame.data_name,
                error=str(exc),
                queue_ms=queue_ms,
                fetch_ms=_ms(time.monotonic() - started),
            )
            return ServiceResponse(
                status=False,
                error=f"frame fetch failed: {exc}",
            )
        fetched = time.monotonic()
        fetch_ms = _ms(fetched - started)
        print_json(
            "gcs.frame.fetched",
            frame=request.frame.data_name,
            bytes=len(frame_payload),
            sha256=header["sha256"],
            width=header.get("width"),
            height=header.get("height"),
            queue_ms=queue_ms,
            fetch_ms=fetch_ms,
        )

        timestamp_ns = gps_time_ns()
        capture_ns = int(header.get("gps_time_ns") or 0)
        # The capture pose travels IN the frame metadata (the drone agent tags
        # every search frame with lat/lon/agl/heading/attitude at capture).
        # Prefer it over the request's pose: the dashboard sends a
        # placeholder (0,0) pose and relies on this metadata. Without it the
        # estimate lands near null island, thousands of km from the fleet,
        # and trips the IUAS investigate max-range guard.
        meta = header.get("metadata", {}) or {}
        pose = request.frame.pose
        cap_lat = float(meta.get("lat_deg", pose.position.lat_deg))
        cap_lon = float(meta.get("lon_deg", pose.position.lon_deg))
        cap_agl = float(meta.get("agl_m", pose.position.alt_m or 0.0))
        cap_heading = meta.get("heading_deg")
        heading = (
            float(cap_heading)
            if cap_heading is not None
            else getattr(pose, "yaw_deg", None)
        )
        # attitude at capture: a translating multirotor is NOT level, and the
        # tilt swings the belly camera's ground footprint
        roll_deg = float(meta.get("roll_deg", 0.0))
        pitch_deg = float(meta.get("pitch_deg", 0.0))
        capture_pose = {
            "lat": cap_lat, "lon": cap_lon, "agl_m": cap_agl,
            "heading_deg": heading,
        }

        image = None
        detections = []
        all_classes = []
        infer_ms = 0.0
        if detector is not None:
            image = decode_image(frame_body(frame_payload))
            if image is None:
                return ServiceResponse(
                    status=False, error="frame body is not a decodable image"
                )
            image_size = (int(image.shape[1]), int(image.shape[0]))
            detections = detector.detect(image)
            all_classes = detector.last_all_detections
            infer_ms = _ms(time.monotonic() - fetched)
        else:
            image_size = (
                int(header.get("width") or 0), int(header.get("height") or 0)
            )

        # Operator-placed sim targets (the dashboard's ground truth rides
        # every request). Whatever the detector, a placed visual target that
        # projects inside THIS frame, at its capture pose and through the same
        # camera model real detections use, becomes a SIMULATED detection.
        # In the field (yolo) the anomalies used to be ignored entirely —
        # only the stub read them — so a placed target could never produce
        # a hit (anom-1 placed 16:50:06 on 2026-10-13: nothing).
        sim = None
        if request.anomalies:
            sim = simulated_target_in_frame(
                request.anomalies,
                cap_lat=cap_lat, cap_lon=cap_lon, agl_m=max(cap_agl, 0.0),
                image_size=image_size, hfov_deg=args.hfov_deg,
                heading_deg=heading, pitch_deg=pitch_deg, roll_deg=roll_deg,
                cam_yaw_offset_deg=args.cam_yaw_offset_deg,
            )
        sim_info = None
        if sim is not None:
            anomaly, pixel, sim_offset = sim
            sim_info = {
                "anomaly_id": str(anomaly.get("id", "")),
                "signature": str(anomaly.get("signature", "") or ""),
                "center_px": [round(pixel[0], 1), round(pixel[1], 1)],
                "offset_m": round(sim_offset, 2),
            }
        capture_age_ms = (
            round((gps_time_ns() - capture_ns) / 1e6) if capture_ns else None
        )
        print_json(
            "gcs.detection.inference",
            frame=request.frame.data_name,
            detector=detector_name,
            detections=[d.as_dict() for d in detections],
            all_classes=[d.as_dict() for d in all_classes],
            simulated=sim_info,
            queue_ms=queue_ms,
            fetch_ms=fetch_ms,
            infer_ms=infer_ms,
            handler_ms=_ms(time.monotonic() - admitted_at),
            capture_age_ms=capture_age_ms,
            capture_pose=capture_pose,
        )

        if image is not None:
            # debugging/dashboard breadcrumb: the exact frame as analyzed,
            # with every above-threshold box drawn (target class in green)
            # and any simulated target marked in magenta
            try:
                import cv2
                annotated = image.copy()
                for det in all_classes:
                    x, y, w, h = det.box_xywh
                    hit = det in detections
                    color = (0, 255, 0) if hit else (0, 165, 255)
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
                    shown = (
                        det.label if det.raw_label in ("", det.label)
                        else f"{det.label} ({det.raw_label})"
                    )
                    cv2.putText(
                        annotated,
                        f"{shown} {det.confidence:.2f}",
                        (x, max(y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )
                if sim is not None:
                    _draw_sim_marker(annotated, sim, cv2)
                cv2.imwrite("/tmp/muas-last-detect.jpg", annotated)
                _save_analyzed_frame(
                    args.frames_dir, args.frames_keep,
                    request.mission_id, request.frame.data_name, annotated,
                    {
                        "frame": request.frame.data_name,
                        "mission": request.mission_id,
                        "gps_time_ns": timestamp_ns,
                        "capture_pose": capture_pose,
                        "detections": [d.as_dict() for d in detections],
                        "all_classes": [d.as_dict() for d in all_classes],
                        "simulated": sim_info,
                    },
                    cv2,
                    raw_jpeg=frame_body(frame_payload),
                )
            except Exception:
                pass

        if detections:
            # A real detection wins its frame; a sim target in the same
            # frame is only logged (sim_info above) — the next frames over
            # it still report it.
            best = detections[0]
            north_m, east_m = project_ground(
                best.center_px,
                image_size,
                agl_m=max(cap_agl, 0.0),
                hfov_deg=args.hfov_deg,
                heading_deg=heading,
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                cam_yaw_offset_deg=args.cam_yaw_offset_deg,
            )
            lat_deg, lon_deg = offset_latlon(cap_lat, cap_lon, north_m, east_m)
            # In-frame offset magnitude: a nadir single-frame fix is most
            # trustworthy when the object is near frame center (small
            # offset) and degrades toward the edges (AGL/heading error is
            # amplified by the lever arm). Report it so the dashboard can
            # weight multi-frame detections of the same object.
            offset_mag_m = math.hypot(north_m, east_m)
            # Full provenance for the estimate: which pixel, at what pose,
            # produced what ground offset and final fix.
            print_json(
                "gcs.detection.projection",
                frame=request.frame.data_name,
                center_px=[round(best.center_px[0], 1), round(best.center_px[1], 1)],
                image_px=list(image_size),
                capture_lat=cap_lat, capture_lon=cap_lon, capture_agl=cap_agl,
                heading_deg=heading,
                pitch_deg=pitch_deg, roll_deg=roll_deg,
                offset_m={"north": round(north_m, 2), "east": round(east_m, 2)},
                estimate={"lat": lat_deg, "lon": lon_deg},
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
                offset_m=round(offset_mag_m, 2),
            )
            print_json(
                "gcs.detection.completed",
                frame=request.frame.data_name,
                detector="yolo",
                simulated=False,
                label=best.label,
                raw_label=best.raw_label,
                confidence=round(best.confidence, 4),
                box_xywh=list(best.box_xywh),
                offset_m={"north": round(north_m, 2), "east": round(east_m, 2)},
                estimate={"lat": lat_deg, "lon": lon_deg},
                evidence=response.evidence_ref,
                capture_age_ms=capture_age_ms,
            )
            return response.to_bytes()

        if sim is not None:
            anomaly, _pixel, sim_offset = sim
            # "SIM-" in the object id AND the simulated flag: every surface
            # that shows the id (events, cards, task ids) says it is fake
            object_id = "SIM-" + (sim_info["signature"] or "target").replace(" ", "-")
            response = DetectionResponse(
                mission_id=request.mission_id,
                object_id=object_id,
                confidence=0.93,
                estimate=GeoPoint(
                    lat_deg=float(anomaly["lat_deg"]),
                    lon_deg=float(anomaly["lon_deg"]),
                    alt_m=0.0,
                ),
                evidence_ref=mission_evidence_name(
                    request.mission_id, object_id, timestamp_ns
                ),
                offset_m=round(sim_offset, 2),
                simulated=True,
            )
            print_json(
                "gcs.detection.completed",
                frame=request.frame.data_name,
                detector="simulated",
                simulated=True,
                anomaly_id=sim_info["anomaly_id"],
                object_id=object_id,
                offset_m=sim_info["offset_m"],
                estimate={"lat": response.estimate.lat_deg, "lon": response.estimate.lon_deg},
                evidence=response.evidence_ref,
                confidence=response.confidence,
                capture_age_ms=capture_age_ms,
            )
            return response.to_bytes()

        if detector is not None:
            return ServiceResponse(status=False, error="no-detection")
        if request.anomalies:
            # stub with placed targets: hits come ONLY from the ground truth
            return ServiceResponse(
                status=False, error="no-detection (no placed target in frame)"
            )

        # Legacy stub (no vision, no placed targets): always "finds" a target
        # a fixed offset from the capture position. Synthetic by
        # construction, so flagged simulated like a placed target.
        response = DetectionResponse(
            mission_id=request.mission_id,
            object_id="target-001",
            confidence=0.91,
            estimate=GeoPoint(
                lat_deg=cap_lat + args.lat_offset_deg,
                lon_deg=cap_lon + args.lon_offset_deg,
                alt_m=0.0,
            ),
            evidence_ref=mission_evidence_name(
                request.mission_id,
                "target-001",
                timestamp_ns,
            ),
            simulated=True,
        )
        print_json(
            "gcs.detection.completed",
            frame=request.frame.data_name,
            detector="stub",
            simulated=True,
            frame_bytes=len(frame_payload),
            evidence=response.evidence_ref,
            confidence=response.confidence,
        )
        return response.to_bytes()

    def _run_detect(payload: bytes, admitted_at: float) -> bytes | ServiceResponse:
        try:
            return _detect_impl(payload, admitted_at)
        except Exception as exc:
            # an answer, not a silent handler death the dashboard can only
            # time out on
            print_json("gcs.detection.error", error=repr(exc))
            return ServiceResponse(status=False, error=f"detector error: {exc}")

    # Every service handler runs each command once however often it is
    # delivered: the dashboard re-issues a command until it is answered
    # (commands.py).
    executed_once = CommandDeduplicator(log=print_json).wrap

    @provider.handler(args.service)
    @executed_once
    def detect_object(payload: bytes) -> bytes | ServiceResponse:
        # Admission gate: at most `admit` frames are in the worker's queue;
        # excess is fast-rejected so a framework thread isn't parked behind a
        # backlog. The dashboard keeps no more than that in flight and
        # re-queues a "detector busy" frame, so the reject is a pacing
        # signal, never a lost frame (on 2026-10-13 it was final: 61 frames
        # rejected, the racket frames among them).
        request = DetectionRequest.from_bytes(payload)
        future = submit_admitted(
            _infer_pool, _detect_gate, _run_detect, payload, time.monotonic()
        )
        if future is None:
            print_json(
                "gcs.detection.rejected", reason="detector-busy",
                frame=request.frame.data_name,
            )
            return ServiceResponse(status=False, error="detector busy")
        print_json(
            "gcs.request.received",
            mission=request.mission_id,
            frame=request.frame.data_name,
            anomalies=len(request.anomalies),
        )
        try:
            return future.result(timeout=wait_ceiling_s)
        except FuturesTimeoutError:
            print_json(
                "gcs.detection.timeout", timeout_s=wait_ceiling_s,
                frame=request.frame.data_name,
            )
            return ServiceResponse(status=False, error="detector timeout")

    with optional_local_nfd(args.start_local_nfd):
        print_json("gcs.provider.starting", service=args.service)
        try:
            # Serve this provider's journal over NDN so the dashboard's
            # mission-bundle sweep can pull it without SSH (node id "gcs").
            # After the provider exists: the journal producer opens a second
            # ndn::Face, and building it during provider construction coincided
            # with a 300 s command blackout on an agent (run_drone_agent.py).
            start_journal_publisher("gcs", args.session)
            return provider.run(args.service)
        finally:
            flush_json_log()


if __name__ == "__main__":
    raise SystemExit(main())
