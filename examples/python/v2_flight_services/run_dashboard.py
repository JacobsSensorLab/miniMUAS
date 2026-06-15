#!/usr/bin/env python3
"""miniMUAS v2 GCS dashboard: web UI backend + mission orchestrator.

One process, three jobs:

  1. Web server (aiohttp) at http://0.0.0.0:8080 serving the single-page
     Leaflet UI (dashboard.html beside this file) and a WebSocket that
     carries everything: telemetry, search status, events, detections,
     video frames (binary), and operator commands.

  2. NDNSF user (/muas/v2/gcs): polls vehicle telemetry/search/video
     status objects, relays MJPEG video frames, and issues all service
     requests (raster-search, detect-object, investigate, rtl/land/hold,
     video control) via the wrapper's async API.

  3. Mission state machine — the detect->dispatch brain the agents
     deliberately don't have:

       idle -> searching: operator commits a raster (area+params) ->
               raster-search request to the WUAS (long timeout)
       searching: every NEW frame name in the WUAS SearchStatus spawns an
               async detect-object request (the raster never waits; NDNSF
               adds a ~constant per-request latency)
       hit (confidence >= threshold): hold the WUAS, drop the detection
               marker (with trigger-frame thumbnail), state -> dispatching
       dispatching -> investigating: investigate request to the IUAS with
               the operator's orbit tunables
       investigating -> done: result + capture artifact relayed to the UI

Threading: NDNSF blocking calls run in a ThreadPoolExecutor; NDNSF async
callbacks land on framework threads and are marshalled onto the asyncio
loop with call_soon_threadsafe. The UI only ever talks to the loop.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from contracts import (
    DetectionRequest,
    DetectionResponse,
    FrameRef,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    RasterSearchRequest,
    SearchArea,
    SearchStatus,
    TelemetrySample,
    VideoControlRequest,
    VideoStatus,
    gcs_detection_service,
    gps_time_ns,
    vehicle_flight_service,
    vehicle_search_status_name,
    vehicle_telemetry_live_name,
    vehicle_video_live_name,
    vehicle_video_service,
)
from dataplane import FRAME_CONTENT_TYPE, fetch_segmented, frame_body
from raster import build_raster, estimate_duration_s
from ndnsf_runtime import add_common_arguments, add_ndnsf_path, print_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="miniMUAS v2 GCS dashboard")
    add_common_arguments(parser)
    parser.add_argument("--user", default="/muas/v2/gcs")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--wuas-id", default="wuas-01")
    parser.add_argument("--iuas-id", default="iuas-01")
    parser.add_argument("--detect-timeout-ms", type=int, default=30000)
    parser.add_argument("--search-margin-s", type=float, default=60.0)
    parser.add_argument("--investigate-timeout-ms", type=int, default=120000)
    parser.add_argument(
        "--confirm-count", type=int, default=2,
        help="Independent detections (within target_separation_m) required "
        "before a candidate becomes a dispatched target. Guards against "
        "single-frame false positives launching the IUAS — a real object "
        "is seen on many consecutive frames; texture noise is not.",
    )
    parser.add_argument(
        "--html",
        default=None,
        help="Path to dashboard.html (default: alongside this script)",
    )
    parser.add_argument(
        "--tiles-dir",
        default="/var/lib/minimuas/tiles",
        help="Local satellite tile cache served at /tiles/{z}/{x}/{y}",
    )
    parser.add_argument(
        "--tile-upstream",
        default=(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
        help=(
            "Upstream tile URL template ({z}/{x}/{y} placeholders). When a "
            "tile is missing locally and the upstream is reachable (bench "
            "with internet), it is fetched once and cached — bench panning "
            "warms the cache the offline field deployment serves from. "
            "Empty string disables proxying (pure offline)."
        ),
    )
    return parser


_M_PER_DEG_LAT = 111_111.0


def _dist_m(lat_a, lon_a, lat_b, lon_b) -> float:
    dn = (lat_a - lat_b) * _M_PER_DEG_LAT
    de = (lon_a - lon_b) * _M_PER_DEG_LAT * max(
        math.cos(math.radians((lat_a + lat_b) / 2.0)), 1e-6
    )
    return math.hypot(dn, de)


class Dashboard:
    def __init__(self, args, user) -> None:
        self.args = args
        self.user = user
        self.vehicles = [args.wuas_id, args.iuas_id]
        self.loop: asyncio.AbstractEventLoop | None = None
        self.executor = ThreadPoolExecutor(max_workers=8)
        self.clients: set = set()

        # mission state machine (multi-target):
        #   searching: raster in progress; every deduped hit becomes a
        #     target and the search CONTINUES — the IUAS works the target
        #     queue in parallel, one investigation at a time
        #   investigating: raster finished, queue still draining
        #   done: raster finished and every target investigated
        self.mission = {
            "state": "idle",   # idle|searching|investigating|done|aborted
            "mission_id": "",
            "params": {},
            "search_done": False,
            "targets": [],      # {index, object_id, confidence, lat, lon,
                                #  frame, status: queued|investigating|done|
                                #  failed, artifacts: []}
        }
        self.targets_lock = threading.Lock()
        self.candidates: list[dict] = []  # pre-confirmation hits
        # per-vehicle enable gate (dashboard-side). A disabled vehicle
        # stays fully alive (telemetry, video) but the orchestrator will
        # not auto-dispatch to it and refuses manual flight commands to
        # it — this is how you fly WUAS-only: disable the IUAS so a
        # detection confirms and queues but never launches it.
        self.enabled: dict[str, bool] = {v: True for v in self.vehicles}
        self.seen_frames: set[str] = set()
        self.detects_pending = 0
        self.detects_done = 0
        self.video_relays: dict[str, dict] = {}  # vid -> {"enabled": bool, "seq": int}
        self.telemetry_age: dict[str, float] = {}
        # link health is measured on OUR clock only: cross-node wall-clock
        # differencing just reports clock skew on an RTC-less fleet (clocks
        # are set from GPS/FC/HTTPS-Date and are never aligned to better
        # than seconds-to-minutes). vid -> {last_ns, changed_mono}
        self.sample_state: dict[str, dict] = {}

    # ---- WS plumbing ------------------------------------------------------

    def _send_loop(self, payload) -> None:
        """Schedule a broadcast from any thread."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(self.broadcast(payload))
            )

    async def broadcast(self, payload) -> None:
        message = json.dumps(payload) if isinstance(payload, dict) else payload
        dead = []
        for ws in self.clients:
            try:
                if isinstance(message, bytes):
                    await ws.send_bytes(message)
                else:
                    await ws.send_str(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    def event(self, kind: str, **fields) -> None:
        record = {"type": "event", "kind": kind, "t": time.time(), **fields}
        print_json(f"dash.{kind}", **fields)
        self._send_loop(record)

    # ---- pollers (framework threads) ---------------------------------------

    def poll_forever(self) -> None:
        while True:
            for vid in self.vehicles:
                self._poll_vehicle(vid)
            time.sleep(1.0)

    def _poll_vehicle(self, vid: str) -> None:
        try:
            payload = fetch_segmented(
                vehicle_telemetry_live_name(vid), timeout_ms=800
            )
            sample = TelemetrySample.from_bytes(payload)
            now = time.monotonic()
            state = self.sample_state.setdefault(
                vid, {"last_ns": None, "changed_mono": now}
            )
            if sample.gps_time_ns != state["last_ns"]:
                state["last_ns"] = sample.gps_time_ns
                state["changed_mono"] = now
            # freshness on the dashboard's own clock: seconds since the
            # last NEW sample was observed (skew-immune)
            age_s = now - state["changed_mono"]
            # clock skew, reported separately as a time-subsystem diagnostic
            skew_s = (gps_time_ns() - sample.gps_time_ns) / 1e9
            self.telemetry_age[vid] = now
            self._send_loop({
                "type": "telemetry",
                "vehicle": vid,
                "sample": json.loads(payload.decode()),
                "age_s": round(age_s, 1),
                "skew_s": round(skew_s, 1),
            })
        except Exception:
            # stale-marker danger: tell the UI explicitly how old we are
            last = self.telemetry_age.get(vid)
            silent_s = None if last is None else time.monotonic() - last
            self._send_loop({
                "type": "telemetry_stale",
                "vehicle": vid,
                "silent_s": None if silent_s is None else round(silent_s, 1),
            })

        if vid == self.args.wuas_id and self.mission["state"] == "searching":
            try:
                payload = fetch_segmented(
                    vehicle_search_status_name(vid), timeout_ms=800
                )
                status = SearchStatus.from_bytes(payload)
                self._send_loop({
                    "type": "search_status",
                    "vehicle": vid,
                    "status": json.loads(payload.decode()),
                    "detects_pending": self.detects_pending,
                    "detects_done": self.detects_done,
                })
                for frame in status.last_frames:
                    if frame not in self.seen_frames:
                        self.seen_frames.add(frame)
                        self._detect_frame(frame)
            except Exception:
                pass

    # ---- detection fan-out ---------------------------------------------------

    def _detect_frame(self, frame_name: str) -> None:
        params = self.mission["params"]
        request = DetectionRequest(
            mission_id=self.mission["mission_id"],
            frame=FrameRef(
                data_name=frame_name,
                gps_time_ns=gps_time_ns(),
                seq=1,
                camera_id="bottom",
                # placeholder pose; the GCS provider prefers the true
                # capture pose embedded in the frame metadata by the agent
                pose=Pose(position=GeoPoint(0.0, 0.0, 0.0), yaw_deg=0.0),
                content_type=FRAME_CONTENT_TYPE,
            ),
            object_query=params.get("object_query", "tennis racket"),
        )
        self.detects_pending += 1
        self.event("detect.sent", frame=frame_name)

        def on_response(response) -> None:
            self.detects_pending -= 1
            self.detects_done += 1
            if not response.status:
                self.event("detect.miss", frame=frame_name, error=response.error)
                return
            detection = DetectionResponse.from_bytes(response.payload)
            self.event(
                "detect.hit",
                frame=frame_name,
                object_id=detection.object_id,
                confidence=round(detection.confidence, 4),
                lat=detection.estimate.lat_deg,
                lon=detection.estimate.lon_deg,
                offset_m=round(detection.offset_m, 2),
            )
            min_conf = float(params.get("min_confidence", 0.3))
            if (
                detection.confidence >= min_conf
                and self.mission["state"] in ("searching", "investigating")
            ):
                self._on_detect_hit(detection, frame_name)

        def on_timeout(_request_id: str) -> None:
            self.detects_pending -= 1
            self.detects_done += 1
            self.event("detect.timeout", frame=frame_name)

        self.user.request_service_async(
            gcs_detection_service(),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=self.args.detect_timeout_ms,
        )

    # ---- multi-target machinery -------------------------------------------

    def _on_detect_hit(self, detection: DetectionResponse, frame: str) -> None:
        """Confirm-then-queue. A hit first reinforces a CANDIDATE; only a
        candidate seen on `confirm_count` separate frames is promoted to a
        dispatched target. This is the guard against the field failure
        where a single 99% texture false-positive launched the IUAS.

        Dedup is by ground distance: hits within `target_separation_m`
        belong to the same candidate/target, best-confidence estimate
        kept. Already-dispatched targets just absorb further hits.
        """
        sep = float(self.mission["params"].get("target_separation_m", 5.0))
        need = max(1, int(self.args.confirm_count))
        lat, lon = detection.estimate.lat_deg, detection.estimate.lon_deg
        with self.targets_lock:
            # already a confirmed target nearby? absorb + maybe refine.
            for target in self.mission["targets"]:
                if _dist_m(target["lat"], target["lon"], lat, lon) <= sep:
                    cand_conf = max(target["confidence"], detection.confidence)
                    target["confidence"] = cand_conf
                    # refine position only from a BETTER-localized sighting,
                    # and only while not yet flown
                    if (
                        target["status"] == "queued"
                        and detection.offset_m < target.get("best_offset", 1e9)
                    ):
                        target["best_offset"] = detection.offset_m
                        target["lat"], target["lon"], target["frame"] = lat, lon, frame
                        self.event(
                            "target.updated", index=target["index"],
                            confidence=round(target["confidence"], 4),
                            lat=target["lat"], lon=target["lon"], frame=frame,
                            best_offset_m=round(target["best_offset"], 2),
                        )
                    return
            # otherwise reinforce / create a candidate.
            cand = None
            for c in self.candidates:
                if _dist_m(c["lat"], c["lon"], lat, lon) <= sep:
                    cand = c
                    break
            if cand is None:
                cand = {
                    "hits": 0, "object_id": detection.object_id,
                    "confidence": detection.confidence,
                    "lat": lat, "lon": lon, "frame": frame,
                    "best_offset": detection.offset_m,
                    "frames": set(),
                }
                self.candidates.append(cand)
            cand["frames"].add(frame)
            cand["hits"] = len(cand["frames"])
            cand["confidence"] = max(cand["confidence"], detection.confidence)
            # POSITION comes from the best-localized sighting (object
            # nearest frame center => smallest nadir offset => least
            # AGL/heading lever-arm error), NOT the highest confidence.
            # This is what fixes the field symptom: the racquet's fix
            # snaps to the pass where it was directly underneath, instead
            # of a corner glimpse where it sat at the frame edge.
            if detection.offset_m < cand["best_offset"]:
                cand["best_offset"] = detection.offset_m
                cand["lat"], cand["lon"], cand["frame"] = lat, lon, frame
            self.event(
                "detect.candidate", object_id=cand["object_id"],
                hits=cand["hits"], need=need,
                confidence=round(cand["confidence"], 4),
                lat=cand["lat"], lon=cand["lon"],
                best_offset_m=round(cand["best_offset"], 2),
            )
            if cand["hits"] < need:
                return
            # promote candidate -> target
            self.candidates.remove(cand)
            target = {
                "index": len(self.mission["targets"]),
                "object_id": cand["object_id"],
                "confidence": cand["confidence"],
                "lat": cand["lat"], "lon": cand["lon"],
                "frame": cand["frame"],
                "best_offset": cand["best_offset"],
                "status": "queued",
                "artifacts": [],
            }
            self.mission["targets"].append(target)
        self.event(
            "mission.target_found",
            index=target["index"], object_id=target["object_id"],
            confidence=round(target["confidence"], 4),
            lat=target["lat"], lon=target["lon"], frame=target["frame"],
            hits=need,
        )
        self._pump_dispatch()

    def _pump_dispatch(self) -> None:
        """Send the next queued target to the IUAS if it's idle and enabled."""
        with self.targets_lock:
            if self.mission["state"] not in ("searching", "investigating"):
                return  # operator aborted: stop draining the queue
            if not self.enabled.get(self.args.iuas_id, True):
                # IUAS disabled (e.g. WUAS-only flight): targets confirm
                # and queue but are never launched. Mission can still
                # complete — a queued-but-undispatchable target isn't
                # "in flight", so check completion explicitly.
                if self.mission["search_done"] and not any(
                    t["status"] == "investigating"
                    for t in self.mission["targets"]
                ):
                    self._complete_locked(note="iuas-disabled")
                return
            if any(
                t["status"] == "investigating"
                for t in self.mission["targets"]
            ):
                return
            target = next(
                (t for t in self.mission["targets"] if t["status"] == "queued"),
                None,
            )
            if target is None:
                self._maybe_complete_locked()
                return
            target["status"] = "investigating"
        self._dispatch_iuas(target)

    def _maybe_complete_locked(self) -> None:
        """Caller holds targets_lock. Mission ends when the raster is done
        and no target is queued or in flight."""
        if not self.mission["search_done"]:
            return
        if self.mission["state"] not in ("searching", "investigating"):
            return
        if any(
            t["status"] in ("queued", "investigating")
            for t in self.mission["targets"]
        ):
            self.mission["state"] = "investigating"
            return
        self._complete_locked()

    def _complete_locked(self, note: str = "") -> None:
        """Caller holds targets_lock. Mark mission done and announce."""
        if self.mission["state"] not in ("searching", "investigating"):
            return
        self.mission["state"] = "done"
        targets = self.mission["targets"]
        self._send_loop({"type": "event", "kind": "mission.completed",
                         "t": time.time(),
                         "targets": len(targets),
                         "investigated": sum(
                             1 for t in targets if t["status"] == "done"
                         ),
                         "note": note})
        print_json(
            "dash.mission.completed",
            targets=len(targets),
            investigated=sum(1 for t in targets if t["status"] == "done"),
            note=note,
        )

    def _dispatch_iuas(self, target: dict) -> None:
        params = self.mission["params"]
        request = InvestigatePointRequest(
            mission_id=self.mission["mission_id"],
            source_detection_id=f"{target['object_id']}-{target['index']}",
            target=GeoPoint(
                lat_deg=target["lat"], lon_deg=target["lon"], alt_m=0.0
            ),
            approach_alt_m=float(params.get("orbit_agl_m", 8.0)),
            standoff_m=float(params.get("orbit_radius_m", 6.0)),
            circle_radius_m=float(params.get("orbit_radius_m", 6.0)),
            circle_count=float(params.get("orbit_count", 1.0)),
            sensor_plan=["front"],
        )
        self.event(
            "target.dispatch",
            index=target["index"],
            vehicle=self.args.iuas_id,
            lat=request.target.lat_deg,
            lon=request.target.lon_deg,
            radius_m=request.circle_radius_m,
            agl_m=request.approach_alt_m,
        )

        def finish(status: str, artifacts: list[str], note: str = "") -> None:
            with self.targets_lock:
                target["status"] = status
                target["artifacts"] = artifacts
            self.event(
                "target.completed" if status == "done" else "target.failed",
                index=target["index"],
                artifacts=artifacts,
                note=note,
            )
            self._pump_dispatch()

        def on_response(response) -> None:
            if not response.status:
                finish("failed", [], note=response.error)
                return
            from contracts import FlightTaskResult

            result = FlightTaskResult.from_bytes(response.payload)
            finish(
                "done" if result.status == "completed" else "failed",
                [a.data_name for a in result.artifacts],
                note=result.notes,
            )

        def on_timeout(_request_id: str) -> None:
            finish("failed", [], note="timeout")

        self.user.request_service_async(
            vehicle_flight_service(self.args.iuas_id, "investigate"),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=self.args.investigate_timeout_ms,
        )

    # ---- operator commands (from the WS) ----------------------------------------

    def start_mission(self, params: dict) -> None:
        if self.mission["state"] in ("searching", "investigating"):
            self.event("mission.rejected", reason=f"state={self.mission['state']}")
            return
        mission_id = f"mission-{int(time.time())}"
        with self.targets_lock:
            self.mission.update(
                state="searching",
                mission_id=mission_id,
                params=params,
                search_done=False,
                targets=[],
            )
        self.seen_frames.clear()
        self.candidates.clear()
        self.detects_pending = 0
        self.detects_done = 0

        area = SearchArea.from_dict(params.get("area", {}))
        request = RasterSearchRequest(
            mission_id=mission_id,
            area=area,
            agl_m=float(params.get("agl_m", 6.0)),
            leg_spacing_m=float(params.get("leg_spacing_m", 5.0)),
            speed_m_s=float(params.get("speed_m_s", 2.0)),
            capture_every_m=float(params.get("capture_every_m", 4.0)),
            object_query=str(params.get("object_query", "tennis racket")),
            min_confidence=float(params.get("min_confidence", 0.3)),
            max_duration_s=float(params.get("max_duration_s", 600.0)),
        )
        timeout_ms = int((request.max_duration_s + self.args.search_margin_s) * 1000)
        self.event(
            "mission.started",
            mission_id=mission_id,
            vehicle=self.args.wuas_id,
            agl_m=request.agl_m,
        )

        def on_response(response) -> None:
            if response.status:
                from contracts import RasterSearchResult

                result = RasterSearchResult.from_bytes(response.payload)
                self.event(
                    "mission.search_finished",
                    status=result.status,
                    frames=result.frames_captured,
                )
            else:
                self.event("mission.search_failed", error=response.error)
            with self.targets_lock:
                self.mission["search_done"] = True
                if self.mission["state"] == "searching" and any(
                    t["status"] in ("queued", "investigating")
                    for t in self.mission["targets"]
                ):
                    self.mission["state"] = "investigating"
            # drain (or immediately complete) the target queue
            self._pump_dispatch()

        def on_timeout(_request_id: str) -> None:
            self.event("mission.search_timeout")
            with self.targets_lock:
                self.mission["search_done"] = True
                if self.mission["state"] == "searching" and any(
                    t["status"] in ("queued", "investigating")
                    for t in self.mission["targets"]
                ):
                    self.mission["state"] = "investigating"
            self._pump_dispatch()

        self.user.request_service_async(
            vehicle_flight_service(self.args.wuas_id, "raster-search"),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=timeout_ms,
        )

    def _flight_command(self, vid: str, command: str, params: dict | None = None) -> None:
        self.event("command.sent", vehicle=vid, command=command)
        payload = b"{}"
        if command == "takeoff":
            from contracts import TakeoffRequest

            agl = float((params or {}).get("target_agl_m", 5.0))
            payload = TakeoffRequest(target_agl_m=agl).to_bytes()

        def on_response(response) -> None:
            self.event(
                "command.result",
                vehicle=vid,
                command=command,
                ok=bool(response.status),
                error=response.error,
            )

        def on_timeout(_request_id: str) -> None:
            self.event("command.timeout", vehicle=vid, command=command)

        self.user.request_service_async(
            vehicle_flight_service(vid, command),
            payload,
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=20000 if command == "takeoff" else 15000,
        )

    def set_video(self, vid: str, params: dict) -> None:
        request = VideoControlRequest(
            enable=bool(params.get("enable", False)),
            width=int(params.get("width", 320)),
            height=int(params.get("height", 240)),
            fps=float(params.get("fps", 5.0)),
            quality=int(params.get("quality", 40)),
        )
        relay = self.video_relays.setdefault(vid, {"enabled": False, "seq": 0})
        relay["enabled"] = request.enable
        self.event("video.control", vehicle=vid, enable=request.enable)

        def on_response(response) -> None:
            if response.status:
                status = VideoStatus.from_bytes(response.payload)
                relay["seq"] = status.seq
                if request.enable and not relay.get("thread_alive"):
                    relay["thread_alive"] = True
                    threading.Thread(
                        target=self._video_relay, args=(vid, relay), daemon=True
                    ).start()
            else:
                self.event("video.control_failed", vehicle=vid, error=response.error)

        def on_timeout(_request_id: str) -> None:
            self.event("video.control_timeout", vehicle=vid)

        self.user.request_service_async(
            vehicle_video_service(vid),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=15000,
        )

    def _video_relay(self, vid: str, relay: dict) -> None:
        """Poll the vehicle's latest-wins live name and forward new frames.

        fetch_segmented on the base name runs version discovery, so every
        poll returns the NEWEST published frame — latency is one fetch,
        independent of how long the stream has run. The 8-byte seq header
        drops duplicates (same version fetched twice, possibly from the
        local NFD content store within the freshness window) and the rare
        out-of-order race during producer handover. Binary WS message:
        1 byte vehicle index + JPEG.
        """
        name = vehicle_video_live_name(vid)
        last_seq = 0
        window_t0 = time.monotonic()
        window_bytes = 0
        window_frames = 0
        while relay["enabled"]:
            try:
                payload = fetch_segmented(name, timeout_ms=1000)
                seq = int.from_bytes(payload[:8], "big")
                if seq <= last_seq and seq != 0:
                    time.sleep(0.08)  # nothing new yet; cheap local re-poll
                    continue
                last_seq = seq
                jpeg = payload[8:]
                window_bytes += len(jpeg)
                window_frames += 1
                index = self.vehicles.index(vid)
                self._send_loop(bytes([index]) + jpeg)
                now = time.monotonic()
                if now - window_t0 >= 2.0:
                    self._send_loop({
                        "type": "video_stats",
                        "vehicle": vid,
                        "fps": round(window_frames / (now - window_t0), 1),
                        "kbps": round(window_bytes * 8 / (now - window_t0) / 1000),
                        "seq": seq,
                    })
                    window_t0, window_bytes, window_frames = now, 0, 0
            except Exception:
                # stream gap (producer restarting, radio loss): brief pause,
                # then re-poll — the next success is the live frame, never
                # a backlog
                time.sleep(0.15)
        relay["thread_alive"] = False

    def fetch_artifact_jpeg(self, name: str) -> bytes | None:
        try:
            payload = fetch_segmented(name, timeout_ms=15000)
            return frame_body(payload)
        except Exception as exc:
            self.event("artifact.fetch_failed", name=name, error=str(exc))
            return None

    def handle_command(self, message: dict) -> dict | None:
        kind = message.get("cmd")
        if kind == "preview_raster":
            area = SearchArea.from_dict(message.get("area", {}))
            plan = build_raster(
                area,
                leg_spacing_m=float(message.get("leg_spacing_m", 5.0)),
                capture_every_m=float(message.get("capture_every_m", 4.0)),
            )
            return {
                "type": "raster_preview",
                "plan": plan.as_dict(),
                "estimate_s": round(
                    estimate_duration_s(
                        plan, speed_m_s=float(message.get("speed_m_s", 2.0))
                    ),
                    1,
                ),
            }
        if kind == "start_mission":
            self.start_mission(message.get("params", {}))
        elif kind == "set_enabled":
            vid = message.get("vehicle", "")
            if vid in self.vehicles:
                self.enabled[vid] = bool(message.get("enabled", True))
                self.event(
                    "vehicle.enabled" if self.enabled[vid] else "vehicle.disabled",
                    vehicle=vid,
                )
                # re-enabling the IUAS mid-mission should pick up any
                # targets that queued while it was disabled
                if self.enabled[vid]:
                    self._pump_dispatch()
        elif kind == "flight":
            vid = message.get("vehicle", "")
            command = message.get("command", "")
            if command in ("rtl", "land", "hold", "takeoff") and vid in self.vehicles:
                if not self.enabled.get(vid, True):
                    # safety actions (rtl/land/hold) are ALWAYS allowed,
                    # even to a disabled vehicle — disable must never trap
                    # an aircraft in the air. Only takeoff is blocked.
                    if command == "takeoff":
                        self.event(
                            "command.rejected", vehicle=vid,
                            command=command, reason="vehicle disabled",
                        )
                        return None
                if (
                    self.mission["state"] == "searching"
                    and vid == self.args.wuas_id
                    and command in ("rtl", "land")
                ):
                    self.mission["state"] = "aborted"
                self._flight_command(vid, command, message.get("params"))
        elif kind == "all":
            command = message.get("command", "")
            if command in ("rtl", "land", "hold"):
                if self.mission["state"] in ("searching", "investigating"):
                    self.mission["state"] = "aborted"
                for vid in self.vehicles:
                    self._flight_command(vid, command)
        elif kind == "video":
            vid = message.get("vehicle", "")
            if vid in self.vehicles:
                self.set_video(vid, message.get("params", {}))
        return None


async def run_web(dash: Dashboard, args) -> None:
    from aiohttp import WSMsgType, web

    html_path = Path(
        args.html or Path(__file__).resolve().parent / "dashboard.html"
    )

    async def index(_request):
        return web.Response(
            text=html_path.read_text(), content_type="text/html"
        )

    async def artifact(request):
        name = request.query.get("name", "")
        body = await asyncio.get_event_loop().run_in_executor(
            dash.executor, dash.fetch_artifact_jpeg, name
        )
        if body is None:
            return web.Response(status=404, text="artifact unavailable")
        return web.Response(body=body, content_type="image/jpeg")

    async def tile(request):
        """Serve satellite tiles: local cache first, then (if configured
        and reachable) the upstream — caching what it fetches so the field
        deployment serves the same tiles with no internet."""
        try:
            z = int(request.match_info["z"])
            x = int(request.match_info["x"])
            y = int(request.match_info["y"])
        except (KeyError, ValueError):
            return web.Response(status=400)
        if not (0 <= z <= 20):
            return web.Response(status=400)
        path = Path(args.tiles_dir) / str(z) / str(x) / f"{y}.jpg"
        if path.exists():
            return web.Response(
                body=path.read_bytes(),
                content_type="image/jpeg",
                headers={"Cache-Control": "max-age=86400"},
            )
        if args.tile_upstream:
            import aiohttp

            url = args.tile_upstream.format(z=z, x=x, y=y)
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.get(
                        url, timeout=aiohttp.ClientTimeout(total=4)
                    ) as upstream:
                        if upstream.status == 200:
                            body = await upstream.read()
                            try:
                                path.parent.mkdir(parents=True, exist_ok=True)
                                path.write_bytes(body)
                            except Exception:
                                pass  # cache write failure isn't fatal
                            return web.Response(
                                body=body,
                                content_type="image/jpeg",
                                headers={"Cache-Control": "max-age=86400"},
                            )
            except Exception:
                pass  # offline / filtered: fall through to 404 -> grid
        return web.Response(status=404)

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        dash.clients.add(ws)
        await ws.send_str(json.dumps({
            "type": "hello",
            "vehicles": dash.vehicles,
            "enabled": dash.enabled,
            "mission": {
                "state": dash.mission["state"],
                "mission_id": dash.mission["mission_id"],
                "targets": dash.mission["targets"],
            },
        }))
        try:
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    try:
                        parsed = json.loads(message.data)
                    except Exception:
                        continue
                    reply = await asyncio.get_event_loop().run_in_executor(
                        dash.executor, dash.handle_command, parsed
                    )
                    if reply is not None:
                        await ws.send_str(json.dumps(reply))
        finally:
            dash.clients.discard(ws)
        return ws

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/artifact", artifact)
    app.router.add_get("/tiles/{z}/{x}/{y}", tile)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.http_host, args.http_port)
    await site.start()
    print_json("dash.serving", host=args.http_host, port=args.http_port)
    while True:
        await asyncio.sleep(3600)


def main() -> int:
    args = build_parser().parse_args()
    if args.dry_run:
        print_json("dash.dry_run", user=args.user, port=args.http_port)
        return 0

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import ServiceUser
    from ndnsf_runtime import user_kwargs

    user = ServiceUser(**user_kwargs(args, args.user))
    user.start()  # background event loop for request_service_async

    dash = Dashboard(args, user)
    threading.Thread(target=dash.poll_forever, daemon=True).start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    dash.loop = loop
    try:
        loop.run_until_complete(run_web(dash, args))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
