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
    vehicle_video_frame_name,
    vehicle_video_service,
    vehicle_video_status_name,
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
        "--html",
        default=None,
        help="Path to dashboard.html (default: alongside this script)",
    )
    return parser


class Dashboard:
    def __init__(self, args, user) -> None:
        self.args = args
        self.user = user
        self.vehicles = [args.wuas_id, args.iuas_id]
        self.loop: asyncio.AbstractEventLoop | None = None
        self.executor = ThreadPoolExecutor(max_workers=8)
        self.clients: set = set()

        # mission state machine
        self.mission = {
            "state": "idle",          # idle|searching|dispatching|investigating|done|aborted
            "mission_id": "",
            "params": {},             # last committed mission params (UI dict)
            "detection": None,        # {lat, lon, confidence, object_id, frame}
            "investigation": None,    # result dict
        }
        self.seen_frames: set[str] = set()
        self.detects_pending = 0
        self.detects_done = 0
        self.video_relays: dict[str, dict] = {}  # vid -> {"enabled": bool, "seq": int}
        self.telemetry_age: dict[str, float] = {}

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
            age_s = max(0.0, (gps_time_ns() - sample.gps_time_ns) / 1e9)
            self.telemetry_age[vid] = time.monotonic()
            self._send_loop({
                "type": "telemetry",
                "vehicle": vid,
                "sample": json.loads(payload.decode()),
                "age_s": round(age_s, 1),
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
            )
            min_conf = float(params.get("min_confidence", 0.3))
            if (
                detection.confidence >= min_conf
                and self.mission["state"] == "searching"
            ):
                self._on_target_found(detection, frame_name)

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

    # ---- state machine transitions --------------------------------------------

    def _on_target_found(self, detection: DetectionResponse, frame: str) -> None:
        self.mission["state"] = "dispatching"
        self.mission["detection"] = {
            "object_id": detection.object_id,
            "confidence": detection.confidence,
            "lat": detection.estimate.lat_deg,
            "lon": detection.estimate.lon_deg,
            "frame": frame,
        }
        self.event(
            "mission.target_found",
            **self.mission["detection"],
        )
        # stop the searcher where it is; operator decides RTL later
        self._flight_command(self.args.wuas_id, "hold")
        self._dispatch_iuas(detection)

    def _dispatch_iuas(self, detection: DetectionResponse) -> None:
        params = self.mission["params"]
        request = InvestigatePointRequest(
            mission_id=self.mission["mission_id"],
            source_detection_id=detection.object_id,
            target=GeoPoint(
                lat_deg=detection.estimate.lat_deg,
                lon_deg=detection.estimate.lon_deg,
                alt_m=0.0,
            ),
            approach_alt_m=float(params.get("orbit_agl_m", 8.0)),
            standoff_m=float(params.get("orbit_radius_m", 6.0)),
            circle_radius_m=float(params.get("orbit_radius_m", 6.0)),
            circle_count=float(params.get("orbit_count", 1.0)),
            sensor_plan=["front"],
        )
        self.mission["state"] = "investigating"
        self.event(
            "mission.dispatch",
            vehicle=self.args.iuas_id,
            lat=request.target.lat_deg,
            lon=request.target.lon_deg,
            radius_m=request.circle_radius_m,
            agl_m=request.approach_alt_m,
        )

        def on_response(response) -> None:
            if not response.status:
                self.mission["state"] = "done"
                self.event("mission.investigate_failed", error=response.error)
                return
            from contracts import FlightTaskResult

            result = FlightTaskResult.from_bytes(response.payload)
            artifacts = [a.data_name for a in result.artifacts]
            self.mission["state"] = "done"
            self.mission["investigation"] = {
                "task_id": result.task_id,
                "status": result.status,
                "artifacts": artifacts,
            }
            self.event(
                "mission.completed",
                status=result.status,
                artifacts=artifacts,
            )

        def on_timeout(_request_id: str) -> None:
            self.mission["state"] = "done"
            self.event("mission.investigate_timeout")

        self.user.request_service_async(
            vehicle_flight_service(self.args.iuas_id, "investigate"),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=self.args.investigate_timeout_ms,
        )

    # ---- operator commands (from the WS) ----------------------------------------

    def start_mission(self, params: dict) -> None:
        if self.mission["state"] in ("searching", "dispatching", "investigating"):
            self.event("mission.rejected", reason=f"state={self.mission['state']}")
            return
        mission_id = f"mission-{int(time.time())}"
        self.mission.update(
            state="searching",
            mission_id=mission_id,
            params=params,
            detection=None,
            investigation=None,
        )
        self.seen_frames.clear()
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
            if self.mission["state"] == "searching":
                # search ended without a dispatch: report its own outcome
                if response.status:
                    from contracts import RasterSearchResult

                    result = RasterSearchResult.from_bytes(response.payload)
                    self.mission["state"] = "done"
                    self.event(
                        "mission.search_finished",
                        status=result.status,
                        frames=result.frames_captured,
                    )
                else:
                    self.mission["state"] = "done"
                    self.event("mission.search_failed", error=response.error)

        def on_timeout(_request_id: str) -> None:
            if self.mission["state"] == "searching":
                self.mission["state"] = "done"
                self.event("mission.search_timeout")

        self.user.request_service_async(
            vehicle_flight_service(self.args.wuas_id, "raster-search"),
            request.to_bytes(),
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=timeout_ms,
        )

    def _flight_command(self, vid: str, command: str) -> None:
        self.event("command.sent", vehicle=vid, command=command)

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
            b"{}",
            on_response=on_response,
            on_timeout=on_timeout,
            timeout_ms=15000,
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
        """Chase /video/<seq> and push JPEG frames over the WS as binary.

        Binary message layout: 1 byte vehicle index + JPEG bytes.
        """
        misses = 0
        window_t0 = time.monotonic()
        window_bytes = 0
        window_frames = 0
        while relay["enabled"]:
            seq = relay["seq"] + 1
            try:
                payload = fetch_segmented(
                    vehicle_video_frame_name(vid, seq), timeout_ms=1500
                )
                relay["seq"] = seq
                misses = 0
                window_bytes += len(payload)
                window_frames += 1
                index = self.vehicles.index(vid)
                self._send_loop(bytes([index]) + payload)
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
                misses += 1
                if misses % 5 == 0:
                    # resync from the published status (stream may have
                    # restarted or jumped)
                    try:
                        status = VideoStatus.from_bytes(
                            fetch_segmented(
                                vehicle_video_status_name(vid), timeout_ms=800
                            )
                        )
                        relay["seq"] = max(relay["seq"], status.seq)
                    except Exception:
                        pass
                time.sleep(0.2)
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
        elif kind == "flight":
            vid = message.get("vehicle", "")
            command = message.get("command", "")
            if command in ("rtl", "land", "hold") and vid in self.vehicles:
                if self.mission["state"] == "searching" and vid == self.args.wuas_id:
                    self.mission["state"] = "aborted"
                self._flight_command(vid, command)
        elif kind == "all":
            command = message.get("command", "")
            if command in ("rtl", "land", "hold"):
                if self.mission["state"] == "searching":
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

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        dash.clients.add(ws)
        await ws.send_str(json.dumps({
            "type": "hello",
            "vehicles": dash.vehicles,
            "mission": {
                k: v for k, v in dash.mission.items() if k != "params"
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
