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
import os
import re
import shutil
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from contracts import (
    CapabilityProfile,
    DetectionRequest,
    DetectionResponse,
    FrameRef,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    RasterSearchRequest,
    SearchArea,
    SearchStatus,
    SensorCaptureRequest,
    SensorCaptureResult,
    TelemetrySample,
    VideoControlRequest,
    VideoStatus,
    gcs_detection_service,
    gps_time_ns,
    vehicle_flight_service,
    vehicle_search_status_name,
    vehicle_sensor_event_name,
    vehicle_sensor_service,
    vehicle_system_service,
    vehicle_journal_name,
    vehicle_telemetry_live_name,
    vehicle_telemetry_state_name,
    vehicle_video_live_name,
    vehicle_video_service,
)
from dataplane import (
    FRAME_CONTENT_TYPE,
    fetch_segmented,
    frame_body,
    parse_frame,
)
from raster import build_raster, estimate_duration_s
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    flush_json_log,
    print_json,
    start_nfd_counter_scrape,
    start_role_journal,
)
import metrics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="miniMUAS v2 GCS dashboard")
    add_common_arguments(parser)
    parser.add_argument("--user", default="/muas/v2/gcs")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--wuas-id", default="wuas-01")
    parser.add_argument("--iuas-id", default="iuas-01")
    parser.add_argument(
        "--iuas-ids", default=None,
        help="Comma-separated IUAS vehicle ids (e.g. iuas-01,iuas-02). "
        "Targets dispatch per requested sensor to whichever idle enabled "
        "IUAS advertises it — one drone can carry the camera and another "
        "the microphone. Default: just --iuas-id.",
    )
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
        "--record-dir",
        default="/var/lib/minimuas/replays",
        help="Mission recorder: every dashboard broadcast (telemetry, "
        "events, detections, sensor data — everything except binary "
        "video) is appended to a timestamped JSONL here, replayable in "
        "the UI via the Replay button. Unwritable directory disables "
        "recording; empty string disables explicitly.",
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
    parser.add_argument(
        "--log-dir",
        default="/var/lib/minimuas/log",
        help="Directory for the fsync-per-line metrics/event journal "
        "(empty string disables).",
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
        self.iuas_ids = (
            [v.strip() for v in args.iuas_ids.split(",") if v.strip()]
            if args.iuas_ids
            else [args.iuas_id]
        )
        self.vehicles = [args.wuas_id] + self.iuas_ids
        # vid -> set of investigation sensors the vehicle advertises
        # ("camera", "audio"); populated from CapabilityProfile extras
        self.capabilities: dict[str, set] = {}
        # vid -> advertised sensor_meta dict (camera FoV / audio reach) for
        # the dashboard's coverage layer; populated from CapabilityProfile.
        self.sensor_meta: dict[str, dict] = {}
        # operator-placed sim ground-truth anomalies (targets the synthetic
        # detector finds): {id, kind, lat_deg, lon_deg, size_m|loudness_db,
        # signature, created_ns}. The dashboard IS the v2 sim operator, so it
        # owns this world model; it rides each detect request to the GCS.
        self.anomalies: list[dict] = []
        self.anomalies_lock = threading.Lock()
        self._anomaly_seq = 0
        # everything captured this session, mission or operator-tasked:
        # {vehicle, sensor, kind, name, lat, lon, t, source, label}
        # — feeds the map's sensor-data layer and the playback modal
        self.sensor_data: list[dict] = []
        self.sensor_data_lock = threading.Lock()
        # last decoded telemetry per vehicle (armed guard for shutdown)
        self.last_sample: dict[str, dict] = {}
        # mission recorder: every broadcast dict -> timestamped JSONL
        self.record_dir: Path | None = (
            Path(args.record_dir) if args.record_dir else None
        )
        self.record_lock = threading.Lock()
        self.record_file = None
        self.record_path: Path | None = None
        self._record_synced = 0.0
        # imported mission bundle (sim-mode replay): when set, /artifact
        # resolves stored media from here before touching the fabric.
        self.bundle = None
        self.bundle_dir: Path | None = None
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
                                #  failed, artifacts: [], jobs: [...]}
            # End-of-raster candidates that fell short of confirm_count.
            # Surfaced to the operator (never auto-dispatched, never blocking
            # completion) with promote ("investigate anyway") / dismiss.
            "unconfirmed": [],  # {index, object_id, confidence, lat, lon,
                                #  frame, best_offset, hits, need,
                                #  status: unconfirmed|promoted|dismissed}
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
        # Live video is latest-wins per vehicle. A single drainer coalesces
        # frames and applies per-send backpressure, so a slow/mesh WS client
        # drops stale frames instead of piling a broadcast task per frame onto
        # the loop (which starved the event loop and hung all HTTP).
        self.video_slots: dict[int, bytes] = {}
        self._video_drain_task = None
        # ONE shared relay thread for all feeds (never one per vehicle): the
        # NDN fetch holds the GIL, so concurrent relay threads starved the
        # asyncio HTTP loop. See _ensure_video_thread / _video_relay_loop.
        self._video_thread = None
        self.telemetry_age: dict[str, float] = {}
        # link health is measured on OUR clock only: cross-node wall-clock
        # differencing just reports clock skew on an RTC-less fleet (clocks
        # are set from GPS/FC/HTTPS-Date and are never aligned to better
        # than seconds-to-minutes). vid -> {last_ns, changed_mono}
        self.sample_state: dict[str, dict] = {}
        # first monotonic time we tried to poll each vehicle, so a fleet that
        # is still coming up doesn't flash "no link" before its first fix
        self.first_poll: dict[str, float] = {}

    # link is only declared lost after SUSTAINED silence, never on a single
    # dropped poll: telemetry publishes at 4 Hz and we poll at ~3 Hz, so one
    # missed fetch is normal jitter, not an offline vehicle. 2.5 s is ~8
    # consecutive misses — unambiguous silence — which keeps a healthy fleet
    # from blinking online/offline at the poll rate.
    STALE_AFTER_S = 2.5

    # ---- mission recorder ----------------------------------------------------

    def _record(self, payload: dict) -> None:
        if self.record_dir is None:
            return
        try:
            with self.record_lock:
                if self.record_file is None:
                    self.record_dir.mkdir(parents=True, exist_ok=True)
                    self.record_path = self.record_dir / time.strftime(
                        "dash-%Y%m%d-%H%M%S.jsonl"
                    )
                    self.record_file = open(self.record_path, "a")
                    print_json("dash.record.started", path=str(self.record_path))
                self.record_file.write(json.dumps(
                    {"ts": time.time(), "m": payload},
                    separators=(",", ":"),
                ) + "\n")
                # flush every line (survives a dashboard crash); fsync at
                # most every 2 s (survives a GCS power pull, cheaply)
                self.record_file.flush()
                now = time.monotonic()
                if now - self._record_synced > 2.0:
                    os.fsync(self.record_file.fileno())
                    self._record_synced = now
        except Exception as exc:
            print_json("dash.record.disabled", error=str(exc))
            self.record_dir = None

    def record_sync(self) -> None:
        with self.record_lock:
            if self.record_file is not None:
                try:
                    self.record_file.flush()
                    os.fsync(self.record_file.fileno())
                except Exception:
                    pass

    # ---- WS plumbing ------------------------------------------------------

    def _send_loop(self, payload) -> None:
        """Schedule a send from any thread.

        Binary payloads (live video frames) are coalesced per vehicle through
        a single drainer with per-send backpressure — high-rate frames on a
        slow WS drop the stale frame instead of flooding the loop with a task
        per frame (which starved the event loop and hung all HTTP). Dict
        payloads (telemetry/events) keep the simple per-message broadcast.
        """
        if self.loop is None:
            return
        if isinstance(payload, (bytes, bytearray)):
            self.loop.call_soon_threadsafe(self._enqueue_video, bytes(payload))
        else:
            self.loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(self.broadcast(payload))
            )

    def _enqueue_video(self, frame: bytes) -> None:
        """Latest-wins per-vehicle slot; (re)start the single drainer. Runs on
        the loop thread (via call_soon_threadsafe), so no lock is needed."""
        if not frame:
            return
        self.video_slots[frame[0]] = frame  # frame[0] = vehicle index header
        if self._video_drain_task is None or self._video_drain_task.done():
            self._video_drain_task = asyncio.ensure_future(self._video_drainer())

    async def _video_drainer(self) -> None:
        """Send the newest frame per vehicle to all clients, at most one send
        in flight per client; frames that arrive mid-send are dropped."""
        while self.video_slots:
            frames = list(self.video_slots.values())
            self.video_slots.clear()
            for frame in frames:
                clients = list(self.clients)
                if not clients:
                    continue
                results = await asyncio.gather(
                    *(self._safe_send_bytes(ws, frame) for ws in clients),
                    return_exceptions=True,
                )
                for ws, ok in zip(clients, results):
                    if ok is not True:
                        self.clients.discard(ws)
            await asyncio.sleep(0)  # yield so HTTP handlers never starve

    async def _safe_send_bytes(self, ws, frame: bytes) -> bool:
        try:
            await asyncio.wait_for(ws.send_bytes(frame), timeout=2.0)
            return True
        except Exception:
            return False

    async def broadcast(self, payload) -> None:
        if isinstance(payload, dict):
            self._record(payload)
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
        # one poller thread PER STREAM: the old single loop fetched both
        # vehicles' telemetry and the search status serially (800 ms
        # timeout each) then slept 1 s — a slow vehicle stalled everyone
        # and markers updated every 2-3 s. Independent threads at ~3 Hz
        # follow the agents' 4 Hz publications closely.
        for vid in self.vehicles:
            threading.Thread(
                target=self._poll_telemetry_forever, args=(vid,), daemon=True
            ).start()
        threading.Thread(target=self._poll_search_forever, daemon=True).start()
        threading.Thread(
            target=self._poll_capabilities_forever, daemon=True
        ).start()
        threading.Thread(
            target=self._poll_sensor_events_forever, daemon=True
        ).start()
        while True:
            time.sleep(3600)

    def _poll_sensor_events_forever(self) -> None:
        """Relay tasked-capture results the service response can't carry
        (opportunistic watchpoints fire long after their ack)."""
        seen: dict[str, tuple] = {}
        while True:
            for vid in self.vehicles:
                try:
                    payload = fetch_segmented(
                        vehicle_sensor_event_name(vid), timeout_ms=700
                    )
                    result = SensorCaptureResult.from_bytes(payload)
                    key = (result.request_id, result.gps_time_ns, result.status)
                    if seen.get(vid) == key:
                        continue
                    seen[vid] = key
                    self._on_sensor_result(vid, result)
                except Exception:
                    pass
            time.sleep(1.5)

    def _poll_capabilities_forever(self) -> None:
        """Track which investigation sensors each IUAS advertises.

        The agents publish a CapabilityProfile once at startup (long-lived
        producer); extras carry sensor strings ("camera", "audio"). An
        agent predating sensor advertisement gets the legacy assumption:
        camera only.
        """
        while True:
            for vid in self.vehicles:
                try:
                    payload = fetch_segmented(
                        vehicle_telemetry_state_name(vid), timeout_ms=800
                    )
                    profile = CapabilityProfile.from_bytes(payload)
                    sensors = {
                        s for s in ("camera", "audio")
                        if s in (profile.extras or [])
                    } or {"camera"}
                    meta = profile.sensor_meta or {}
                    changed = sensors != self.capabilities.get(vid)
                    meta_changed = meta != self.sensor_meta.get(vid)
                    if changed or meta_changed:
                        self.capabilities[vid] = sensors
                        self.sensor_meta[vid] = meta
                        # sensor_meta rides the same capabilities broadcast so
                        # the coverage layer updates in lockstep with the tag
                        self._send_loop({
                            "type": "capabilities",
                            "vehicle": vid,
                            "sensors": sorted(sensors),
                            "sensor_meta": meta,
                        })
                        if changed:
                            self._pump_dispatch()  # a new capability may unblock a job
                except Exception:
                    pass
            time.sleep(10.0)

    def _poll_telemetry_forever(self, vid: str) -> None:
        period = 0.3
        while True:
            t0 = time.monotonic()
            self._poll_vehicle(vid)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _poll_search_forever(self) -> None:
        vid = self.args.wuas_id
        while True:
            if self.mission["state"] == "searching":
                self._poll_search(vid)
            time.sleep(0.5)

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
            sample_dict = json.loads(payload.decode())
            self.last_sample[vid] = sample_dict
            self._send_loop({
                "type": "telemetry",
                "vehicle": vid,
                "sample": sample_dict,
                "age_s": round(age_s, 1),
                "skew_s": round(skew_s, 1),
            })
        except Exception:
            # A single dropped 0.3 s poll must NOT flip the vehicle offline.
            # Only surface staleness after SUSTAINED silence (STALE_AFTER_S);
            # until then the UI keeps easing the last-known marker, so a
            # healthy fleet holds a steady link instead of flapping at the
            # poll rate on one contended/timed-out fetch.
            now = time.monotonic()
            first = self.first_poll.setdefault(vid, now)
            last = self.telemetry_age.get(vid)
            # how long we've been dark: since the last good fix if we've ever
            # had one, else since we first started polling this vehicle
            silent_s = (now - last) if last is not None else (now - first)
            if silent_s < self.STALE_AFTER_S:
                return  # transient gap — leave the marker live
            self._send_loop({
                "type": "telemetry_stale",
                "vehicle": vid,
                "silent_s": None if last is None else round(now - last, 1),
            })

    def _poll_search(self, vid: str) -> None:
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
            # last_frames is newest-first; dispatch oldest-first so
            # detections leave (and usually return) in capture order
            for frame in reversed(status.last_frames):
                if frame not in self.seen_frames:
                    self.seen_frames.add(frame)
                    self._detect_frame(frame)
        except Exception:
            pass

    # ---- timed async service requests --------------------------------------

    def _timed_async(
        self, service: str, label: str, payload: bytes,
        on_response, on_timeout, *, timeout_ms: int, **ctx,
    ) -> None:
        """Submit a request_service_async, recording per-call latency.

        Stamps CLOCK_REALTIME before submit and emits a `metric.latency`
        (stage=service) from the framework callback: on_response records the
        round trip plus the provider four-point breakdown when the wrapper
        attaches response.timing; on_timeout records total + status=False.
        The caller's own callbacks run afterwards, unchanged.
        """
        sent = metrics.stamp()

        def wrapped_response(response) -> None:
            metrics.record_service_result(
                label, sent, response, service=service, **ctx
            )
            on_response(response)

        def wrapped_timeout(request_id) -> None:
            metrics.record_service_timeout(label, sent, service=service, **ctx)
            on_timeout(request_id)

        self.user.request_service_async(
            service,
            payload,
            on_response=wrapped_response,
            on_timeout=wrapped_timeout,
            timeout_ms=timeout_ms,
        )

    # ---- detection fan-out ---------------------------------------------------

    @staticmethod
    def _frame_seq(frame_name: str) -> int:
        """Capture sequence number from a frame name (.../frame/<ts>/<seq>)."""
        try:
            return int(frame_name.rsplit("/", 1)[-1])
        except (ValueError, IndexError):
            return -1

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
            # ship the current sim ground truth so the synthetic detector
            # finds operator-placed targets (empty => legacy detector path)
            anomalies=self._anomaly_snapshot(),
        )
        seq = self._frame_seq(frame_name)
        self.detects_pending += 1
        self.event("detect.sent", frame=frame_name, seq=seq)

        def on_response(response) -> None:
            self.detects_pending -= 1
            self.detects_done += 1
            if not response.status:
                self.event(
                    "detect.miss", frame=frame_name, seq=seq,
                    error=response.error,
                )
                return
            detection = DetectionResponse.from_bytes(response.payload)
            self.event(
                "detect.hit",
                frame=frame_name,
                seq=seq,
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
            self.event("detect.timeout", frame=frame_name, seq=seq)

        self._timed_async(
            gcs_detection_service(),
            "detect",
            request.to_bytes(),
            on_response,
            on_timeout,
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
            # promote candidate -> target, with one investigation JOB per
            # sensor the operator asked for; each job is dispatched to an
            # IUAS advertising that sensor (possibly different vehicles)
            self.candidates.remove(cand)
            sensors = self._mission_sensors()
            target = {
                "index": len(self.mission["targets"]),
                "object_id": cand["object_id"],
                "confidence": cand["confidence"],
                "lat": cand["lat"], "lon": cand["lon"],
                "frame": cand["frame"],
                "best_offset": cand["best_offset"],
                "status": "queued",
                "artifacts": [],
                "jobs": [
                    {"sensor": s, "vehicle": "", "status": "queued",
                     "artifacts": []}
                    for s in sensors
                ],
            }
            self.mission["targets"].append(target)
        self.event(
            "mission.target_found",
            index=target["index"], object_id=target["object_id"],
            confidence=round(target["confidence"], 4),
            lat=target["lat"], lon=target["lon"], frame=target["frame"],
            hits=need, sensors=sensors,
        )
        self._pump_dispatch()

    def _mission_sensors(self) -> list[str]:
        wanted = self.mission["params"].get("investigate_sensors") or ["camera"]
        sensors = [s for s in wanted if s in ("camera", "audio")]
        return sensors or ["camera"]

    # ---- end-of-raster unconfirmed disposition (operator inputs) -----------

    def _finish_search_disposition_locked(self) -> None:
        """Caller holds targets_lock. Convert leftover candidates (each seen
        on fewer than confirm_count frames — the geometric trap where a
        footprint narrower than the leg spacing can only ever see an object
        on one pass) into operator-facing `unconfirmed` entries. Surfaced,
        NOT auto-dispatched, NOT blocking completion. An aborted mission
        drops its candidates silently (matches v3 mission.rs finish_search)."""
        if self.mission["state"] not in ("searching", "investigating"):
            self.candidates.clear()
            return
        need = max(1, int(self.args.confirm_count))
        for cand in self.candidates:
            u = {
                "index": len(self.mission["unconfirmed"]),
                "object_id": cand["object_id"],
                "confidence": cand["confidence"],
                "lat": cand["lat"],
                "lon": cand["lon"],
                "frame": cand["frame"],
                "best_offset": cand.get("best_offset", 0.0),
                "hits": len(cand["frames"]),
                "need": need,
                "status": "unconfirmed",
            }
            self.mission["unconfirmed"].append(u)
            self.event(
                "target.unconfirmed",
                index=u["index"], hits=u["hits"], need=u["need"],
                object_id=u["object_id"],
                confidence=round(u["confidence"], 4),
                lat=u["lat"], lon=u["lon"], frame=u["frame"],
            )
        self.candidates.clear()

    def promote_unconfirmed(self, index: int) -> None:
        """Operator "Investigate anyway": promote an unconfirmed candidate
        through the NORMAL target/job path (one queued job per requested
        sensor). A completed mission reopens (done -> investigating) so the
        completion predicate re-runs. Idempotent — only an `unconfirmed`
        entry in a non-aborted mission promotes (mirrors v3
        mission.rs promote_unconfirmed)."""
        target = None
        u_index = -1
        u_hits = 0
        sensors: list[str] = []
        with self.targets_lock:
            if self.mission["state"] not in (
                "searching", "investigating", "done"
            ):
                return
            u = next(
                (x for x in self.mission["unconfirmed"]
                 if x["index"] == index and x["status"] == "unconfirmed"),
                None,
            )
            if u is None:
                return
            u["status"] = "promoted"
            u_index, u_hits = u["index"], u["hits"]
            if self.mission["state"] == "done":
                self.mission["state"] = "investigating"
            sensors = self._mission_sensors()
            target = {
                "index": len(self.mission["targets"]),
                "object_id": u["object_id"],
                "confidence": u["confidence"],
                "lat": u["lat"], "lon": u["lon"],
                "frame": u["frame"],
                "best_offset": u.get("best_offset", 0.0),
                "status": "queued",
                "artifacts": [],
                "jobs": [
                    {"sensor": s, "vehicle": "", "status": "queued",
                     "artifacts": []}
                    for s in sensors
                ],
            }
            self.mission["targets"].append(target)
        self.event(
            "target.promoted",
            index=u_index, target_index=target["index"],
            lat=target["lat"], lon=target["lon"],
        )
        # same wire shape as a confirm-count promotion, plus provenance
        self.event(
            "mission.target_found",
            index=target["index"], object_id=target["object_id"],
            confidence=round(target["confidence"], 4),
            lat=target["lat"], lon=target["lon"], frame=target["frame"],
            hits=u_hits, sensors=sensors, promoted_from=u_index,
        )
        self._pump_dispatch()

    def dismiss_unconfirmed(self, index: int) -> None:
        """Operator "Dismiss": terminal — the candidate can no longer be
        promoted and nothing else ever touches it."""
        payload = None
        with self.targets_lock:
            u = next(
                (x for x in self.mission["unconfirmed"]
                 if x["index"] == index and x["status"] == "unconfirmed"),
                None,
            )
            if u is None:
                return
            u["status"] = "dismissed"
            payload = {"index": u["index"], "lat": u["lat"], "lon": u["lon"]}
        self.event("target.dismissed", **payload)

    # ---- sensor data registry (map layer + playback modal) ------------------

    def add_sensor_data(self, item: dict) -> None:
        with self.sensor_data_lock:
            if any(d["name"] == item["name"] for d in self.sensor_data):
                return
            self.sensor_data.append(item)
            del self.sensor_data[:-500]
        self._send_loop({"type": "sensor_data", "item": item})

    def _on_sensor_result(self, vid: str, result: SensorCaptureResult) -> None:
        fields = dict(
            vehicle=vid,
            request=result.request_id,
            sensor=result.sensor,
            status=result.status,
        )
        if result.message:
            fields["message"] = result.message
        if result.status == "captured":
            fields["lat"] = result.lat_deg
            fields["lon"] = result.lon_deg
        self.event("sensor.result", **fields)
        if result.status != "captured":
            return
        for name in result.artifacts:
            self.add_sensor_data({
                "vehicle": vid,
                "sensor": result.sensor,
                "kind": (
                    "audio/wav" if result.sensor == "audio" else "image/jpeg"
                ),
                "name": name,
                "lat": result.lat_deg,
                "lon": result.lon_deg,
                "t": time.time(),
                "source": "tasked",
                "label": f"tasked {result.sensor}",
            })

    def request_sensor_capture(self, vid: str, params: dict) -> None:
        request_id = f"cap-{int(time.time() * 1000) % 100_000_000}"
        target = params.get("target")
        request = SensorCaptureRequest(
            request_id=request_id,
            sensor=str(params.get("sensor", "camera")),
            mode=str(params.get("mode", "now")),
            duration_s=float(params.get("duration_s", 6.0)),
            target=(
                None if not target else GeoPoint(
                    lat_deg=float(target["lat"]),
                    lon_deg=float(target["lon"]),
                    alt_m=0.0,
                )
            ),
            radius_m=float(params.get("radius_m", 6.0)),
            expires_s=float(params.get("expires_s", 600.0)),
            note=str(params.get("note", "")),
        )
        fields = dict(
            vehicle=vid, request=request_id,
            sensor=request.sensor, mode=request.mode,
        )
        if request.target is not None:
            fields["lat"] = request.target.lat_deg
            fields["lon"] = request.target.lon_deg
        self.event("sensor.request", **fields)

        def on_response(response) -> None:
            if not response.status:
                self.event(
                    "sensor.failed", vehicle=vid, request=request_id,
                    error=response.error,
                )
                return
            self._on_sensor_result(
                vid, SensorCaptureResult.from_bytes(response.payload)
            )

        def on_timeout(_request_id: str) -> None:
            self.event("sensor.timeout", vehicle=vid, request=request_id)

        timeout_ms = 300_000 if request.mode == "override" else 60_000
        self._timed_async(
            vehicle_sensor_service(vid),
            "sensor",
            request.to_bytes(),
            on_response,
            on_timeout,
            timeout_ms=timeout_ms,
            vehicle=vid,
        )

    def _pump_dispatch(self) -> None:
        """Assign queued jobs to idle, enabled, capability-matching IUAS.

        Each target carries one job per requested sensor; every idle IUAS
        that advertises a queued job's sensor gets one — so a camera
        drone and a microphone drone work the same target concurrently,
        or one dual-sensor drone flies the jobs back to back. Jobs whose
        sensor no enabled vehicle carries stay queued (and stop blocking
        completion once nothing else is in flight)."""
        to_dispatch = []
        with self.targets_lock:
            if self.mission["state"] not in ("searching", "investigating"):
                return  # operator aborted: stop draining the queue
            busy = {
                j["vehicle"]
                for t in self.mission["targets"]
                for j in t["jobs"]
                if j["status"] == "investigating"
            }
            for target in self.mission["targets"]:
                for job in target["jobs"]:
                    if job["status"] != "queued":
                        continue
                    vid = self._pick_vehicle_locked(job["sensor"], busy)
                    if vid is None:
                        continue
                    job["status"] = "investigating"
                    job["vehicle"] = vid
                    busy.add(vid)
                    target["status"] = "investigating"
                    to_dispatch.append((target, job, vid))
            if not to_dispatch:
                self._maybe_complete_locked()
        for target, job, vid in to_dispatch:
            self._dispatch_iuas(target, job, vid)

    def _pick_vehicle_locked(self, sensor: str, busy: set) -> str | None:
        """First idle, enabled IUAS advertising `sensor`; None if none."""
        for vid in self.iuas_ids:
            if vid in busy or not self.enabled.get(vid, True):
                continue
            caps = self.capabilities.get(vid, {"camera"})
            if sensor in caps:
                return vid
        return None

    def _maybe_complete_locked(self) -> None:
        """Caller holds targets_lock. Mission ends when the raster is done,
        nothing is in flight, and no queued job could ever be served by a
        currently enabled vehicle (disabled/absent capability must not
        hold the mission open forever)."""
        if not self.mission["search_done"]:
            return
        if self.mission["state"] not in ("searching", "investigating"):
            return
        jobs = [j for t in self.mission["targets"] for j in t["jobs"]]
        if any(j["status"] == "investigating" for j in jobs):
            self.mission["state"] = "investigating"
            return
        serviceable = [
            j for j in jobs
            if j["status"] == "queued"
            and self._pick_vehicle_locked(j["sensor"], set()) is not None
        ]
        if serviceable:
            self.mission["state"] = "investigating"
            return
        unserved = sum(1 for j in jobs if j["status"] == "queued")
        self._complete_locked(
            note=f"unserviceable-jobs:{unserved}" if unserved else ""
        )

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

    def _dispatch_iuas(self, target: dict, job: dict, vid: str) -> None:
        params = self.mission["params"]
        request = InvestigatePointRequest(
            mission_id=self.mission["mission_id"],
            source_detection_id=(
                f"{target['object_id']}-{target['index']}-{job['sensor']}"
            ),
            target=GeoPoint(
                lat_deg=target["lat"], lon_deg=target["lon"], alt_m=0.0
            ),
            approach_alt_m=float(params.get("orbit_agl_m", 8.0)),
            standoff_m=float(params.get("orbit_radius_m", 6.0)),
            circle_radius_m=float(params.get("orbit_radius_m", 6.0)),
            circle_count=float(params.get("orbit_count", 1.0)),
            sensor_plan=[job["sensor"]],
        )
        self.event(
            "target.dispatch",
            index=target["index"],
            sensor=job["sensor"],
            vehicle=vid,
            lat=request.target.lat_deg,
            lon=request.target.lon_deg,
            radius_m=request.circle_radius_m,
            agl_m=request.approach_alt_m,
        )

        def finish(
            status: str, artifacts: list[str], note: str = "",
            artifact_objs=(),
        ) -> None:
            with self.targets_lock:
                job["status"] = status
                job["artifacts"] = artifacts
                jobs = target["jobs"]
                target["artifacts"] = [
                    a for j in jobs for a in j["artifacts"]
                ]
                terminal = all(
                    j["status"] in ("done", "failed") for j in jobs
                )
                if terminal:
                    target["status"] = (
                        "done"
                        if all(j["status"] == "done" for j in jobs)
                        else "failed"
                    )
            self.event(
                "target.job_completed" if status == "done"
                else "target.job_failed",
                index=target["index"],
                sensor=job["sensor"],
                vehicle=vid,
                artifacts=artifacts,
                note=note,
                lat=target["lat"], lon=target["lon"],
            )
            if terminal:
                self.event(
                    "target.completed" if target["status"] == "done"
                    else "target.failed",
                    index=target["index"],
                    artifacts=target["artifacts"],
                    note=note,
                    lat=target["lat"], lon=target["lon"],
                )
            # mission evidence joins the sensor-data layer, pinned at the
            # capture pose the artifact itself carries
            for a in artifact_objs:
                pos = a.pose.position
                self.add_sensor_data({
                    "vehicle": vid,
                    "sensor": job["sensor"],
                    "kind": a.kind,
                    "name": a.data_name,
                    "lat": pos.lat_deg,
                    "lon": pos.lon_deg,
                    "t": time.time(),
                    "source": "mission",
                    "label": (
                        f"target #{target['index']} {target['object_id']} "
                        f"({(target['confidence'] * 100):.0f}%)"
                    ),
                })
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
                artifact_objs=result.artifacts,
            )

        def on_timeout(_request_id: str) -> None:
            finish("failed", [], note="timeout")

        self._timed_async(
            vehicle_flight_service(vid, "investigate"),
            "investigate",
            request.to_bytes(),
            on_response,
            on_timeout,
            timeout_ms=self.args.investigate_timeout_ms,
            vehicle=vid,
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
                unconfirmed=[],
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
                # leftover candidates (hits < confirm_count) become
                # operator-facing unconfirmed markers/cards
                self._finish_search_disposition_locked()
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
                # leftover candidates (hits < confirm_count) become
                # operator-facing unconfirmed markers/cards
                self._finish_search_disposition_locked()
            self._pump_dispatch()

        self._timed_async(
            vehicle_flight_service(self.args.wuas_id, "raster-search"),
            "raster-search",
            request.to_bytes(),
            on_response,
            on_timeout,
            timeout_ms=timeout_ms,
            vehicle=self.args.wuas_id,
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

        self._timed_async(
            vehicle_flight_service(vid, command),
            f"flight:{command}",
            payload,
            on_response,
            on_timeout,
            timeout_ms=20000 if command == "takeoff" else 15000,
            vehicle=vid,
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
                if request.enable:
                    # one shared, paced relay thread for the whole fleet — never
                    # a blocking-fetch thread per vehicle (that starved HTTP).
                    self._ensure_video_thread()
            else:
                self.event("video.control_failed", vehicle=vid, error=response.error)

        def on_timeout(_request_id: str) -> None:
            self.event("video.control_timeout", vehicle=vid)

        self._timed_async(
            vehicle_video_service(vid),
            "video",
            request.to_bytes(),
            on_response,
            on_timeout,
            timeout_ms=15000,
            vehicle=vid,
        )

    def _ensure_video_thread(self) -> None:
        """Start the single shared relay thread if it isn't already running.

        Per-vehicle relay threads each ran a blocking NDN fetch that HOLDS the
        GIL (the wrapper can't release it — ndn-cxx isn't thread-safe for
        concurrent use), so several at once starved the asyncio HTTP loop and
        hung the server. One thread round-robins the enabled feeds and paces
        itself to a bounded aggregate fetch rate, so the interpreter is never
        monopolised. Live video is latest-wins, so the only cost is a lower
        per-vehicle framerate.
        """
        t = self._video_thread
        if t is not None and t.is_alive():
            return
        self._video_thread = threading.Thread(
            target=self._video_relay_loop, daemon=True
        )
        self._video_thread.start()

    def _video_relay_loop(self) -> None:
        """Round-robin the enabled feeds from one paced thread. Poll each
        vehicle's latest-wins live name (version discovery -> newest frame),
        drop duplicate seqs, forward new JPEGs through the coalescing WS
        drainer. The per-fetch sleep is where the asyncio loop gets to run, so
        the single GIL-holding fetch never starves HTTP.
        """
        # aggregate fetch cap across ALL enabled feeds; per-vehicle fps ~= /N
        min_period = 1.0 / 8.0
        last_seq: dict[str, int] = {}
        stat_t0 = time.monotonic()
        stat: dict[str, list] = {}  # vid -> [frames, bytes]
        while True:
            enabled = [
                vid for vid in self.vehicles
                if self.video_relays.get(vid, {}).get("enabled")
            ]
            if not enabled:
                return  # set_video restarts us on the next enable
            for vid in enabled:
                if not self.video_relays.get(vid, {}).get("enabled"):
                    continue
                t0 = time.monotonic()
                try:
                    payload = fetch_segmented(
                        vehicle_video_live_name(vid), timeout_ms=700
                    )
                    seq = int.from_bytes(payload[:8], "big")
                    if seq != last_seq.get(vid) or seq == 0:
                        last_seq[vid] = seq
                        jpeg = payload[8:]
                        self._send_loop(bytes([self.vehicles.index(vid)]) + jpeg)
                        s = stat.setdefault(vid, [0, 0])
                        s[0] += 1
                        s[1] += len(jpeg)
                except Exception:
                    pass  # stream gap; the next success is the live frame
                time.sleep(max(0.0, min_period - (time.monotonic() - t0)))
            now = time.monotonic()
            if now - stat_t0 >= 2.0:
                for vid, (frames, nbytes) in stat.items():
                    self._send_loop({
                        "type": "video_stats", "vehicle": vid,
                        "fps": round(frames / (now - stat_t0), 1),
                        "kbps": round(nbytes * 8 / (now - stat_t0) / 1000),
                        "seq": last_seq.get(vid, 0),
                    })
                stat_t0, stat = now, {}

    def fetch_artifact(self, name: str) -> tuple[bytes, str] | None:
        """Artifact body + declared content type (image/jpeg, audio/wav...).

        Prefers a loaded mission bundle (sim-mode import) so replay serves the
        recorded media with the fabric disconnected; falls through to the live
        fabric when no bundle is loaded or it never carried this name.
        """
        bundle = self.bundle
        if bundle is not None:
            hit = bundle.artifact(name)
            if hit is not None:
                return hit
        try:
            payload = fetch_segmented(name, timeout_ms=15000)
            header = parse_frame(payload)
            kind = str(header.get("kind") or "image/jpeg")
            return frame_body(payload), kind
        except Exception as exc:
            self.event("artifact.fetch_failed", name=name, error=str(exc))
            return None

    # ---- mission data bundle (NDN-native collection + sim-mode import) ------

    def build_mission_bundle(self, session: str) -> tuple[str, bytes]:
        """Run the fetch-sweep and return (filename, .tar.gz bytes).

        Sweeps every fleet node's journal over NDN, fetches every artifact the
        mission referenced (persisting media + pose/time/hfov metadata),
        includes the dashboard's own recording, and packs a coherent archive.
        A powered-down node is marked ``missing`` in the manifest, not fatal.
        Runs in the executor (blocking NDN fetches).
        """
        from bundle import assemble_bundle, bundle_filename, tar_gz_bytes

        self.record_sync()  # complete the live recording before capturing it
        with self.sensor_data_lock:
            artifacts = list(self.sensor_data)

        def journal_fetcher(node: str):
            for sess in (session, "latest"):
                if not sess:
                    continue
                try:
                    return fetch_segmented(
                        vehicle_journal_name(node, sess), timeout_ms=8000
                    )
                except Exception:
                    continue
            return None

        def artifact_fetcher(name: str):
            try:
                return fetch_segmented(name, timeout_ms=15000)
            except Exception:
                return None

        staging = Path(tempfile.mkdtemp(prefix="muas-bundle-"))
        try:
            manifest = assemble_bundle(
                staging,
                session=session,
                fleet=self.vehicles,
                artifacts=artifacts,
                journal_fetcher=journal_fetcher,
                artifact_fetcher=artifact_fetcher,
                dashboard_jsonl_path=self.record_path,
                extra_journal_nodes=["gcs"],
            )
            data = tar_gz_bytes(staging)
        finally:
            shutil.rmtree(staging, ignore_errors=True)
        self.event(
            "mission.bundle.built",
            session=session,
            bytes=len(data),
            journals_ok=manifest["counts"].get("journals_ok", 0),
            artifacts_ok=manifest["counts"].get("artifacts_ok", 0),
        )
        return bundle_filename(session), data

    def load_bundle(self, archive_bytes: bytes):
        """Extract an uploaded archive into sim-mode: /artifact now resolves
        from it. Returns the BundleView. Replaces any previously loaded one."""
        from bundle import extract_bundle

        old_dir = self.bundle_dir
        dest = Path(tempfile.mkdtemp(prefix="muas-import-"))
        view = extract_bundle(archive_bytes, dest)
        self.bundle = view
        self.bundle_dir = dest
        if old_dir is not None:
            shutil.rmtree(old_dir, ignore_errors=True)
        self.event(
            "mission.bundle.imported",
            session=view.session,
            artifacts=len(view.index),
        )
        return view

    # ---- sim ground truth (operator-placed targets) -----------------------

    def _anomaly_snapshot(self) -> list[dict]:
        with self.anomalies_lock:
            return [dict(a) for a in self.anomalies]

    def _broadcast_anomalies(self) -> None:
        self._send_loop({"type": "sim_anomalies", "anomalies": self._anomaly_snapshot()})

    def place_anomaly(self, params: dict) -> None:
        """Drop a ground-truth anomaly into the running sim (v3 parity: the
        operator places targets the synthetic detector then finds). Broadcasts
        the updated truth so every client re-renders the map + list."""
        try:
            lat = float(params["lat"])
            lon = float(params["lon"])
        except (KeyError, TypeError, ValueError):
            return
        kind = "audio" if str(params.get("kind", "visual")) == "audio" else "visual"
        with self.anomalies_lock:
            self._anomaly_seq += 1
            anomaly = {
                "id": f"anom-{self._anomaly_seq}",
                "kind": kind,
                "lat_deg": lat,
                "lon_deg": lon,
                "signature": str(params.get("signature", "")),
                "created_ns": gps_time_ns(),
            }
            if kind == "audio":
                anomaly["loudness_db"] = float(params.get("loudness_db", 80.0))
            else:
                anomaly["size_m"] = float(params.get("size_m", 4.0))
            self.anomalies.append(anomaly)
        self.event(
            "sim.anomaly_placed", anomaly_id=anomaly["id"], anomaly_kind=kind,
            lat=lat, lon=lon, signature=anomaly["signature"],
        )
        self._broadcast_anomalies()

    def remove_anomaly(self, anomaly_id: str) -> None:
        with self.anomalies_lock:
            before = len(self.anomalies)
            self.anomalies = [a for a in self.anomalies if a.get("id") != anomaly_id]
            removed = len(self.anomalies) != before
        if removed:
            self.event("sim.anomaly_removed", anomaly_id=anomaly_id)
            self._broadcast_anomalies()

    def clear_anomalies(self) -> None:
        with self.anomalies_lock:
            count = len(self.anomalies)
            self.anomalies = []
        if count:
            self.event("sim.anomalies_cleared", count=count)
            self._broadcast_anomalies()

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
        elif kind == "sensor":
            vid = message.get("vehicle", "")
            if vid in self.vehicles:
                if not self.enabled.get(vid, True):
                    self.event(
                        "sensor.rejected", vehicle=vid,
                        reason="vehicle disabled",
                    )
                else:
                    self.request_sensor_capture(vid, message.get("params", {}))
        elif kind == "candidate_promote":
            self.promote_unconfirmed(int(message.get("index", -1)))
        elif kind == "candidate_dismiss":
            self.dismiss_unconfirmed(int(message.get("index", -1)))
        elif kind == "task_abort":
            # scoped abort from the commands log: halt the vehicle (a safe,
            # existing flight action) rather than RTL. v2 has no per-task
            # cancellation, so "hold" is the closest honest stop.
            vid = message.get("vehicle", "")
            if vid in self.vehicles:
                self._flight_command(vid, "hold")
        elif kind == "sim":
            op = message.get("op", "")
            params = message.get("params", {}) or {}
            if op == "place_anomaly":
                self.place_anomaly(params)
            elif op == "remove_anomaly":
                self.remove_anomaly(str(params.get("id", "")))
            elif op == "clear_anomalies":
                self.clear_anomalies()
        elif kind == "system":
            vid = message.get("vehicle", "")
            if vid in self.vehicles and message.get("command") == "shutdown":
                # double authorization: the UI already made the operator
                # type the vehicle id; the agent re-verifies it AND its
                # own armed/busy state before doing anything
                if message.get("confirm", "") != vid:
                    self.event(
                        "system.rejected", vehicle=vid,
                        reason="confirm phrase mismatch",
                    )
                elif self.last_sample.get(vid, {}).get("armed"):
                    self.event(
                        "system.rejected", vehicle=vid,
                        reason="vehicle is armed",
                    )
                else:
                    self._system_shutdown(vid)
        return None

    def _system_shutdown(self, vid: str) -> None:
        self.event("system.shutdown_sent", vehicle=vid)
        self.record_sync()  # the recording should hold this moment

        def on_response(response) -> None:
            if not response.status:
                self.event(
                    "system.shutdown_failed", vehicle=vid,
                    error=response.error,
                )
                return
            from contracts import FlightCommandResult

            result = FlightCommandResult.from_bytes(response.payload)
            self.event(
                "system.shutdown_result", vehicle=vid,
                status=result.status, message=result.message,
            )

        def on_timeout(_request_id: str) -> None:
            self.event("system.shutdown_timeout", vehicle=vid)

        self._timed_async(
            vehicle_system_service(vid, "shutdown"),
            "shutdown",
            json.dumps({"confirm": vid}).encode(),
            on_response,
            on_timeout,
            timeout_ms=15000,
            vehicle=vid,
        )


def make_app(dash: Dashboard, args):
    """Build the aiohttp application (routes + handlers).

    Split out of run_web so a headless test harness can drive the endpoints
    with an aiohttp TestClient (no TCP bind, no NDN stack).
    """
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
        result = await asyncio.get_event_loop().run_in_executor(
            dash.executor, dash.fetch_artifact, name
        )
        if result is None:
            return web.Response(status=404, text="artifact unavailable")
        body, kind = result
        return web.Response(body=body, content_type=kind)

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

    async def replays_index(_request):
        items = []
        if dash.record_dir is not None and dash.record_dir.exists():
            for p in sorted(dash.record_dir.glob("*.jsonl"), reverse=True):
                try:
                    st = p.stat()
                except OSError:
                    continue
                items.append({
                    "name": p.name,
                    "bytes": st.st_size,
                    "mtime": st.st_mtime,
                    "recording": p == dash.record_path,
                })
        return web.json_response({"replays": items})

    async def replay_file(request):
        name = request.match_info.get("name", "")
        if not re.fullmatch(r"[A-Za-z0-9._-]+\.jsonl", name):
            return web.Response(status=400, text="bad replay name")
        if dash.record_dir is None:
            return web.Response(status=404, text="recording disabled")
        path = dash.record_dir / name
        if not path.exists():
            return web.Response(status=404, text="no such replay")
        if path == dash.record_path:
            dash.record_sync()  # replaying the live recording: complete it
        return web.FileResponse(path)

    async def mission_bundle(request):
        """Download the ENTIRE mission over NDN as one .tar.gz (no SSH)."""
        session = (
            request.query.get("session")
            or dash.mission.get("mission_id")
            or "mission"
        )
        try:
            fname, data = await asyncio.get_event_loop().run_in_executor(
                dash.executor, dash.build_mission_bundle, session
            )
        except Exception as exc:
            return web.Response(status=500, text=f"bundle failed: {exc}")
        return web.Response(
            body=data,
            content_type="application/gzip",
            headers={
                "Content-Disposition": f'attachment; filename="{fname}"',
            },
        )

    async def mission_import(request):
        """Load an uploaded mission archive into sim-mode replay. Returns the
        dashboard recording (fed to the existing replay machinery) + summary;
        /artifact now resolves from the bundle."""
        try:
            reader = await request.multipart()
        except Exception:
            return web.Response(status=400, text="expected multipart upload")
        data = None
        while True:
            field = await reader.next()
            if field is None:
                break
            if field.name == "bundle" or field.filename:
                data = await field.read(decode=False)
                break
        if not data:
            return web.Response(status=400, text="no bundle file in upload")
        try:
            view = await asyncio.get_event_loop().run_in_executor(
                dash.executor, dash.load_bundle, data
            )
            jsonl = await asyncio.get_event_loop().run_in_executor(
                dash.executor, view.dashboard_jsonl_text
            )
        except Exception as exc:
            return web.Response(status=400, text=f"import failed: {exc}")
        return web.json_response({
            "session": view.session,
            "manifest": view.manifest,
            "artifacts": len(view.index),
            "jsonl": jsonl,
        })

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        dash.clients.add(ws)
        await ws.send_str(json.dumps({
            "type": "hello",
            "vehicles": dash.vehicles,
            "enabled": dash.enabled,
            "capabilities": {
                v: sorted(c) for v, c in dash.capabilities.items()
            },
            "sensor_meta": dict(dash.sensor_meta),
            "anomalies": dash._anomaly_snapshot(),
            "sensor_data": list(dash.sensor_data),
            "mission": {
                "state": dash.mission["state"],
                "mission_id": dash.mission["mission_id"],
                "targets": dash.mission["targets"],
                "unconfirmed": dash.mission.get("unconfirmed", []),
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
    app.router.add_get("/replays", replays_index)
    app.router.add_get("/replays/{name}", replay_file)
    app.router.add_get("/mission/bundle", mission_bundle)
    app.router.add_post("/mission/import", mission_import)
    app.router.add_get("/ws", ws_handler)
    return app


async def run_web(dash: Dashboard, args) -> None:
    from aiohttp import web

    app = make_app(dash, args)
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

    start_role_journal("gcs-dashboard", args.log_dir)
    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

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
    finally:
        flush_json_log()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
