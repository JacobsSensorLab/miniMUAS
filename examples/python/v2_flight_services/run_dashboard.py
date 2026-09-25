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
import uuid
from statistics import median
from collections import deque
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from commands import is_in_progress, with_command_id
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
    vehicle_telemetry_state_name,
    vehicle_video_live_name,
    vehicle_video_service,
)
from dataplane import (
    FRAME_CONTENT_TYPE,
    fetch_segmented,
    frame_body,
    parse_frame,
    set_runtime,
)
from raster import build_raster, estimate_duration_s
from timesync import ClockMonitor
from telemetry_stream import TelemetryFeed
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
    parser.add_argument(
        "--command-mode",
        choices=["targeted", "two-phase"],
        default="targeted",
        help="INITIAL command routing (toggled live from the dashboard "
        "afterwards): 'targeted' skips the two-phase discovery/ACK for "
        "known-provider commands (NDNSF fast path), 'two-phase' always discovers.",
    )
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument(
        "--poll-start-delay-s", type=float, default=8.0,
        help="Max seconds the NDN pollers wait for the HTTP server to bind "
        "before starting (they hold the GIL, so this guarantees :8080 comes up "
        "even when every vehicle is unreachable at startup).",
    )
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


def _provider_of(service: str):
    """The single provider identity that serves a name, or None for a shared
    multi-provider namespace. Per-vehicle /muas/v2/<vid>/... and the GCS
    /muas/v2/gcs/... are single-provider (a targeted request can go straight to
    /muas/v2/<vid|gcs>); mission/group are not."""
    parts = service.strip("/").split("/")
    if (
        len(parts) >= 4 and parts[0] == "muas" and parts[1] == "v2"
        and parts[2] not in ("mission", "group")
    ):
        return "/muas/v2/" + parts[2]
    return None


class Dashboard:
    def __init__(self, args, user) -> None:
        self.args = args
        self.user = user
        # command routing, toggled live from the console (no redeploy):
        # "targeted" skips the two-phase handshake for known-provider commands,
        # "two-phase" always discovers. Only the initial value comes from args.
        self._command_mode = getattr(args, "command_mode", "targeted")
        # set True once :8080 is bound; the pollers wait on it so a fleet of
        # unreachable vehicles can't starve the HTTP bind (see poll_forever)
        self._http_ready = False
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
        # Predictive-stream subscribers, one per vehicle whose video/status
        # advertises transport=="stream". These push frames into the SAME
        # _send_loop path as the segmented poller, so the WS side is identical;
        # only the fabric-facing half differs (subscribe vs poll).
        # vid -> {"consumer", "descriptor", "fps", "log_key", "log_t"}; health is
        # polled at 1 Hz by _watch_video_subs_forever.
        self.video_subs: dict[str, dict] = {}
        # Last resubscribe per vehicle. Kept outside the subscription record
        # because a resubscribe replaces that record, and the 3 s throttle has
        # to span the replacement.
        self.video_resub_at: dict[str, float] = {}
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
        # vid -> TelemetryFeed (one NDNSF telemetry stream subscription each)
        self.telemetry_feeds: dict[str, TelemetryFeed] = {}
        # Link health is measured on OUR monotonic clock only, so it never
        # depends on clock sync. Clock offset comes from chrony on each end
        # (timesync.py), not from differencing wall clocks across the link.
        # vid -> {last_ns, changed_mono, age}
        self.sample_state: dict[str, dict] = {}
        self.clock = ClockMonitor()
        # when the feeds started, so a fleet that is still coming up doesn't
        # flash "no link" before its first sample
        self.telemetry_since = time.monotonic()

    # link is only declared lost after SUSTAINED silence: 2.5 s is 10 missed
    # samples at 4 Hz, well past a single lost sample (fast retransmit after 3
    # later samples), so a healthy fleet never blinks online/offline.
    STALE_AFTER_S = 2.5

    # Resubscribe a stream at the live edge once it has fallen this far behind
    # (VideoStreamConsumer.lag_ms). Well above a healthy stream's frame-to-frame
    # jitter (p95 gap 0.12-0.22 s on the fleet), far below the ~20 s of lag
    # that built up before the producer's retention cut the stream off.
    VIDEO_LAG_RESUBSCRIBE_MS = 1500
    # At most one resubscribe per vehicle per this many seconds, so a stream
    # that cannot keep up degrades to periodic skips instead of thrashing.
    VIDEO_RESUBSCRIBE_MIN_S = 3.0

    # Re-issue an unanswered command this often (commands.py). A healthy
    # targeted round trip is 50-100 ms and a provider's first request after
    # start took 11.6 s (fleet 2026-09-25), so 2 s re-issues quickly without
    # piling duplicates onto a provider that is merely slow.
    COMMAND_RETRY_S = 2.0
    # Don't start an attempt that could not complete before the deadline.
    COMMAND_MIN_ATTEMPT_MS = 500

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
        # Let the HTTP server bind first. The pollers do blocking NDN fetches
        # that HOLD the GIL (ndn-cxx Faces aren't thread-safe, so the fetch
        # can't release it); when every vehicle is unreachable (e.g. agents not
        # yet up at field-test start) those fetches all time out and starve the
        # main thread before it can bind :8080 — the dashboard then never comes
        # up. A short head start lets the web server bind, after which it stays
        # reachable even while the pollers churn.
        for _ in range(int(self.args.poll_start_delay_s * 10)):
            if getattr(self, "_http_ready", False):
                break
            time.sleep(0.1)
        # One telemetry stream subscription per vehicle, each with its own
        # watch + drain thread, so a vehicle that is down never delays another.
        self.telemetry_since = time.monotonic()
        for vid in self.vehicles:
            self.telemetry_feeds[vid] = TelemetryFeed(
                self.user, vid, fetch=fetch_segmented,
                log=lambda event, **kw: print_json("dashboard." + event, **kw),
                on_sample=lambda payload, _sample, vid=vid: self._on_telemetry(
                    vid, payload
                ),
            ).start()
        threading.Thread(
            target=self._watch_telemetry_forever, name="telemetry-watch",
            daemon=True,
        ).start()
        threading.Thread(target=self._poll_search_forever, daemon=True).start()
        threading.Thread(
            target=self._poll_capabilities_forever, daemon=True
        ).start()
        threading.Thread(
            target=self._poll_sensor_events_forever, daemon=True
        ).start()
        threading.Thread(
            target=self._watch_video_subs_forever, name="video-watch", daemon=True
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

    def _poll_search_forever(self) -> None:
        vid = self.args.wuas_id
        while True:
            if self.mission["state"] == "searching":
                self._poll_search(vid)
            time.sleep(0.5)

    def _on_telemetry(self, vid: str, payload: bytes) -> None:
        """One delivered telemetry stream sample (the feed's drain thread)."""
        sample = TelemetrySample.from_bytes(payload)
        now = time.monotonic()
        state = self.sample_state.setdefault(
            vid, {"last_ns": None, "changed_mono": now, "age": deque(maxlen=31)}
        )
        if sample.gps_time_ns != state["last_ns"]:
            state["last_ns"] = sample.gps_time_ns
            state["changed_mono"] = now
        # freshness on the dashboard's own clock: seconds since the
        # last NEW sample was observed (independent of clock sync)
        age_s = now - state["changed_mono"]
        # Node-minus-GCS clock offset from chrony on both ends: each
        # measures itself against its reference (drones against the GCS),
        # so the difference is exact to chrony's ~0.1 ms. The old figure,
        # our clock minus the sample's stamp, was 300-700 ms of sample age
        # (publish + fetch + poll staleness) read as "clock error".
        ours = self.clock.reading()
        clock_ms = (
            sample.clock_offset_ms - ours.offset_ms
            if sample.clock_ref and ours.known else None
        )
        # With the offset known, our clock minus the stamp is the sample's
        # true age from publish to delivery. Median: a sample released after
        # a retransmission carries that stall.
        if clock_ms is not None:
            stamp_ms = (gps_time_ns() - sample.gps_time_ns) / 1e6
            state["age"].append(stamp_ms + clock_ms)
        sample_dict = json.loads(payload.decode())
        self.last_sample[vid] = sample_dict
        self._send_loop({
            "type": "telemetry",
            "vehicle": vid,
            "sample": sample_dict,
            "age_s": round(age_s, 1),
            "clock_ms": round(clock_ms, 3) if clock_ms is not None else None,
            "clock_ref": sample.clock_ref,
            "clock_rms_ms": sample.clock_rms_ms if sample.clock_rms_ms >= 0 else None,
            "sample_age_ms": round(median(state["age"])) if state["age"] else None,
        })

    def _watch_telemetry_forever(self) -> None:
        """Report a vehicle stale only after SUSTAINED silence (STALE_AFTER_S);
        until then the UI keeps easing the last-known marker."""
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            for vid, feed in list(self.telemetry_feeds.items()):
                silent = feed.silent_s()
                # dark since the last sample, or since the feeds started
                dark = silent if silent is not None else now - self.telemetry_since
                if dark < self.STALE_AFTER_S:
                    continue
                self._send_loop({
                    "type": "telemetry_stale",
                    "vehicle": vid,
                    "silent_s": None if silent is None else round(silent, 1),
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
        """Deliver one command at least once, recording its latency.

        NDNSF delivers a request at most once and never retransmits it
        (commands.py has the fleet measurement), so the same command -- same
        `command_id`, which vehicles execute once -- is re-issued every
        COMMAND_RETRY_S until the first response or the deadline. The caller's
        on_response runs once, for the first response; on_timeout runs once,
        only after every attempt has expired. Each outcome is logged as
        `command.ok` / `command.failed` with attempts and latency, and each
        re-issue as `command.retry`, plus the `metric.latency` (stage=service)
        record for the winning response.
        """
        sent = metrics.stamp()
        started = time.monotonic()
        deadline = started + timeout_ms / 1000.0
        command_id = uuid.uuid4().hex
        payload = with_command_id(payload, command_id)

        # A named service under a single KNOWN provider (a per-vehicle
        # /muas/v2/<vid>/... command, or the GCS detector /muas/v2/gcs/...) can
        # skip the two-phase discovery/ACK via a targeted request. mission/group
        # are genuinely multi-provider, so they stay two-phase.
        provider = _provider_of(service)
        use_targeted = provider is not None and self._command_mode == "targeted"
        mode = "targeted" if use_targeted else "two-phase"
        state = {"done": False, "attempts": 0}
        lock = threading.Lock()

        def wrapped_response(response) -> None:
            if is_in_progress(response):
                # The vehicle has it and is still executing; keep re-issuing
                # until a re-issue returns the stored result (commands.py).
                return
            with lock:
                if state["done"]:
                    return
                state["done"] = True
                attempts = state["attempts"]
            self.event("command.ok", label=label, service=service,
                       command_id=command_id, attempts=attempts,
                       latency_ms=round((time.monotonic() - started) * 1000),
                       status=bool(getattr(response, "status", True)), **ctx)
            metrics.record_service_result(
                label, sent, response, service=service, mode=mode, **ctx
            )
            on_response(response)

        def wrapped_timeout(request_id) -> None:
            # Each attempt expires at the shared deadline; only the first
            # expiry at or after it ends the command.
            with lock:
                if state["done"] or time.monotonic() < deadline - 0.05:
                    return
                state["done"] = True
                attempts = state["attempts"]
            self.event("command.failed", label=label, service=service,
                       command_id=command_id, attempts=attempts,
                       latency_ms=round((time.monotonic() - started) * 1000), **ctx)
            metrics.record_service_timeout(
                label, sent, service=service, mode=mode, **ctx
            )
            on_timeout(request_id)

        def attempt() -> None:
            with lock:
                remaining_ms = int((deadline - time.monotonic()) * 1000)
                if state["done"] or remaining_ms < self.COMMAND_MIN_ATTEMPT_MS:
                    return
                state["attempts"] += 1
                n = state["attempts"]
            if n > 1:
                self.event("command.retry", label=label, service=service,
                           command_id=command_id, attempt=n, **ctx)
            if use_targeted:
                self.user.request_service_targeted_async(
                    provider, service, payload,
                    on_response=wrapped_response, on_timeout=wrapped_timeout,
                    timeout_ms=remaining_ms,
                )
            else:
                self.user.request_service_async(
                    service, payload,
                    on_response=wrapped_response, on_timeout=wrapped_timeout,
                    timeout_ms=remaining_ms,
                )
            retry = threading.Timer(self.COMMAND_RETRY_S, attempt)
            retry.daemon = True
            retry.start()

        attempt()

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
        # Default to the predictive stream. Measured on the fleet: stream runs
        # 9.9 fps with 0.10 s median frame spacing and no stutter, while the
        # segmented poll path manages ~1.2 fps with multi-second stalls (it
        # re-fetches a latest-wins name per frame and races the producer
        # churn). The operator can still pick "poll" in the UI.
        transport = params.get("transport", "stream")
        if transport not in ("segmented", "stream"):
            transport = "segmented"
        request = VideoControlRequest(
            enable=bool(params.get("enable", False)),
            width=int(params.get("width", 320)),
            height=int(params.get("height", 240)),
            fps=float(params.get("fps", 5.0)),
            quality=int(params.get("quality", 40)),
            transport=transport,
        )
        relay = self.video_relays.setdefault(vid, {"enabled": False, "seq": 0})
        relay["enabled"] = request.enable
        # Any transport change or a disable tears down an existing stream
        # subscription; the segmented poller stops itself when relay disabled.
        if not request.enable or transport != "stream":
            self._stop_video_sub(vid)
        self.event("video.control", vehicle=vid, enable=request.enable,
                   transport=transport)

        def on_response(response) -> None:
            if response.status:
                status = VideoStatus.from_bytes(response.payload)
                relay["seq"] = status.seq
                if not request.enable:
                    return
                # Tell the UI what the vehicle ACTUALLY applied. It clamps
                # (1280x800, 30 fps, q95) and the camera derives height from
                # width to preserve the sensor aspect ratio, so the applied
                # settings can differ from what was requested — and until now
                # nothing reported them, which is why the feed's resolution was
                # not visible anywhere.
                self._send_loop({
                    "type": "video_settings", "vehicle": vid,
                    "width": status.width, "height": status.height,
                    "fps": status.fps, "quality": status.quality,
                    "transport": status.transport,
                })
                if status.transport == "stream" and status.descriptor:
                    # Subscribe once to the vehicle's predictive stream; frames
                    # arrive on framework threads and feed the same WS drainer.
                    self._start_video_sub(vid, status.descriptor,
                                          fps=status.fps)
                else:
                    # segmented: one shared, paced relay thread for the whole
                    # fleet — never a blocking-fetch thread per vehicle (that
                    # starved HTTP).
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

    def _start_video_sub(self, vid: str, descriptor_json: str,
                         *, fps: float | None = None) -> None:
        """Subscribe to a vehicle's predictive video stream (idempotent).

        Replaces any prior subscription (a re-enable mints a new descriptor).
        The on_item callback runs on framework threads and forwards each frame
        through the same coalescing WS drainer the segmented poller uses, so
        the browser side is transport-agnostic.
        """
        if vid not in self.vehicles:
            return
        self._stop_video_sub(vid)
        try:
            from video_stream import VideoStreamConsumer

            idx = self.vehicles.index(vid)
            # Per-vehicle stream is a single subscriber thread, so these
            # counters need no lock. Emit the same video_stats the poll loop
            # does (~2 s window) so the UI fps/kbps indicator works on stream.
            stat = {"frames": 0, "bytes": 0, "t0": None}
            holder: dict = {"consumer": None}

            def on_frame(cursor: int, jpeg: bytes) -> None:
                self._send_loop(bytes([idx]) + jpeg)
                now = time.monotonic()
                if stat["t0"] is None:
                    stat["t0"] = now
                stat["frames"] += 1
                stat["bytes"] += len(jpeg)
                dt = now - stat["t0"]
                if dt >= 2.0:
                    consumer = holder["consumer"]
                    self._send_loop({
                        "type": "video_stats", "vehicle": vid,
                        "fps": round(stat["frames"] / dt, 1),
                        "kbps": round(stat["bytes"] * 8 / dt / 1000),
                        "seq": cursor,
                        "lag_ms": consumer.lag_ms if consumer is not None else None,
                    })
                    stat["frames"], stat["bytes"], stat["t0"] = 0, 0, now

            # Pass the vehicle's frame rate so the prefetch depth and Interest
            # lifetime are sized from the item rate. A fixed depth over-reaches
            # at low rates and every prefetch Interest expires before its frame
            # exists.
            consumer = VideoStreamConsumer(self.user, descriptor_json, on_frame, fps=fps)
            holder["consumer"] = consumer
            self.video_subs[vid] = {
                "consumer": consumer, "descriptor": descriptor_json, "fps": fps,
                "log_key": None, "log_t": 0.0,
            }
            self.event("video.stream_subscribed", vehicle=vid)
        except Exception as exc:
            self.event("video.stream_subscribe_failed", vehicle=vid,
                       error=str(exc))

    def _stop_video_sub(self, vid: str) -> None:
        sub = self.video_subs.pop(vid, None)
        if sub is not None:
            try:
                sub["consumer"].stop()
            except Exception:
                pass

    def _watch_video_subs_forever(self) -> None:
        """Check every stream subscription once a second.

        Polled rather than NDNSF's status callback: that fires on EVERY drained
        item and takes the GIL each time, on the one IO thread all three
        streams share. Measured: ~230 items/s total across three streams
        (76 each) against 165-207 items/s for one stream alone.
        """
        while True:
            time.sleep(1.0)
            for vid, sub in list(self.video_subs.items()):
                try:
                    self._check_video_sub(vid, sub)
                except Exception:
                    pass

    def _check_video_sub(self, vid: str, sub: dict) -> None:
        consumer = sub["consumer"]
        status = consumer.status()
        g = lambda k: getattr(status, k, None)
        state_s, reason_s = str(g("state")), str(g("reason"))
        now = time.monotonic()

        # Resubscribe decision FIRST, above the log-dedup return.
        #
        # A single unrecoverable frame also ends the subscription for good
        # ("terminal-gap:timeout") — the producer pausing longer than the retry
        # budget, a camera hiccup, a busy moment on the drone — which is what
        # "video rarely works" looked like in the field. Resubscribing with
        # start="latest" picks up at the live edge, so the operator sees a
        # sub-second hiccup instead of a dead feed.
        #
        # A FAILED lifecycle state is terminal by construction — NDNSF's
        # PredictiveStreamSubscriber raises on any attempt to restart after
        # stop/failure — so the only recovery is a fresh subscription. Keying
        # this off the STATE rather than the reason text matters: the most
        # common opening failure is "predictive frontier unavailable after
        # timeout", which contains no "terminal" and so was never retried
        # (measured as first_frame=124s against NFD's 1-2s).
        #
        # Falling behind live is resubscribed the same way: NDNSF cannot see it
        # (VideoStreamConsumer.lag_ms), and left alone the lag grew ~0.6 s per
        # second until the producer evicted the next chunk, then a 12-17 s
        # freeze, on a ~53 s cycle.
        if "FAILED" in state_s.upper() or "terminal" in reason_s.lower():
            why = reason_s or state_s
        elif consumer.lag_ms > self.VIDEO_LAG_RESUBSCRIBE_MS:
            why = f"lag:{consumer.lag_ms}ms"
        else:
            why = None
        if (
            why is not None
            and self.video_relays.get(vid, {}).get("enabled")
            and self.video_subs.get(vid) is sub
            and now - self.video_resub_at.get(vid, 0.0) > self.VIDEO_RESUBSCRIBE_MIN_S
        ):
            self.video_resub_at[vid] = now
            self.event("video.stream_resubscribe", vehicle=vid, reason=why)
            threading.Thread(
                target=self._start_video_sub,
                args=(vid, sub["descriptor"]),
                kwargs={"fps": sub["fps"]},
                name=f"video-resub-{vid}",
                daemon=True,
            ).start()
            return

        key = (state_s, reason_s)
        if key == sub["log_key"] and now - sub["log_t"] < 10.0:
            return
        sub["log_key"], sub["log_t"] = key, now
        self.event(
            "video.stream_status", vehicle=vid,
            state=state_s, reason=reason_s,
            delivered=g("delivered"), rejected=g("rejected"),
            timeouts=g("timeouts"), nacks=g("nacks"),
            next_cursor=g("next_deliver_cursor"),
            oldest_ready=g("oldest_ready_cursor"),
            ready_q=g("ready_queue_depth"),
            in_flight=g("in_flight"),
            pending=g("pending_interests"),
            retry_exh=g("retry_exhaustions"),
            recov_exh=g("recovery_exhaustions"),
            missing=g("terminal_missing_sources"),
            stale_drops=g("stale_ready_drops"),
            lag_ms=consumer.lag_ms,
            dropped=consumer.dropped,
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
        elif kind == "command_mode":
            # live toggle (no redeploy): route known-provider commands via the
            # targeted fast path or the two-phase handshake. metric.latency is
            # tagged with the mode, so flipping this mid-run yields the A/B.
            mode = message.get("mode", "")
            if mode in ("targeted", "two-phase"):
                self._command_mode = mode
                self.event("command_mode", mode=mode)
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
            # capability gates for the converged (v2+v3) dashboard.html:
            # each flag lights up a v2-only surface; a backend that omits
            # them serves the same file with those surfaces hidden.
            "bundle": True,          # GET /mission/bundle + POST /mission/import
            "video_transport": True, # per-vehicle segmented/stream selector
            "command_mode": True,    # targeted vs two-phase routing toggle
            # the dashboard IS the v2 sim operator (anomalies are always
            # placeable — there is no separate virtual-deployment mode), so
            # the sim panel is unconditionally on, as v2's old panel was
            "sim": True,
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
    dash._http_ready = True  # :8080 is bound — release the NDN pollers
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
    # Route dataplane fetches (telemetry/video/status pollers) through this
    # user's Face — one Face/process, so the pollers can't crash the runtime.
    set_runtime(user)
    user.start()  # background event loop for request_service_async
    if hasattr(user, "set_use_tokens"):
        # tokens off so a targeted request always takes the direct fast path (no
        # per-provider bootstrap); two-phase works token-free too, so the console
        # can flip modes live for a clean targeted-vs-two-phase comparison
        user.set_use_tokens(False)
        print_json("dash.use_tokens.disabled", reason="targeted-experiment")

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
