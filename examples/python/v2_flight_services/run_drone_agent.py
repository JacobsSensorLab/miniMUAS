#!/usr/bin/env python3
"""miniMUAS v2 drone agent: telemetry, raster search, flight commands, video.

One NDNSF provider per drone, registering every vehicle service:

  wuas + iuas : flight/rtl, flight/land, flight/hold, video/control
  wuas only   : flight/raster-search (lawnmower + capture-per-point)
  iuas only   : flight/investigate (same execution path as the validated
                run_iuas_provider: investigate_plan over sim or MAVLink)

Continuous publications (segmented objects, short freshness, latest-wins):

  /muas/v2/<vid>/telemetry/live   1 Hz TelemetrySample (dashboard cards,
                                  link health = sample age)
  /muas/v2/<vid>/telemetry/state  CapabilityProfile (once, at startup)
  /muas/v2/<vid>/search/status    1 Hz SearchStatus while a raster runs
  /muas/v2/<vid>/video/<seq>      MJPEG frames while video is enabled
  /muas/v2/<vid>/video/status     VideoStatus on every stream change

Division of labor (deliberate): the agent FLIES and PUBLISHES; the GCS
dashboard — an NDNSF user — issues detect-object per published search
frame and owns the detect->dispatch state machine. The agent never plays
user and provider in one process. A detection hit reaches the agent as a
hold/rtl command, which the raster loop honors within one capture cycle.

Flight backends: with --mavlink-endpoint the raster runs goto/position
waits on the real autopilot (same LoggingFlightLink the HITL probe
validated); without it a built-in kinematic simulator moves the vehicle
at the commanded speed so the bench dashboard shows live motion.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path

from contracts import (
    CapabilityProfile,
    FlightCommandResult,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    RasterSearchRequest,
    RasterSearchResult,
    SearchStatus,
    TakeoffRequest,
    TelemetrySample,
    VideoControlRequest,
    VideoStatus,
    gps_time_ns,
    mission_frame_name,
    mission_sensor_name,
    vehicle_flight_service,
    vehicle_search_status_name,
    vehicle_telemetry_live_name,
    vehicle_telemetry_state_name,
    vehicle_video_live_name,
    vehicle_video_service,
    vehicle_video_status_name,
)
from camera import frame_source_from_spec
from dataplane import build_frame_bytes, publish_segmented
from raster import build_raster
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    optional_local_nfd,
    print_json,
    provider_kwargs,
)

EARTH_M_PER_DEG_LAT = 111_111.0


def _m_per_deg_lon(lat: float) -> float:
    return EARTH_M_PER_DEG_LAT * max(math.cos(math.radians(lat)), 1e-6)


def _dist_m(lat_a, lon_a, lat_b, lon_b) -> float:
    dn = (lat_a - lat_b) * EARTH_M_PER_DEG_LAT
    de = (lon_a - lon_b) * _m_per_deg_lon((lat_a + lat_b) / 2.0)
    return math.hypot(dn, de)


# ---------------------------------------------------------------------------
# Flight backends
# ---------------------------------------------------------------------------


class SimFlightBackend:
    """Kinematic bench vehicle: moves toward goto targets at cruise speed.

    Gives the dashboard genuine live motion without an autopilot. AGL is
    tracked directly; armed/mode mimic a guided multirotor's surface.
    """

    source = "sim"

    def __init__(self, lat: float, lon: float) -> None:
        self._lock = threading.Lock()
        self._lat, self._lon = lat, lon
        self._agl = 0.0
        self._target = None  # (lat, lon, agl)
        self._speed = 2.0
        self.armed = False
        self.mode = "STABILIZE"
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        dt = 0.2
        while not self._stop.wait(dt):
            with self._lock:
                if self._target is None:
                    continue
                t_lat, t_lon, t_agl = self._target
                # vertical
                dz = t_agl - self._agl
                max_dz = 1.5 * dt
                self._agl += max(-max_dz, min(max_dz, dz))
                # horizontal
                dist = _dist_m(self._lat, self._lon, t_lat, t_lon)
                step = self._speed * dt
                if dist <= step or dist < 0.05:
                    self._lat, self._lon = t_lat, t_lon
                else:
                    f = step / dist
                    self._lat += (t_lat - self._lat) * f
                    self._lon += (t_lon - self._lon) * f
                if self.mode == "RTL" and dist < 0.1 and abs(dz) < 0.1:
                    if self._agl <= 0.05:
                        self.armed = False
                        self.mode = "STABILIZE"
                        self._target = None

    def position(self):
        with self._lock:
            return (self._lat, self._lon, self._agl)

    def set_cruise_speed(self, speed: float) -> None:
        with self._lock:
            self._speed = max(0.2, float(speed))

    def ensure_airborne(self, agl: float) -> bool:
        with self._lock:
            self.armed = True
            self.mode = "GUIDED"
            self._home = (self._lat, self._lon)
            self._target = (self._lat, self._lon, agl)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if abs(self.position()[2] - agl) < 0.3:
                return True
            time.sleep(0.2)
        return False

    def takeoff(self, agl: float) -> bool:
        # standalone manual takeoff: same path as ensure_airborne for the
        # kinematic sim (arm, climb to agl, hold).
        return self.ensure_airborne(agl)

    def goto(self, lat: float, lon: float, agl: float) -> None:
        with self._lock:
            self.mode = "GUIDED"
            self._target = (lat, lon, agl)

    def at_target(self, lat, lon, agl, tol_m=1.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - agl) <= max(0.5, tol_m / 2)
        )

    def rtl(self) -> bool:
        with self._lock:
            home = getattr(self, "_home", (self._lat, self._lon))
            self.mode = "RTL"
            self._target = (home[0], home[1], 0.0)
        return True

    def land(self) -> bool:
        with self._lock:
            self.mode = "LAND"
            self._target = (self._lat, self._lon, 0.0)
        return True

    def hold(self) -> bool:
        with self._lock:
            self.mode = "GUIDED"
            self._target = (self._lat, self._lon, self._agl)
        return True

    def telemetry(self) -> dict:
        lat, lon, agl = self.position()
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,
            "agl_m": agl,
            "armed": self.armed,
            "mode": self.mode,
        }


class MavlinkFlightBackend:
    """Real-autopilot backend over the validated LoggingFlightLink.

    Altitude frame, stated once so it can't drift: this backend works
    ENTIRELY in AGL (metres above the takeoff point). It pins the inner
    MavlinkDroneLink to home_alt_m=0, which makes the link's reported
    `pos.alt` the raw ArduCopter relative-to-home altitude == AGL, and
    makes every `goto(.., alt)` send `alt` straight through as the
    RELATIVE_ALT wire value. No home-altitude is captured at startup and
    no ASL<->AGL arithmetic happens anywhere in the agent.

    This is a deliberate departure from connect_flight_link's
    auto-detect-home behaviour, which was the cause of the 2026-06-15
    field crash: it captured home_alt from the FIRST position fix, but
    on an agent that reconnects to an already-settled FC that fix can
    carry a nonzero relative_alt, baking a spurious offset into every
    subsequent AGL. With home_alt pinned to 0 there is nothing to
    capture and nothing to get wrong.
    """

    source = "mavlink"

    def __init__(self, endpoint: str, vehicle_id: str, uas_root) -> None:
        import mavlink_flight

        self._mod = mavlink_flight
        # home_alt_m=0.0 -> link reports & accepts AGL directly.
        self._link, self._vehicle, self._home_alt = (
            mavlink_flight.connect_flight_link(
                endpoint,
                vehicle_id=vehicle_id,
                uas_ipbrc_root=uas_root,
                home_alt_m=0.0,
            )
        )
        # _home_alt is 0.0 by construction; kept as a field only so the
        # rest of the class can read it without branching. Never re-derive.
        self._home_alt = 0.0

    def position(self):
        p = self._vehicle.position
        # pos.alt is already AGL (link pinned to home_alt_m=0).
        return (p.lat, p.lon, max(p.alt, 0.0))

    def set_cruise_speed(self, speed: float) -> None:
        self._link.set_cruise_speed_m_s(speed)

    def ensure_airborne(self, agl: float) -> bool:
        return self._mod.ensure_airborne(
            self._link,
            self._vehicle,
            target_agl_m=agl,
            home_alt_m=self._home_alt,
        )

    def takeoff(self, agl: float) -> bool:
        # standalone manual takeoff == the same arm+climb path the
        # raster/investigate use, exposed as its own command.
        return self.ensure_airborne(agl)

    def goto(self, lat: float, lon: float, agl: float) -> None:
        # agl passes straight through: link sends it as the RELATIVE_ALT
        # wire value (home_alt_m=0).
        self._link.goto(lat, lon, agl)

    def at_target(self, lat, lon, agl, tol_m=2.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - agl) <= max(1.0, tol_m)
        )

    def rtl(self) -> bool:
        return bool(self._link.rtl())

    def land(self) -> bool:
        return bool(self._link.land())

    def hold(self) -> bool:
        # GUIDED + retarget current position = position hold. Retarget at
        # the CURRENT AGL so a hold never commands a climb or descent.
        p = self.position()
        if not self._link.set_mode_guided():
            return False
        self._link.goto(p[0], p[1], p[2])
        return True

    def telemetry(self) -> dict:
        lat, lon, agl = self.position()
        inner = self._link._inner
        battery_pct = -1.0
        try:
            bp = inner.battery_pct()
            if bp is not None:
                battery_pct = float(bp)
        except Exception:
            pass
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,  # AGL frame throughout
            "agl_m": agl,
            "armed": bool(self._link.is_armed()),
            "mode": str(getattr(inner, "mode", "") or ""),
            "battery_pct": battery_pct,
        }


# ---------------------------------------------------------------------------
# Camera hub: one device, many consumers (search captures + video stream)
# ---------------------------------------------------------------------------


class CameraHub:
    """Continuously reads the camera; consumers take the latest frame.

    A V4L2 device supports one capture client, but the agent needs both
    full-res search captures and a downscaled video stream — so one reader
    thread owns the device and everyone else copies `latest`.
    """

    def __init__(self, spec: str) -> None:
        self._source = frame_source_from_spec(spec)
        self._cv2 = None
        self._lock = threading.Lock()
        self._latest = None  # BGR ndarray
        self._latest_ts = 0.0
        self._stop = threading.Event()
        if hasattr(self._source, "_capture"):  # OpenCV-backed: live hub
            import cv2

            self._cv2 = cv2
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()

    def describe(self):
        return self._source.describe()

    def _reader(self) -> None:
        cap = self._source._capture
        while not self._stop.is_set():
            ok, frame = cap.read()
            if ok and frame is not None:
                with self._lock:
                    self._latest = frame
                    self._latest_ts = time.monotonic()
            else:
                time.sleep(0.05)

    def latest_bgr(self):
        with self._lock:
            return (
                None if self._latest is None else self._latest.copy(),
                self._latest_ts,
            )

    def jpeg(self, *, width=None, height=None, quality=85):
        """Latest frame as JPEG bytes (optionally downscaled); None if dry."""
        frame, ts = self.latest_bgr()
        if frame is None or self._cv2 is None:
            return None, 0.0
        if width and height:
            frame = self._cv2.resize(frame, (int(width), int(height)))
        ok, buf = self._cv2.imencode(
            ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
        return (buf.tobytes() if ok else None), ts

    def capture_frame_payload(self, **kwargs) -> bytes:
        """Full frame-container payload via the underlying source.

        For OpenCV sources the hub's reader keeps the buffer fresh; for
        synthetic/file sources this just delegates.
        """
        if self._cv2 is not None:
            body, _ = self.jpeg(quality=85)
            if body is not None:
                frame, _ = self.latest_bgr()
                h, w = frame.shape[:2]
                return build_frame_bytes(
                    body, kind="image/jpeg", width=w, height=h, **kwargs
                )
        return self._source.capture(**kwargs)

    # frame-source surface for investigate_plan.execute_investigation
    capture = capture_frame_payload

    @property
    def spec(self) -> str:
        return getattr(self._source, "spec", "camera-hub")

    def close(self):
        self._stop.set()
        try:
            self._source.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------


class LatestPublisher:
    """Republishes one name with fresh content; previous producer stopped."""

    def __init__(self, name: str, freshness_ms: int = 1500) -> None:
        self.name = name
        self.freshness_ms = freshness_ms
        self._producer = None

    def publish(self, payload: bytes) -> None:
        old = self._producer
        self._producer = publish_segmented(
            self.name, payload, freshness_ms=self.freshness_ms
        )
        if old is not None:
            try:
                old.stop()
            except Exception:
                pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="miniMUAS v2 drone agent")
    add_common_arguments(parser)
    parser.add_argument("--role", choices=["wuas", "iuas"], required=True)
    parser.add_argument("--vehicle-id", default=None)
    parser.add_argument("--camera", default="synthetic")
    parser.add_argument("--mavlink-endpoint", default=None)
    parser.add_argument("--uas-ipbrc-root", default=None)
    parser.add_argument("--telemetry-hz", type=float, default=1.0)
    parser.add_argument(
        "--sim-lat", type=float, default=None,
        help="Sim start latitude (default: bench home)",
    )
    parser.add_argument(
        "--sim-lon", type=float, default=None,
        help="Sim start longitude (default: bench home, offset ~8 m east "
        "for the iuas so co-located sim markers don't overprint)",
    )
    parser.add_argument(
        "--search-frame-width", type=int, default=640,
        help="Search captures are published at model resolution: the GCS "
        "detector letterboxes to 640 anyway, so full-res frames only "
        "slow the radio fetch.",
    )
    parser.add_argument("--search-frame-height", type=int, default=400)
    parser.add_argument("--search-frame-quality", type=int, default=80)
    parser.add_argument(
        "--max-range-m", type=float, default=300.0,
        help="Field-safety guard: reject search areas / investigate "
        "targets whose reference point is farther than this from the "
        "vehicle's current position. A typo'd rectangle or a wild "
        "geo-estimate must be rejected at the ack, never flown.",
    )
    parser.add_argument(
        "--max-agl-m", type=float, default=20.0,
        help="Field-safety guard: reject requested altitudes above this.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    vehicle_id = args.vehicle_id or f"{args.role}-01"
    prefix = f"/muas/v2/{vehicle_id}"
    uas_root = (
        Path(args.uas_ipbrc_root).expanduser() if args.uas_ipbrc_root else None
    )

    if args.dry_run:
        print_json("agent.dry_run", role=args.role, vehicle=vehicle_id)
        return 0

    # ---- camera + flight backend -----------------------------------------
    try:
        camera = CameraHub(args.camera)
    except Exception as exc:
        print_json("agent.camera.unavailable", camera=args.camera, error=str(exc))
        return 2
    print_json("agent.camera.ready", **camera.describe())

    if args.mavlink_endpoint:
        try:
            flight = MavlinkFlightBackend(
                args.mavlink_endpoint, vehicle_id, uas_root
            )
        except Exception as exc:
            print_json(
                "agent.mavlink.connect_failed",
                endpoint=args.mavlink_endpoint,
                error=str(exc),
            )
            return 2
    else:
        sim_lat = args.sim_lat if args.sim_lat is not None else 35.1208
        sim_lon = args.sim_lon if args.sim_lon is not None else (
            -89.9347 + (0.00009 if args.role == "iuas" else 0.0)
        )
        flight = SimFlightBackend(sim_lat, sim_lon)
    print_json("agent.flight.ready", backend=flight.source)

    add_ndnsf_path(args.ndnsf_root)
    from ndnsf import AckDecision, ServiceProvider, ServiceResponse

    provider = ServiceProvider(**provider_kwargs(args, prefix, ""))

    # ---- shared state ------------------------------------------------------
    state_lock = threading.Lock()
    busy = {"task": ""}              # "", "raster-search", "investigate"
    abort = threading.Event()        # raised by hold/rtl/land during a task
    search_status = {"value": None}  # latest SearchStatus or None
    video_cfg = {
        "enabled": False, "width": 320, "height": 240,
        "fps": 5.0, "quality": 40, "seq": 0,
    }
    producers_keepalive: list[object] = []  # frames must outlive handlers

    telemetry_pub = LatestPublisher(vehicle_telemetry_live_name(vehicle_id))
    search_pub = LatestPublisher(vehicle_search_status_name(vehicle_id))
    video_status_pub = LatestPublisher(vehicle_video_status_name(vehicle_id))

    def set_busy(task: str) -> bool:
        with state_lock:
            if task and busy["task"]:
                return False
            busy["task"] = task
            return True

    # ---- telemetry loop ----------------------------------------------------
    def telemetry_loop() -> None:
        period = 1.0 / max(args.telemetry_hz, 0.2)
        while True:
            try:
                t = flight.telemetry()
                sample = TelemetrySample(
                    vehicle_id=vehicle_id,
                    gps_time_ns=gps_time_ns(),
                    source=flight.source,
                    busy=busy["task"],
                    **{k: v for k, v in t.items()},
                )
                telemetry_pub.publish(sample.to_bytes())
            except Exception as exc:
                print_json("agent.telemetry.error", error=str(exc))
            time.sleep(period)

    # ---- video loop --------------------------------------------------------
    def publish_video_status() -> None:
        status = VideoStatus(
            vehicle_id=vehicle_id,
            gps_time_ns=gps_time_ns(),
            enabled=video_cfg["enabled"],
            seq=video_cfg["seq"],
            width=video_cfg["width"],
            height=video_cfg["height"],
            fps=video_cfg["fps"],
            quality=video_cfg["quality"],
        )
        video_status_pub.publish(status.to_bytes())

    def video_loop() -> None:
        # Live video is latest-wins: every frame republishes the SAME
        # well-known name with a new version and short freshness. Keeping
        # exactly one previous producer alive covers fetches in flight;
        # anything older is stopped — no history tail, so a consumer can
        # never accumulate a playback backlog.
        prev = None
        curr = None
        while True:
            if not video_cfg["enabled"]:
                time.sleep(0.25)
                continue
            t0 = time.monotonic()
            jpeg, ts = camera.jpeg(
                width=video_cfg["width"],
                height=video_cfg["height"],
                quality=video_cfg["quality"],
            )
            if jpeg is not None:
                video_cfg["seq"] += 1
                payload = video_cfg["seq"].to_bytes(8, "big") + jpeg
                try:
                    producer = publish_segmented(
                        vehicle_video_live_name(vehicle_id),
                        payload,
                        freshness_ms=300,
                    )
                    if prev is not None:
                        try:
                            prev.stop()
                        except Exception:
                            pass
                    prev, curr = curr, producer
                except Exception as exc:
                    print_json("agent.video.publish_failed", error=str(exc))
                if video_cfg["seq"] % 50 == 1:
                    publish_video_status()
            delay = (1.0 / max(video_cfg["fps"], 0.5)) - (time.monotonic() - t0)
            if delay > 0:
                time.sleep(delay)

    # ---- service: video/control -------------------------------------------
    @provider.handler(vehicle_video_service(vehicle_id))
    def video_control(payload: bytes) -> bytes:
        request = VideoControlRequest.from_bytes(payload)
        video_cfg.update(
            enabled=request.enable,
            width=max(120, min(request.width, 1280)),
            height=max(90, min(request.height, 800)),
            fps=max(0.5, min(request.fps, 15.0)),
            quality=max(10, min(request.quality, 95)),
        )
        publish_video_status()
        print_json(
            "agent.video.control",
            enabled=video_cfg["enabled"],
            w=video_cfg["width"], h=video_cfg["height"],
            fps=video_cfg["fps"], q=video_cfg["quality"],
        )
        return VideoStatus(
            vehicle_id=vehicle_id,
            gps_time_ns=gps_time_ns(),
            enabled=video_cfg["enabled"],
            seq=video_cfg["seq"],
            width=video_cfg["width"],
            height=video_cfg["height"],
            fps=video_cfg["fps"],
            quality=video_cfg["quality"],
        ).to_bytes()

    # ---- services: rtl / land / hold ----------------------------------------
    def flight_command(command: str) -> bytes:
        abort.set()  # any running task loop terminates at its next check
        try:
            ok = {"rtl": flight.rtl, "land": flight.land, "hold": flight.hold}[
                command
            ]()
        except Exception as exc:
            ok, message = False, str(exc)
        else:
            message = ""
        print_json("agent.command", command=command, ok=bool(ok))
        return FlightCommandResult(
            vehicle_id=vehicle_id,
            command=command,
            status="accepted" if ok else "failed",
            message=message,
        ).to_bytes()

    @provider.handler(vehicle_flight_service(vehicle_id, "rtl"))
    def cmd_rtl(payload: bytes) -> bytes:
        return flight_command("rtl")

    @provider.handler(vehicle_flight_service(vehicle_id, "land"))
    def cmd_land(payload: bytes) -> bytes:
        return flight_command("land")

    @provider.handler(vehicle_flight_service(vehicle_id, "hold"))
    def cmd_hold(payload: bytes) -> bytes:
        return flight_command("hold")

    # ---- service: takeoff (standalone, guarded) ----------------------------
    @provider.ack_handler(vehicle_flight_service(vehicle_id, "takeoff"))
    def ack_takeoff(payload: bytes) -> AckDecision:
        request = TakeoffRequest.from_bytes(payload)
        if busy["task"]:
            return AckDecision(status=False, message=f"busy:{busy['task']}")
        if not (0.5 <= request.target_agl_m <= args.max_agl_m):
            return AckDecision(
                status=False,
                message=f"agl {request.target_agl_m} outside 0.5..{args.max_agl_m}",
            )
        return AckDecision(status=True, message=f"agl={request.target_agl_m}")

    @provider.handler(vehicle_flight_service(vehicle_id, "takeoff"))
    def cmd_takeoff(payload: bytes) -> bytes:
        request = TakeoffRequest.from_bytes(payload)
        if not (0.5 <= request.target_agl_m <= args.max_agl_m):
            return FlightCommandResult(
                vehicle_id=vehicle_id, command="takeoff", status="rejected",
                message=f"agl {request.target_agl_m} outside guard",
            ).to_bytes()
        # takeoff occupies the vehicle like a task: refuse if mid-mission,
        # and clear any stale abort so the climb isn't instantly cancelled
        if not set_busy("takeoff"):
            return FlightCommandResult(
                vehicle_id=vehicle_id, command="takeoff", status="rejected",
                message=f"busy:{busy['task']}",
            ).to_bytes()
        abort.clear()
        try:
            ok = flight.takeoff(request.target_agl_m)
        except Exception as exc:
            ok, message = False, str(exc)
        else:
            message = "" if ok else "takeoff did not reach target AGL"
        finally:
            set_busy("")
        print_json(
            "agent.command", command="takeoff",
            agl=request.target_agl_m, ok=bool(ok),
        )
        return FlightCommandResult(
            vehicle_id=vehicle_id,
            command="takeoff",
            status="accepted" if ok else "failed",
            message=message,
        ).to_bytes()

    services = [
        vehicle_video_service(vehicle_id),
        vehicle_flight_service(vehicle_id, "rtl"),
        vehicle_flight_service(vehicle_id, "land"),
        vehicle_flight_service(vehicle_id, "hold"),
        vehicle_flight_service(vehicle_id, "takeoff"),
    ]

    # ---- wuas: raster-search -------------------------------------------------
    if args.role == "wuas":
        search_service = vehicle_flight_service(vehicle_id, "raster-search")
        services.append(search_service)

        @provider.ack_handler(search_service)
        def ack_search(payload: bytes) -> AckDecision:
            request = RasterSearchRequest.from_bytes(payload)
            if busy["task"]:
                return AckDecision(status=False, message=f"busy:{busy['task']}")
            if not (0.5 <= request.agl_m <= args.max_agl_m):
                return AckDecision(
                    status=False,
                    message=f"agl {request.agl_m} outside 0.5..{args.max_agl_m}",
                )
            from raster import resolve_area

            center_lat, center_lon, _w, _h = resolve_area(request.area)
            here = flight.position()
            range_m = _dist_m(here[0], here[1], center_lat, center_lon)
            if range_m > args.max_range_m:
                return AckDecision(
                    status=False,
                    message=f"area {range_m:.0f}m away > {args.max_range_m:.0f}m guard",
                )
            plan = build_raster(
                request.area,
                leg_spacing_m=request.leg_spacing_m,
                capture_every_m=request.capture_every_m,
            )
            if not plan.captures:
                return AckDecision(status=False, message="empty raster")
            return AckDecision(
                status=True, message=f"legs={len(plan.legs)}"
            )

        @provider.handler(search_service)
        def raster_search(payload: bytes) -> bytes:
            request = RasterSearchRequest.from_bytes(payload)
            if not set_busy("raster-search"):
                return ServiceResponse(
                    status=False, error=f"busy:{busy['task']}"
                )
            abort.clear()
            plan = build_raster(
                request.area,
                leg_spacing_m=request.leg_spacing_m,
                capture_every_m=request.capture_every_m,
            )
            task_id = f"{vehicle_id}-search-{request.mission_id}"
            frames = 0
            recent: list[str] = []
            deadline = time.monotonic() + request.max_duration_s
            status_state = {"state": "transit", "leg": 0}

            def push_status(note: str = "") -> None:
                status = SearchStatus(
                    vehicle_id=vehicle_id,
                    mission_id=request.mission_id,
                    gps_time_ns=gps_time_ns(),
                    state=status_state["state"],
                    leg=status_state["leg"],
                    legs_total=len(plan.legs),
                    frames_captured=frames,
                    last_frames=list(recent[:6]),
                    last_note=note,
                )
                search_status["value"] = status
                try:
                    search_pub.publish(status.to_bytes())
                except Exception:
                    pass

            print_json(
                "agent.search.started",
                task=task_id, legs=len(plan.legs), captures=len(plan.captures),
                agl=request.agl_m, speed=request.speed_m_s,
            )
            push_status("starting")

            def _publish_capture(cp, frame_index, here) -> None:
                """Grab the latest frame and publish it, tagged with the
                vehicle's pose AT CAPTURE (lat/lon/agl/heading). The GCS
                geo-projects detections from this embedded pose, so the
                ~10 s detection round-trip never corrupts the estimate —
                the pose is frozen with the image, not read late."""
                ts = gps_time_ns()
                name = mission_frame_name(
                    request.mission_id, vehicle_id, "bottom", ts, frame_index
                )
                jpeg, _ = camera.jpeg(
                    width=args.search_frame_width,
                    height=args.search_frame_height,
                    quality=args.search_frame_quality,
                )
                if jpeg is None:
                    payload_bytes = camera.capture_frame_payload(
                        mission_id=request.mission_id,
                        vehicle_id=vehicle_id,
                        sensor_id="bottom",
                        gps_time_ns=ts,
                        metadata={"heading_deg": str(cp.heading_deg)},
                    )
                else:
                    payload_bytes = build_frame_bytes(
                        jpeg,
                        mission_id=request.mission_id,
                        vehicle_id=vehicle_id,
                        sensor_id="bottom",
                        gps_time_ns=ts,
                        kind="image/jpeg",
                        width=args.search_frame_width,
                        height=args.search_frame_height,
                        metadata={
                            "lat_deg": f"{here[0]:.7f}",
                            "lon_deg": f"{here[1]:.7f}",
                            "agl_m": f"{here[2]:.2f}",
                            "heading_deg": str(cp.heading_deg),
                        },
                    )
                try:
                    producer = publish_segmented(name, payload_bytes)
                    producers_keepalive.append(producer)
                    if len(producers_keepalive) > 60:
                        old = producers_keepalive.pop(0)
                        try:
                            old.stop()
                        except Exception:
                            pass
                    recent.insert(0, name)
                    del recent[12:]
                except Exception as exc:
                    print_json(
                        "agent.search.publish_failed",
                        frame=name, error=str(exc),
                    )
            try:
                flight.set_cruise_speed(request.speed_m_s)
                if not flight.ensure_airborne(request.agl_m):
                    push_status("airborne failed")
                    return RasterSearchResult(
                        task_id=task_id, status="failed",
                        notes="could not reach search altitude",
                    ).to_bytes()

                outcome = "completed"
                # Fly each leg as ONE continuous motion to its far
                # endpoint, capturing on the fly when passing near a
                # pending capture point. The previous design issued a
                # goto per capture point and waited for arrival at each
                # — that produced the field stutter (accelerate, brake,
                # pitch back, repeat) and the pitch oscillation ruined
                # the nadir frames. A multirotor holds a far better
                # constant-velocity attitude across a whole leg.
                cap_radius = max(request.capture_every_m * 0.5, 1.5)
                legs = plan.legs
                # group capture points by leg, preserving order
                caps_by_leg: dict[int, list] = {}
                for cp in plan.captures:
                    caps_by_leg.setdefault(cp.leg, []).append(cp)

                aborted = False
                for leg_index, (leg_start, leg_end) in enumerate(legs):
                    if abort.is_set():
                        outcome = "aborted"
                        break
                    if time.monotonic() > deadline:
                        outcome = "failed"
                        break
                    status_state.update(state="searching", leg=leg_index)
                    # command the far end of the leg ONCE; the vehicle
                    # cruises the whole leg without stopping
                    flight.goto(leg_end[0], leg_end[1], request.agl_m)
                    pending = list(caps_by_leg.get(leg_index, []))
                    leg_deadline = time.monotonic() + max(
                        60.0, _dist_m(*leg_start, *leg_end) / 0.3
                    )
                    while pending or not flight.at_target(
                        leg_end[0], leg_end[1], request.agl_m, tol_m=2.0
                    ):
                        if abort.is_set():
                            outcome = "aborted"
                            aborted = True
                            break
                        if time.monotonic() > deadline or time.monotonic() > leg_deadline:
                            break
                        here = flight.position()
                        # capture any pending point we're now near
                        fired = [
                            cp for cp in pending
                            if _dist_m(here[0], here[1], cp.lat_deg, cp.lon_deg)
                            <= cap_radius
                        ]
                        for cp in fired:
                            pending.remove(cp)
                            frames += 1
                            _publish_capture(cp, frames, here)
                            push_status()
                        if not pending and flight.at_target(
                            leg_end[0], leg_end[1], request.agl_m, tol_m=2.0
                        ):
                            break
                        time.sleep(0.1)
                    if aborted:
                        break
                    # any capture points not reached (overshoot / GPS
                    # scatter): take them at the leg end so coverage is
                    # never silently dropped
                    for cp in pending:
                        if abort.is_set():
                            break
                        frames += 1
                        _publish_capture(cp, frames, flight.position())
                        push_status()
                status_state["state"] = (
                    "done" if outcome == "completed" else outcome
                )
                push_status(outcome)
                print_json(
                    "agent.search.finished",
                    task=task_id, outcome=outcome, frames=frames,
                )
                return RasterSearchResult(
                    task_id=task_id,
                    status=outcome,
                    frames_captured=frames,
                    notes=f"legs={len(plan.legs)}",
                ).to_bytes()
            finally:
                set_busy("")

    # ---- iuas: investigate (mirrors the validated run_iuas_provider) --------
    if args.role == "iuas":
        investigate_service = vehicle_flight_service(vehicle_id, "investigate")
        services.append(investigate_service)

        import investigate_plan

        investigate_plan.add_flight_path(uas_root)

        def active_profile():
            if flight.source == "mavlink":
                import mavlink_flight

                return mavlink_flight.mavlink_capability_profile()
            return investigate_plan.default_capability_profile(native_orbit=True)

        @provider.ack_handler(investigate_service)
        def ack_investigate(payload: bytes) -> AckDecision:
            request = InvestigatePointRequest.from_bytes(payload)
            if busy["task"]:
                return AckDecision(status=False, message=f"busy:{busy['task']}")
            if request.circle_radius_m <= 0 or request.approach_alt_m <= 0:
                return AckDecision(status=False, message="invalid request geometry")
            if request.approach_alt_m > args.max_agl_m:
                return AckDecision(
                    status=False,
                    message=f"agl {request.approach_alt_m} > {args.max_agl_m} guard",
                )
            here = flight.position()
            range_m = _dist_m(
                here[0], here[1],
                request.target.lat_deg, request.target.lon_deg,
            )
            if range_m > args.max_range_m:
                return AckDecision(
                    status=False,
                    message=f"target {range_m:.0f}m away > {args.max_range_m:.0f}m guard",
                )
            compiled = investigate_plan.compile_investigation(
                request, vehicle_id=vehicle_id, profile=active_profile()
            )
            if compiled.rejected:
                return AckDecision(
                    status=False, message=compiled.reason or compiled.mode
                )
            return AckDecision(status=True, message=compiled.mode)

        def _sim_investigate(request: InvestigatePointRequest):
            """Fly the investigation on the agent's own sim vehicle.

            investigate_plan's internal executor simulates a *separate*
            vehicle, which completes in milliseconds and never moves the
            telemetry the dashboard watches. Driving SimFlightBackend
            instead makes the bench IUAS visibly take off, transit, and
            orbit at real speed on the map.
            """
            import types
            from contracts import FlightTaskResult, SensorArtifact

            started = gps_time_ns()
            tgt = request.target
            agl = request.approach_alt_m
            radius = max(request.circle_radius_m, 1.0)
            flight.set_cruise_speed(3.0)
            status = "completed"
            if not flight.ensure_airborne(agl):
                status = "failed"
            else:
                # orbit entry north of target, then waypoints around it
                steps_per_orbit = 12
                total = max(1, int(round(request.circle_count * steps_per_orbit)))
                for k in range(total + 1):
                    if abort.is_set():
                        status = "aborted"
                        break
                    ang = 2.0 * math.pi * k / steps_per_orbit
                    dn = radius * math.cos(ang)
                    de = radius * math.sin(ang)
                    lat = tgt.lat_deg + dn / EARTH_M_PER_DEG_LAT
                    lon = tgt.lon_deg + de / _m_per_deg_lon(tgt.lat_deg)
                    flight.goto(lat, lon, agl)
                    settle = time.monotonic() + 30
                    while not flight.at_target(lat, lon, agl, tol_m=0.8):
                        if abort.is_set() or time.monotonic() > settle:
                            break
                        time.sleep(0.2)
            # capture from the orbit (whatever the camera sees on the bench)
            here = flight.position()
            artifact_time = gps_time_ns()
            artifact = SensorArtifact(
                data_name=mission_sensor_name(
                    request.mission_id, vehicle_id, "front", "frame",
                    artifact_time, 1,
                ),
                kind="image/jpeg",
                gps_time_ns=artifact_time,
                pose=Pose(
                    position=GeoPoint(
                        lat_deg=here[0], lon_deg=here[1], alt_m=here[2]
                    ),
                    yaw_deg=0.0,
                ),
                metadata={"target_id": request.source_detection_id},
            )
            payload = camera.capture_frame_payload(
                mission_id=request.mission_id,
                vehicle_id=vehicle_id,
                sensor_id="front",
                gps_time_ns=artifact_time,
                metadata={"target_id": request.source_detection_id},
            )
            result = FlightTaskResult(
                task_id=(
                    f"{vehicle_id}-investigate-{request.source_detection_id}"
                ),
                status=status,
                started_at_gps_ns=started,
                completed_at_gps_ns=gps_time_ns(),
                artifacts=[artifact] if status != "failed" else [],
                notes="sim-circle-mode",
            )
            return types.SimpleNamespace(
                result=result,
                artifact_payloads=(
                    [payload] if status != "failed" else []
                ),
                mode="sim-circle-mode",
                command_log=[],
            )

        @provider.handler(investigate_service)
        def investigate(payload: bytes) -> bytes:
            request = InvestigatePointRequest.from_bytes(payload)
            if not set_busy("investigate"):
                return ServiceResponse(status=False, error=f"busy:{busy['task']}")
            abort.clear()
            try:
                if flight.source == "mavlink":
                    import mavlink_flight

                    backend: MavlinkFlightBackend = flight  # type: ignore
                    if not backend.ensure_airborne(request.approach_alt_m):
                        return FlightTaskResultFailed(request, vehicle_id)
                    # Everything is AGL (link pinned to home_alt_m=0):
                    # pass the request through unchanged, target on the
                    # ground (alt 0 AGL), approach at the requested AGL.
                    flown = InvestigatePointRequest(
                        mission_id=request.mission_id,
                        source_detection_id=request.source_detection_id,
                        target=GeoPoint(
                            lat_deg=request.target.lat_deg,
                            lon_deg=request.target.lon_deg,
                            alt_m=0.0,
                        ),
                        approach_alt_m=request.approach_alt_m,
                        standoff_m=request.standoff_m,
                        circle_radius_m=request.circle_radius_m,
                        circle_count=request.circle_count,
                        sensor_plan=list(request.sensor_plan),
                        constraints=request.constraints,
                    )
                    outcome = investigate_plan.execute_investigation(
                        flown,
                        vehicle_id=vehicle_id,
                        uas_ipbrc_root=uas_root,
                        link=backend._link,
                        vehicle=backend._vehicle,
                        profile=mavlink_flight.mavlink_capability_profile(),
                        realtime=True,
                        frame_source=camera,
                    )
                else:
                    outcome = _sim_investigate(request)
                for artifact, art_payload in zip(
                    outcome.result.artifacts, outcome.artifact_payloads
                ):
                    try:
                        producer = publish_segmented(artifact.data_name, art_payload)
                        producers_keepalive.append(producer)
                        print_json(
                            "agent.artifact.published",
                            artifact=artifact.data_name,
                            bytes=len(art_payload),
                        )
                    except Exception as exc:
                        print_json(
                            "agent.artifact.publish_failed",
                            artifact=artifact.data_name, error=str(exc),
                        )
                print_json(
                    "agent.investigation.completed",
                    task_id=outcome.result.task_id,
                    status=outcome.result.status,
                    execution=outcome.mode,
                )
                return outcome.result.to_bytes()
            finally:
                set_busy("")

        def FlightTaskResultFailed(request, vid):
            from contracts import FlightTaskResult

            now = gps_time_ns()
            return FlightTaskResult(
                task_id=f"{vid}-investigate-{request.source_detection_id}",
                status="failed",
                started_at_gps_ns=now,
                completed_at_gps_ns=gps_time_ns(),
                artifacts=[],
                notes="mavlink preflight failed",
            ).to_bytes()

    # ---- capability + run ----------------------------------------------------
    with optional_local_nfd(args.start_local_nfd):
        profile = CapabilityProfile(
            vehicle_id=vehicle_id,
            gps_time_ns=gps_time_ns(),
            position=True,
            velocity=True,
            yaw_control=True,
            mode_control=True,
            extras=["orbit"] if args.role == "iuas" else [],
        )
        try:
            producers_keepalive.append(
                publish_segmented(
                    vehicle_telemetry_state_name(vehicle_id), profile.to_bytes()
                )
            )
            print_json("agent.capability.published", vehicle=vehicle_id)
        except Exception as exc:
            print_json("agent.capability.publish_failed", error=str(exc))

        threading.Thread(target=telemetry_loop, daemon=True).start()
        threading.Thread(target=video_loop, daemon=True).start()

        # Register EVERY service before entering the native loop. run(service)
        # only registers one; the per-service registrar is the supported path
        # for multi-service providers in the python wrapper.
        for service in services:
            provider._register_service(service)
        print_json("agent.starting", role=args.role, services=services)
        provider._native.run()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
