#!/usr/bin/env python3
"""miniMUAS v2 drone agent: telemetry, raster search, flight commands, video.

One NDNSF provider per drone, registering every vehicle service:

  wuas + iuas : flight/rtl, flight/land, flight/hold, video/control
  wuas only   : flight/raster-search (lawnmower + capture-per-point)
  iuas only   : flight/investigate (climb + continuous carrot orbit,
                streamed guided targets on sim or MAVLink)

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
import json
import math
import os
import subprocess
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
    SensorCaptureRequest,
    SensorCaptureResult,
    TakeoffRequest,
    TelemetrySample,
    VideoControlRequest,
    VideoStatus,
    gps_time_ns,
    mission_frame_name,
    mission_sensor_name,
    tasked_sensor_name,
    vehicle_flight_service,
    vehicle_search_status_name,
    vehicle_sensor_event_name,
    vehicle_sensor_service,
    vehicle_system_service,
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
    enable_json_log,
    flush_json_log,
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


def _leg_axis(leg_start, leg_end):
    """Unit vector (north, east) and length in metres of one raster leg."""
    lat0 = (leg_start[0] + leg_end[0]) / 2.0
    dn = (leg_end[0] - leg_start[0]) * EARTH_M_PER_DEG_LAT
    de = (leg_end[1] - leg_start[1]) * _m_per_deg_lon(lat0)
    length = math.hypot(dn, de)
    if length < 1e-6:
        return (1.0, 0.0), 0.0
    return (dn / length, de / length), length


def _along_leg_m(leg_start, axis, lat, lon) -> float:
    """Projection of (lat, lon) onto the leg axis, metres from leg_start."""
    dn = (lat - leg_start[0]) * EARTH_M_PER_DEG_LAT
    de = (lon - leg_start[1]) * _m_per_deg_lon(leg_start[0])
    return dn * axis[0] + de * axis[1]


def fly_raster(
    flight,
    plan,
    *,
    agl_m: float,
    speed_m_s: float,
    abort: threading.Event,
    deadline_mono: float,
    on_capture,
    on_leg,
    service_interrupt=None,
) -> str:
    """Fly the serpentine raster as one continuous motion.

    Field debrief 2026-07: the previous loop commanded only each leg's far
    end (so the vehicle cut diagonals that never overflew the capture
    points), fired captures on a proximity radius those diagonals never
    entered, and then refused to leave the leg end while captures were
    still "pending" — with a leg deadline that assumed 0.3 m/s. Net
    effect: fly to one waypoint, hover for minutes. This version

      * transits to each leg's START, so the leg itself is flown on-line;
      * fires captures by ALONG-TRACK progress — a point is captured the
        moment the vehicle passes abeam of it, regardless of cross-track
        GPS scatter, with the actual pose stamped into the frame;
      * never waits at a leg end: on arrival any not-yet-fired points are
        captured immediately and the next leg is commanded;
      * re-sends the position target every 2 s so one lost
        SET_POSITION_TARGET cannot strand the vehicle in a hover;
      * sizes deadlines from the commanded speed, not a 0.3 m/s floor.

    `service_interrupt`, when given, is polled each control tick: it
    services any pending operator override (fly to a point, capture,
    fly back to where the raster was interrupted) and returns True if it
    did — the loop then re-commands its current target and the sweep
    resumes exactly where it left off.

    Returns "completed", "aborted", or "timeout".
    """
    speed = max(float(speed_m_s), 0.3)
    caps_by_leg: dict[int, list] = {}
    for cp in plan.captures:
        caps_by_leg.setdefault(cp.leg, []).append(cp)

    def cruise(t_lat, t_lon, fire_from=None) -> str:
        here = flight.position()
        travel_deadline = (
            time.monotonic()
            + _dist_m(here[0], here[1], t_lat, t_lon) / (0.5 * speed)
            + 45.0
        )
        next_send = 0.0
        while True:
            if abort.is_set():
                return "aborted"
            now = time.monotonic()
            if now > deadline_mono:
                return "timeout"
            if service_interrupt is not None and service_interrupt():
                next_send = 0.0  # we were flown elsewhere: re-command
                continue
            if now >= next_send:
                flight.goto(t_lat, t_lon, agl_m)
                next_send = now + 2.0
            here = flight.position()
            if fire_from is not None:
                leg_start, axis, pending = fire_from
                along = _along_leg_m(leg_start, axis, here[0], here[1])
                while pending and pending[0][0] <= along:
                    _, cp = pending.pop(0)
                    on_capture(cp, here)
            if flight.at_target(t_lat, t_lon, agl_m, tol_m=2.5):
                return "arrived"
            if now > travel_deadline:
                # blocked short of the target (wind, EKF disagreement):
                # move on rather than hover — the caller captures any
                # stragglers so coverage is not silently dropped
                return "arrived"
            time.sleep(0.1)

    for leg_index, (leg_start, leg_end) in enumerate(plan.legs):
        on_leg(leg_index)
        outcome = cruise(leg_start[0], leg_start[1])
        if outcome in ("aborted", "timeout"):
            return outcome
        axis, _length = _leg_axis(leg_start, leg_end)
        pending = sorted(
            (
                (_along_leg_m(leg_start, axis, cp.lat_deg, cp.lon_deg), cp)
                for cp in caps_by_leg.get(leg_index, [])
            ),
            key=lambda item: item[0],
        )
        outcome = cruise(leg_end[0], leg_end[1], (leg_start, axis, pending))
        for _, cp in pending:
            if abort.is_set():
                return "aborted"
            on_capture(cp, flight.position())
        if outcome in ("aborted", "timeout"):
            return outcome
    return "completed"


def fly_orbit(
    flight,
    *,
    center_lat: float,
    center_lon: float,
    agl_m: float,
    radius_m: float,
    turns: float,
    speed_m_s: float,
    abort: threading.Event,
    tick_s: float = 0.4,
) -> str:
    """Continuous carrot-chasing orbit around a ground point.

    The old segmented waypoint ring (12–16 position targets per lap, each
    with an arrival wait) made the vehicle brake and pitch at every
    vertex. Instead, stream guided position targets: each tick, read the
    vehicle's ACTUAL bearing from the center and command the point on the
    circle a fixed lead-arc ahead, yaw facing the center. The autopilot
    chases a smoothly moving target, so the path is a clean circle.
    Closed-loop on measured bearing (not open-loop time) so wind cannot
    detach the carrot; sweep accumulates from measured motion, so `turns`
    means what it says. Returns "completed", "aborted", or "timeout".
    """
    radius = max(float(radius_m), 2.0)
    speed = min(max(float(speed_m_s), 0.5), 8.0)
    turns = max(float(turns), 0.25)
    m_lon = _m_per_deg_lon(center_lat)

    def bearing_dist(lat, lon):
        dn = (lat - center_lat) * EARTH_M_PER_DEG_LAT
        de = (lon - center_lon) * m_lon
        return math.atan2(de, dn), math.hypot(dn, de)

    def circle_point(ang):
        return (
            center_lat + radius * math.cos(ang) / EARTH_M_PER_DEG_LAT,
            center_lon + radius * math.sin(ang) / m_lon,
        )

    # enter at the nearest point of the circle (due north when starting
    # from over the center, where "nearest" is undefined)
    here = flight.position()
    ang, dist = bearing_dist(here[0], here[1])
    if dist < 1.0:
        ang = 0.0
    elat, elon = circle_point(ang)
    entry_deadline = (
        time.monotonic()
        + max(_dist_m(here[0], here[1], elat, elon), 5.0) / (0.5 * speed)
        + 30.0
    )
    next_send = 0.0
    while not flight.at_target(elat, elon, agl_m, tol_m=2.5):
        if abort.is_set():
            return "aborted"
        now = time.monotonic()
        if now > entry_deadline:
            return "timeout"
        if now >= next_send:
            flight.goto(elat, elon, agl_m)
            next_send = now + 2.0
        time.sleep(0.2)

    # lead arc ~1.5 s of travel, clamped so the carrot stays meaningfully
    # ahead without pulling the track inside the circle (the vehicle flies
    # the chord to the carrot; chord depth grows with the lead angle)
    lead = min(0.8, max(0.25, speed * 1.5 / radius))
    goal = 2.0 * math.pi * turns
    swept = 0.0
    here = flight.position()
    prev_ang, _ = bearing_dist(here[0], here[1])
    budget = time.monotonic() + ((goal + lead) * radius / speed) * 3.0 + 60.0
    while swept < goal:
        if abort.is_set():
            return "aborted"
        if time.monotonic() > budget:
            return "timeout"
        here = flight.position()
        ang, _ = bearing_dist(here[0], here[1])
        delta = ang - prev_ang
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        swept = max(0.0, swept + delta)  # clockwise = increasing bearing
        prev_ang = ang
        t_lat, t_lon = circle_point(ang + lead)
        yaw = (math.degrees(ang) + 180.0) % 360.0  # face the center
        flight.goto(t_lat, t_lon, agl_m, yaw_deg=yaw)
        time.sleep(tick_s)
    return "completed"


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
        self._heading = 0.0
        self._yaw_cmd = None
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
                if self._yaw_cmd is not None:
                    self._heading = self._yaw_cmd  # guided yaw override
                elif dist > 0.2:
                    self._heading = math.degrees(math.atan2(
                        (t_lon - self._lon) * _m_per_deg_lon(self._lat),
                        (t_lat - self._lat) * EARTH_M_PER_DEG_LAT,
                    )) % 360.0
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

    def goto(self, lat: float, lon: float, agl: float, *, yaw_deg=None) -> None:
        with self._lock:
            self.mode = "GUIDED"
            self._target = (lat, lon, agl)
            self._yaw_cmd = yaw_deg

    def at_target(self, lat, lon, agl, tol_m=1.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - agl) <= max(0.5, tol_m / 2)
        )

    def heading(self) -> float | None:
        with self._lock:
            return self._heading

    def attitude(self):
        """(roll_deg, pitch_deg); the bench vehicle is always level."""
        return (0.0, 0.0)

    def rtl(self) -> bool:
        with self._lock:
            home = getattr(self, "_home", (self._lat, self._lon))
            self.mode = "RTL"
            self._target = (home[0], home[1], 0.0)
            self._yaw_cmd = None
        return True

    def land(self) -> bool:
        with self._lock:
            self.mode = "LAND"
            self._target = (self._lat, self._lon, 0.0)
            self._yaw_cmd = None
        return True

    def hold(self) -> bool:
        with self._lock:
            self.mode = "GUIDED"
            self._target = (self._lat, self._lon, self._agl)
            self._yaw_cmd = None
        return True

    def telemetry(self) -> dict:
        lat, lon, agl = self.position()
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,
            "agl_m": agl,
            "heading_deg": self.heading() or 0.0,
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

    def goto(self, lat: float, lon: float, agl: float, *, yaw_deg=None) -> None:
        # agl passes straight through: link sends it as the RELATIVE_ALT
        # wire value (home_alt_m=0).
        self._link.goto(lat, lon, agl, yaw_deg=yaw_deg)

    def at_target(self, lat, lon, agl, tol_m=2.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - agl) <= max(1.0, tol_m)
        )

    def heading(self) -> float | None:
        """Heading at the last telemetry drain, degrees, or None.

        MavlinkDroneLink doesn't decode GLOBAL_POSITION_INT.hdg, but
        pymavlink caches the last message of every type on the
        connection; position() above already drained it. 65535 = compass
        unknown/failed — fall back to the ground-track course from the
        link's cached GLOBAL_POSITION_INT velocities when the vehicle is
        actually moving, so the map indicator still turns.
        """
        try:
            msg = self._link._inner._conn.messages.get("GLOBAL_POSITION_INT")
            hdg = getattr(msg, "hdg", None)
            if hdg is not None and int(hdg) < 65535:
                return (int(hdg) % 36000) / 100.0
            vel = getattr(self._link._inner, "_last_velocity_enu", None)
            if vel is not None and math.hypot(vel[0], vel[1]) > 0.5:
                return math.degrees(math.atan2(vel[1], vel[0])) % 360.0
            return None
        except Exception:
            return None

    def attitude(self):
        """(roll_deg, pitch_deg) from the ATTITUDE cache, or None.

        A translating multirotor is NOT level — nose-down pitch swings a
        belly camera's footprint backward by AGL·tan(pitch). Captured
        with every frame so the GCS ray-casts through the true attitude.
        """
        try:
            msg = self._link._inner._conn.messages.get("ATTITUDE")
            if msg is None:
                return None
            return (
                math.degrees(float(msg.roll)),
                math.degrees(float(msg.pitch)),
            )
        except Exception:
            return None

    def rangefinder_m(self) -> float:
        """Downward rangefinder AGL, metres; -1 when not fitted/no data.

        These airframes fly WITHOUT rangefinders today — every consumer
        treats -1 as "absent" and falls back to baro AGL, so this must
        never gate a mission. If one is ever fitted it starts feeding the
        low-altitude cross-check for free.
        """
        try:
            msgs = self._link._inner._conn.messages
            m = msgs.get("RANGEFINDER")
            if m is not None and float(m.distance) > 0.0:
                return float(m.distance)
            m = msgs.get("DISTANCE_SENSOR")
            if (
                m is not None
                and int(getattr(m, "orientation", 25)) == 25  # facing down
                and int(m.current_distance) > 0
            ):
                return int(m.current_distance) / 100.0
        except Exception:
            pass
        return -1.0

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
        rf = self.rangefinder_m()
        # cross-check only when a rangefinder exists AND is in its
        # trustworthy band; drones without one (rf = -1) never alarm
        alarm = 0.0 < rf < 8.0 and abs(rf - agl) > 2.0
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,  # AGL frame throughout
            "agl_m": agl,
            "heading_deg": self.heading() or 0.0,
            "armed": bool(self._link.is_armed()),
            "mode": str(getattr(inner, "mode", "") or ""),
            "battery_pct": battery_pct,
            "rangefinder_m": rf,
            "agl_alarm": bool(alarm),
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

    def jpeg(self, *, width=None, quality=85):
        """Latest frame as JPEG, downscaled to `width` preserving aspect.

        Returns (bytes, (w, h), ts) or (None, None, 0.0) when dry. The
        aspect ratio is ALWAYS preserved: the old fixed width×height
        resize silently squashed non-8:5 sensors, which broke the nadir
        projection's square-pixel assumption (vertical ground offsets
        scaled by the squash factor). Callers embed the returned actual
        dimensions in the frame header.
        """
        frame, ts = self.latest_bgr()
        if frame is None or self._cv2 is None:
            return None, None, 0.0
        h0, w0 = frame.shape[:2]
        if width and int(width) < w0:
            w = int(width)
            h = max(2, int(round(w * h0 / w0)))
            frame = self._cv2.resize(frame, (w, h))
        else:
            w, h = w0, h0
        ok, buf = self._cv2.imencode(
            ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
        return (buf.tobytes() if ok else None), (w, h), ts

    def capture_frame_payload(self, **kwargs) -> bytes:
        """Full frame-container payload via the underlying source.

        For OpenCV sources the hub's reader keeps the buffer fresh; for
        synthetic/file sources this just delegates.
        """
        if self._cv2 is not None:
            body, dims, _ = self.jpeg(quality=85)
            if body is not None:
                return build_frame_bytes(
                    body, kind="image/jpeg",
                    width=dims[0], height=dims[1], **kwargs
                )
        return self._source.capture(**kwargs)

    # frame-source surface (capture(**kwargs) -> frame payload bytes)
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
    parser.add_argument(
        "--audio", default="none",
        help="Microphone source for the iuas audio capability (audio.py): "
        "none, synthetic[:hz], or alsa:<device>[?rate=16000]. When set, "
        "the vehicle advertises 'audio' and an investigate whose "
        "sensor_plan includes audio records a WAV clip from the orbit.",
    )
    parser.add_argument(
        "--audio-seconds", type=float, default=6.0,
        help="Length of the audio clip recorded per investigation.",
    )
    parser.add_argument(
        "--audio-range-m", type=float, default=30.0,
        help="Audio captures tied to a target point only record while the "
        "vehicle is within this range of it — the microphone is never hot "
        "outside a tasked window. 0 disables the guard.",
    )
    parser.add_argument("--mavlink-endpoint", default=None)
    parser.add_argument("--uas-ipbrc-root", default=None)
    parser.add_argument(
        "--telemetry-hz", type=float, default=4.0,
        help="Telemetry publish rate. 4 Hz keeps the dashboard marker "
        "moving smoothly; samples are ~300 bytes, so the radio cost is "
        "negligible next to one video frame.",
    )
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
    parser.add_argument(
        "--log-dir", default="/var/lib/minimuas/log",
        help="Directory for the agent's persistent event journal "
        "(fsync-per-line JSONL that survives a pulled battery, unlike "
        "journald + the page cache). Unwritable directory just disables "
        "it. Empty string disables explicitly.",
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

    if args.log_dir:
        try:
            log_path = enable_json_log(
                Path(args.log_dir) / f"{vehicle_id}-agent.jsonl"
            )
            print_json("agent.journal.ready", path=str(log_path))
        except Exception as exc:
            print_json(
                "agent.journal.disabled", dir=args.log_dir, error=str(exc)
            )

    # ---- camera + flight backend -----------------------------------------
    try:
        camera = CameraHub(args.camera)
    except Exception as exc:
        print_json("agent.camera.unavailable", camera=args.camera, error=str(exc))
        return 2
    print_json("agent.camera.ready", **camera.describe())

    audio_src = None
    if args.audio and args.audio != "none":
        from audio import audio_source_from_spec

        try:
            audio_src = audio_source_from_spec(args.audio)
        except Exception as exc:
            # a broken microphone must not ground the aircraft: fly
            # camera-only and say so, loudly
            print_json(
                "agent.audio.unavailable", audio=args.audio, error=str(exc)
            )
        else:
            print_json("agent.audio.ready", **audio_src.describe())

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

    # Floor for commandable altitudes. On a real autopilot this must stay
    # above 3 m: MavlinkDroneLink.goto() suppresses ALL position targets
    # until the vehicle is 3 m off the ground after a takeoff (protecting
    # ArduCopter's guided-takeoff sub-state) — a mission commanded below
    # that never receives a single goto and hovers at the takeoff point
    # forever. Reject it at the ack instead.
    min_agl = 3.5 if flight.source == "mavlink" else 0.5

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

    telemetry_pub = LatestPublisher(
        vehicle_telemetry_live_name(vehicle_id), freshness_ms=700
    )
    search_pub = LatestPublisher(vehicle_search_status_name(vehicle_id))
    video_status_pub = LatestPublisher(vehicle_video_status_name(vehicle_id))
    sensor_event_pub = LatestPublisher(
        vehicle_sensor_event_name(vehicle_id), freshness_ms=3000
    )

    # ---- operator sensor tasking --------------------------------------------
    # Sensors are mission-controlled: nothing records or captures outside
    # an explicit window. This is the tasking state — a single pending
    # raster OVERRIDE (serviced by fly_raster between control ticks) and
    # any number of OPPORTUNISTIC watchpoints (fired by the monitor loop
    # whenever the vehicle happens to pass within range, whatever it is
    # doing at the time).
    tasking_lock = threading.Lock()
    tasking = {"override": None, "watchpoints": []}
    sensor_seq = {"n": 0}

    def agent_sensors() -> set:
        return {"camera"} | ({"audio"} if audio_src else set())

    def do_sensor_capture(req: SensorCaptureRequest) -> SensorCaptureResult:
        """Capture `req.sensor` at the CURRENT position and publish it."""
        here = flight.position()
        ts = gps_time_ns()
        heading = flight.heading() if hasattr(flight, "heading") else None
        sensor_seq["n"] += 1
        meta = {
            "request_id": req.request_id,
            "note": req.note,
            "lat_deg": f"{here[0]:.7f}",
            "lon_deg": f"{here[1]:.7f}",
            "agl_m": f"{here[2]:.2f}",
            **({"heading_deg": f"{heading:.1f}"} if heading is not None else {}),
        }
        try:
            if req.sensor == "audio":
                if audio_src is None:
                    raise RuntimeError("no audio capability")
                if (
                    req.target is not None
                    and args.audio_range_m > 0
                    and _dist_m(
                        here[0], here[1],
                        req.target.lat_deg, req.target.lon_deg,
                    ) > args.audio_range_m
                ):
                    raise RuntimeError(
                        f"outside audio range ({args.audio_range_m:.0f} m) "
                        "of the requested target"
                    )
                body = audio_src.record_wav(
                    max(1.0, min(req.duration_s, 30.0))
                )
                name = tasked_sensor_name(
                    vehicle_id, "mic", "audio", ts, sensor_seq["n"]
                )
                payload_bytes = build_frame_bytes(
                    body,
                    mission_id="tasked",
                    vehicle_id=vehicle_id,
                    sensor_id="mic",
                    gps_time_ns=ts,
                    kind="audio/wav",
                    metadata=meta,
                )
            else:
                payload_bytes = camera.capture_frame_payload(
                    mission_id="tasked",
                    vehicle_id=vehicle_id,
                    sensor_id="bottom",
                    gps_time_ns=ts,
                    metadata=meta,
                )
                name = tasked_sensor_name(
                    vehicle_id, "bottom", "frame", ts, sensor_seq["n"]
                )
            producer = publish_segmented(name, payload_bytes)
            producers_keepalive.append(producer)
            result = SensorCaptureResult(
                request_id=req.request_id,
                vehicle_id=vehicle_id,
                sensor=req.sensor,
                status="captured",
                artifacts=[name],
                lat_deg=here[0], lon_deg=here[1], agl_m=here[2],
                gps_time_ns=ts,
            )
        except Exception as exc:
            result = SensorCaptureResult(
                request_id=req.request_id,
                vehicle_id=vehicle_id,
                sensor=req.sensor,
                status="failed",
                message=str(exc),
                lat_deg=here[0], lon_deg=here[1], agl_m=here[2],
                gps_time_ns=ts,
            )
        try:
            sensor_event_pub.publish(result.to_bytes())
        except Exception:
            pass
        print_json(
            "agent.sensor.capture",
            request=req.request_id, sensor=req.sensor,
            status=result.status, message=result.message,
            artifacts=result.artifacts,
        )
        return result

    def goto_and_wait(lat: float, lon: float, agl: float,
                      tol_m: float = 2.5, timeout_s: float = 120.0) -> bool:
        flight.goto(lat, lon, agl)
        next_send = time.monotonic() + 2.0
        deadline = time.monotonic() + timeout_s
        while not flight.at_target(lat, lon, agl, tol_m=tol_m):
            if abort.is_set() or time.monotonic() > deadline:
                return False
            if time.monotonic() >= next_send:
                flight.goto(lat, lon, agl)
                next_send = time.monotonic() + 2.0
            time.sleep(0.2)
        return True

    def service_override() -> bool:
        """fly_raster hook: serve one pending override capture, then
        return the vehicle to where the sweep was interrupted."""
        with tasking_lock:
            entry = tasking["override"]
            tasking["override"] = None
        if entry is None:
            return False
        req, done, slot = entry
        resume = flight.position()
        print_json(
            "agent.sensor.override_start",
            request=req.request_id, sensor=req.sensor,
        )
        if req.target is not None and goto_and_wait(
            req.target.lat_deg, req.target.lon_deg, resume[2],
            tol_m=max(2.5, req.radius_m),
        ):
            slot["result"] = do_sensor_capture(req)
        else:
            slot["result"] = SensorCaptureResult(
                request_id=req.request_id, vehicle_id=vehicle_id,
                sensor=req.sensor, status="failed",
                message="could not reach override target (aborted or timed out)",
            )
        # resume: fly back to the interrupt point before handing control
        # back to the sweep
        goto_and_wait(resume[0], resume[1], resume[2])
        done.set()
        return True

    def watchpoint_loop() -> None:
        while True:
            time.sleep(0.5)
            with tasking_lock:
                pending = list(tasking["watchpoints"])
            if not pending:
                continue
            here = flight.position()
            now = time.monotonic()
            for wp in pending:
                req = wp["req"]
                fire = expired = False
                if now > wp["expires"]:
                    expired = True
                elif req.target is not None and _dist_m(
                    here[0], here[1],
                    req.target.lat_deg, req.target.lon_deg,
                ) <= max(req.radius_m, 1.0):
                    fire = True
                if not (fire or expired):
                    continue
                with tasking_lock:
                    if wp in tasking["watchpoints"]:
                        tasking["watchpoints"].remove(wp)
                    else:
                        continue  # raced with another disposition
                if expired:
                    try:
                        sensor_event_pub.publish(SensorCaptureResult(
                            request_id=req.request_id,
                            vehicle_id=vehicle_id,
                            sensor=req.sensor,
                            status="failed",
                            message="watchpoint expired before the vehicle "
                            "passed within range",
                        ).to_bytes())
                    except Exception:
                        pass
                else:
                    do_sensor_capture(req)

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
            jpeg, _dims, ts = camera.jpeg(
                width=video_cfg["width"],
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
        if not (min_agl <= request.target_agl_m <= args.max_agl_m):
            return AckDecision(
                status=False,
                message=f"agl {request.target_agl_m} outside {min_agl}..{args.max_agl_m}",
            )
        return AckDecision(status=True, message=f"agl={request.target_agl_m}")

    @provider.handler(vehicle_flight_service(vehicle_id, "takeoff"))
    def cmd_takeoff(payload: bytes) -> bytes:
        request = TakeoffRequest.from_bytes(payload)
        if not (min_agl <= request.target_agl_m <= args.max_agl_m):
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

    # ---- service: sensor/capture (operator tasking, all roles) -------------
    sensor_service = vehicle_sensor_service(vehicle_id)

    @provider.ack_handler(sensor_service)
    def ack_sensor(payload: bytes) -> AckDecision:
        req = SensorCaptureRequest.from_bytes(payload)
        if req.sensor not in agent_sensors():
            return AckDecision(
                status=False,
                message=f"sensor {req.sensor!r} not carried "
                f"(have: {sorted(agent_sensors())})",
            )
        if req.mode not in ("now", "override", "opportunistic"):
            return AckDecision(status=False, message=f"unknown mode {req.mode!r}")
        if req.mode in ("override", "opportunistic") and req.target is None:
            return AckDecision(status=False, message=f"{req.mode} needs a target")
        if req.target is not None:
            here = flight.position()
            range_m = _dist_m(
                here[0], here[1], req.target.lat_deg, req.target.lon_deg
            )
            if range_m > args.max_range_m:
                return AckDecision(
                    status=False,
                    message=f"target {range_m:.0f}m away > "
                    f"{args.max_range_m:.0f}m guard",
                )
        if req.mode == "override" and busy["task"] == "investigate":
            return AckDecision(
                status=False, message="override rejected mid-investigation"
            )
        return AckDecision(status=True, message=req.mode)

    @provider.handler(sensor_service)
    def sensor_capture(payload: bytes) -> bytes:
        req = SensorCaptureRequest.from_bytes(payload)
        if req.mode == "opportunistic":
            with tasking_lock:
                tasking["watchpoints"].append({
                    "req": req,
                    "expires": time.monotonic() + max(30.0, req.expires_s),
                })
            print_json(
                "agent.sensor.watchpoint",
                request=req.request_id, sensor=req.sensor,
                radius_m=req.radius_m, expires_s=req.expires_s,
            )
            return SensorCaptureResult(
                request_id=req.request_id, vehicle_id=vehicle_id,
                sensor=req.sensor, status="queued",
                message="watchpoint armed; fires when the vehicle passes "
                f"within {req.radius_m:.0f} m",
            ).to_bytes()

        if req.mode == "now" or req.target is None:
            # capture where we are — the "directly requested" case
            return do_sensor_capture(req).to_bytes()

        # override with a target
        if busy["task"] == "raster-search":
            done = threading.Event()
            slot: dict = {}
            with tasking_lock:
                if tasking["override"] is not None:
                    return SensorCaptureResult(
                        request_id=req.request_id, vehicle_id=vehicle_id,
                        sensor=req.sensor, status="rejected",
                        message="another override is already pending",
                    ).to_bytes()
                tasking["override"] = (req, done, slot)
            if not done.wait(timeout=240.0):
                with tasking_lock:
                    if (
                        tasking["override"] is not None
                        and tasking["override"][0] is req
                    ):
                        tasking["override"] = None
                return SensorCaptureResult(
                    request_id=req.request_id, vehicle_id=vehicle_id,
                    sensor=req.sensor, status="failed",
                    message="override not serviced (raster ended or busy); "
                    "re-issue as mode=now near the point",
                ).to_bytes()
            return slot["result"].to_bytes()

        # idle vehicle: fly there, capture, hold at the point
        if not set_busy("sensor-capture"):
            return SensorCaptureResult(
                request_id=req.request_id, vehicle_id=vehicle_id,
                sensor=req.sensor, status="rejected",
                message=f"busy:{busy['task']}",
            ).to_bytes()
        abort.clear()
        try:
            here = flight.position()
            agl = max(here[2], min_agl)
            if not flight.ensure_airborne(agl):
                return SensorCaptureResult(
                    request_id=req.request_id, vehicle_id=vehicle_id,
                    sensor=req.sensor, status="failed",
                    message="could not get airborne",
                ).to_bytes()
            if not goto_and_wait(
                req.target.lat_deg, req.target.lon_deg, agl,
                tol_m=max(2.5, req.radius_m),
            ):
                return SensorCaptureResult(
                    request_id=req.request_id, vehicle_id=vehicle_id,
                    sensor=req.sensor, status="failed",
                    message="could not reach the target point",
                ).to_bytes()
            return do_sensor_capture(req).to_bytes()
        finally:
            set_busy("")

    # ---- service: system/shutdown (authorized companion power-off) ----------
    # SD-card filesystems rewind to their last real flush when the battery
    # is pulled — journals, tasked captures, and tiles written since then
    # vanish. A clean poweroff (sync + unmount) is the only real
    # assurance, so give the operator one: guarded (never while armed or
    # mid-task) and authorized (the request must carry the vehicle id as
    # a typed confirm phrase — no accidental single click can take a
    # companion down).
    shutdown_service = vehicle_system_service(vehicle_id, "shutdown")

    def _shutdown_guard(payload: bytes) -> str:
        """Empty string when shutdown is permitted, else the refusal."""
        try:
            confirm = str(json.loads(payload.decode() or "{}").get("confirm", ""))
        except Exception:
            confirm = ""
        if confirm != vehicle_id:
            return f"confirm phrase must be the vehicle id ({vehicle_id!r})"
        t = flight.telemetry()
        if t.get("armed"):
            return "refused: vehicle is ARMED"
        if busy["task"]:
            return f"refused: busy:{busy['task']}"
        return ""

    @provider.ack_handler(shutdown_service)
    def ack_shutdown(payload: bytes) -> AckDecision:
        reason = _shutdown_guard(payload)
        if reason:
            return AckDecision(status=False, message=reason)
        return AckDecision(status=True, message="will sync + poweroff")

    @provider.handler(shutdown_service)
    def cmd_shutdown(payload: bytes) -> bytes:
        reason = _shutdown_guard(payload)  # re-check: state may have changed
        if reason:
            return FlightCommandResult(
                vehicle_id=vehicle_id, command="shutdown",
                status="rejected", message=reason,
            ).to_bytes()
        print_json("agent.system.shutdown", vehicle=vehicle_id)
        flush_json_log()

        def poweroff() -> None:
            # flush everything the kernel holds, then a clean poweroff
            # (which also syncs + unmounts). The 3 s delay lets the NDN
            # response reach the GCS before the network goes away.
            try:
                os.sync()
            except Exception:
                pass
            for cmd in (
                ["systemctl", "poweroff", "--no-wall"],
                ["poweroff"],
                ["shutdown", "-h", "now"],
            ):
                try:
                    if subprocess.run(cmd, timeout=20).returncode == 0:
                        return
                except Exception:
                    continue

        threading.Timer(3.0, poweroff).start()
        return FlightCommandResult(
            vehicle_id=vehicle_id, command="shutdown",
            status="accepted", message="syncing and powering off in 3 s",
        ).to_bytes()

    services = [
        vehicle_video_service(vehicle_id),
        vehicle_sensor_service(vehicle_id),
        vehicle_system_service(vehicle_id, "shutdown"),
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
            if not (min_agl <= request.agl_m <= args.max_agl_m):
                return AckDecision(
                    status=False,
                    message=f"agl {request.agl_m} outside {min_agl}..{args.max_agl_m}",
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
                # heading: the ACTUAL compass heading at capture when the
                # FC provides one — the planned leg heading was wrong
                # whenever the vehicle was crabbing, turning, or hovering,
                # and every degree of error swings the geo-projection by
                # the in-frame lever arm
                heading = flight.heading() if hasattr(flight, "heading") else None
                if heading is None:
                    heading = cp.heading_deg
                attitude = (
                    flight.attitude() if hasattr(flight, "attitude") else None
                )
                jpeg, dims, _ = camera.jpeg(
                    width=args.search_frame_width,
                    quality=args.search_frame_quality,
                )
                if jpeg is None:
                    payload_bytes = camera.capture_frame_payload(
                        mission_id=request.mission_id,
                        vehicle_id=vehicle_id,
                        sensor_id="bottom",
                        gps_time_ns=ts,
                        metadata={"heading_deg": f"{heading:.1f}"},
                    )
                else:
                    payload_bytes = build_frame_bytes(
                        jpeg,
                        mission_id=request.mission_id,
                        vehicle_id=vehicle_id,
                        sensor_id="bottom",
                        gps_time_ns=ts,
                        kind="image/jpeg",
                        width=dims[0],
                        height=dims[1],
                        metadata={
                            "lat_deg": f"{here[0]:.7f}",
                            "lon_deg": f"{here[1]:.7f}",
                            "agl_m": f"{here[2]:.2f}",
                            "heading_deg": f"{heading:.1f}",
                            **(
                                {
                                    "roll_deg": f"{attitude[0]:.1f}",
                                    "pitch_deg": f"{attitude[1]:.1f}",
                                }
                                if attitude is not None
                                else {}
                            ),
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

                def on_leg(leg_index: int) -> None:
                    status_state.update(state="searching", leg=leg_index)
                    push_status()

                def on_capture(cp, here) -> None:
                    nonlocal frames
                    frames += 1
                    _publish_capture(cp, frames, here)
                    push_status()

                outcome = fly_raster(
                    flight,
                    plan,
                    agl_m=request.agl_m,
                    speed_m_s=request.speed_m_s,
                    abort=abort,
                    deadline_mono=deadline,
                    on_capture=on_capture,
                    on_leg=on_leg,
                    service_interrupt=service_override,
                )
                if outcome == "timeout":
                    outcome = "failed"
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

    # ---- iuas: investigate (continuous carrot orbit on either backend) ------
    if args.role == "iuas":
        investigate_service = vehicle_flight_service(vehicle_id, "investigate")
        services.append(investigate_service)

        @provider.ack_handler(investigate_service)
        def ack_investigate(payload: bytes) -> AckDecision:
            request = InvestigatePointRequest.from_bytes(payload)
            if busy["task"]:
                return AckDecision(status=False, message=f"busy:{busy['task']}")
            if request.circle_radius_m <= 0 or request.circle_count <= 0:
                return AckDecision(status=False, message="invalid request geometry")
            wanted = _investigate_sensors(request)
            unknown = wanted - {"camera", "audio"}
            if unknown:
                return AckDecision(
                    status=False,
                    message=f"unknown sensors: {sorted(unknown)}",
                )
            if "audio" in wanted and audio_src is None:
                return AckDecision(
                    status=False, message="no audio capability on this vehicle"
                )
            if not (min_agl <= request.approach_alt_m <= args.max_agl_m):
                return AckDecision(
                    status=False,
                    message=(
                        f"agl {request.approach_alt_m} outside "
                        f"{min_agl}..{args.max_agl_m} guard"
                    ),
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
            return AckDecision(status=True, message="carrot-orbit")

        def _investigate_sensors(request: InvestigatePointRequest) -> set:
            # legacy requesters say "front" for the camera
            plan = request.sensor_plan or ["camera"]
            return {"camera" if s in ("front", "camera") else s for s in plan}

        def _run_investigate(request: InvestigatePointRequest):
            """Fly the investigation directly on the agent's flight backend.

            One path for sim and MAVLink: climb, then a CONTINUOUS carrot
            orbit (fly_orbit) around the target with yaw facing it. The
            previous MAVLink path went through relay.flight's
            guided-yaw-path, a 16-waypoint ring with an arrival wait at
            every vertex — the field IUAS flew a stop-and-go polygon.
            Streaming guided targets makes the lap one smooth circle.
            Everything is AGL (mavlink link pinned to home_alt_m=0).
            """
            import types
            from contracts import FlightTaskResult, SensorArtifact

            started = gps_time_ns()
            tgt = request.target
            agl = request.approach_alt_m
            speed = request.constraints.max_speed_mps or 3.0
            flight.set_cruise_speed(speed)
            if not flight.ensure_airborne(agl):
                status, note = "failed", "could not reach approach altitude"
            else:
                orbit = fly_orbit(
                    flight,
                    center_lat=tgt.lat_deg,
                    center_lon=tgt.lon_deg,
                    agl_m=agl,
                    radius_m=request.circle_radius_m,
                    turns=request.circle_count,
                    speed_m_s=speed,
                    abort=abort,
                )
                status = {
                    "completed": "completed", "aborted": "aborted"
                }.get(orbit, "failed")
                note = (
                    "carrot-orbit" if orbit == "completed"
                    else f"carrot-orbit: {orbit}"
                )
            # capture per requested sensor from the orbit's end pose: a
            # camera frame, an audio clip, or both — whatever this
            # vehicle carries and the request asked for
            artifacts: list[SensorArtifact] = []
            payloads: list[bytes] = []
            here = flight.position()
            heading = flight.heading() if hasattr(flight, "heading") else None
            pose = Pose(
                position=GeoPoint(
                    lat_deg=here[0], lon_deg=here[1], alt_m=here[2]
                ),
                yaw_deg=heading if heading is not None else 0.0,
            )
            base_meta = {
                "target_id": request.source_detection_id,
                "lat_deg": f"{here[0]:.7f}",
                "lon_deg": f"{here[1]:.7f}",
                "agl_m": f"{here[2]:.2f}",
            }
            sensor_errors: list[str] = []
            if status != "failed":
                for i, sensor in enumerate(sorted(_investigate_sensors(request))):
                    artifact_time = gps_time_ns()
                    if sensor == "audio":
                        if audio_src is None:
                            sensor_errors.append("audio: not fitted")
                            continue
                        if args.audio_range_m > 0 and _dist_m(
                            here[0], here[1],
                            request.target.lat_deg, request.target.lon_deg,
                        ) > args.audio_range_m:
                            sensor_errors.append(
                                f"audio: outside {args.audio_range_m:.0f} m "
                                "listen range of the target"
                            )
                            continue
                        try:
                            wav = audio_src.record_wav(args.audio_seconds)
                        except Exception as exc:
                            sensor_errors.append(f"audio: {exc}")
                            continue
                        payloads.append(build_frame_bytes(
                            wav,
                            mission_id=request.mission_id,
                            vehicle_id=vehicle_id,
                            sensor_id="mic",
                            gps_time_ns=artifact_time,
                            kind="audio/wav",
                            metadata={
                                **base_meta,
                                "seconds": f"{args.audio_seconds:g}",
                            },
                        ))
                        artifacts.append(SensorArtifact(
                            data_name=mission_sensor_name(
                                request.mission_id, vehicle_id, "mic",
                                "audio", artifact_time, i + 1,
                            ),
                            kind="audio/wav",
                            gps_time_ns=artifact_time,
                            pose=pose,
                            metadata={
                                "target_id": request.source_detection_id
                            },
                        ))
                    else:
                        try:
                            payloads.append(camera.capture_frame_payload(
                                mission_id=request.mission_id,
                                vehicle_id=vehicle_id,
                                sensor_id="front",
                                gps_time_ns=artifact_time,
                                metadata=dict(base_meta),
                            ))
                        except Exception as exc:
                            sensor_errors.append(f"camera: {exc}")
                            continue
                        artifacts.append(SensorArtifact(
                            data_name=mission_sensor_name(
                                request.mission_id, vehicle_id, "front",
                                "frame", artifact_time, i + 1,
                            ),
                            kind="image/jpeg",
                            gps_time_ns=artifact_time,
                            pose=pose,
                            metadata={
                                "target_id": request.source_detection_id
                            },
                        ))
                if sensor_errors:
                    # the flight succeeded but the evidence didn't: a job
                    # whose sensor produced nothing must not report done
                    if not artifacts:
                        status = "failed"
                    note = f"{note}; " + "; ".join(sensor_errors)
            result = FlightTaskResult(
                task_id=(
                    f"{vehicle_id}-investigate-{request.source_detection_id}"
                ),
                status=status,
                started_at_gps_ns=started,
                completed_at_gps_ns=gps_time_ns(),
                artifacts=artifacts if status != "failed" else [],
                notes=note,
            )
            return types.SimpleNamespace(
                result=result,
                artifact_payloads=payloads if status != "failed" else [],
                mode="carrot-orbit",
                command_log=[],
            )

        @provider.handler(investigate_service)
        def investigate(payload: bytes) -> bytes:
            request = InvestigatePointRequest.from_bytes(payload)
            if not set_busy("investigate"):
                return ServiceResponse(status=False, error=f"busy:{busy['task']}")
            abort.clear()
            try:
                outcome = _run_investigate(request)
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

    # ---- capability + run ----------------------------------------------------
    with optional_local_nfd(args.start_local_nfd):
        profile = CapabilityProfile(
            vehicle_id=vehicle_id,
            gps_time_ns=gps_time_ns(),
            position=True,
            velocity=True,
            yaw_control=True,
            mode_control=True,
            # investigation/tasking sensors ride in extras so the GCS can
            # route per-sensor jobs and tasked captures to a vehicle that
            # actually carries them
            extras=(
                (["orbit"] if args.role == "iuas" else [])
                + sorted(agent_sensors())
            ),
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
        threading.Thread(target=watchpoint_loop, daemon=True).start()

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
