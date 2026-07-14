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
    default_audio_meta,
    default_camera_meta,
    gps_time_ns,
    mission_frame_name,
    mission_sensor_name,
    tasked_sensor_name,
    vehicle_flight_service,
    vehicle_coord_status_name,
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
from dataplane import build_frame_bytes, fetch_segmented, publish_segmented
from raster import build_raster
from ndnsf_runtime import (
    add_common_arguments,
    add_ndnsf_path,
    enable_json_log,
    flush_json_log,
    optional_local_nfd,
    print_json,
    provider_kwargs,
    start_journal_publisher,
    start_nfd_counter_scrape,
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


def fly_flyover(
    flight,
    *,
    waypoints,
    speed_m_s: float,
    abort: threading.Event,
    on_dip_center=None,
) -> str:
    """Fly the acoustic dip-flyover waypoint list (backport of v3's
    `fly_flyover` / uas-flight `flyover_targets`).

    An audio target is interrogated with a low straight pass through the
    point, NOT a camera-style carrot orbit: the vehicle lines up behind the
    target at cruise AGL, descends to the commandable floor for a run
    directly over it, then climbs out — repeated for each pass, rotating
    90° between passes. Waypoints are flown with the same guided-cruise
    discipline as the raster/orbit legs: a target re-sent every 2 s, a 2.5 m
    arrival tolerance, and a per-leg deadline that "moves on" rather than
    hovering a leg the vehicle cannot make (so one blocked leg never strands
    the pass). The de-confliction altitude overlay rides underneath every
    `goto` exactly as it does for the orbit.

    v3 left the acoustic capture a stub (its WaypointFlight drops the phase
    tags and never fires the mic). Here `on_dip_center(waypoint)` is invoked
    at each dip_center the vehicle actually reaches — that is where the WAV
    is recorded and published. Returns "completed", "aborted", or "timeout".
    """

    speed = min(max(float(speed_m_s), 0.5), 8.0)
    tol_m = 2.5
    path_len = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        path_len += _dist_m(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg)
    budget = time.monotonic() + 3.0 * path_len / speed + 90.0
    for wp in waypoints:
        here = flight.position()
        leg_deadline = (
            time.monotonic()
            + max(_dist_m(here[0], here[1], wp.lat_deg, wp.lon_deg), 5.0)
            / (0.5 * speed)
            + 45.0
        )
        next_send = 0.0
        arrived = False
        while True:
            if abort.is_set():
                return "aborted"
            now = time.monotonic()
            if now > budget:
                return "timeout"
            if flight.at_target(wp.lat_deg, wp.lon_deg, wp.agl_m, tol_m=tol_m):
                arrived = True
                break
            if now > leg_deadline:
                # never hover a leg we cannot make — advance like the raster
                break
            if now >= next_send:
                flight.goto(
                    wp.lat_deg, wp.lon_deg, wp.agl_m, yaw_deg=wp.bearing_deg
                )
                next_send = now + 2.0
            time.sleep(0.2)
        if arrived and wp.phase == "dip_center" and on_dip_center is not None:
            on_dip_center(wp)
    return "completed"


# ---------------------------------------------------------------------------
# Fleet coordination: separation by communication
# ---------------------------------------------------------------------------

# Below ArduCopter's hover noise (~±0.5 m): a maneuver whose own-side bias is
# smaller than this never engages — it would churn the altitude controller
# without buying separation. The peer's half (or its uncooperative escalation)
# carries the pair. This is the guard against the hover-oscillation limit cycle.
MIN_ENGAGE_BIAS_M = 0.75
# The merged altitude bias handed to the flight overlay is clamped to this
# band so stacked conflicts can't command an unbounded climb/descent.
ALT_BIAS_MIN_M = -4.0
ALT_BIAS_MAX_M = 8.0

# A busy task that neither heartbeats (busy_touch, called from its motion loop)
# nor keeps its owner thread alive for this long is treated as stranded, and the
# flag is reclaimed — otherwise one hung/dead task bricks the vehicle until a
# power-cycle (observed: a stuck takeoff left busy set, so the next mission was
# rejected). Longer than any single flight primitive: takeoff's ensure_airborne
# deadline is 120 s, and every goto_and_wait heartbeats every 0.2 s.
BUSY_STALE_S = 150.0

# Collision-avoidance priority tiers, advertised in telemetry so peers know who
# yields. A drone WORKING a task (e.g. a camera raster whose whole value is a
# steady low search altitude) outranks one merely NAVIGATING to its task zone,
# which outranks an IDLE loiterer. Higher tier holds its course/altitude; the
# lower-tier peer yields — maneuvering if it is navigating/idle, or pausing in
# place if it is itself mid-task. Same-tier conflicts fall back to the
# deconfliction module's symmetric cooperative plan.
AVOID_IDLE, AVOID_NAV, AVOID_WORK = 1, 2, 3

# Which avoidance tier each busy task advertises. A camera raster and an active
# investigation/capture are altitude/position-critical -> WORK (they hold, and
# peers yield to them). A takeoff is transient transit -> NAV. Anything not
# listed defaults to WORK (hold) rather than silently yielding.
_TASK_TIER = {
    "raster-search": AVOID_WORK,
    "investigate": AVOID_WORK,
    "sensor-capture": AVOID_WORK,
    "takeoff": AVOID_NAV,
}


class PeerGuard:
    """Watches peer telemetry, predicts conflicts, flies vertical avoidance.

    These airframes have no useful perception sensors, so separation is a
    communication problem: fetch each peer's telemetry on an adaptive
    schedule (relay.flight.deconflict.peer_poll_interval_s — distant or
    opening peers cost one fetch per ~5 s, an imminent one is watched at
    2 Hz), extrapolate with constant-velocity physics, and when the
    predicted closest approach violates the separation envelope, apply a
    vertical bias through the flight backend's altitude overlay.

    Coordination protocol (data-plane only, no request/response): the
    cooperative pair plan is DETERMINISTIC AND SYMMETRIC — both vehicles
    compute identical roles from each other's telemetry — so each side
    just applies its own role immediately and publishes the maneuver on
    its coord/status name. Seeing the peer's matching entry inside the
    grace window confirms cooperation; not seeing it escalates to the
    uncooperative plan (take the whole burden upward, with headroom).
    A peer-published entry naming US is adopted even before our own
    detector fires, so whichever side notices first drags both.

    Transport is injected (fetch_telemetry / fetch_coord / publish_coord
    callables) so the whole loop runs against bench sim backends in
    tests with no NDN anywhere.
    """

    def __init__(
        self,
        vehicle_id: str,
        flight,
        peer_ids,
        *,
        fetch_telemetry,
        fetch_coord,
        publish_coord,
        deconflict_module,
        envelope=None,
        on_event=None,
        own_tier=None,
        pause_event=None,
        min_airborne_agl_m: float = 2.0,
        floor_agl_m: float = 3.5,
        grace_s: float = 2.5,
        tick_s: float = 0.1,
    ) -> None:
        self.vehicle_id = vehicle_id
        self.flight = flight
        self.peer_ids = list(peer_ids)
        self.fetch_telemetry = fetch_telemetry
        self.fetch_coord = fetch_coord
        self.publish_coord = publish_coord
        self.dc = deconflict_module
        self.envelope = envelope or deconflict_module.DeconflictionEnvelope()
        self.on_event = on_event or (lambda **kw: None)
        # own_tier() -> our current AVOID_* priority; pause_event is set when a
        # higher-priority peer requires us to hold position mid-task
        self.own_tier = own_tier or (lambda: AVOID_IDLE)
        self.pause_event = pause_event
        self.min_airborne = min_airborne_agl_m
        # fleet-wide flight floor: the cooperative plan never asks a
        # descender to give altitude it doesn't have above this (the
        # climber absorbs the shortfall). Must match the backends' goto
        # floor and be the SAME on every vehicle.
        self.floor_agl_m = floor_agl_m
        self.grace_s = grace_s
        self.tick_s = tick_s
        self._peers = {
            vid: {"due": 0.0, "sample": None, "seen_mono": 0.0}
            for vid in self.peer_ids
        }
        # active avoidance per peer:
        #   {mode: coop-pending|coop|unco, bias, started, clear_since,
        #    hold_s, expires}
        self._active: dict[str, dict] = {}
        # GPS time (ns) at which we last released a maneuver against each peer.
        # A peer's cached coord entry is only adopted if it is NEWER than this,
        # so a stale "coop" entry can't ping-pong a pair we just settled.
        self._released_ns: dict[str, int] = {}
        self._stop = threading.Event()
        self._thread = None

    # -- lifecycle -----------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    # -- geometry ------------------------------------------------------

    def _relative(self, sample: dict):
        own = self.flight.position()
        own_v = (
            self.flight.velocity() if hasattr(self.flight, "velocity")
            else (0.0, 0.0)
        )
        return self.dc.RelativeState(
            north_m=(sample["lat_deg"] - own[0]) * EARTH_M_PER_DEG_LAT,
            east_m=(sample["lon_deg"] - own[1]) * _m_per_deg_lon(own[0]),
            up_m=sample.get("agl_m", 0.0) - own[2],
            vnorth_m_s=sample.get("vn_m_s", 0.0) - own_v[0],
            veast_m_s=sample.get("ve_m_s", 0.0) - own_v[1],
            vup_m_s=0.0,
        ), own

    # -- bias + coord publication ---------------------------------------

    def _apply(self) -> None:
        biases = [entry["bias"] for entry in self._active.values()]
        if not biases:
            bias = 0.0
        else:
            ups = [b for b in biases if b > 0]
            # when several conflicts disagree, climbing wins: descending
            # into one conflict to solve another is never the answer
            bias = max(ups) if ups else min(biases)
        # keep stacked conflicts from commanding an unbounded excursion
        bias = max(ALT_BIAS_MIN_M, min(ALT_BIAS_MAX_M, bias))
        if hasattr(self.flight, "set_alt_bias"):
            self.flight.set_alt_bias(bias)
        # a "pause" entry means a higher-priority peer requires us to hold
        # position mid-task; drive the shared pause flag the task loop honors
        if self.pause_event is not None:
            if any(e["mode"] == "pause" for e in self._active.values()):
                self.pause_event.set()
            else:
                self.pause_event.clear()
        entries = [
            {
                "from_id": self.vehicle_id,
                "to_id": peer,
                "biases": {self.vehicle_id: entry["bias"]},
                "mode": entry["mode"],
                "gps_time_ns": gps_time_ns(),
            }
            for peer, entry in self._active.items()
        ]
        try:
            self.publish_coord(entries)
        except Exception:
            pass

    def _engage(self, peer: str, mode: str, bias: float, hold_s: float,
                reason: str) -> None:
        now = time.monotonic()
        self._active[peer] = {
            "mode": mode, "bias": bias, "started": now,
            "clear_since": None, "hold_s": hold_s, "expires": now + 60.0,
        }
        self._apply()
        self.on_event(
            kind=f"coord.{mode}", peer=peer,
            bias_m=round(bias, 2), reason=reason,
        )

    def _release(self, peer: str, why: str) -> None:
        if self._active.pop(peer, None) is not None:
            # stamp the release so a stale peer coord entry can't immediately
            # re-adopt us into the maneuver we just cleared (see _adopt_remote)
            self._released_ns[peer] = gps_time_ns()
            self._apply()
            self.on_event(kind="coord.clear", peer=peer, reason=why)

    # -- main loop -------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.wait(self.tick_s):
            try:
                self._step(time.monotonic())
            except Exception as exc:
                self.on_event(kind="coord.error", error=str(exc))

    def _step(self, now: float) -> None:
        own_pos = self.flight.position()
        airborne = own_pos[2] >= self.min_airborne
        for peer in self.peer_ids:
            state = self._peers[peer]
            if now < state["due"]:
                continue
            sample = None
            try:
                sample = self.fetch_telemetry(peer)
            except Exception:
                sample = None
            if sample is not None:
                state["sample"] = sample
                state["seen_mono"] = now
            elif now - state["seen_mono"] > 20.0:
                state["sample"] = None
            sample = state["sample"]
            if sample is None or not airborne or (
                sample.get("agl_m", 0.0) < self.min_airborne
            ):
                # nothing to separate from: relaxed re-check, and any
                # active maneuver against this peer expires below
                state["due"] = now + 3.0
                self._expire(peer, now)
                continue
            rel, own = self._relative(sample)
            cpa = self.dc.closest_point_of_approach(rel)
            entry = self._active.get(peer)
            if entry is None:
                if self.dc.in_conflict(cpa, self.envelope):
                    self._on_conflict(peer, sample, own)
                else:
                    # not in conflict: adopt a peer-initiated maneuver if
                    # the peer's coord status names us (it noticed first)
                    self._adopt_remote(peer)
            else:
                self._update_active(peer, entry, cpa, now)
            interval = self.dc.peer_poll_interval_s(rel)
            if peer in self._active:
                interval = min(interval, 0.5)
            state["due"] = now + interval
        self._expire_all(now)

    def _on_conflict(self, peer: str, sample: dict, own) -> None:
        # Priority arbitration first: WORKING outranks NAVIGATING outranks IDLE.
        # Both sides read the same tiers from shared telemetry.
        own_tier = self.own_tier()
        peer_tier = int(sample.get("avoid_tier", AVOID_IDLE))
        if own_tier > peer_tier:
            # higher priority: hold our course/altitude, let the peer yield —
            # this keeps a searching camera drone on its search altitude instead
            # of being shoved up by an idle/navigating peer.
            if peer in self._active:
                self._release(peer, "priority-hold")
            return
        if own_tier < peer_tier:
            # lower priority: take the whole avoidance burden by maneuvering,
            # since the higher-priority peer holds
            plan = self.dc.uncooperative_plan(
                self.vehicle_id, peer, envelope=self.envelope,
            )
            self._engage(peer, "yield", plan.biases[self.vehicle_id],
                         plan.hold_s, "yield: maneuver (outranked)")
            return
        if own_tier >= AVOID_WORK:
            # two drones both mid-task can't both hold; break the tie
            # deterministically (lower vehicle id holds) so exactly one pauses in
            # place while the other continues its task.
            if self.vehicle_id < peer:
                if peer in self._active:
                    self._release(peer, "priority-hold")
                return
            self._engage(peer, "pause", 0.0, self.grace_s,
                         "yield: pause (working, tie)")
            return

        # both navigating or both idle: the symmetric cooperative plan, unchanged
        plan = self.dc.cooperative_plan(
            self.vehicle_id, own[2], peer, sample.get("agl_m", 0.0),
            envelope=self.envelope, floor_agl_m=self.floor_agl_m,
        )
        own_bias = plan.biases[self.vehicle_id]
        if abs(own_bias) < MIN_ENGAGE_BIAS_M:
            # our share of the separation is below hover noise (e.g. we are the
            # descender pinned near the floor): don't churn the controller —
            # the peer's larger half, or its uncooperative escalation, opens
            # the full gap. Re-evaluated every poll, so it engages if geometry
            # later hands us a real share.
            return
        self._engage(
            peer, "coop-pending", own_bias,
            plan.hold_s, plan.reason,
        )

    def _adopt_remote(self, peer: str) -> None:
        try:
            entries = self.fetch_coord(peer) or []
        except Exception:
            return
        for entry in entries:
            if entry.get("to_id") != self.vehicle_id:
                continue
            # only adopt an entry that is NEWER than our last release against
            # this peer — otherwise a stale cached "coop" entry re-engages a
            # pair we just settled and the two ping-pong forever. (Fresh CPA
            # violations still re-engage ungated via _on_conflict.)
            if entry.get("gps_time_ns", 0) <= self._released_ns.get(peer, 0):
                continue
            own_pos = self.flight.position()
            sample = self._peers[peer]["sample"] or {}
            if self.own_tier() > int(sample.get("avoid_tier", AVOID_IDLE)):
                # we outrank the peer: ignore its adopt request and hold course
                return
            plan = self.dc.cooperative_plan(
                self.vehicle_id, own_pos[2],
                peer, sample.get("agl_m", 0.0),
                envelope=self.envelope, floor_agl_m=self.floor_agl_m,
            )
            own_bias = plan.biases[self.vehicle_id]
            if abs(own_bias) < MIN_ENGAGE_BIAS_M:
                # sub-hover-noise share: let the peer carry it (see _on_conflict)
                return
            self._engage(
                peer, "coop", own_bias,
                plan.hold_s, "adopted peer-initiated maneuver",
            )
            return

    def _update_active(self, peer: str, entry: dict, cpa, now: float) -> None:
        if entry["mode"] == "coop-pending":
            confirmed = False
            try:
                for e in self.fetch_coord(peer) or []:
                    if e.get("to_id") == self.vehicle_id or (
                        e.get("from_id") == peer
                        and e.get("to_id") == self.vehicle_id
                    ):
                        confirmed = True
            except Exception:
                pass
            if confirmed:
                entry["mode"] = "coop"
                # re-emit so the CONFIRMED state is visible on our coord/status
                # wire — a peer adopting us must see "coop", not stale "pending"
                self._apply()
                self.on_event(kind="coord.confirmed", peer=peer)
            elif now - entry["started"] > self.grace_s:
                # peer never joined: assume it holds course, take the
                # whole burden upward with headroom
                plan = self.dc.uncooperative_plan(
                    self.vehicle_id, peer, envelope=self.envelope,
                )
                self._engage(
                    peer, "unco", plan.biases[self.vehicle_id],
                    plan.hold_s, plan.reason,
                )
                return
        if self.dc.conflict_cleared(cpa, self.envelope):
            if entry["clear_since"] is None:
                entry["clear_since"] = now
            elif now - entry["clear_since"] >= entry["hold_s"]:
                self._release(peer, "cpa passed")
        else:
            entry["clear_since"] = None
            entry["expires"] = now + 60.0

    def _expire(self, peer: str, now: float) -> None:
        entry = self._active.get(peer)
        if entry is not None and now > entry["expires"]:
            self._release(peer, "expired")

    def _expire_all(self, now: float) -> None:
        for peer in list(self._active):
            self._expire(peer, now)


def smart_rtl(
    flight,
    vehicle_id: str,
    fleet_ids,
    *,
    deconflict_module,
    cancel: threading.Event,
    base_agl_m: float = 8.0,
    separation_m: float = 3.0,
    on_event=None,
) -> str:
    """Layered return-to-launch: collision-free by construction.

    The autopilot's native RTL climbs to ONE configured altitude and
    flies straight home — with several vehicles that leaves the crossing
    to chance. Here every vehicle computes the same deterministic
    altitude slot table (sorted fleet ids, `separation_m` apart), climbs
    in place to ITS slot, cruises home at it, and lands — so simultaneous
    returns cross at guaranteed vertical separation no matter how the
    horizontal paths intersect, with the PeerGuard still biasing
    underneath as a second layer. Falls back to the autopilot's RTL when
    home is unknown, and backs off entirely if the operator takes the
    aircraft out of GUIDED (the RC pilot always wins).
    """

    emit = on_event or (lambda **kw: None)
    slots = deconflict_module.rtl_altitude_slots(
        fleet_ids, base_agl_m=base_agl_m, separation_m=separation_m
    )
    slot = slots[vehicle_id]
    home = flight.home() if hasattr(flight, "home") else None
    here = flight.position()
    if here[2] < 2.0:
        emit(kind="rtl.on_ground")
        return "on-ground"
    if home is None:
        emit(kind="rtl.fallback", reason="home unknown")
        flight.rtl()
        return "fallback"
    emit(kind="rtl.smart", slot_agl_m=slot, home=list(home))

    def pilot_took_over() -> bool:
        mode = str(flight.telemetry().get("mode", "") or "").upper()
        return mode not in ("", "GUIDED")

    def cruise(lat, lon, agl, deadline_s, tol_m) -> bool:
        deadline = time.monotonic() + deadline_s
        next_send = 0.0
        while not flight.at_target(lat, lon, agl, tol_m=tol_m):
            if cancel.is_set():
                return False
            if pilot_took_over():
                emit(kind="rtl.takeover")
                return False
            if time.monotonic() > deadline:
                return False
            if time.monotonic() >= next_send:
                flight.goto(lat, lon, agl)
                next_send = time.monotonic() + 2.0
            time.sleep(0.2)
        return True

    # climb in place to the slot, cruise home at it, then land
    if not cruise(here[0], here[1], slot,
                  abs(slot - here[2]) / 0.5 + 30.0, tol_m=3.0):
        if not cancel.is_set() and not pilot_took_over():
            emit(kind="rtl.fallback", reason="climb failed")
            flight.rtl()
        return "fallback"
    dist = _dist_m(here[0], here[1], home[0], home[1])
    if not cruise(home[0], home[1], slot, dist / 1.0 + 60.0, tol_m=3.0):
        if not cancel.is_set() and not pilot_took_over():
            emit(kind="rtl.fallback", reason="cruise failed")
            flight.rtl()
        return "fallback"
    if cancel.is_set() or pilot_took_over():
        return "cancelled"
    flight.land()
    emit(kind="rtl.landing", home=list(home))
    return "landing"


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
        self._vn = 0.0
        self._ve = 0.0
        self._alt_bias = 0.0
        self._last_cmd = None  # raw (lat, lon, agl, yaw) pre-bias
        self.armed = False
        self.mode = "STABILIZE"
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        dt = 0.2
        while not self._stop.wait(dt):
            with self._lock:
                prev = (self._lat, self._lon)
                if self._target is None:
                    self._vn = self._ve = 0.0
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
                self._vn = (self._lat - prev[0]) * EARTH_M_PER_DEG_LAT / dt
                self._ve = (self._lon - prev[1]) * _m_per_deg_lon(self._lat) / dt
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

    def _effective_agl(self, agl: float) -> float:
        return max(0.5, float(agl) + self._alt_bias)

    def goto(self, lat: float, lon: float, agl: float, *, yaw_deg=None) -> None:
        with self._lock:
            self.mode = "GUIDED"
            self._last_cmd = (lat, lon, agl, yaw_deg)
            self._target = (lat, lon, self._effective_agl(agl))
            self._yaw_cmd = yaw_deg

    def at_target(self, lat, lon, agl, tol_m=1.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - self._effective_agl(agl)) <= max(0.5, tol_m / 2)
        )

    def set_alt_bias(self, bias_m: float) -> None:
        """Vertical de-confliction overlay: every commanded altitude is
        shifted by this until cleared; the current target is re-issued
        immediately so the maneuver starts now, not at the next resend."""
        with self._lock:
            self._alt_bias = max(-4.0, min(float(bias_m), 8.0))
            cmd = self._last_cmd
        if cmd is not None:
            self.goto(cmd[0], cmd[1], cmd[2], yaw_deg=cmd[3])

    def avoid_bias(self) -> float:
        return self._alt_bias

    def velocity(self):
        """Ground velocity (north, east) m/s."""
        with self._lock:
            return (self._vn, self._ve)

    def home(self):
        h = getattr(self, "_home", None)
        return None if h is None else (h[0], h[1])

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
        vn, ve = self.velocity()
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,
            "agl_m": agl,
            "heading_deg": self.heading() or 0.0,
            "armed": self.armed,
            "mode": self.mode,
            "vn_m_s": vn,
            "ve_m_s": ve,
            "avoid_bias_m": self._alt_bias,
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
        self._alt_bias = 0.0
        self._last_cmd = None  # raw (lat, lon, agl, yaw) pre-bias
        self._home = None      # (lat, lon) captured at last ground arm

    def position(self):
        p = self._vehicle.position
        # pos.alt is already AGL (link pinned to home_alt_m=0).
        return (p.lat, p.lon, max(p.alt, 0.0))

    def set_cruise_speed(self, speed: float) -> None:
        self._link.set_cruise_speed_m_s(speed)

    def ensure_airborne(self, agl: float) -> bool:
        if not self._link.is_armed():
            # about to launch from here: THIS is home for smart RTL
            p = self.position()
            self._home = (p[0], p[1])
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

    def _effective_agl(self, agl: float) -> float:
        # floor 3.5: the link suppresses gotos below the 3 m takeoff gate
        return max(3.5, float(agl) + self._alt_bias)

    def goto(self, lat: float, lon: float, agl: float, *, yaw_deg=None) -> None:
        # agl passes straight through (link pinned to home_alt_m=0),
        # shifted by the de-confliction bias when one is active.
        self._last_cmd = (lat, lon, agl, yaw_deg)
        t_lat, t_lon = lat, lon
        if self._alt_bias:
            # SITL 2026-07-09: ArduCopter guided flies a straight 3D line
            # to its target, so an avoidance climb attached to a far
            # waypoint is spread over the WHOLE leg — at a mid-leg
            # crossing barely half the bias was realized. While a bias is
            # active, cap the horizontal lead so the vertical maneuver
            # completes within ~lead metres of travel (seconds, not the
            # whole leg); the mission's 2 s re-sends keep walking the
            # capped carrot toward the real target.
            here = self.position()
            dist = _dist_m(here[0], here[1], lat, lon)
            lead = 15.0
            if dist > lead:
                f = lead / dist
                t_lat = here[0] + (lat - here[0]) * f
                t_lon = here[1] + (lon - here[1]) * f
        self._link.goto(t_lat, t_lon, self._effective_agl(agl), yaw_deg=yaw_deg)

    def at_target(self, lat, lon, agl, tol_m=2.0) -> bool:
        p = self.position()
        return (
            _dist_m(p[0], p[1], lat, lon) <= tol_m
            and abs(p[2] - self._effective_agl(agl)) <= max(1.0, tol_m)
        )

    def set_alt_bias(self, bias_m: float) -> None:
        """Vertical de-confliction overlay (see SimFlightBackend)."""
        self._alt_bias = max(-4.0, min(float(bias_m), 8.0))
        cmd = self._last_cmd
        if cmd is not None:
            self.goto(cmd[0], cmd[1], cmd[2], yaw_deg=cmd[3])

    def avoid_bias(self) -> float:
        return self._alt_bias

    def velocity(self):
        """Ground velocity (north, east) m/s from GLOBAL_POSITION_INT."""
        vel = getattr(self._link._inner, "_last_velocity_enu", None)
        if vel is None:
            return (0.0, 0.0)
        return (float(vel[0]), float(vel[1]))

    def home(self):
        return self._home

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

    def _mode_string(self) -> str:
        """Human ArduCopter flight mode from the cached HEARTBEAT.

        The v3 agent reports the flight mode in telemetry
        (`muas-agent/src/telemetry.rs` build_sample copies `t.mode`, filled
        by the mavlink backend from the heartbeat's custom_mode). The old v2
        code read `getattr(inner, "mode", "")`, but MavlinkDroneLink exposes
        no such attribute — only the integer `_last_heartbeat_custom_mode`,
        refreshed on every telemetry drain (each `position()` call). Resolve
        that integer to a name through pymavlink's per-vehicle
        `mode_mapping()` (name->number; e.g. STABILIZE=0, AUTO=3, GUIDED=4,
        RTL=6, LAND=9). The link's cached custom_mode is sysid-filtered, so
        (unlike `conn.flightmode`) it is never stomped by another GCS's
        heartbeat on the shared mavproxy fan-out. Empty string when the
        mapping is not yet known so the dashboard cell shows its "-" default.
        """
        inner = self._link._inner
        try:
            conn = inner._conn
            custom = int(getattr(inner, "_last_heartbeat_custom_mode", 0))
            mapping = conn.mode_mapping() or {}
            for name, number in mapping.items():
                if int(number) == custom:
                    return str(name)
        except Exception:
            pass
        return ""

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
        vn, ve = self.velocity()
        return {
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": agl,  # AGL frame throughout
            "agl_m": agl,
            "heading_deg": self.heading() or 0.0,
            "armed": bool(self._link.is_armed()),
            "mode": self._mode_string(),
            "battery_pct": battery_pct,
            "rangefinder_m": rf,
            "agl_alarm": bool(alarm),
            "vn_m_s": vn,
            "ve_m_s": ve,
            "avoid_bias_m": self._alt_bias,
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
        "--cam-hfov-deg", type=float, default=70.0,
        help="Camera horizontal FOV advertised in the sensor_meta capability "
        "key (the dashboard's coverage layer renders the ground footprint "
        "from this; matches the GCS nadir-projection default of 70 deg).",
    )
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
        "--sensors", default="auto",
        help="Investigation sensors this vehicle advertises: auto "
        "(camera, plus audio when --audio is fitted) or an explicit "
        "comma list, e.g. 'audio' for a microphone-only IUAS whose "
        "camera should not receive capture jobs.",
    )
    parser.add_argument(
        "--fleet-ids", default="",
        help="Comma-separated ids of EVERY vehicle in the fleet (self "
        "included). Enables fleet coordination: adaptive peer telemetry "
        "watching, physics-based conflict prediction, cooperative/"
        "uncooperative vertical avoidance, and slot-layered smart RTL. "
        "Empty disables (single-vehicle behavior).",
    )
    parser.add_argument(
        "--coord-hsep-m", type=float, default=5.0,
        help="Horizontal separation minimum for conflict prediction "
        "(small-airframe default; raise for larger/faster vehicles).",
    )
    parser.add_argument(
        "--coord-vsep-m", type=float, default=2.0,
        help="Vertical separation minimum for conflict prediction. The "
        "yielder climbs vsep*1.5 (uncooperative) or the pair splits vsep*1.25 "
        "(cooperative); 2 m suits the small quads (~3 m climb).",
    )
    parser.add_argument(
        "--coord-horizon-s", type=float, default=20.0,
        help="Reaction horizon: predicted approaches beyond this are "
        "watched, not acted on.",
    )
    parser.add_argument(
        "--rtl-base-agl-m", type=float, default=8.0,
        help="Smart RTL: lowest return-cruise altitude slot.",
    )
    parser.add_argument(
        "--rtl-sep-m", type=float, default=3.0,
        help="Smart RTL: vertical spacing between fleet altitude slots.",
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

    start_nfd_counter_scrape(args.nfd_metrics_interval, enabled=args.nfd_metrics)

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
    # Serve this vehicle's agent journal (events + metrics + logs) over NDN
    # under /muas/v2/<vehicle_id>/journal/<session> so the dashboard's mission
    # bundle sweep can pull the whole flight record without SSH.
    start_journal_publisher(vehicle_id, args.session)
    from ndnsf import AckDecision, ServiceProvider, ServiceResponse

    provider = ServiceProvider(**provider_kwargs(args, prefix, ""))

    # ---- shared state ------------------------------------------------------
    state_lock = threading.Lock()
    busy = {"task": "", "owner": None, "beat": 0.0}  # "", "raster-search", ...
    abort = threading.Event()        # raised by hold/rtl/land during a task
    avoid = {"tier": AVOID_IDLE}     # collision priority advertised to peers
    avoid_pause = threading.Event()  # PeerGuard sets this to pause our task motion
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
        available = {"camera"} | ({"audio"} if audio_src else set())
        if args.sensors and args.sensors != "auto":
            wanted = {s.strip() for s in args.sensors.split(",") if s.strip()}
            return (wanted & available) or {"camera"}
        return available

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
        pause_anchor = None
        while not flight.at_target(lat, lon, agl, tol_m=tol_m):
            if abort.is_set() or time.monotonic() > deadline:
                return False
            busy_touch()  # heartbeat the busy flag while actively flying
            if avoid_pause.is_set():
                # yield-by-pausing: hold where we are until the higher-priority
                # peer clears, rather than flying our task path into the conflict
                if pause_anchor is None:
                    pause_anchor = flight.position()
                    print_json("agent.avoid.pause_hold", lat=pause_anchor[0],
                               lon=pause_anchor[1], agl_m=pause_anchor[2])
                flight.goto(pause_anchor[0], pause_anchor[1], pause_anchor[2])
                deadline += 0.2  # a legitimate pause must not exhaust the budget
                next_send = time.monotonic() + 2.0
                time.sleep(0.2)
                continue
            if pause_anchor is not None:
                pause_anchor = None  # conflict cleared: resume toward the target
                print_json("agent.avoid.pause_resume")
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
        now = time.monotonic()
        reclaimed = None
        with state_lock:
            if not task:
                # release: only the current owner (or nobody) may clear, so a
                # stale thread's late finally can't wipe a successor's task
                if busy["owner"] in (None, threading.current_thread()):
                    busy.update(task="", owner=None, beat=0.0)
                    avoid["tier"] = AVOID_IDLE
                    avoid_pause.clear()
                return True
            if busy["task"]:
                owner = busy["owner"]
                fresh = (
                    owner is not None
                    and owner.is_alive()
                    and now - busy["beat"] <= BUSY_STALE_S
                )
                if fresh:
                    return False
                reclaimed = busy["task"]  # stranded: dead owner or no heartbeat
            busy.update(task=task, owner=threading.current_thread(), beat=now)
            avoid["tier"] = _TASK_TIER.get(task, AVOID_WORK)
            if avoid["tier"] < AVOID_WORK:
                avoid_pause.clear()
        if reclaimed is not None:
            print_json("agent.busy.reclaimed", was=reclaimed, by=task)
        return True

    def busy_touch() -> None:
        # heartbeat from the owning task's motion loop, so the watchdog can tell
        # a live, progressing task from a stalled or dead one
        cur = threading.current_thread()
        with state_lock:
            if busy["owner"] is cur:
                busy["beat"] = time.monotonic()

    # ---- fleet coordination: PeerGuard + smart RTL ---------------------------
    fleet_ids = [v.strip() for v in args.fleet_ids.split(",") if v.strip()]
    peer_ids = [v for v in fleet_ids if v != vehicle_id]
    deconflict = None
    peer_guard = None
    rtl_cancel = threading.Event()
    rtl_thread: dict = {"t": None}
    if peer_ids:
        try:
            from investigate_plan import add_flight_path

            add_flight_path(uas_root)
            import relay.flight.deconflict as deconflict
        except Exception as exc:
            print_json(
                "agent.coord.disabled",
                error=str(exc),
                note="relay.flight.deconflict unavailable — flying "
                "WITHOUT fleet separation",
            )
    if deconflict is not None:
        coord_pub = LatestPublisher(
            vehicle_coord_status_name(vehicle_id), freshness_ms=700
        )

        def _fetch_peer_telemetry(vid: str):
            try:
                return json.loads(fetch_segmented(
                    vehicle_telemetry_live_name(vid), timeout_ms=600
                ).decode())
            except Exception:
                return None

        def _fetch_peer_coord(vid: str):
            try:
                return json.loads(fetch_segmented(
                    vehicle_coord_status_name(vid), timeout_ms=600
                ).decode())
            except Exception:
                return None

        def _publish_coord(entries) -> None:
            coord_pub.publish(json.dumps(entries).encode())

        peer_guard = PeerGuard(
            vehicle_id,
            flight,
            peer_ids,
            fetch_telemetry=_fetch_peer_telemetry,
            fetch_coord=_fetch_peer_coord,
            publish_coord=_publish_coord,
            deconflict_module=deconflict,
            envelope=deconflict.DeconflictionEnvelope(
                horizontal_sep_m=args.coord_hsep_m,
                vertical_sep_m=args.coord_vsep_m,
                horizon_s=args.coord_horizon_s,
            ),
            floor_agl_m=min_agl,  # same constant the goto floor enforces
            on_event=lambda **kw: print_json(
                "agent." + str(kw.pop("kind")), **kw
            ),
            own_tier=lambda: avoid["tier"],
            pause_event=avoid_pause,
        )
        print_json(
            "agent.coord.ready", peers=peer_ids,
            hsep_m=args.coord_hsep_m, vsep_m=args.coord_vsep_m,
            horizon_s=args.coord_horizon_s,
        )

    def _start_smart_rtl() -> bool:
        prev = rtl_thread["t"]
        if prev is not None and prev.is_alive():
            return True  # already returning; don't restart the sequence
        rtl_cancel.clear()

        def run() -> None:
            outcome = smart_rtl(
                flight,
                vehicle_id,
                fleet_ids,
                deconflict_module=deconflict,
                cancel=rtl_cancel,
                base_agl_m=args.rtl_base_agl_m,
                separation_m=args.rtl_sep_m,
                on_event=lambda **kw: print_json(
                    "agent." + str(kw.pop("kind")), **kw
                ),
            )
            print_json("agent.rtl.finished", outcome=outcome)

        thread = threading.Thread(target=run, daemon=True)
        rtl_thread["t"] = thread
        thread.start()
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
                    avoid_tier=avoid["tier"],
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
        if command != "rtl":
            rtl_cancel.set()  # land/hold override an in-progress smart RTL
        try:
            if command == "rtl" and fleet_ids and deconflict is not None:
                # layered return: deterministic altitude slot per vehicle,
                # collision-free by construction on simultaneous RTL
                ok = _start_smart_rtl()
            else:
                ok = {
                    "rtl": flight.rtl,
                    "land": flight.land,
                    "hold": flight.hold,
                }[command]()
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
        # Accept here; the gate lives in cmd_takeoff. A negative AckDecision is
        # silently dropped by the NDNSF consumer (ServiceUser logs "Permission
        # Denied" and returns without invoking the caller's handler), so the
        # requester never learns the reason — it just times out. Only a
        # request-handler response is delivered, so cmd_takeoff re-runs the
        # busy + agl guards and returns a visible rejection.
        return AckDecision(status=True)

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

    def _sensor_guard(req) -> str:
        """Empty string when the capture is permitted, else the refusal."""
        if req.sensor not in agent_sensors():
            return (
                f"sensor {req.sensor!r} not carried "
                f"(have: {sorted(agent_sensors())})"
            )
        if req.mode not in ("now", "override", "opportunistic"):
            return f"unknown mode {req.mode!r}"
        if req.mode in ("override", "opportunistic") and req.target is None:
            return f"{req.mode} needs a target"
        if req.target is not None:
            here = flight.position()
            range_m = _dist_m(
                here[0], here[1], req.target.lat_deg, req.target.lon_deg
            )
            if range_m > args.max_range_m:
                return (
                    f"target {range_m:.0f}m away > "
                    f"{args.max_range_m:.0f}m guard"
                )
        if req.mode == "override" and busy["task"] == "investigate":
            return "override rejected mid-investigation"
        return ""

    @provider.ack_handler(sensor_service)
    def ack_sensor(payload: bytes) -> AckDecision:
        # Gate in sensor_capture, not here: a negative ack never reaches the
        # requester (the NDNSF consumer drops it -> the request just times out).
        return AckDecision(status=True)

    @provider.handler(sensor_service)
    def sensor_capture(payload: bytes) -> bytes:
        req = SensorCaptureRequest.from_bytes(payload)
        reason = _sensor_guard(req)
        if reason:
            return SensorCaptureResult(
                request_id=req.request_id, vehicle_id=vehicle_id,
                sensor=req.sensor, status="rejected", message=reason,
            ).to_bytes()
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
        # Gate in cmd_shutdown, not here: a negative AckDecision is dropped by
        # the NDNSF consumer, so a refused shutdown (armed / busy / bad confirm)
        # would just time out with no reason on the dashboard. cmd_shutdown
        # re-runs _shutdown_guard and returns a visible "rejected" result.
        return AckDecision(status=True)

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

        def _search_guard(request) -> str:
            """Empty string when the search is permitted, else the refusal."""
            if busy["task"]:
                return f"busy:{busy['task']}"
            if not (min_agl <= request.agl_m <= args.max_agl_m):
                return f"agl {request.agl_m} outside {min_agl}..{args.max_agl_m}"
            from raster import resolve_area

            center_lat, center_lon, _w, _h = resolve_area(request.area)
            here = flight.position()
            range_m = _dist_m(here[0], here[1], center_lat, center_lon)
            if range_m > args.max_range_m:
                return f"area {range_m:.0f}m away > {args.max_range_m:.0f}m guard"
            plan = build_raster(
                request.area,
                leg_spacing_m=request.leg_spacing_m,
                capture_every_m=request.capture_every_m,
            )
            if not plan.captures:
                return "empty raster"
            return ""

        @provider.ack_handler(search_service)
        def ack_search(payload: bytes) -> AckDecision:
            # Gate in raster_search: a negative ack is dropped by the consumer.
            return AckDecision(status=True)

        @provider.handler(search_service)
        def raster_search(payload: bytes) -> bytes:
            request = RasterSearchRequest.from_bytes(payload)
            reason = _search_guard(request)
            if reason:
                return ServiceResponse(status=False, error=reason)
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
                    # Synthetic-camera path: still tag the capture pose, same
                    # as the real-jpeg branch below. Without lat/lon/agl the
                    # GCS geo-projects every detection off a (0,0) placeholder
                    # pose, landing the estimate at ~null island — thousands
                    # of km from the fleet — which trips the IUAS investigate
                    # max-range guard so a dispatched investigation always
                    # fails and the IUAS never take off.
                    payload_bytes = camera.capture_frame_payload(
                        mission_id=request.mission_id,
                        vehicle_id=vehicle_id,
                        sensor_id="bottom",
                        gps_time_ns=ts,
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

        def _investigate_guard(request) -> str:
            """Empty string when the investigation is permitted, else refusal."""
            if busy["task"]:
                return f"busy:{busy['task']}"
            if request.circle_radius_m <= 0 or request.circle_count <= 0:
                return "invalid request geometry"
            wanted = _investigate_sensors(request)
            unknown = wanted - {"camera", "audio"}
            if unknown:
                return f"unknown sensors: {sorted(unknown)}"
            if "audio" in wanted and audio_src is None:
                return "no audio capability on this vehicle"
            if not (min_agl <= request.approach_alt_m <= args.max_agl_m):
                return (
                    f"agl {request.approach_alt_m} outside "
                    f"{min_agl}..{args.max_agl_m} guard"
                )
            here = flight.position()
            range_m = _dist_m(
                here[0], here[1],
                request.target.lat_deg, request.target.lon_deg,
            )
            if range_m > args.max_range_m:
                return f"target {range_m:.0f}m away > {args.max_range_m:.0f}m guard"
            return ""

        @provider.ack_handler(investigate_service)
        def ack_investigate(payload: bytes) -> AckDecision:
            # Gate in the investigate handler: a negative ack is dropped by the
            # NDNSF consumer, so the requester would just time out with no reason.
            return AckDecision(status=True)

        def _investigate_sensors(request: InvestigatePointRequest) -> set:
            # legacy requesters say "front" for the camera
            plan = request.sensor_plan or ["camera"]
            return {"camera" if s in ("front", "camera") else s for s in plan}

        def _select_investigate_pattern(request: InvestigatePointRequest) -> str:
            """Pick the flight geometry (v3 `select_investigate_pattern`).

            An audio-only interrogation on an audio-capable airframe (the
            iuas-02 mic) flies the acoustic DIP FLYOVER; everything else —
            any camera work — keeps the carrot orbit. The choice keys on the
            vehicle's own sensors so a camera IUAS never dips.
            """
            wanted = _investigate_sensors(request)
            vehicle = agent_sensors()
            audio_only = bool(wanted) and wanted <= {"audio"}
            has_audio = audio_src is not None and "audio" in vehicle
            if audio_only and has_audio:
                return "flyover"
            return "orbit"

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
            from investigate_plan import build_flyover_waypoints

            started = gps_time_ns()
            tgt = request.target
            agl = request.approach_alt_m
            speed = request.constraints.max_speed_mps or 3.0
            pattern = _select_investigate_pattern(request)
            task_id = f"{vehicle_id}-investigate-{request.source_detection_id}"
            flight.set_cruise_speed(speed)

            artifacts: list[SensorArtifact] = []
            payloads: list[bytes] = []
            sensor_errors: list[str] = []

            def _capture_audio(seq_index: int, phase: str) -> bool:
                """Record + stage a WAV at the CURRENT position and pose.

                This is where the acoustic interrogation actually fires. v3
                left the flyover's capture a stub (its WaypointFlight drops
                the phase tags and never records); here it is driven from the
                dip-center callback, so the mic is hot directly over the
                target at the lowest point of the pass.
                """
                if audio_src is None:
                    sensor_errors.append("audio: not fitted")
                    return False
                here = flight.position()
                if args.audio_range_m > 0 and _dist_m(
                    here[0], here[1], tgt.lat_deg, tgt.lon_deg,
                ) > args.audio_range_m:
                    sensor_errors.append(
                        f"audio: outside {args.audio_range_m:.0f} m "
                        "listen range of the target"
                    )
                    return False
                try:
                    wav = audio_src.record_wav(args.audio_seconds)
                except Exception as exc:
                    sensor_errors.append(f"audio: {exc}")
                    return False
                artifact_time = gps_time_ns()
                heading = flight.heading() if hasattr(flight, "heading") else None
                pose = Pose(
                    position=GeoPoint(
                        lat_deg=here[0], lon_deg=here[1], alt_m=here[2]
                    ),
                    yaw_deg=heading if heading is not None else 0.0,
                )
                payloads.append(build_frame_bytes(
                    wav,
                    mission_id=request.mission_id,
                    vehicle_id=vehicle_id,
                    sensor_id="mic",
                    gps_time_ns=artifact_time,
                    kind="audio/wav",
                    metadata={
                        "target_id": request.source_detection_id,
                        "lat_deg": f"{here[0]:.7f}",
                        "lon_deg": f"{here[1]:.7f}",
                        "agl_m": f"{here[2]:.2f}",
                        "phase": phase,
                        "seconds": f"{args.audio_seconds:g}",
                    },
                ))
                artifacts.append(SensorArtifact(
                    data_name=mission_sensor_name(
                        request.mission_id, vehicle_id, "mic",
                        "audio", artifact_time, seq_index,
                    ),
                    kind="audio/wav",
                    gps_time_ns=artifact_time,
                    pose=pose,
                    metadata={
                        "target_id": request.source_detection_id,
                        "phase": phase,
                    },
                ))
                print_json(
                    "agent.investigate.audio_captured",
                    task_id=task_id, phase=phase,
                    lat_deg=round(here[0], 7), lon_deg=round(here[1], 7),
                    agl_m=round(here[2], 2), seconds=args.audio_seconds,
                )
                return True

            if not flight.ensure_airborne(agl):
                status, note, mode = (
                    "failed", "could not reach approach altitude", pattern
                )
            elif pattern == "flyover":
                mode = "dip-flyover"
                # Approach along the vehicle's natural inbound bearing; dip to
                # the commandable floor (min_agl, already the goto floor and
                # the fleet AGL guard) for a low run directly over the target.
                here = flight.position()
                approach = math.degrees(math.atan2(
                    (tgt.lon_deg - here[1]) * _m_per_deg_lon(here[0]),
                    (tgt.lat_deg - here[0]) * EARTH_M_PER_DEG_LAT,
                )) % 360.0
                passes = max(int(round(request.circle_count)), 1)
                dip_agl = min_agl
                waypoints = build_flyover_waypoints(
                    target_lat=tgt.lat_deg,
                    target_lon=tgt.lon_deg,
                    approach_bearing_deg=approach,
                    cruise_agl_m=agl,
                    dip_agl_m=dip_agl,
                    radius_m=request.circle_radius_m,
                    passes=passes,
                )
                print_json(
                    "agent.investigate.flyover",
                    task_id=task_id, waypoints=len(waypoints), passes=passes,
                    cruise_agl_m=agl, dip_agl_m=dip_agl,
                    radius_m=request.circle_radius_m,
                    approach_bearing_deg=round(approach, 1),
                )
                dip_seq = {"n": 0}

                def _on_dip_center(wp) -> None:
                    dip_seq["n"] += 1
                    print_json(
                        "agent.investigate.dip_center",
                        task_id=task_id, pass_index=wp.pass_index,
                        agl_m=round(wp.agl_m, 2),
                        lat_deg=round(wp.lat_deg, 7),
                        lon_deg=round(wp.lon_deg, 7),
                        bearing_deg=round(wp.bearing_deg, 1),
                    )
                    _capture_audio(dip_seq["n"], "dip_center")

                fly = fly_flyover(
                    flight,
                    waypoints=waypoints,
                    speed_m_s=speed,
                    abort=abort,
                    on_dip_center=_on_dip_center,
                )
                status = {
                    "completed": "completed", "aborted": "aborted"
                }.get(fly, "failed")
                note = "dip-flyover" if fly == "completed" else f"dip-flyover: {fly}"
                # Captures already fired at the dip centers; no post-flight
                # sensor loop for this pattern. Evidence-or-fail: a pass that
                # never reached a dip center produced nothing.
                if status != "failed" and not artifacts:
                    status = "failed"
                    note = f"{note}; no audio captured at dip center"
                if sensor_errors:
                    note = f"{note}; " + "; ".join(sensor_errors)
            else:
                mode = "carrot-orbit"
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
                if status != "failed":
                    for i, sensor in enumerate(
                        sorted(_investigate_sensors(request))
                    ):
                        artifact_time = gps_time_ns()
                        if sensor == "audio":
                            _capture_audio(i + 1, "orbit")
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
                task_id=task_id,
                status=status,
                started_at_gps_ns=started,
                completed_at_gps_ns=gps_time_ns(),
                artifacts=artifacts if status != "failed" else [],
                notes=note,
            )
            return types.SimpleNamespace(
                result=result,
                artifact_payloads=payloads if status != "failed" else [],
                mode=mode,
                command_log=[],
            )

        @provider.handler(investigate_service)
        def investigate(payload: bytes) -> bytes:
            request = InvestigatePointRequest.from_bytes(payload)
            reason = _investigate_guard(request)
            if reason:
                return ServiceResponse(status=False, error=reason)
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
    # sensor_meta: the airframe advertises the FACTS its sensors have so the
    # dashboard's coverage layer renders them (no airframe knowledge in the
    # renderer). camera => nadir footprint quad; audio => omni reach circle.
    def build_sensor_meta() -> dict:
        sensors = agent_sensors()
        meta: dict = {}
        if "camera" in sensors:
            meta["camera"] = default_camera_meta(
                args.cam_hfov_deg,
                args.search_frame_width,
                args.search_frame_height,
                facing="down",
            )
        if "audio" in sensors and audio_src is not None:
            meta["audio"] = default_audio_meta(args.audio_range_m)
        return meta

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
            sensor_meta=build_sensor_meta(),
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
        if peer_guard is not None:
            peer_guard.start()

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
