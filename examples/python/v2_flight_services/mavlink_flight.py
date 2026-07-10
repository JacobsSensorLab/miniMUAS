"""MAVLink-backed flight link for the v2 investigate slice.

Bridges `relay.drone.mavlink.MavlinkDroneLink` (UAS-IPBRC's pymavlink
DroneLink, identical against ArduPilot SITL and real autopilots) to the
duck-typed `FlightCommandLink` surface the relay.flight executor drives.

By design this changes nothing about plan compilation: the same
`compile_investigation` output executes here, with `plan_orbit` selecting a
guided fallback because the link (correctly) exposes no native `orbit`
method. Position truth comes from GLOBAL_POSITION_INT telemetry instead of
teleportation, and the runner ticks on the wall clock.

Altitude frames: the flight plan speaks absolute ASL; ArduCopter's guided
targets travel in relative-to-home. `MavlinkDroneLink(home_alt_m=...)` does
the translation, so callers must know the ground ASL — `connect_flight_link`
auto-detects it from the first position fix when not supplied.
"""

from __future__ import annotations

import json
import time
from typing import Any

from investigate_plan import add_flight_path


class LoggingFlightLink:
    """FlightCommandLink surface over a MavlinkDroneLink, with a command log.

    Method-for-method the same surface the simulated link exposes — except
    `orbit`, deliberately absent so the capability ladder selects a guided
    execution mode. `goto`'s signature carries the yaw kwargs, which is what
    the executor inspects to allow yaw-dependent primitives.
    """

    def __init__(self, inner: Any) -> None:
        self._inner = inner
        self.command_log: list[tuple[str, dict[str, object]]] = []

    # -- FlightCommandLink surface ------------------------------------

    def arm(self) -> bool:
        ok = bool(self._inner.arm())
        self.command_log.append(("arm", {"ok": ok}))
        return ok

    def takeoff(self, alt_m: float) -> bool:
        ok = bool(self._inner.takeoff(float(alt_m)))
        self.command_log.append(("takeoff", {"alt_m": float(alt_m), "ok": ok}))
        return ok

    def set_cruise_speed_m_s(self, speed_m_s: float) -> bool:
        ok = bool(self._inner.set_cruise_speed_m_s(float(speed_m_s)))
        self.command_log.append(
            ("set_speed", {"speed_m_s": float(speed_m_s), "ok": ok})
        )
        return ok

    def goto(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        *,
        vel_n_m_s: float = 0.0,
        vel_e_m_s: float = 0.0,
        vel_u_m_s: float = 0.0,
        yaw_deg: float | None = None,
        yaw_rate_deg_s: float | None = None,
    ) -> None:
        self._inner.goto(
            lat,
            lon,
            alt_m,
            vel_n_m_s=vel_n_m_s,
            vel_e_m_s=vel_e_m_s,
            vel_u_m_s=vel_u_m_s,
            yaw_deg=yaw_deg,
            yaw_rate_deg_s=yaw_rate_deg_s,
        )
        self.command_log.append(
            ("goto", {"lat": lat, "lon": lon, "alt_m": alt_m, "yaw_deg": yaw_deg})
        )

    def land(self) -> bool:
        ok = bool(self._inner.land())
        self.command_log.append(("land", {"ok": ok}))
        return ok

    def rtl(self) -> bool:
        ok = bool(self._inner.rtl())
        self.command_log.append(("rtl", {"ok": ok}))
        return ok

    def set_mode_guided(self) -> bool:
        """Switch the autopilot to GUIDED, if the inner link can.

        ArduCopter refuses MAV_CMD_COMPONENT_ARM_DISARM in non-armable
        modes — a vehicle that finished an RTL is parked in RTL — and
        ignores guided position targets outside GUIDED. MavlinkDroneLink
        keeps mode switching private (`_set_mode`, retry-until-heartbeat
        semantics), so reach it defensively; links without it are
        assumed to not need mode management.
        """

        setter = getattr(self._inner, "_set_mode", None)
        if setter is None:
            return True
        ok = bool(setter("GUIDED"))
        self.command_log.append(("set_mode", {"mode": "GUIDED", "ok": ok}))
        return ok

    # -- telemetry passthroughs (not logged) ---------------------------

    def position(self):
        return self._inner.position()

    def is_armed(self) -> bool:
        return bool(self._inner.is_armed())

    def close(self) -> None:
        self._inner.close()


class LinkVehicle:
    """Vehicle snapshot whose position is live MAVLink telemetry.

    Drop-in for the executor's vehicle objects: `.id`, `.position`, and
    `.armed` attributes, with position/armed reading the link on access.
    Falls back to the last known position when telemetry is momentarily
    absent so primitives never see None mid-plan.
    """

    def __init__(
        self,
        vehicle_id: str,
        link: LoggingFlightLink,
        position_type: type,
        initial_position: Any,
    ) -> None:
        self.id = vehicle_id
        self._link = link
        self._position_type = position_type
        self._last = initial_position

    @property
    def position(self):
        value = self._link.position()
        if value is not None:
            self._last = self._position_type(
                float(value.lat), float(value.lon), float(value.alt)
            )
        return self._last

    @property
    def armed(self) -> bool:
        return self._link.is_armed()


def mavlink_capability_profile():
    """Capability profile for a MavlinkDroneLink-backed vehicle.

    Position, velocity, yaw, and mode control are real (telemetry,
    SET_POSITION_TARGET yaw fields, GUIDED mode switching). No native
    orbit — ArduCopter GUIDED has no orbit command on this link — so the
    plan_orbit ladder lands on guided-yaw-path.
    """

    from relay.flight import FlightCapabilityProfile

    return FlightCapabilityProfile(
        position=True,
        velocity=True,
        yaw_control=True,
        mode_control=True,
    )


def connect_flight_link(
    endpoint: str,
    *,
    vehicle_id: str = "iuas-01",
    home_alt_m: float | None = None,
    connect_timeout_s: float = 10.0,
    position_timeout_s: float = 30.0,
    uas_ipbrc_root=None,
):
    """Connect to an autopilot and return (link, vehicle, home_alt_m).

    One connection only: ArduPilot's TCP serial ports serve a single
    client, and a probe/close/reconnect dance races the listener (worse
    through Docker's host proxy, which can delay the FIN). When
    `home_alt_m` is not supplied the link runs with home_alt_m=0 — the
    plan's "ASL" frame then coincides with ArduCopter's relative-to-home
    frame — and ground level is read from the first position fix on the
    same connection (~0 for a grounded vehicle). When it is supplied, the
    plan frame is true ASL. Either way, commands and telemetry pass
    through one consistent translation. Auto-detection assumes the
    vehicle is on the ground at connect time.
    """

    add_flight_path(uas_ipbrc_root)
    from relay.core.geo import Position
    from relay.drone.mavlink import MavlinkDroneLink

    inner = MavlinkDroneLink(
        node_id=vehicle_id,
        endpoint=endpoint,
        home_alt_m=float(home_alt_m) if home_alt_m is not None else 0.0,
    )
    inner.connect(timeout_s=connect_timeout_s)
    try:
        start = _wait_position(inner, position_timeout_s, phase="primary")
    except Exception:
        # close on failure or the dangling connection (and its 1 Hz
        # heartbeat thread) poisons every retry: SITL's single-client
        # serial port keeps talking to the corpse
        try:
            inner.close()
        except Exception:
            pass
        raise
    if home_alt_m is None:
        # Grounded vehicle: its current altitude in the link frame IS
        # ground level for all subsequent AGL math.
        home_alt_m = float(start.alt)

    link = LoggingFlightLink(inner)
    vehicle = LinkVehicle(
        vehicle_id,
        link,
        Position,
        Position(float(start.lat), float(start.lon), float(start.alt)),
    )
    return link, vehicle, float(home_alt_m)


def ensure_airborne(
    link: LoggingFlightLink,
    vehicle: LinkVehicle,
    *,
    target_agl_m: float,
    home_alt_m: float,
    timeout_s: float = 120.0,
    settle_m: float = 2.0,
    ground_agl_tolerance_m: float = 1.5,
    climb_check_s: float = 12.0,
    climb_check_min_gain_m: float = 1.0,
) -> bool:
    """Arm and take off to `target_agl_m` if not already flying.

    Idempotent: an armed vehicle meaningfully off the ground is left
    alone — the plan's own ChangeAltitude step handles altitude from
    there. GUIDED is forced up front (no-op when already there): a
    vehicle that finished a previous flight sits in RTL or LAND, where
    ArduCopter rejects arming and ignores guided targets.

    Altitude-source sanity (fleet has flaky barometers; a past crash hit
    the ground while reported AGL was still positive):

      * ground check — a DISARMED vehicle is on the ground by
        definition. If it nonetheless reports AGL beyond
        `ground_agl_tolerance_m`, the altitude estimate is lying and
        every subsequent AGL-referenced command would be biased by that
        error; refuse to launch.
      * climb check — after NAV_TAKEOFF, if reported AGL hasn't gained
        `climb_check_min_gain_m` within `climb_check_s`, either the
        vehicle isn't climbing or the altitude source is stuck; abort
        the mission start rather than fly on a suspect estimate.
    """

    if not link.set_mode_guided():
        return False
    agl = vehicle.position.alt - home_alt_m
    if vehicle.armed and agl >= 3.0:
        return True

    if not vehicle.armed:
        if abs(agl) > ground_agl_tolerance_m:
            link.command_log.append((
                "takeoff_refused",
                {"reason": "grounded vehicle reports nonzero AGL",
                 "agl_m": round(agl, 2)},
            ))
            print(
                json.dumps({
                    "event": "flight.takeoff_refused",
                    "reason": "altitude sensor disagrees with ground",
                    "reported_agl_m": round(agl, 2),
                    "tolerance_m": ground_agl_tolerance_m,
                }),
                flush=True,
            )
            return False
        if not link.arm():
            return False
    start_agl = vehicle.position.alt - home_alt_m
    if not link.takeoff(target_agl_m):
        return False

    deadline = time.monotonic() + timeout_s
    climb_deadline = time.monotonic() + climb_check_s
    while time.monotonic() < deadline:
        agl = vehicle.position.alt - home_alt_m
        if agl >= target_agl_m - settle_m:
            return True
        if (
            time.monotonic() > climb_deadline
            and agl - start_agl < climb_check_min_gain_m
        ):
            print(
                json.dumps({
                    "event": "flight.climb_stall",
                    "reason": "no observed climb after takeoff — altitude "
                    "source suspect; aborting mission start",
                    "gain_m": round(agl - start_agl, 2),
                    "after_s": climb_check_s,
                }),
                flush=True,
            )
            return False
        time.sleep(0.5)
    return False


def _wait_position(link: Any, timeout_s: float, *, phase: str = ""):
    """Wait for the first position fix, nudging stream rates as needed.

    Over mavproxy-fanned UDP, telemetry already flows (mavproxy requested
    the streams). On a raw TCP serial connection to SITL or an autopilot,
    nothing streams until a client asks: `health_check()` publicly issues
    REQUEST_DATA_STREAM, so it doubles as the stream-rate nudge here.
    """

    deadline = time.monotonic() + timeout_s
    next_nudge = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_nudge and hasattr(link, "health_check"):
            try:
                link.health_check(timeout_s=0.2)
            except Exception:
                pass
            next_nudge = time.monotonic() + 3.0
        value = link.position()
        if value is not None:
            return value
        time.sleep(0.2)
    raise TimeoutError(
        f"no GLOBAL_POSITION_INT telemetry within {timeout_s:.0f}s"
        + (f" (phase: {phase})" if phase else "")
    )
