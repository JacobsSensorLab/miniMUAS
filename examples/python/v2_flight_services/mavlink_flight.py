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
from collections import deque
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
        #: Why the last `ensure_airborne` refused or failed ("" = it did
        #: not). Callers surface this instead of a generic "could not get
        #: airborne" so the operator sees the real gate.
        self.last_failure: str = ""
        #: The autopilot's own STATUSTEXTs, (unix time, severity, text).
        #: ArduCopter says WHY it refuses to arm only here ("PreArm: ...",
        #: "Arm: ..."); nothing kept them, so on 2026-10-13 wuas-01 refused
        #: six takeoffs (15:38-15:53) with no reason anywhere.
        self._fc_text: deque = deque(maxlen=64)
        self._fc_text_logged: dict[str, float] = {}
        #: HEARTBEAT.system_status of the FC (MAV_STATE_*; None = unseen).
        #: CRITICAL (5) = a failsafe is active: MavlinkDroneLink.arm then
        #: gives up without sending the arm command at all.
        self.fc_system_status: int | None = None
        conn = getattr(inner, "_conn", None)
        hooks = getattr(conn, "message_hooks", None)
        if hooks is not None:
            # pymavlink runs message hooks for every message ANY thread
            # reads, so no STATUSTEXT is lost to the competing drains
            hooks.append(self._on_message)

    def _on_message(self, conn: Any, msg: Any) -> None:
        try:
            if msg.get_srcSystem() != getattr(conn, "target_system", None):
                return
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                if msg.get_srcComponent() == 1:  # MAV_COMP_ID_AUTOPILOT1
                    self.fc_system_status = int(msg.system_status)
                return
            if kind != "STATUSTEXT":
                return
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("ascii", errors="replace")
            text = str(text).rstrip("\x00").strip()
            severity = int(msg.severity)
        except Exception:
            return
        now = time.time()
        self._fc_text.append((now, severity, text))
        # journal warnings and arming verdicts; ArduCopter repeats a failing
        # PreArm every 30 s while disarmed, so log each text once a minute
        if severity <= 4 or text.startswith(("PreArm", "Arm")):
            if now - self._fc_text_logged.get(text, 0.0) >= 60.0:
                self._fc_text_logged[text] = now
                print(
                    json.dumps({"event": "flight.fc_text",
                                "severity": severity, "text": text}),
                    flush=True,
                )

    def fc_reason(self, since: float) -> str:
        """The autopilot's latest arming verdict or warning since `since`
        (unix time), "" if it said nothing relevant."""
        arming = warning = ""
        for t, severity, text in list(self._fc_text):
            if t < since:
                continue
            if text.startswith(("PreArm", "Arm")):
                arming = text
            elif severity <= 4:
                warning = text
        return arming or warning

    def _acked_since(self, cmd_id: int, sent_at: float) -> bool:
        """True if the autopilot ACCEPTED `cmd_id` at or after `sent_at`.

        MavlinkDroneLink waits for COMMAND_ACK with `recv_match`, but the
        agent's telemetry, PeerGuard and watchpoint threads drain the SAME
        pymavlink connection (`position()` -> `_drain()`), and whichever
        thread reads the ACK first consumes it. pymavlink still caches the
        last message of every type in `conn.messages` no matter which
        thread read it, so a "timed out" command can be re-judged here.
        Field 2026-10-13 (wuas-01 16:37:35 and 16:47:10 CDT): NAV_TAKEOFF
        "failed" exactly 3 s (the ACK timeout) after arming while the
        vehicle climbed to the commanded altitude and hovered.
        """
        conn = getattr(self._inner, "_conn", None)
        messages = getattr(conn, "messages", None) or {}
        msg = messages.get("COMMAND_ACK")
        if msg is None:
            return False
        try:
            return (
                int(msg.command) == int(cmd_id)
                and float(getattr(msg, "_timestamp", 0.0) or 0.0) >= sent_at
                and int(msg.result) == 0  # MAV_RESULT_ACCEPTED
            )
        except (TypeError, ValueError):
            return False

    def _command(self, name: str, cmd_id: int, send, fields: dict) -> bool:
        sent_at = time.time()
        ok = bool(send())
        recovered = False
        if not ok and self._acked_since(cmd_id, sent_at):
            ok = recovered = True
        entry = {**fields, "ok": ok}
        if recovered:
            entry["ack"] = "recovered-from-cache"
            print(
                json.dumps({
                    "event": "flight.ack_recovered",
                    "command": name,
                    "cmd_id": cmd_id,
                    "reason": "COMMAND_ACK consumed by another reader of "
                    "the shared MAVLink connection",
                }),
                flush=True,
            )
        self.command_log.append((name, entry))
        return ok

    # -- FlightCommandLink surface ------------------------------------

    def arm(self) -> bool:
        return self._command("arm", 400, self._inner.arm, {})

    def disarm(self) -> bool:
        """MAV_CMD_COMPONENT_ARM_DISARM(0) through the inner link's private
        command path (MavlinkDroneLink exposes no public disarm)."""
        send_cmd = getattr(self._inner, "_send_cmd_long", None)
        if send_cmd is None:
            return False
        return self._command(
            "disarm", 400,
            lambda: send_cmd(400, params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            {},
        )

    def takeoff(self, alt_m: float) -> bool:
        return self._command(
            "takeoff", 22, lambda: self._inner.takeoff(float(alt_m)),
            {"alt_m": float(alt_m)},
        )

    def set_cruise_speed_m_s(self, speed_m_s: float) -> bool:
        return self._command(
            "set_speed", 178,
            lambda: self._inner.set_cruise_speed_m_s(float(speed_m_s)),
            {"speed_m_s": float(speed_m_s)},
        )

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


def _airborne_failed(link: LoggingFlightLink, stage: str, reason: str,
                     *, since: float, **fields: object) -> bool:
    """Record why ensure_airborne gave up (command_log + journal) -> False.

    Every exit used to be a bare `return False` that the agent reported as
    "could not reach search altitude" — on 2026-10-13 the operator saw a
    drone take off and hover with no way to tell what had failed. The
    autopilot's own verdict since `since` (its "PreArm: ..."/"Arm: ..."
    STATUSTEXT) is appended: that is the only place it says why.
    """
    fc_reason = getattr(link, "fc_reason", None)
    fc = fc_reason(since) if fc_reason is not None else ""
    if fc:
        reason = f"{reason} — FC: {fc}"
    link.last_failure = f"{stage}: {reason}"
    link.command_log.append(
        ("takeoff_refused", {"stage": stage, "reason": reason, **fields})
    )
    print(
        json.dumps({"event": "flight.takeoff_refused", "stage": stage,
                    "reason": reason, **fields}),
        flush=True,
    )
    return False


def _wait_for(predicate, timeout_s: float, poll_s: float = 0.2) -> bool:
    deadline = time.monotonic() + timeout_s
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_s)


def ensure_airborne(
    link: LoggingFlightLink,
    vehicle: LinkVehicle,
    *,
    target_agl_m: float,
    home_alt_m: float,
    timeout_s: float = 120.0,
    settle_m: float = 2.0,
    ground_agl_tolerance_m: float = 1.5,
    confirm_s: float = 3.0,
    climb_check_s: float = 12.0,
    climb_check_min_gain_m: float = 1.0,
) -> bool:
    """Arm and take off to `target_agl_m` if not already flying.

    Idempotent: an armed vehicle meaningfully off the ground is left
    alone — the plan's own ChangeAltitude step handles altitude from
    there. GUIDED is forced up front (no-op when already there): a
    vehicle that finished a previous flight sits in RTL or LAND, where
    ArduCopter rejects arming and ignores guided targets.

    Success and failure are judged from TELEMETRY, not from COMMAND_ACKs:
    the ACKs race the agent's other readers of the shared connection (see
    LoggingFlightLink._acked_since). A missing ARM ack is re-checked
    against the heartbeat's armed flag; a missing NAV_TAKEOFF ack falls
    through to the climb check, which is the real proof either way.

    Altitude-source sanity (fleet has flaky barometers; a past crash hit
    the ground while reported AGL was still positive):

      * ground check — just after arming the vehicle is on the ground and
        ArduCopter has re-zeroed home altitude (AP_Arming_Copter::arm
        resets home to the current location unless home is locked), so
        relative AGL must read ~0. If it reads beyond
        `ground_agl_tolerance_m` the AGL frame every later command uses is
        biased: disarm and refuse. Checking BEFORE arming refused good
        vehicles: disarmed baro drift read 1.53-2.27 m on the 2026-10-13
        field day (iuas-01/iuas-02 "could not get airborne"), and that
        drift is exactly what the arming home-reset removes.
      * climb check — after NAV_TAKEOFF, if reported AGL hasn't gained
        `climb_check_min_gain_m` within `climb_check_s`, either the
        vehicle isn't climbing or the altitude source is stuck; abort
        the mission start rather than fly on a suspect estimate (and
        disarm if it never left the ground).

    On False, `link.last_failure` says which gate failed.
    """

    link.last_failure = ""
    # ArduCopter repeats a failing PreArm every 30 s while disarmed: look
    # back far enough to catch the last report even if the refusal itself
    # sent none
    since = time.time() - 35.0

    def agl_now() -> float:
        return vehicle.position.alt - home_alt_m

    if not link.set_mode_guided():
        return _airborne_failed(
            link, "mode", "autopilot did not confirm GUIDED", since=since,
        )
    agl = agl_now()
    if vehicle.armed and agl >= 3.0:
        return True

    if not vehicle.armed:
        disarmed_agl = agl
        if not link.arm() and not _wait_for(lambda: vehicle.armed, confirm_s):
            status = getattr(link, "fc_system_status", None)
            if status in (5, 6):  # MAV_STATE_CRITICAL / EMERGENCY
                why = ("autopilot reports a failsafe (MAV_STATE_"
                       + ("CRITICAL" if status == 5 else "EMERGENCY")
                       + "); arming not attempted")
            else:
                why = "autopilot did not arm (pre-arm checks or arming refused)"
            return _airborne_failed(
                link, "arm", why, since=since, fc_system_status=status,
            )
        since = time.time() - 1.0  # armed: older PreArm texts are stale
        # ArduCopter re-zeroes home altitude at arming; give the next
        # GLOBAL_POSITION_INT a moment to carry the reset before judging.
        if not _wait_for(
            lambda: abs(agl_now()) <= ground_agl_tolerance_m, confirm_s
        ):
            agl = agl_now()
            link.disarm()
            return _airborne_failed(
                link, "ground_agl",
                "altitude sensor disagrees with ground after arming",
                since=since,
                agl_m=round(agl, 2), disarmed_agl_m=round(disarmed_agl, 2),
                tolerance_m=ground_agl_tolerance_m,
            )
    start_agl = agl_now()
    if not link.takeoff(target_agl_m):
        # No ACK is not a refusal: let the climb check decide from telemetry.
        print(
            json.dumps({
                "event": "flight.takeoff_unacked",
                "note": "NAV_TAKEOFF not acknowledged; judging the climb "
                "from telemetry",
            }),
            flush=True,
        )

    deadline = time.monotonic() + timeout_s
    climb_deadline = time.monotonic() + climb_check_s
    while time.monotonic() < deadline:
        agl = agl_now()
        if agl >= target_agl_m - settle_m:
            return True
        if (
            time.monotonic() > climb_deadline
            and agl - start_agl < climb_check_min_gain_m
        ):
            if agl < 1.0:
                # still on the ground: don't leave it armed in GUIDED
                # (DISARM_DELAY may be 0, so the FC won't disarm itself)
                link.disarm()
            return _airborne_failed(
                link, "climb",
                "no observed climb after takeoff (takeoff refused or "
                "altitude source stuck)", since=since,
                gain_m=round(agl - start_agl, 2), after_s=climb_check_s,
            )
        time.sleep(0.5)
    return _airborne_failed(
        link, "timeout", f"did not reach {target_agl_m:.1f} m AGL",
        since=since, agl_m=round(agl_now(), 2), after_s=timeout_s,
    )


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
