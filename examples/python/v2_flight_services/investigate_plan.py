"""Compile and execute `InvestigatePointRequest` with relay.flight primitives.

This module is the missing middle of the miniMUAS v2 slice: it turns the
NDNSF-facing task contract into an actual primitive plan and runs it to a
terminal status, instead of fabricating a `FlightTaskResult`.

The plan shape follows `docs/v2_flight_services.md`:

    climb to approach altitude
    -> fly to a standoff point near the target
    -> orbit the target (native circle mode when the vehicle advertises it,
       otherwise a guided yaw-facing or position-only waypoint path)
    -> emit the sensor-plan capture command

`plan_orbit` performs the capability mapping, so the execution mode reported
back to the requester (`circle-mode`, `guided-yaw-path`,
`guided-position-only`, or `reject`) reflects what actually ran.

The flight library lives in the UAS-IPBRC repository (`relay.flight`). Point
`UAS_IPBRC_ROOT` at a checkout, or pass `--uas-ipbrc-root` to the role
scripts. Execution here runs against an in-process `SimFlightLink`; swapping
in a MAVLink-backed link with the same method surface is the SITL/hardware
step and requires no changes to this module's plan compilation.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
import os
from pathlib import Path
import sys
import time
from typing import Any

from contracts import (
    FlightTaskResult as WireFlightTaskResult,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    SensorArtifact,
    gps_time_ns,
    mission_sensor_name,
)
from dataplane import synthetic_frame_bytes


DEFAULT_UAS_IPBRC_ROOT = Path(
    os.environ.get("UAS_IPBRC_ROOT", "~/Documents/Dev/UAS-IPBRC")
).expanduser()

CAPTURE_COMMAND_KIND = "capture_still"
DEFAULT_ORBIT_SPEED_M_S = 3.0

# Flat-earth constant shared with run_drone_agent (v2 uses 111 111 m/deg).
_EARTH_M_PER_DEG_LAT = 111_111.0
# Approach/climb-out standoff as a multiple of the dip radius: the vehicle
# lines up 3R behind the target and climbs out 3R past it, so the low pass
# is a straight run through the target rather than a hover-and-drop. Mirror
# of v3 uas-flight `FLYOVER_APPROACH_FACTOR` (patterns.rs).
FLYOVER_APPROACH_FACTOR = 3.0


@dataclass(frozen=True)
class FlyoverWaypoint:
    """One waypoint of the acoustic dip-flyover profile.

    Tagged with its `phase` (approach / dip_entry / dip_center / dip_exit /
    climb_out), the `pass_index` it belongs to, and the compass `bearing`
    of that pass's run — parity with v3 uas-flight `flyover_targets`, whose
    MotionTargets carry the same phase/pass/bearing tags.
    """

    lat_deg: float
    lon_deg: float
    agl_m: float
    phase: str
    pass_index: int
    bearing_deg: float


def _offset_point(
    lat: float, lon: float, along_m: float, bearing_deg: float
) -> tuple[float, float]:
    """Point `along_m` metres from (lat, lon) along `bearing_deg` (compass)."""

    import math

    theta = math.radians(bearing_deg)
    dn = along_m * math.cos(theta)
    de = along_m * math.sin(theta)
    m_per_deg_lon = _EARTH_M_PER_DEG_LAT * max(math.cos(math.radians(lat)), 1e-6)
    return (
        lat + dn / _EARTH_M_PER_DEG_LAT,
        lon + de / m_per_deg_lon,
    )


def build_flyover_waypoints(
    *,
    target_lat: float,
    target_lon: float,
    approach_bearing_deg: float,
    cruise_agl_m: float,
    dip_agl_m: float,
    radius_m: float,
    passes: int,
) -> list[FlyoverWaypoint]:
    """Acoustic dip-flyover waypoint list (backport of v3 `flyover_targets`).

    Each pass is five waypoints along its run bearing: approach (-3R, cruise
    AGL) -> dip_entry (-R, dip AGL) -> dip_center (0, directly over the
    target, dip AGL, the lowest and only capture point) -> dip_exit (+R, dip
    AGL) -> climb_out (+3R, cruise AGL). `passes` repeats the profile, each
    pass rotated 90° (`cross_axis`) from the previous so an omnidirectional
    mic hears the target from crossed baselines. The dip AGL is the caller's
    commandable floor, so the pass is as low as the fleet guard allows.
    """

    radius = max(float(radius_m), 2.0)
    n = max(int(passes), 1)
    stand = FLYOVER_APPROACH_FACTOR * radius
    # (along-track offset metres, AGL, phase) for one pass, front to back.
    legs = [
        (-stand, cruise_agl_m, "approach"),
        (-radius, dip_agl_m, "dip_entry"),
        (0.0, dip_agl_m, "dip_center"),
        (radius, dip_agl_m, "dip_exit"),
        (stand, cruise_agl_m, "climb_out"),
    ]
    waypoints: list[FlyoverWaypoint] = []
    for p in range(n):
        bearing = (float(approach_bearing_deg) + 90.0 * p) % 360.0
        for along_m, agl, phase in legs:
            lat, lon = _offset_point(target_lat, target_lon, along_m, bearing)
            waypoints.append(
                FlyoverWaypoint(
                    lat_deg=lat,
                    lon_deg=lon,
                    agl_m=float(agl),
                    phase=phase,
                    pass_index=p,
                    bearing_deg=bearing,
                )
            )
    return waypoints


def add_flight_path(root: Path | None = None) -> None:
    """Make `relay.flight` importable from a UAS-IPBRC checkout."""

    resolved = (root or DEFAULT_UAS_IPBRC_ROOT).expanduser().resolve()
    if not (resolved / "relay" / "flight").exists():
        raise RuntimeError(f"relay.flight not found under: {resolved}")
    root_str = str(resolved)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)


@dataclass
class SimVehicle:
    """Minimal vehicle snapshot the flight primitives can read."""

    id: str
    position: Any
    armed: bool = True


class SimFlightLink:
    """Instant-motion `FlightCommandLink` for contract-level execution.

    `goto` teleports the vehicle to the commanded point so each motion
    primitive completes on its next tick. `orbit` accepts the native command
    and reports success; the `Orbit` primitive's simulated-time completion
    does the rest. Replace this with a MAVLink-backed link exposing the same
    methods to fly the identical plan on SITL or hardware.
    """

    def __init__(self, vehicle: SimVehicle, position_type: type) -> None:
        self._vehicle = vehicle
        self._position_type = position_type
        self.command_log: list[tuple[str, dict[str, object]]] = []

    def arm(self) -> bool:
        self._vehicle.armed = True
        self.command_log.append(("arm", {}))
        return True

    def takeoff(self, alt_m: float) -> bool:
        position = self._vehicle.position
        self._vehicle.position = self._position_type(
            position.lat, position.lon, alt_m
        )
        self.command_log.append(("takeoff", {"alt_m": alt_m}))
        return True

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
        del vel_n_m_s, vel_e_m_s, vel_u_m_s, yaw_rate_deg_s
        self._vehicle.position = self._position_type(lat, lon, alt_m)
        self.command_log.append(
            ("goto", {"lat": lat, "lon": lon, "alt_m": alt_m, "yaw_deg": yaw_deg})
        )

    def orbit(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        radius_m: float,
        *,
        turns: float = 1.0,
        speed_m_s: float | None = None,
        clockwise: bool = True,
    ) -> bool:
        self.command_log.append(
            (
                "orbit",
                {
                    "lat": lat,
                    "lon": lon,
                    "alt_m": alt_m,
                    "radius_m": radius_m,
                    "turns": turns,
                    "speed_m_s": speed_m_s,
                    "clockwise": clockwise,
                },
            )
        )
        return True

    def land(self) -> bool:
        self._vehicle.armed = False
        self.command_log.append(("land", {}))
        return True

    def rtl(self) -> bool:
        self.command_log.append(("rtl", {}))
        return True


@dataclass(frozen=True)
class CompiledInvestigation:
    """Result of compiling one investigation request."""

    primitive: object | None
    mode: str
    reason: str | None = None

    @property
    def rejected(self) -> bool:
        return self.primitive is None


@dataclass(frozen=True)
class InvestigationOutcome:
    """Terminal record for one executed (or rejected) investigation."""

    result: WireFlightTaskResult
    mode: str
    event_names: tuple[str, ...] = ()
    command_log: tuple[tuple[str, dict[str, object]], ...] = ()
    artifact_payloads: tuple[bytes, ...] = ()

    @property
    def ok(self) -> bool:
        return self.result.status == "completed"


def default_capability_profile(*, native_orbit: bool = True):
    """Capability profile for the simulated IUAS vehicle."""

    from relay.flight import FlightCapabilityProfile

    profile = FlightCapabilityProfile(
        position=True,
        mode_control=True,
        yaw_control=True,
    )
    if native_orbit:
        profile = profile.with_extra("orbit")
    return profile


@dataclass(frozen=True)
class _EmitCommandOnce:
    """Tiny primitive: emit one application command, then succeed.

    Demonstrates the open-ended command vocabulary; the runner's
    `FlightCommandExecutor` routes the custom kind to an application handler.
    """

    vehicle_id: str
    kind: str
    params: Mapping[str, object] = field(default_factory=dict)
    reason: str = "sensor_sample"

    @property
    def capabilities(self):
        from relay.flight import CapabilityRequirement

        return CapabilityRequirement(extras=frozenset({"sensor_sample"}))

    def tick(self, context):
        from relay.flight import (
            FlightCommand,
            FlightProgress,
            FlightStatus,
            FlightStep,
        )

        del context
        return FlightStep(
            progress=FlightProgress(
                FlightStatus.SUCCEEDED,
                "sensor command emitted",
            ),
            commands=(
                FlightCommand(
                    self.vehicle_id,
                    self.kind,
                    dict(self.params),
                    reason=self.reason,
                ),
            ),
        )


def compile_investigation(
    request: InvestigatePointRequest,
    *,
    vehicle_id: str,
    profile,
) -> CompiledInvestigation:
    """Compile a request into a primitive plan via the capability ladder."""

    from relay.core.geo import EARTH_M_PER_DEG_LAT, Position
    from relay.flight import (
        ChangeAltitude,
        FlyTo,
        MotionTarget,
        OrbitExecutionMode,
        Sequence,
        plan_orbit,
    )

    if request.approach_alt_m <= 0.0:
        return CompiledInvestigation(
            None,
            OrbitExecutionMode.REJECT,
            "approach altitude must be positive",
        )

    target = Position(
        lat=request.target.lat_deg,
        lon=request.target.lon_deg,
        alt=request.target.alt_m or 0.0,
    )
    speed_m_s = request.constraints.max_speed_mps or DEFAULT_ORBIT_SPEED_M_S

    orbit_plan = plan_orbit(
        vehicle_id=vehicle_id,
        center=target,
        radius_m=request.circle_radius_m,
        profile=profile,
        turns=request.circle_count,
        altitude_m=request.approach_alt_m,
        speed_m_s=speed_m_s,
        reason="investigate_orbit",
        state_key="investigate_orbit_progress",
    )
    if orbit_plan.rejected:
        return CompiledInvestigation(None, orbit_plan.mode, orbit_plan.reason)

    standoff_point = Position(
        lat=target.lat + request.standoff_m / EARTH_M_PER_DEG_LAT,
        lon=target.lon,
        alt=request.approach_alt_m,
    )
    steps = (
        ChangeAltitude(
            vehicle_id,
            target_alt_m=request.approach_alt_m,
            acceptance_radius_m=2.0,
            reason="investigate_climb",
        ),
        FlyTo(
            vehicle_id,
            MotionTarget(
                standoff_point,
                acceptance_radius_m=2.0,
                speed_m_s=speed_m_s,
            ),
            reason="investigate_approach",
        ),
        orbit_plan.primitive,
        _EmitCommandOnce(
            vehicle_id,
            CAPTURE_COMMAND_KIND,
            params={"sensor_plan": list(request.sensor_plan)},
        ),
    )
    return CompiledInvestigation(
        Sequence(steps=steps, state_key="investigate_sequence"),
        orbit_plan.mode,
    )


def execute_investigation(
    request: InvestigatePointRequest,
    *,
    vehicle_id: str = "iuas-01",
    native_orbit: bool = True,
    sensor_id: str = "front",
    tick_dt_s: float = 0.25,
    max_ticks: int = 4000,
    uas_ipbrc_root: Path | None = None,
    link: Any | None = None,
    vehicle: Any | None = None,
    profile: Any | None = None,
    realtime: bool = False,
    frame_source: Any | None = None,
) -> InvestigationOutcome:
    """Run the compiled plan to a terminal status.

    Default: an in-process simulated vehicle on simulated time. Passing a
    real `link` + `vehicle` pair (see `mavlink_flight.connect_flight_link`)
    with `realtime=True` flies the identical plan against an autopilot:
    contexts then carry wall-clock time and live telemetry, with
    `tick_dt_s` as the control period and `max_ticks * tick_dt_s` as the
    wall-time budget. `profile` overrides the default capability profile;
    supply it whenever supplying a link so the orbit ladder matches what
    the link can actually do. `frame_source` (see `camera.py`) supplies
    capture-artifact payloads; default is the synthetic pattern.
    """

    add_flight_path(uas_ipbrc_root)

    from relay.core.geo import EARTH_M_PER_DEG_LAT, Position
    from relay.flight import (
        AltitudeEnvelopeConstraint,
        FlightCommandExecutor,
        FlightCommandResult,
        FlightCommandResultStatus,
        FlightContext,
        FlightPrimitiveRunner,
        FlightStatus,
        task_result_from_run,
    )

    started_ns = gps_time_ns()
    if profile is None:
        profile = default_capability_profile(native_orbit=native_orbit)
    compiled = compile_investigation(
        request,
        vehicle_id=vehicle_id,
        profile=profile,
    )
    if compiled.rejected:
        completed_ns = gps_time_ns()
        return InvestigationOutcome(
            result=WireFlightTaskResult(
                task_id=_task_id(vehicle_id, request),
                status="rejected",
                started_at_gps_ns=started_ns,
                completed_at_gps_ns=completed_ns,
                artifacts=[],
                notes=compiled.reason or compiled.mode,
            ),
            mode=compiled.mode,
        )

    if link is None:
        # Vehicle starts standoff_m south of the target at ground level, armed.
        start_position = Position(
            lat=request.target.lat_deg - request.standoff_m / EARTH_M_PER_DEG_LAT,
            lon=request.target.lon_deg,
            alt=request.target.alt_m or 0.0,
        )
        vehicle = SimVehicle(id=vehicle_id, position=start_position)
        link = SimFlightLink(vehicle, position_type=Position)
    elif vehicle is None:
        raise ValueError("a vehicle must accompany an externally supplied link")

    artifacts: list[SensorArtifact] = []
    artifact_payloads: list[bytes] = []

    def capture_handler(command, capture_link) -> FlightCommandResult:
        del capture_link
        artifact_time = gps_time_ns()
        position = vehicle.position
        capture_kwargs = dict(
            mission_id=request.mission_id,
            vehicle_id=vehicle_id,
            sensor_id=sensor_id,
            gps_time_ns=artifact_time,
            metadata={
                "target_id": request.source_detection_id,
                "lat_deg": f"{position.lat:.8f}",
                "lon_deg": f"{position.lon:.8f}",
                "alt_m": f"{position.alt:.2f}",
            },
        )
        if frame_source is not None:
            artifact_payloads.append(frame_source.capture(**capture_kwargs))
        else:
            artifact_payloads.append(synthetic_frame_bytes(**capture_kwargs))
        artifacts.append(
            SensorArtifact(
                data_name=mission_sensor_name(
                    request.mission_id,
                    vehicle_id,
                    sensor_id,
                    "frame",
                    artifact_time,
                    len(artifacts) + 1,
                ),
                kind="image/jpeg",
                gps_time_ns=artifact_time,
                pose=Pose(
                    position=GeoPoint(
                        lat_deg=position.lat,
                        lon_deg=position.lon,
                        alt_m=position.alt,
                    ),
                    yaw_deg=None,
                ),
                metadata={"target_id": request.source_detection_id},
            )
        )
        return FlightCommandResult(
            command=command,
            status=FlightCommandResultStatus.SUCCEEDED,
            ok=True,
        )

    constraints = ()
    if request.constraints.min_clearance_m is not None:
        floor_alt = (request.target.alt_m or 0.0) + request.constraints.min_clearance_m
        constraints = (AltitudeEnvelopeConstraint(min_alt_m=floor_alt),)

    deadline_s: float | None = None
    if request.constraints.deadline_gps_ns is not None:
        # Map the remaining wall budget 1:1 onto the simulated clock.
        deadline_s = max(
            0.0,
            (request.constraints.deadline_gps_ns - started_ns) / 1e9,
        )

    runner = FlightPrimitiveRunner(
        primitive=compiled.primitive,
        constraints=constraints,
        capability_profiles={
            vehicle_id: profile.with_extra("sensor_sample"),
        },
        links={vehicle_id: link},
        command_executor=FlightCommandExecutor().with_handler(
            CAPTURE_COMMAND_KIND,
            capture_handler,
        ),
        deadline_s=deadline_s,
    )

    def contexts() -> Iterator[FlightContext]:
        if realtime:
            start_t = time.monotonic()
            for _ in range(max_ticks):
                yield FlightContext(
                    now_s=time.monotonic() - start_t,
                    vehicles={vehicle_id: vehicle},
                )
                time.sleep(tick_dt_s)
            return
        for tick in range(max_ticks):
            yield FlightContext(
                now_s=tick * tick_dt_s,
                vehicles={vehicle_id: vehicle},
            )

    # Tick to a terminal status, accumulating the full event log (the
    # library's drive_primitive only surfaces the final tick's events).
    run = None
    event_names: list[str] = []
    active_runner = runner
    for context in contexts():
        run = active_runner.tick(context)
        active_runner = run.runner
        event_names.extend(event.name for event in run.events)
        if run.progress.done:
            break
        if run.progress.status == FlightStatus.BLOCKED:
            break

    completed_ns = gps_time_ns()
    flight_result = task_result_from_run(
        task_id=_task_id(vehicle_id, request),
        run=run,
        started_at_s=0.0,
        completed_at_s=run.context.now_s if run is not None else 0.0,
        notes=compiled.mode,
    )

    notes = compiled.mode
    if flight_result.failure_reason:
        notes = f"{compiled.mode}: {flight_result.failure_reason}"

    return InvestigationOutcome(
        result=WireFlightTaskResult(
            task_id=flight_result.task_id,
            status=flight_result.status,
            started_at_gps_ns=started_ns,
            completed_at_gps_ns=completed_ns,
            artifacts=list(artifacts),
            notes=notes,
        ),
        mode=compiled.mode,
        event_names=tuple(event_names),
        command_log=tuple(getattr(link, "command_log", ()) or ()),
        artifact_payloads=tuple(artifact_payloads),
    )


def _task_id(vehicle_id: str, request: InvestigatePointRequest) -> str:
    return f"{vehicle_id}-investigate-{request.source_detection_id}"


def _smoke_test(argv: list[str]) -> int:
    """Offline check: compile and execute one sample investigation.

    Runs entirely in-process on simulated time (no NDN, no MAVLink):

        python investigate_plan.py
        python investigate_plan.py --no-native-orbit
    """

    import argparse
    from dataclasses import asdict
    import json

    from contracts import ConstraintSet

    parser = argparse.ArgumentParser(description=_smoke_test.__doc__)
    parser.add_argument("--uas-ipbrc-root", default=None)
    parser.add_argument(
        "--native-orbit",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = parser.parse_args(argv)

    request = InvestigatePointRequest(
        mission_id="m-smoke",
        source_detection_id="det-1",
        target=GeoPoint(lat_deg=35.0, lon_deg=-90.0, alt_m=0.0),
        approach_alt_m=25.0,
        standoff_m=40.0,
        circle_radius_m=8.0,
        circle_count=1.0,
        constraints=ConstraintSet(max_speed_mps=4.0, min_clearance_m=10.0),
    )
    outcome = execute_investigation(
        request,
        native_orbit=args.native_orbit,
        uas_ipbrc_root=(
            Path(args.uas_ipbrc_root) if args.uas_ipbrc_root else None
        ),
    )
    print(
        json.dumps(
            {
                "result": asdict(outcome.result),
                "mode": outcome.mode,
                "events": list(outcome.event_names),
                "link_commands": [name for name, _ in outcome.command_log],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if outcome.ok else 1


if __name__ == "__main__":
    raise SystemExit(_smoke_test(sys.argv[1:]))
