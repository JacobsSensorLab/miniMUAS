# Survey: UAS-IPBRC — flight/motion primitives to extract

Surveyed 2026-07-09 (source: ~/Documents/Dev/UAS-IPBRC). Input to the `muas-flight`
Rust library design in ARCHITECTURE.md.

## What it is

"UAS IP Broadcast Relay Chain" — a centrally-coordinated wfb-ng radio relay chain
for UAS backhaul: relay drones position themselves as an airborne mesh backbone
between GCS and a surveyor. Python 3.12, pyproject + Nix flake, pymavlink, ~310
pytest tests. Same code runs in `sim`, `sitl`, `real` modes.

Layered explicitly for reuse (`docs/flight-primitives.md` is the design bible):
transport adapter → vehicle command adapter → primitive → task → mission.
**`relay/flight/` is deliberately above MAVLink and below missions** — it emits
intent (`FlightCommand`), never touches MAVLink, unit-testable without an
autopilot. That layer is the port target.

## Primitive inventory (all under `relay/flight/` unless noted)

- **Core intent model** — `primitives.py`: `FlightPrimitive` protocol with
  `.capabilities` and `.tick(ctx) -> FlightStep`; `FlightStep` = progress +
  emitted commands + events + blackboard `state_updates`. `FlightStatus`
  (running/succeeded/failed/canceled/blocked). `StandardCommand`: ARM, TAKEOFF,
  SET_SPEED, GOTO, HOLD, RTL, LAND, ORBIT (open string vocabulary).
  `Constraint` protocol + `apply_constraints()` pipeline.
- **Motion** — `motion.py`: `FlyTo`, `FlyPath` (once/loop/ping_pong),
  `HoldPosition`, `ChangeAltitude`, `Land`, `ReturnToLaunch`; `MotionTarget`
  (position, frame, ENU velocity, acceptance radius, yaw behavior, speed);
  `YawBehavior` (UNCHANGED/FACE_TRAVEL/FACE_TARGET/FIXED). Velocity rides on
  goto targets — no standalone attitude/rate control exists.
- **Lifecycle** — `lifecycle.py`: `StartupLaunch`, `LaunchIfNeeded`,
  `resume_flight_state` (crash-recovery reconstruction), `LaunchLedger`,
  `RoleLaunchProfile`.
- **Orbit** — `orbit.py`: `Orbit` (native circle mode, time-based completion) +
  `plan_orbit` capability ladder: CIRCLE_MODE → GUIDED_YAW_PATH →
  GUIDED_POSITION_ONLY → REJECT, falling back to `InspectPoint` waypoint circle.
  Encodes the v1 post-mortem (MAV_CMD_DO_ORBIT assumption that ArduPilot broke).
- **Patterns** — `patterns.py`: `OrbitPath`/`orbit_targets`,
  `RasterPath`/`raster_targets` (boustrophedon with lane metadata).
- **Tasks** — `inspection.py` (`InspectPoint`), `survey.py` (`RasterSurvey`,
  `RasterUntilCondition` + `SurveyRepeatPolicy`), `tasks.py` (task envelope,
  `drive_primitive`).
- **Composition** — `sequence.py`: `Sequence` with namespaced child blackboard
  state; `merge_capability_requirements()`.
- **Runner** — `runtime.py`: `FlightPrimitiveRunner` single-tick loop — merge
  blackboard, cancel/deadline halts, capability BLOCKED check, tick, apply
  constraints, execute accepted commands.
- **Constraints** — `constraints.py`: `AltitudeEnvelopeConstraint`,
  `HorizontalRadiusConstraint`, `CommandKindConstraint`; blocked → event
  `flight.constraint_blocked`, clamped → `flight.constraint_adjusted`.
  Battery/link-health/keep-out/operator-override documented but absent.
- **Deconfliction (pure math, strongest reuse)** — `deconflict.py`: CPA
  (`closest_point_of_approach`), `DeconflictionEnvelope` with hysteresis,
  `peer_poll_interval_s` (attention scheduling), `cooperative_plan`
  (deterministic symmetric vertical avoidance, floor-aware, quantized
  tie-break by id), `uncooperative_plan`, `rtl_altitude_slots`.
- **Placement/follow (swarm motion)** — `relay/core/placement.py`:
  `plan_placement()` — mast primitive, uniform spacing along GCS→mast line,
  velocity feed-forward + lookahead lead, keep-out radius, grow/shrink flags.
  Plus `relay/core/relay_recall.py`, `relay/core/geo.py` (flat-earth ENU).
- **Serialization** — `serialization.py`: JSON round-trip of all progress
  dataclasses for resume/replay.
- **Execution bridge** — `execution.py`: `FlightCommandExecutor` handler
  registry; `FlightCommandLink` protocol (adapter surface).

## MAVLink adapter (reference for the Rust adapter, mine for workarounds)

`relay/drone/mavlink.py` (`MavlinkDroneLink`), behind `relay/core/drone.py`
`DroneLink` ABC — the clean seam to reimplement. ArduPilot/ArduCopter semantics
(mode ints 3/4/6/9 hardcoded; PX4 name-only). Field-hardened behaviors to keep:

- HEARTBEAT filtering strictly by `MAV_COMP_ID_AUTOPILOT1` (real field bug:
  sysid flapped to 255 from mavp2p/GCS heartbeats).
- 1 Hz GCS-heartbeat thread so `FS_GCS` failsafe doesn't misfire.
- `_takeoff_in_progress` latch — first position target cancels ArduCopter's
  GUIDED takeoff sub-state, so goto is suppressed until off the ground
  (same family of bug as our 3.5 m goto floor).
- ASL ↔ relative-alt frame translation via `home_alt_m`.
- Arm/mode-set retry state machines with heartbeat confirmation.
- goto via `SET_POSITION_TARGET_GLOBAL_INT` with type_mask for pos / pos+vel /
  yaw; COMMAND_LONG ARM(400)/TAKEOFF(22)/LAND(21)/DO_CHANGE_SPEED(178).

## Upgrade opportunities (port = refactor)

1. Replace duck-typing (`inspect.signature` capability sniffing, untyped
   `FlightContext.vehicles: Mapping[str, object]`, `Mapping[str, object]`
   command params) with typed Rust: `FlightCommand` enum with typed variants +
   `Custom(String, Value)` escape hatch; one `VehicleSnapshot` type; explicit
   `Capabilities` bitflags; `VehicleAdapter` trait.
2. No velocity/attitude/rate control and no trajectory generation or
   obstacle-aware planning exist — decide early if v3 needs them (new
   subsystems, not ports). Deconfliction is vertical-only.
3. Orbit completion is time-based — close the loop on accumulated bearing from
   telemetry instead.
4. Constraint pipeline is minimal (sequential, no priorities) — build a real
   constraint/arbitration layer with battery, link-health, keep-out, operator
   override.
5. Flat-earth geo (valid <~km) — isolate behind a frame trait so a geodesic
   implementation can swap in.
6. Keep radio/chain machinery (`core/chain_*`, `core/radio_*`, `agent/`) out of
   the flight crate — orthogonal.
7. Use a typed MAVLink dialect crate (e.g. `mavlink`/`mavio`) + an autopilot
   mode-map abstraction so PX4 can be added without rewrites.

## Port order (highest value, lowest risk first)

1. `deconflict.py` (pure math) → nearly 1:1.
2. `core/geo.py` + `patterns.py` (generators).
3. `core/placement.py` (mast/follow/lead).
4. `primitives.py` + `motion.py` + `sequence.py` + `runtime.py` (tick core),
   redesigned around typed enums.
5. `lifecycle.py`, `orbit.py` mode ladder, `constraints.py`.
6. Rust MAVLink adapter, mining `drone/mavlink.py` for the workarounds above.

**Porting oracle:** `tests/test_flight_*.py` (motion, orbit, deconflict,
sequence, lifecycle, constraints, survey, patterns, primitives, runtime,
serialization, terminal, capabilities, tasks) — tick primitives without
MAVLink; translate directly into Rust unit tests.
