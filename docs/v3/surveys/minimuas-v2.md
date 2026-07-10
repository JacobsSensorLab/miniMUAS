# Survey: miniMUAS v2 — lossless feature inventory (parity checklist)

Surveyed 2026-07-09. This is the parity bible for the v3 port: every feature
here must exist in v3 (or be consciously superseded and noted in FEEDBACK.md).

## Where v2 lives

- v1 (C++/MAVSDK/CMake, `src/` + `config/app.yml` codegen catalog) is legacy —
  reference only.
- **v2 is `examples/python/v2_flight_services/` (~9,770 LOC Python)** on NDNSF's
  Python wrapper + pymavlink via UAS-IPBRC `relay.flight`, OpenCV/YOLO, aiohttp.
- Cross-repo dependency: v2 imports UAS-IPBRC `relay.flight` (Sequence,
  ChangeAltitude, FlyTo, MotionTarget, plan_orbit, FlightPrimitiveRunner,
  FlightCommandExecutor, AltitudeEnvelopeConstraint, FlightCapabilityProfile),
  `relay.drone.mavlink.MavlinkDroneLink`, `relay.core.geo`,
  `relay.flight.deconflict`. These become `muas-flight` in v3.

## Roles & choreography

GCS (mission authority + offboard detection + dashboard), WUAS (raster search),
IUAS (close inspection/orbit). Versioned task-intent services under
`/muas/v2/...` — the API exposes intent, not MAVLink; provider picks execution
mode (circle-mode / guided-yaw-path / guided-position-only / reject).
Flow: raster → frames published → GCS detects → confirm → IUAS investigate →
artifacts back → RTL.

## Feature checklist (v3 must cover all of this)

### Drone agent (`run_drone_agent.py`, 2,575 LOC)
- Services: `flight/rtl|land|hold` (abort flag, running task terminates ≤1
  cycle), `flight/takeoff` (guarded, AGL-clamped, occupies vehicle),
  `flight/raster-search` (WUAS), `flight/investigate` (IUAS — continuous
  **carrot-chasing orbit** `fly_orbit`, streamed guided targets, yaw at center,
  closed-loop on measured bearing), `sensor/capture` (modes now / override
  [fly-capture-resume; rejected mid-investigation] / opportunistic [watchpoint
  with radius + expiry]; `--audio-range-m` guard), `video/control` (MJPEG
  w/h/fps/q), `system/shutdown`.
- Background publications: `telemetry/live` 4 Hz, `telemetry/state`
  (CapabilityProfile), `search/status` 1 Hz, `video/live` latest-wins,
  `video/status`, `coord/status`.
- `fly_raster`: transit to leg START, captures fired by **along-track
  progress**, no waits at leg ends, **re-send position target every 2 s**,
  deadlines sized from commanded speed. (Fix for stop-and-go pitching.)
- Backends (common duck-typed surface): `SimFlightBackend` (kinematic, 0.2 s
  tick, AGL floor 0.5) and `MavlinkFlightBackend` (**all-AGL, pinned
  `home_alt_m=0`** — 2026-06-15 crash fix; AGL floor **3.5 m**; heading from
  GLOBAL_POSITION_INT.hdg with velocity-course fallback; roll/pitch attitude;
  down-facing rangefinder, −1 when absent; battery %; **goto lead capped ~15 m
  under active avoidance bias**).
- `CameraHub`: single reader thread owns V4L2 device; serves full-res captures
  + downscaled video; aspect-ratio preserving (nadir-projection bug fix).

### Fleet coordination ("separation by communication", `--fleet-ids`)
- **PeerGuard**: adaptive peer telemetry polling (`peer_poll_interval_s` —
  ~1/5 s distant, 2 Hz imminent), constant-velocity extrapolation, CPA vs
  `DeconflictionEnvelope` (hsep 8 m, vsep 4 m, horizon 20 s).
- **Cooperative avoidance**: deterministic symmetric pair plan, each side acts
  immediately + publishes `coord/status`; peer confirmation within 2.5 s grace
  (`coop-pending`→`coop`) else escalate to `unco` (whole burden upward);
  peer entry naming us is adopted before our own detector fires.
- **Altitude-bias overlay**: vertical bias via `set_alt_bias()` clamped
  −4..+8 m; conflicting plans → climbing wins (`max(ups)`); re-issues current
  target immediately; rides telemetry as `avoid_bias_m`, dashboard ⚠.
- **Smart RTL slots**: deterministic `rtl_altitude_slots` table (sorted fleet
  ids, `--rtl-sep-m` 3 above `--rtl-base-agl-m` 8); climb-in-place → cruise at
  slot → land. Fallback to native RTL when home unknown; **backs off if pilot
  leaves GUIDED** (RC always wins).
- **Fleet flight floor**: `floor_agl_m` = min_agl (3.5 on MAVLink) plumbed into
  cooperative planning — descender never asked below floor; climber absorbs
  shortfall. Must be identical fleet-wide.
- Transport injected (fetch/publish callables) → coord loop testable w/o NDN.

### Dashboard (`run_dashboard.py` 1,411 LOC + `dashboard.html` 1,374 LOC)
Backend: aiohttp at :8080, one WS for everything; NDNSF user `/muas/v2/gcs`
(~3 Hz pollers); detect→dispatch mission state machine (agents deliberately
have no brain). Frontend: hand-rolled `<canvas>` map (custom Web-Mercator/ENU),
embedded IBM Carbon g100 dark theme (offline), IBM Plex.

Feature-parity list:
- Map: pan/zoom, satellite tiles `/tiles/{z}/{x}/{y}` (ArcGIS, offline
  cache-first + proxy-cache, translucent veil, 10 m grid fallback), toggles
  imagery/events/data.
- Vehicle markers: heading triangle, **exp smoothing τ≈0.25 s position AND
  heading, shortest-arc easing**, velocity-course fallback, per-vehicle color,
  600-pt trail, stale-grey, AGL label.
- Vehicle tiles: mode (+⚠ bias), AGL (+rf), battery color-graded, armed,
  task/busy, source, clock skew Δ, link tag (green<4 s/yellow<10 s/red),
  sensor tags 📷🎙 (click → browse data), enable/disable toggle (disabled = no
  auto-dispatch/takeoff but RTL/Land/Hold still work), Takeoff (AGL field),
  RTL/Land/Hold, ⏻ power-off companion.
- Video panels: per-vehicle toggle → `video/control`; binary WS frames
  `[vehicleIndex][jpeg]`; fps/kbps/seq stats.
- Search editor: center mode + corners mode with map picking; raster params
  (AGL, spacing, capture-every, speed, object query, min conf, target sep);
  **Preview legs renders the exact RasterPlan flown** + duration.
- IUAS params: orbit AGL/radius/turns; per-sensor checkboxes (camera/audio).
- Mission controls: Start (only confirm dialog), RTL/Land/Hold ALL.
- Sensor tasking panel: vehicle/sensor/mode/duration, capture now, pick point.
- Detection panel: target cards (object_id, confidence, lat/lon, per-job
  sensor tags + artifact links), trigger-frame thumbnail.
- Event log: 250 lines, detect lines re-ordered into frame order, color-coded,
  click-to-locate; georeferenced events = fading map diamonds with tooltips;
  sensor points squares(camera)/circles(audio), hollow=mission/filled=tasked,
  click → playback modal (image `/artifact?name=`, audio playback).
- Mission banner: frames / leg / detects / targets.

### Mission replay & recorder
Backend records every WS broadcast (except binary video) to timestamped JSONL
(`--record-dir`, default /var/lib/minimuas/replays), flush-per-line + fsync
≤2 s. `/replays` index + named fetch. Frontend Replay: picker, resets UI, feeds
recorded messages through the **same dispatch() handlers**; transport bar
(play/pause/speed/scrub with deterministic seek by fast-dispatch-from-start).

### Power-loss-safe journals
Per-line flush+fsync JSONL of every print_json event (`--log-dir`, default
/var/lib/minimuas/log). Failures never kill the process. Flush pre-shutdown.

### Authorized companion shutdown
`system/shutdown`: request must carry `confirm == vehicle_id` (operator types
it); agent re-verifies AND refuses while armed or busy (at ack and handler).
On accept: flush journal, `os.sync()`, 3 s-delayed poweroff so the NDN
response lands first.

### MAVLink / SITL / ArduCopter
- `MavlinkDroneLink.goto()` suppresses position targets until 3 m off ground
  (guided-takeoff sub-state protection); agent enforces **min_agl 3.5 m**
  (0.5 sim), rejects out-of-range AGL at ack.
- `ensure_airborne`: force GUIDED before arm; ground check (disarmed + nonzero
  AGL → refuse); climb check (no altitude gain after NAV_TAKEOFF → abort).
- Request altitudes are AGL, rebased onto ground ASL.
- Diagnostics to keep: `probe_mavlink_stream.py`, `probe2_adapter_path.py`,
  `hitl_probe.py`, `run_sitl_investigation.py`.

### Detection & geo-projection
- YOLOv8 ONNX via cv2.dnn CPU (`--detector yolo:model.onnx?conf&iou&imgsz&
  classes`), letterbox+NMS, logs all pre-filter classes, annotated debug jpg;
  stub detector default.
- `project_ground`: full ray cast pixel→camera→body (mount yaw)→NED via
  roll/pitch/yaw **at capture** (pose frozen in frame metadata — NDN round-trip
  can't corrupt it), ground-plane intersect at AGL, horizon clamp; `offset_m`
  reported; dashboard prefers smallest-offset sighting. `--hfov-deg 70`,
  `--cam-yaw-offset-deg` calibration knob.

### Confirm-then-queue & multi-target dispatch
`--confirm-count 2` frames within `target_separation_m` before promotion
(false-positive guard); position from best-localized sighting; multi-target
multi-sensor: one job per requested sensor per target, dispatched to any idle
enabled capability-matching IUAS (`--iuas-ids` multiple); search continues
while IUAS queue drains; completion = raster done + nothing in flight + no
serviceable job.

### Field-safety rails
`--max-range-m 300`, `--max-agl-m 20`, busy guard, abort ladder (RC override →
per-vehicle RTL/Land/Hold → RTL ALL). Field doctrine: `docs/v2-field-day.md`
(incl. 2026-06-15 four-failure debrief).

### Capability telemetry & mode prediction
IUAS publishes CapabilityProfile (extras: orbit/camera/audio);
`expected_orbit_mode()` mirrors plan_orbit ladder; `mode_as_predicted`
reported. Note: NDNSF `RequestServiceTargeted` was evaluated and **rejected**
(no latency win, against data-centric design).

### Media & data-plane
- `dataplane.py`: `MUASFRAME1\n` self-describing container (JSON header with
  body_len/body_sha256 + opaque body); segmented publish/fetch over NDNSF
  SegmentedObjectProducer (signed, integrity-verified); LatestPublisher.
- `camera.py`: synthetic / file / opencv sources (V4L2, MJPG, shallow buffer +
  drain, by-id symlinks). `audio.py`: synthetic sine / alsa arecord.
- `raster.py`: pure serpentine geometry shared by agent AND dashboard preview
  (what you preview is what flies).
- `fetch_tiles.py`: bulk offline tile prefetch.

## NDN usage in v2

- NDNSF (NDN_Service_Framework) Python wrapper: ServiceProvider/ServiceUser/
  ServiceController/AckDecision/SegmentedObjectProducer/fetch_segmented_object.
  Stack: ndn-cxx fork, ndn-svs (group sync), NDNSD, NAC-ABE+OpenABE, NFD.
- Names (`contracts.py`): services `/muas/v2/<vid>/flight/<svc>`, gcs
  `perception/detect-object`, `sensor/capture`, `system/<action>`,
  `video/control`; latest-wins data telemetry/live|state, search/status,
  coord/status, sensor/last, video/live|status; mission objects
  `/muas/v2/mission/<mid>/<vid>/camera/<cam>/frame/<gps_ns>/<seq>` etc.
- **Group prefix `/muas/v2/group` MUST use NFD multicast strategy** (best-route
  silently breaks sync) — `ensure_multicast_strategy()` via nfdc.
- Authz: `config/v2_minimuas.policies` per-identity provider/user allow-lists;
  trust `config/trust-schema.conf` (hierarchical rules, trust-anchor any).

## Wireless

v2 is plain Wi-Fi AP/STA (GCS node 03 runs AP, drones associate; laptop joins
for :8080). No monitor-mode/named-data-radio code exists in v2 — that's
net-new for v3.

## Parity subtlety

The agent's IUAS path flies its own **continuous carrot-orbit** (`fly_orbit`),
NOT relay.flight's waypoint-ring `execute_investigation`. v3 must preserve the
streaming carrot-orbit as the real flight behavior and keep the plan_orbit
capability ladder as the mode-prediction model.
