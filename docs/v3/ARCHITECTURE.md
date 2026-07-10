# miniMUAS v3 — Architecture

Maps `VISION.md` requirements onto concrete design. Grounded in the four
surveys under `surveys/` (v2 parity bible, ndn-workspace/ndf-rs crate map,
UAS-IPBRC primitive inventory, NDF/waterline docs digest). Read those for the
underlying evidence; this doc is the decisions.

## Layering (mirrors the D-47 axiom)

```
 muas-cyd (ESP32)   muas-dashboard        muas-rc          field tools
        \                |                  /
         `------ muas-contracts (names, wire types, manifests, services)
                         |
   muas-agent ---- muas-flight (primitives) ---- muas-mavlink (vehicle adapter)
                         |
      ndf-apps / ndf-spark / ndf-core          (meaning of bytes)
                         |
      ndn-engine / ndn-service / faces / sync  (moving bytes)
```

Rule of thumb carried from UAS-IPBRC: **muas-flight never touches MAVLink or
NDN**; muas-mavlink never contains mission logic; muas-agent wires them.

## Workspace layout (repo root, branch v3)

```
Cargo.toml            # workspace
crates/
  muas-flight/        # flight/motion primitive library (the UAS-IPBRC extraction)
  muas-mavlink/       # MAVLink vehicle adapter (rust-mavlink; ArduPilot first)
  muas-contracts/     # names, wire types, L1 manifests, service definitions
  muas-agent/         # per-drone agent binary
  muas-dashboard/     # GCS console binary
  muas-rc/            # 1-to-many RC-over-NDN (M6)
  muas-sim/           # ndn-sim + SITL co-simulation scenarios & verification
```

Path deps assume sibling checkouts under `~/Documents/Dev/` (same convention
the refounding crates use):

```toml
ndf-core   = { path = "../ndf-rs/refounding/ndf-core" }
ndf-apps   = { path = "../ndf-rs/refounding/ndf-apps" }
ndf-spark  = { path = "../ndf-rs/refounding/ndf-spark" }
manifest        = { path = "../ndn-workspace/flotilla/crates/core/manifest" }
render-contract = { path = "../ndn-workspace/flotilla/crates/core/render-contract" }
ndn-engine       = { path = "../ndn-workspace/ndn-rs/crates/forwarding/ndn-engine" }
ndn-service-core = { path = "../ndn-workspace/ndn-ext/crates/service/ndn-service-core" }
ndn-sim          = { path = "../ndn-workspace/ndn-sim/crates/ndn-sim", features = ["mavlink"] }
```

Risk to watch: `refounding/` is "design-draft, not ratified" and pins
ndn-workspace HEAD `5798fa3f` — first build must verify the current checkout
still satisfies the path deps (log any breakage in FEEDBACK.md; that's exactly
the feedback they want).

## muas-flight — the primitive library (VISION §1)

Port of UAS-IPBRC `relay/flight` + `relay/core/{geo,placement}` +
miniMUAS-specific behaviors, refactored per the survey's upgrade list:

- `FlightCommand` = typed enum (`Arm`, `Takeoff{..}`, `Goto(MotionTarget)`,
  `Orbit(OrbitParams)`, `Rtl`, `Land`, `Hold`, `SetSpeed`, plus
  `Custom(String, serde_json::Value)` escape hatch).
- `FlightPrimitive` trait: `fn tick(&self, ctx: &FlightContext) -> FlightStep`;
  explicit serializable blackboard (progress structs), no hidden state — this
  is what makes journal/replay/resume work.
- One `VehicleSnapshot` type (v2 had two inconsistent ones); explicit
  `Capabilities` bitflags replacing duck-typed `inspect.signature` sniffing.
- Constraint layer upgraded: priorities + arbitration; ship
  AltitudeEnvelope/HorizontalRadius/CommandKind AND the documented-but-absent
  battery, link-health, keep-out, operator-override constraints.
- `deconflict` module ported near-1:1 (pure math): CPA, envelope hysteresis,
  attention scheduling, cooperative/uncooperative plans (floor-aware,
  quantized tie-break), rtl_altitude_slots.
- `patterns` (orbit ring, raster boustrophedon — shared by agent AND dashboard
  preview: what you preview is what flies), `placement` (mast/follow/lead).
- **Both orbit behaviors**: the plan_orbit capability ladder
  (CIRCLE_MODE → GUIDED_YAW_PATH → GUIDED_POSITION_ONLY → REJECT) for mode
  prediction, and v2's continuous carrot-chasing orbit as the real IUAS flight
  behavior. Upgrade: close orbit completion on accumulated bearing, not time.
- Geo isolated behind a frame trait (flat-earth ENU first, geodesic swappable).
- Porting oracle: translate UAS-IPBRC `tests/test_flight_*.py` into Rust unit
  tests as each module lands.
- Every primitive tick and constraint decision is a `tracing` span/event.

## muas-mavlink — vehicle adapter (VISION §1 "MAVLink everywhere")

`rust-mavlink` (or `mavio`) with a typed ArduPilot mode map (PX4 addable).
Reimplements the field-hardened v2/IPBRC behaviors as explicit, tested logic:

- HEARTBEAT filtering by autopilot component id; pinned target sys/comp.
- 1 Hz GCS-heartbeat task (FS_GCS guard).
- Takeoff latch (no position targets until off ground) + the **3.5 m AGL
  command floor**; all-AGL frame pinned to `home_alt_m = 0`.
- `ensure_airborne` ladder: force GUIDED pre-arm, ground check, climb check.
- Arm/mode-set retry state machines with heartbeat confirmation.
- Goto with pos/pos+vel/yaw type masks; **lead cap (~15 m) under active
  avoidance bias**.
- Heading with velocity-course fallback; rangefinder; battery; attitude.

## muas-contracts — names, wire, manifests, services

- The v2 name tree carried forward as `/muas/v3/...` (same shape: services,
  latest-wins data, mission objects).
- Service definitions as `#[ndn_service]` traits over **ndn-service-core** —
  the contract⇄carrier seam IS the pluggable-backend requirement (VISION §3):
  - carriers: `ndn-rpc` (Tier-0, default), **`ndn-ndnsf`** (faithful NDNSF
    four-phase over SVS — the C++-NDNSF-comparable backend), `ndn-nacabe`
    where ABE-gated. Backend selected by config → head-to-head comparisons.
- **L1 semantic manifests** (flotilla `manifest`) for every published kind:
  telemetry sample, coord status, search status, capability profile, frame
  container, sensor artifact, detection/evidence. Units, precision, datum,
  thresholds signed once. The v2 `MUASFRAME1` container's embedded capture
  pose becomes manifest-described fields.
- v2 ack-gating rules (range guard, AGL guard, busy guard, confirm-phrase
  shutdown) become typed policy in one place.

## Data plane (the NDF-native upgrades)

- **Telemetry, video, RC = Sparks** (`ndf-spark`): 52-byte payloads, windowed
  merkle + signed checkpoint Blocks. *Blocks remember; Sparks move.*
- **Journals & mission record = Block chains** (`ndf-core` via
  `ndf-apps::AppRuntime.publish()`): the v2 fsync-JSONL journal becomes a
  signed append-only chain — power-loss-safe by construction, and **mission
  replay = following the chain** (same dispatch handlers, deterministic seek
  preserved). Keep a local fsync JSONL mirror as the belt-and-suspenders
  field fallback.
- Segmented artifacts (frames, wav) = content-addressed Blocks with
  payload-location; integrity comes from the envelope instead of hand-rolled
  body_sha256.
- Fleet coordination stays **data-plane symmetric** (no request/response):
  `coord/status` published + peers watched, exactly the v2 semantics,
  transport injected so the loop tests without NDN.

## muas-dashboard (VISION §2)

Backend: mission state machine (detect→confirm→queue→dispatch, confirm-count,
best-localized-sighting positioning, multi-target multi-sensor jobs), recorder,
web server. Frontend: v2's canvas map/Carbon g100 UI ported for parity first.

The flotilla integration (per the survey caveat that the render host is
design-only in flotilla, real in ndf-surface):

1. Every dashboard-visible kind gets an L1 manifest + L2 edges (provenance:
   detection is-derived-from frame is-measured-by camera authorized-by GCS).
2. Dashboard views declare **render contracts**; binding is
   match → authorize → instantiate via the Keel matcher
   (`render-contract`), dispatching `Via::Native` ids against an in-process
   renderer registry (map layer, vehicle tile, video panel, event log…).
3. Where mature enough, ride `ndf-surface`'s RenderDaemon/Surface Authority
   instead of our own registry — evaluate and feed back.
4. Present miniMUAS to the waterline suite as **instruments**: the agent and
   dashboard publish measurement/control namespaces + three-layer manifests so
   Sextant/Capstan panels appear with zero suite changes.

Parity is audited against `surveys/minimuas-v2.md` feature-by-feature.

## Observability (VISION §4)

- `tracing` spans throughout every crate from day one; span taxonomy:
  `mission > task > primitive-tick / service-invocation / coord-event`.
- **Cross-node trace propagation over NDN**: carry W3C trace context in
  service invocation metadata so a GCS-dispatch → drone-execution → artifact
  publication is one distributed trace. (If ndn-service-core lacks a metadata
  slot for this, that's a flagship FEEDBACK.md item.)
- Export: `ndn-observability` (spans as NDN data under our prefix) +
  `ndn-otel-bridge` sidecar → Jaeger/Tempo on the GCS; `ndn-sim::otel_export`
  in simulation. Sextant's Observe→Ask trace view is a consumer.

## Simulation & verification (VISION §5)

`muas-sim` scenarios on **ndn-sim (ndn-lab)** with features `mavlink` +
`geometry` (+`lora` later):

- ArduPilot SITL co-sim: real ForwarderEngines + real vehicle mobility from
  SITL — the full v3 stack (agent, coord, dashboard headless) under test.
- Line-of-sight/terrain propagation for realistic link loss during missions;
  `adversary` module for security scenarios.
- Regression suite: the v2 SITL validations (goto floor, avoidance bias lead
  cap, smart RTL slots, cooperative avoidance grace/escalation) as scripted
  scenarios with OTLP traces as the assertion substrate.
- Radio-mode comparison harness (below) runs in sim first, field second.

## Wireless & security (VISION §6)

- **Bearer abstraction from day one**: agent/dashboard bind faces by config —
  `udp/tcp` (AP/STA mode, v2 parity) vs `ndn-face-monitor-wifi` over
  `ndn-radio-drivers` (rtl8812eu ×2 per node) vs `ndn-nan` vs `ndn-face-ble-adv`
  / ESP32 bridges. Comparison = same mission, different bearer, same OTLP
  metrics (latency, loss, sync convergence, video throughput).
- `ndn-radio-cognition` for the named-data-radio MAC control plane;
  `ndn-coding` FEC on lossy bearers.
- Trust: `ndn-security` trust schema + `ndn-cert`/`ndn-identity` fleet
  zero-touch provisioning replaces v2's policies files; onboarding via
  `ndn-trust-envelope` QR/NFC join ceremonies (C2 proximity-tap tier).
  Expiry-by-default keys = field revocation story.
- Command authorization adopts the **P11 actuation-safety record** danger
  tiers: RTL/Hold = D1 (C1), takeoff/mission dispatch = D2 (C2, non-delegable),
  companion shutdown = D3-style confirm (v2's type-the-vehicle-id ceremony
  maps cleanly onto the ceremony vocabulary).
- Named-time: `ndn-time`/`ndn-timekeeper` with GNSS + monitor-wifi TSFT
  sources; dashboard shows holdover/±uncertainty per node (v2's clock-skew
  tile, upgraded to honest intervals).

## RC subsumption (VISION §7)

`muas-rc`: USB game controller on the GCS → per-vehicle or broadcast control
streams over NDN.

- Transport: **Sparks** (ephemeral, sequenced, loss-honest — exactly the RC
  profile). MAVLink `RC_CHANNELS_OVERRIDE`/`MANUAL_CONTROL` at the vehicle end
  = "MAVLink over NDN" with Sparks as the light framing; evaluate whether a
  general MAVLink-over-NDN mapping (all message types) is worth a spec doc.
- 1-to-many: name-addressed (`/muas/v3/<vid>/rc`), selector on the dashboard
  chooses target(s); broadcast bearer means N vehicles hear one transmission.
- Failsafe by construction: Spark stream stops → autopilot RC-loss failsafe +
  agent-level hold/RTL ladder. Rate/e-stop per P11.
- Bridges: ESP32 (ndn-espnow / CRSF out) and LoRa as alternate PHYs;
  DroneBridge/ELRS studied for latency + arming semantics.
- Latency budget measured end-to-end via OTLP before any real flight; sim +
  bench first, RC-into-SITL second, field last.

## Field QoL (VISION §8)

- **ESP32 CYD fleet node**: `ndn-embedded` + monitor-mode/BLE face; palm view
  of fleet stats (telemetry Sparks are 52 B — MCU-friendly by design) + C1/C2
  ceremony surface for enrollment. Track as its own mini-project once M3 data
  plane is stable.
- Deployment: v3 gets its own config-repo branch + flake wiring mirroring the
  v2 flow (push before flake bump).

## Milestones

- **M0** (now): branch, docs, surveys, workspace scaffold compiling.
- **M1**: muas-flight port with translated test oracle (deconflict → geo/
  patterns/placement → tick core → constraints → orbit ladder).
- **M2**: muas-mavlink + SITL: takeoff/goto/raster/carrot-orbit/RTL parity,
  goto-floor and ensure_airborne regressions green.
- **M3**: muas-contracts + muas-agent on ndn-service (rpc carrier), telemetry/
  coord as Sparks, journals as chains, fleet coordination parity in ndn-sim
  co-sim; NDNSF carrier comparison.
- **M4**: dashboard parity (audited against surveys/minimuas-v2.md), manifests
  + render-contract binding, replay-from-chain, OTLP end-to-end.
- **M5**: named-data-radio bearer + AP/STA comparison harness (sim → field).
- **M6**: RC subsumption (bench → SITL → field).
- **M7**: CYD + field QoL; waterline instrument polish.

Every milestone feeds FEEDBACK.md (framework friction) and WATERLINE-INPUT.md
(UI/UX evidence from real fleet ops).
