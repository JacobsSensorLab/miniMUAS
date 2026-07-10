# miniMUAS v3 — Architecture

Maps `VISION.md` requirements onto concrete design. Grounded in the four
surveys under `surveys/` (v2 parity bible, ndn-workspace/ndf-rs crate map,
UAS-IPBRC primitive inventory, NDF/waterline docs digest). Read those for the
underlying evidence; this doc is the decisions.

**Structural rule (see `REPO-TOPOLOGY.md`):** miniMUAS is one *app* on a
use-case-agnostic multi-repo stack — `uas-flight` (vehicle plane) ←
`uas-fleet` (NDN fleet plane) ← `uas-console` (operator plane) ← miniMUAS.
Generic capability goes in the layer repos; this repo holds only the
search-and-investigate use case: mission choreography, app contracts, the
composed binaries, scenarios, and field doctrine.

## Layering (mirrors the D-47 axiom)

```
 muas-agent · muas-dashboard · muas-sim     (this repo: the use case)
        │
 uas-console (view framework)   uas-fleet (data kinds, node fw, coord, RC)
        │                            │
        └──── uas-flight (primitives) ──── uas-mavlink (vehicle adapter)
                         │
      ndf-apps / ndf-spark / ndf-core          (meaning of bytes)
                         │
      ndn-engine / ndn-service / faces / sync  (moving bytes)
```

Rule of thumb carried from UAS-IPBRC: **uas-flight never touches MAVLink or
NDN**; uas-mavlink never contains mission logic; the fleet-node framework
hosts both; the app wires them.

## This repo's workspace (branch v3)

```
Cargo.toml            # workspace; sibling path deps (dev) — REPO-TOPOLOGY.md
crates/
  muas-contracts/     # /muas/v3 names, mission kinds, service definitions
  muas-agent/         # per-drone agent binary (composes uas-fleet-node)
  muas-dashboard/     # GCS console binary (composes uas-console)
  muas-sim/           # mission-level co-sim scenarios & verification
```

Sibling checkouts assumed under `~/Documents/Dev/`: `uas-flight`,
`uas-fleet`, `uas-console`, `ndn-workspace`, `ndf-rs`.

Risk to watch: `ndf-rs/refounding` is "design-draft, not ratified" and pins
ndn-workspace HEAD `5798fa3f` (checkout is ahead). Scaffold-depth deps
(`manifest`, `render-contract`, `ndf-core`) compile clean; the deeper
`ndf-apps`/`ndn-engine` graph gets its first exercise at M3 — log breakage
in FEEDBACK.md.

## uas-flight — the primitive library (VISION §1)

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
- **Future big capabilities land here** (autonomous navigation, trajectory
  planning, obstacle-aware planners) behind the same primitive/planner seam —
  no app rework.

## uas-mavlink — vehicle adapter (VISION §1 "MAVLink everywhere")

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

## uas-fleet — the NDN fleet plane

- **uas-fleet-data**: generic published kinds + their L1 semantic manifests
  (telemetry sample, coord status, capability profile, journal chain entries,
  segmented artifacts). App-specific kinds (detection evidence) stay in
  muas-contracts.
- **uas-fleet-node**: the agent framework — service hosting over
  **ndn-service-core**'s contract⇄carrier seam (VISION §3: carriers `ndn-rpc`
  default, **`ndn-ndnsf`** for the NDNSF comparison, `ndn-nacabe` where
  ABE-gated; selected by config → head-to-head comparisons), the
  flight-backend seam (sim + MAVLink behind one trait), the **PeerGuard**
  coordination loop (v2 semantics exactly: adaptive polling, CPA, cooperative/
  uncooperative escalation with 2.5 s grace, altitude-bias overlay clamped
  −4..+8 with climb-wins, smart RTL slots, fleet flight floor; transport
  injected so it tests without NDN), bearer selection, journals, authorized
  shutdown.
- **uas-rc**: RC subsumption (VISION §7) — USB controller → per-vehicle or
  broadcast Spark streams (`<app>/<vid>/rc`) → RC_CHANNELS_OVERRIDE /
  MANUAL_CONTROL. Loss-honest; failsafe by stream silence + autopilot RC-loss
  ladder; rate/e-stop per P11. ESP32 (CRSF/ndn-espnow) and LoRa bridges as
  alternate PHYs; latency budget measured via OTLP before real flight.

## muas-contracts — the app's names, kinds, services

- The v2 name tree carried forward as `/muas/v3/...` (same shape: services,
  latest-wins data, mission objects) — parity audits map one-to-one.
- Service definitions as `#[ndn_service]` traits; mission-specific kinds
  (frame container with embedded capture pose, detection/evidence) as L1
  manifests.
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
  payload-location; integrity from the envelope instead of hand-rolled
  body_sha256.
- Fleet coordination stays **data-plane symmetric** (no request/response).

## muas-dashboard on uas-console (VISION §2)

Backend: mission state machine (detect→confirm→queue→dispatch, confirm-count,
best-localized-sighting positioning, multi-target multi-sensor jobs),
recorder, web server. Frontend: v2's canvas map/Carbon g100 UI ported for
parity first.

The flotilla integration lives in **uas-console** (generic) with miniMUAS
panels registered on top:

1. Every dashboard-visible kind gets an L1 manifest + L2 edges (provenance:
   detection is-derived-from frame is-measured-by camera authorized-by GCS).
2. Views declare **render contracts**; binding is match → authorize →
   instantiate via the Keel matcher, dispatching `Via::Native` ids against
   the console's renderer registry (map layer, vehicle tile, video panel,
   event log…).
3. Where mature enough, ride `ndf-surface`'s RenderDaemon/Surface Authority
   instead of our own registry — evaluate and feed back.
4. Present agent and dashboard to the waterline suite as **instruments**
   (namespaces + three-layer manifests) so Sextant/Capstan panels appear
   with zero suite changes.

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

`muas-sim` holds *mission-level* scenarios on **ndn-sim (ndn-lab)** with
features `mavlink` + `geometry` (+`lora` later); layer repos keep their own
unit/property tests:

- ArduPilot SITL co-sim: real ForwarderEngines + real vehicle mobility from
  SITL — the full v3 stack (agent, coord, dashboard headless) under test.
- Line-of-sight/terrain propagation for realistic link loss; `adversary`
  module for security scenarios.
- Regression suite: the v2 SITL validations (goto floor, avoidance bias lead
  cap, smart RTL slots, cooperative grace/escalation) as scripted scenarios
  with OTLP traces as the assertion substrate.
- Radio-mode comparison harness runs in sim first, field second.

## Wireless & security (VISION §6)

- **Bearer abstraction from day one** (uas-fleet-node): faces bound by
  config — `udp/tcp` (AP/STA, v2 parity) vs `ndn-face-monitor-wifi` over
  `ndn-radio-drivers` (rtl8812eu ×2 per node) vs `ndn-nan` vs
  `ndn-face-ble-adv` / ESP32 bridges. Comparison = same mission, different
  bearer, same OTLP metrics (latency, loss, sync convergence, throughput).
- `ndn-radio-cognition` for the named-data-radio MAC; `ndn-coding` FEC on
  lossy bearers.
- Trust: `ndn-security` trust schema + `ndn-cert`/`ndn-identity` fleet
  zero-touch provisioning replaces v2's policies files; onboarding via
  `ndn-trust-envelope` QR/NFC join ceremonies (C2 proximity-tap tier).
  Expiry-by-default keys = field revocation story.
- Command authorization adopts the **P11 actuation-safety record** danger
  tiers: RTL/Hold = D1 (C1), takeoff/mission dispatch = D2 (C2,
  non-delegable), companion shutdown = D3-style confirm (v2's
  type-the-vehicle-id ceremony maps cleanly).
- Named-time: `ndn-time`/`ndn-timekeeper` with GNSS + monitor-wifi TSFT
  sources; dashboard shows holdover/±uncertainty per node (v2's clock-skew
  tile, upgraded to honest intervals).

## Field QoL (VISION §8)

- **ESP32 CYD fleet node**: `ndn-embedded` + monitor-mode/BLE face; palm view
  of fleet stats (telemetry Sparks are 52 B — MCU-friendly by design) +
  C1/C2 ceremony surface for enrollment. Generic parts belong in uas-fleet;
  track as its own mini-project once M3 data plane is stable.
- Deployment: per-repo flakes + config-repo composition per
  `REPO-TOPOLOGY.md` (repos own packaging; config repo owns hosts).

## Milestones

- **M0** (done): branch, docs, surveys, multi-repo scaffold, all workspaces
  green.
- **M1** (done 2026-07-10): uas-flight fully ported — 17 modules, the whole
  UAS-IPBRC `test_flight_*` oracle translated (202 tests, zero warnings).
- **M2** (done 2026-07-10): uas-mavlink link core + all-AGL backend +
  FlightCommandLink impl; 40 unit tests; 8/8 SITL checkride checks green
  (record: uas-flight `crates/uas-mavlink/CHECKRIDE.md` — pins the 3.5 m
  goto floor, bias lead cap, ensure_airborne ladder).
- **M3** (done 2026-07-10): fleet plane complete — coordination port,
  backend seam, data kinds + manifests, agent on ndn-service (rpc default,
  ndnsf comparison flag), Spark telemetry lane, chain journals; parity
  verdicts green in ndn-sim co-sim (protocol findings → KNOWN-ISSUES.md).
- **M4** (done 2026-07-10): console framework (Keel matcher binding,
  33 tests) + dashboard parity (25 tests, full v2 feature checklist,
  verified live against a running agent). Replay-from-chain + OTLP
  end-to-end remain follow-ups.
- **M-deploy** (flakes done 2026-07-10; config-repo v3 branch pending
  GitHub remotes): per-repo flakes build real binaries on this machine;
  composition plan in MDEPLOY-PLAN.md.
- **M5** (sim side done 2026-07-10 — docs/v3/radio-comparison.md; field
  side hardware-gated): link-profile matrix quantified coop/telemetry/RTT
  degradation; real monitor-wifi bearer wiring awaits the rtl8812eu rigs.
- **M6** (bench done 2026-07-10: 26-byte frames, ~1 ms loopback,
  failsafe ladder verified; SITL/field halves gated on a USB controller).
- **M7** (software done 2026-07-10: uas-cyd repo with host-tested core +
  xtensa-checked firmware shell, DESIGN.md, FIELD-QOL.md; hardware bring-up
  B1-B5 gated on the CYD unit).

Every milestone feeds FEEDBACK.md (framework friction) and WATERLINE-INPUT.md
(UI/UX evidence from real fleet ops).
