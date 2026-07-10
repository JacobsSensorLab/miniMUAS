# Survey: ndn-workspace + ndf-rs (refounding)

Surveyed 2026-07-09. Input to ARCHITECTURE.md and the workspace scaffold.

## Top-level shape

Neither directory is one Cargo workspace — both are **containers of sibling git
repos** wired with relative `path =` deps. Nothing is published; everything is
`0.1.0`/`publish = false`. Governing axiom (refounding README, D-47):
*moving/storing bytes → ndn-workspace; the meaning of bytes → NDF.*

- `~/Documents/Dev/ndn-workspace` — networking stack: `ndn-rs`, `ndn-ext`,
  `ndn-fwd`, `ndn-sim`, `ndn-repo`, `ndn-mobile`, `ndn-embedded`, `ndn-anchor`,
  `ndn-ripple`, `ndn-radio-drivers`, plus **`flotilla`** (Waterline/Keel
  calculus). Build per-repo or `./build_all.sh`; `mapping.json` maps crates→repos.
- `~/Documents/Dev/ndf-rs` — two generations: `ndf-workspace/` (the frozen
  **oracle**, ~40 crates) and **`refounding/`** (the fresh start v3 targets;
  10 standalone crates, "design-draft, not ratified", pinned to ndn-workspace
  HEAD `5798fa3f` — verify current checkout satisfies the path deps).

Maturity: ~3,800 test attributes in ndn-workspace, ~2,290 in ndf-rs; low
TODO density; `#![deny(missing_docs)]` in ndf-core; fuzz targets on
ndn-packet/ndn-frame-io. WIP is concentrated in labelled stubs (flotilla render
host = design-only; ndf-registry backends stubbed; ndn-dv scaffold).

## ndn-workspace highlights for v3

### ndn-rs (core)
- `ndn-rs-prelude` (crate `ndn`): one-type entry `Node` — `fetch/serve/object/
  publish/subscribe/query`. Primary consumer API.
- `ndn-engine`: `ForwarderEngine` + `EngineBuilder` (real engine also used in sim).
- Core crates: `ndn-packet`, `ndn-tlv`, `ndn-crypto-core` (no_std Ed25519,
  byte-identical native+embedded), `ndn-storage` (async+sync, fjall/redb),
  `ndn-transport` (`Face` trait), `ndn-time` (**named-time**: uncertainty-bounded
  samples, Marzullo), `ndn-radio-hal` (bearer-agnostic TxIntent, 802.11 MCS),
  `ndn-frame-io`, `ndn-signals-core`.
- Security: `ndn-security` (trust schema, ABE feature), `ndn-cert` (NDNCERT CA),
  `ndn-identity` (**fleet zero-touch provisioning**).
- `ndn-sync` (SVS, PSync; two-phase-ack `SyncHandle`), **`ndn-observability`**
  (NDN-native OTLP span publisher served under an NDN prefix), `ndn-app`.

### ndn-ext (extensions)
- **`ndn-service-core`** — the pluggable-backend seam: traits `Carrier`,
  `SelectCarrier`, `HintedCarrier`, `Dispatch`, `Frame`; `#[ndn_service]`
  proc-macro generates per-op frames + carrier-generic client. Carriers today:
  `ndn-rpc` (Tier-0), **`ndn-ndnsf`** (faithful NDNSF four-phase over SVS — the
  comparison backend), `ndn-nacabe` (NAC + CP/KP-ABE).
- **Radio faces**: `ndn-face-monitor-wifi` (802.11 monitor-mode raw injection at
  chosen MCS, no association), `ndn-nan`/`ndn-face-wifi-aware`,
  `ndn-face-ble-adv`, `ndn-radio-cognition` (sense→decide→act data-centric MAC),
  `ndn-signal-sources`. Also AF_XDP, SHM zero-copy (`ndn-surface`
  NamedPublisher/Subscriber), QUIC, serial (COBS), full browser stack, ESP32
  targets (webble; plus ndn-embedded ESP32 BLE example).
- `ndn-compute` (fuel-metered WASM executor), `ndn-sealed-box`, `ndn-coding`
  (FEC), `ndn-timekeeper`/`ndn-time-sources` (GNSS/RTC), `ndn-trust-envelope`
  (`ndn-trust://` pairing), `ndn-python` (PyO3), `ndn-wasm`.

### ndn-fwd
Standalone forwarder, `ndn-tools` (peek/put/ping/ls), **`ndn-otel-bridge`**
(NDN spans → OTLP/HTTP → Jaeger/Tempo/Honeycomb), `ndn-trust-context`
(trust schema → QR/NFC join payload).

### ndn-sim (tool: **ndn-lab**) — drone-fleet critical
Multi-node networks of real `ForwarderEngine`s on pluggable `SimKernel`
(wall/virtual clock); `Simulation` → `RunningSimulation: FabricControl`.
Key modules: **`mavlink`** (ArduPilot SITL mobility adapter, feature `mavlink`),
**`geometry`** (line-of-sight propagation obstruction), `lora`, `wifi`,
**`otel_export`** (OTLP/HTTP to a real collector), `keel` (first consumer of
manifest/render-contract for self-describing telemetry), `adversary`,
`scenario`, `cosim`, `replay`, `mcp`.

### ndn-radio-drivers
Userspace USB monitor-mode backends: **RTL8812EU**, 8822E, 8821CU, MT7612U,
8812AU, 8731BU, 8733BU — over `ndn-radio-hal`. Matches our airframes' 2×
rtl8812eu.

### flotilla (Waterline/Keel calculus)
Standalone workspace, depends on nothing in the NDN stack.
- `manifest` — frozen 32-term model V0.2, canonical codec R1–R13, `FrozenDag`,
  zero-dep no_std.
- `render-contract` — Keel matcher: verdicts **Express/Approximate/Refuse/
  Unresolved**; returns verdict + inert `Via::Native|Wasm`, does NOT execute.
- `manifest-derive` (`#[derive(Manifest)]`), `explain`, conformance vectors +
  FREEZE.md.
- **Caveat:** the render host (WASM sandbox, ViewBlock typing, capability
  grants, Surface Authority) is design-only in flotilla; the Surface Authority
  lives in `ndf-rs/refounding/ndf-surface`. Today's pattern: match → select →
  dispatch `Via::Native` ids against your own in-process registry.

## ndf-rs refounding (what v3 consumes)

- **`ndf-core`** — substrate: `NdfHeader`, envelope (NDN Data + Ed25519),
  chain/fork, Kind registry, AC.12 10-step verifier, capability, authority.
  no_std+alloc, deny(missing_docs).
- **`ndf-policy`** — sovereignty posture → mechanism config projections;
  `ChainGate`/`IngestVerdict`. 10 modules green, 68 tests.
- **`ndf-surface`** — the render host flotilla stops short of. SD-1: *Authority
  enforces; Behavior expresses.* `RenderDaemon`, Keel matcher + `TrustFrontier`,
  `Via::Wasm` hash-gated on ndn-compute, sealed frames (ChaCha20-Poly1305) over
  ndn-surface SHM, AppKit/Wayland host adapters. 45 tests.
- **`ndf-apps`** — **primary consumer SDK**. AD-0: one model,
  location-transparent. `AppRuntime::attach(engine)` via `EngineAppExt`;
  `.publish()` → `PublishReceipt`, `.serve()`, `.follow()`/`.follow_gated()`
  (ChainReplicator, D-46 Carry/Reach presence), `.resolve_trusted()`, `.head()`,
  `.forks()`, `.identity()`, `.store()`. 8 tests (5 E2E over ndn-sim).
- **`ndf-spark`** — the sparkstreams: `SparkPayload` (52 B vs 194 B Block, no
  per-item signature), windowed merkle + one signed checkpoint Block, rollback,
  predicate. 13 tests. → natural fit for telemetry AND MAVLink-over-NDN.
- `ndf-replication-transport` (ChainGate × ndn-sync SyncHandle), `ndf-nfn`
  (verifiable in-network compute), `ndf-manifest`(+derive) — strict-canonical
  TLV app-manifest codec with must-understand/may-ignore tags.

Oracle workspace (`ndf-workspace/`) worth mining: `ndf-query` (live IVM/DBSP
query engine + `query!{}` Datalog, tantivy/HNSW/minhash indices), `ndf-bevy`
(`NdfQuery<T>`/`NdfStream<T>` ECS bindings), `ndf-render-contract-wit-bindgen`
(WASM component renderers), `ndf-identity` (did:ndn), `ndf-pseudonym` (BBS+).

## How miniMUAS v3 depends on it

Path deps; miniMUAS at `~/Documents/Dev/miniMUAS` is at the right depth:

```toml
ndn        = { path = "../ndn-workspace/ndn-rs/crates/app/ndn-rs-prelude" }
ndn-engine = { path = "../ndn-workspace/ndn-rs/crates/forwarding/ndn-engine" }
ndn-service-core = { path = "../ndn-workspace/ndn-ext/crates/service/ndn-service-core" }
ndn-sim    = { path = "../ndn-workspace/ndn-sim/crates/ndn-sim", features = ["mavlink"] }
manifest   = { path = "../ndn-workspace/flotilla/crates/core/manifest" }
render-contract = { path = "../ndn-workspace/flotilla/crates/core/render-contract" }
ndf-core   = { path = "../ndf-rs/refounding/ndf-core" }
ndf-apps   = { path = "../ndf-rs/refounding/ndf-apps" }
ndf-spark  = { path = "../ndf-rs/refounding/ndf-spark" }
```

Feature flags that matter: `ndn-storage` sync/redb/fjall, `ndn-security` abe,
`ndn-compute` wasm-exec, `ndn-sim` mavlink, `ndf-core` std vs no_std.

Recommended stack: `ndf-apps::AppRuntime` over `ForwarderEngine` as app SDK;
ndf-spark for motion/telemetry streams + ndf-core Blocks for commitments;
ndf-surface for the render plane; ndn-sim (mavlink+geometry+lora) for co-sim;
monitor-wifi/NAN/BLE faces + ndn-radio-drivers for real named-data radio;
ndn-observability + ndn-otel-bridge (or ndn-sim otel_export) for tracing.

## Unique-vs-mainline features to exercise and report on

Named-data radio bearers (monitor-mode injection, NAN, BLE adv, LoRa),
`ndn-radio-cognition` cognitive MAC, named-time, SHM zero-copy surfaces,
AF_XDP, WASM hot-loadable strategies, service contract⇄carrier seam,
fuel-metered in-network compute, NDN-native OTLP spans, the manifest/
render-contract calculus, MAVLink/geometry co-simulation.
