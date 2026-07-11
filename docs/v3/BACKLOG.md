# v3 backlog — the honest ledger

Purpose: stop eliding. Every capability the owner has asked for across the
session that is **designed-but-not-built**, **built-but-not-surfaced**,
**stubbed**, or **wrong and needs correction** — tracked here so it can't
silently slip again. Updated every round. `DONE` items live in git log /
ARCHITECTURE.md milestones, not here.

Status key: 🔴 not started · 🟡 partial/stub · 🟠 built wrong, correcting ·
🔵 designed only · ⚫ hardware-gated.

## Corrections (built, but wrong — highest priority)

- 🟠 **RC transport must be named-data over the NDN fabric, not UDP.**
  RC frames currently ride a direct dashboard→agent UDP socket that
  bypasses the fabric (no named addressing, no NDN security, no
  broadcast-native 1-to-many). This contradicts the project's north star
  (data-centric network→physical) and RC-CONTROL.md's own vision. UDP must
  be demoted to an explicit *comparison bearer* only (as AP/STA is for the
  network layer); named data over the engine/faces is the default. See
  RC-CONTROL.md "Transport correction". **In progress.**

## Named but not built

- 🔴 **Strategy authoring tools** (owner asked explicitly, twice). Pluggable
  authoring frontends that all emit strategy records: forms/JSON editor
  first (cheap), then LLM-assisted (draft-from-intent, Pilot-style —
  drafts never sign), then node-graph. None exist yet — only the record
  format + evaluator. Must be manifested/render-contract-surfaced, not
  dashboard-hardcoded. **Starting now.**
- 🔵 **Onboard objective/metric-based autonomy.** `ObjectiveRecord` is a
  record-only stub; the onboard interpreter that plans from objectives over
  uas-flight primitives (so external command and onboard autonomy are two
  interpreters of one authored intent) is unbuilt.
- 🔵 **Builder-mode / malleable dashboard** (task #23): namespace browsing
  → data/capability/manifest discovery → render-contract picking (gauge vs
  dial) → place/pop-out; kind-scoped (not device-scoped) bindings with the
  Express→Approximate→text/value degradation ladder; manifest-powered help
  (element inspection/tooltips). Designed in ROUND-3 §3; catalog is the
  read-only seed; placement/binding UI unbuilt.
- 🔵 **Widget placement.** `/catalog.json` advertises native widgets but
  they are not yet placeable onto a surface (the builder-mode milestone).
- 🔵 **Companion-computer dynamics (helm).** COMPANION-DYNAMICS.md designed;
  the helm supervisor (follow node-config chain → apply from pre-built
  closure set → applied-state receipt) is unbuilt.
- 🔵 **MAVLink-over-NDN general mapping.** RC frames are a bespoke light
  frame; a general MAVLink-over-NDN carriage (missions, params, ftp) —
  NDF Sparkstreams or lighter — is queued (RC-CONTROL.md note).

## Built but not surfaced

- 🟡 **Fleet lifecycle records.** `uas-fleet-records` crate + the
  `uas-fleet-instrument` Sextant adapter exist and test green, but the
  records are not surfaced in the miniMUAS dashboard (builder-mode drone
  cards) and no live waterline docking has been run.
- 🟡 **Sensor viz fine trapezoid.** The dashboard renders the attitude-
  driven trapezoid the moment telemetry carries roll/pitch; the agent does
  not yet publish them (renders coarse/heading-only until it does). Small
  agent-side additive change.
- 🟡 **Dispatcher idle-first ranking is evaluator-only live.**
  `rank_candidates` runs, but the dashboard still can't dispatch to a busy
  vehicle (queues job-side until busy→idle), so idle-first is a no-op on
  the live path. Needs the dashboard to learn remote queueing (the agent
  queue engine already supports it).

## Stubs / partial

- 🟡 **Digital-twin detection & anomaly backends.** `DetectionProvider` /
  `AnomalySource` are trait-pluggable; only Simple/synthetic impls exist.
- 🟡 **Audio flyover captures.** Flyover flies the dip profile but does not
  yet fire an acoustic capture at each dip center (noted follow-up).
- 🟡 **NdnsfCarrier `.insecure()`.** Signed mode + trust schema is the
  security milestone (KNOWN-ISSUES #6).

## Network / observability tail

- 🔵 **Network R2b**: span-fed named-data traceroute + data-centric ping
  (NETWORK-VIZ.md phase 2). R2a fields/lens shipped.
- ⚫ **Network R3**: real radio telemetry (channel/MCS/RSSI) from
  ndn-radio-cognition/signal-sources — hardware-gated, never synthesized.
- 🔵 **Live OTLP end-to-end.** tracing spans exist throughout; the
  ndn-observability → ndn-otel-bridge → Jaeger/Tempo export and the
  "Observe→Ask" trace queries are not wired to a live collector. This is
  the "next-level opentelemetry no other NDN app has" showcase — still
  latent.

## Hardware-gated (⚫ — real gear required)

- ⚫ **Named data radio bearer**: monitor-mode wifi, rtl8812eu ×2 per node,
  vs AP/STA comparison in the field (M5 field half).
- ⚫ **RC R3–R5**: SITL stick-fly regression → C2 ceremony keying
  (unbind-by-expiry) → ESP32 CRSF bridge + LoRa bearers (the ELRS
  binding-phrase replacement).
- ⚫ **ESP32 CYD field node**: bring-up B1–B5 (uas-cyd repo scaffolded).
- ⚫ **Field QoL tooling**: FIELD-QOL.md doctrine → physical tools/scripts.

## Process note

If a future round documents a capability, it either gets built in that
round or lands here the same commit. "Designed" is a status, not a
completion.
