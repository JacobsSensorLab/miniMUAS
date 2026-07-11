# v3 backlog — the honest ledger

Purpose: stop eliding. Every capability the owner has asked for across the
session that is **designed-but-not-built**, **built-but-not-surfaced**,
**stubbed**, or **wrong and needs correction** — tracked here so it can't
silently slip again. Updated every round. `DONE` items live in git log /
ARCHITECTURE.md milestones, not here.

Status key: 🔴 not started · 🟡 partial/stub · 🟠 built wrong, correcting ·
🔵 designed only · ⚫ hardware-gated.

## Corrections

- ✅ **RC transport is now named-data Sparkstream over the NDN fabric**
  (done 2026-07-11, miniMUAS `62391dc` / uas-fleet `fc46bab`). RC rides
  ndf-spark carried over the ForwarderEngine as named Data under a
  `/muas/v3/rc/<vid>` control plane, crossing the same SimLinks/faces as
  every other stream — with SP-3 replay refusal, merkle windows, and
  checkpoint Blocks. Frame-as-Data (`--rc-data`) and UDP (`--rc-udp`) are
  explicit comparison bearers. Live-verified 13/13: engaged, crossed the
  SimLinks (nettap), checkpoint anchor-verified. Two bugs fixed en route:
  the UDP-bypass (the original mark-miss) and a self-shadow (frames named
  under the vehicle's own served prefix, so the agent fetched itself).

## Named but not built

- ✅ **Strategy authoring tools** (done 2026-07-11, uas-console `1193d41` /
  uas-fleet `c380fdf`). `StrategyAuthor` trait with Forms (deterministic
  editor + validators), LLM (LlmBackend seam + RuleStubLlm; authors hold
  no runtime so they structurally cannot sign) and NodeGraph (typed graph
  → compiler) backends — all emit identical records (backend-agnostic).
  review→diff-vs-active→sign→publish is the only signing path; NDF-surfaced
  as uas-console render contracts (`strategy.autosign` is a Refuse
  verdict). Objective-record authoring seam present for onboard autonomy.
  Remaining: mount the authoring surface in the miniMUAS dashboard (a
  builder-mode concern), and a real LLM/node-graph editor behind the seams.
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
