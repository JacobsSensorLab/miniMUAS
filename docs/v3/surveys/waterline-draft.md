# Survey: waterline suite — first code draft

Surveyed 2026-07-10 at `~/Documents/Dev/waterline` (14 commits, M0–M9, all
same-day). `cargo check` clean; **77/77 tests green**. Compact, disciplined,
demo-shaped draft. They treat miniMUAS as their reference consumer
throughout (ride-map.md; our FEEDBACK/CHECKRIDE patterns adopted; C2
typed-confirm credits our "sunlight ceremony"; demo fleet named iuas-NN).

## What exists

8 crates, path-deps on the same sibling checkouts we use:

- **waterline-strata** — 6 authored `.ndfs` strata (console-timeseries,
  console-live-track, console-log, console-chain, console-command,
  console-video) + 3 contracts; compiled `.ndf` bytes committed with
  tool-generated hash pins (src/pins.rs, Atelier.lock, STRATA.md — the
  worked pin-lifecycle example flotilla itself lacks). Typed builders in
  src/decl.rs, facts in src/facts.rs. 21/27 conformance vectors (shortfall
  honestly ledgered).
- **waterline-ceremony-floor** — U-3 safing ladder: Floor::preflight,
  FloorTable/FloorRuling, S3 HardwareInterlock. 19 deny-path tests.
- **waterline-instrument** — THE integration plane: `Instrument` over
  `ndf_apps::AppRuntime`; declarations published on
  `<principal>/instrument/<device>` (MIME_KEEL_DOCUMENT), facts on
  `<principal>/<app>/<device>`; `Adoption::consider()` runs the REAL Keel
  matcher against the pinned console DAG; `dock()` via select_best_for.
  **Adoption IS the matcher finding Express — no registry, no descriptor
  file, no JSON.** Their plan.md: "a real miniMUAS agent swaps in by
  keeping the declarations and replacing ScriptedFlight."
- **waterline-console-core** — ConsolePort trait + FabricPort (real
  AppRuntime follows) + ScriptPort; chain-fold scrub (replay); layouts;
  abort chrome.
- **sextant-tty / anchor-tty / capstan-tty** — running ratatui TUIs
  (Adopt·Panels·Why·Gate·Scrub·Verbs; AuthorityRecord cards + fleet bead +
  C1/C2/C4 ceremonies; Bench·Straits·Rotate·Compat·Features·Interlock).
- **waterline-trials** — 5 ndn-sim scenarios + two-process demo
  (scripts/demo.sh, UDP bridge — no SHM anywhere despite the design docs).

NOT built (explicit non-goals): Sextant Tab/browser, GUI/RenderDaemon
hosts, fields/ripples fleet map, Pilot, placement management, Trust
Studio, time panel, QR/NFC/trust-envelope onboarding, Anchor ring model,
NAC-ABE, real radios.

## Render pipeline

Same as ours: flotilla `manifest` + `render-contract` (r#match →
select_best_for), `explain::trace` for the Why view. DAG = pinned bytes
compiled in (`waterline_strata::dag::console_dag()`); ndf-surface used
only for MIME constants. RenderSession deferred behind ConsolePort.

## Integration facts for us

**(a) Docking miniMUAS as instruments — path is waterline-instrument, NOT
/instrument.json.** Map: battery → SeriesDecl::battery + BandedSample;
position → TrackDecl{three_d} + TrackFix (Band uncertainty ← EKF
variance); journal → LogDecl/LogEvent + ChainDecl{replay_capable} (our
power-loss journals ARE their scrub input); RTL/hold/terminate →
VerbDecl with safing/interim/rate (their Floor::preflight implements the
S1-interim bridge our smart-RTL maps to). Wire compat: we must emit their
pinned term hashes → depend on waterline-strata by path. Console-side
floor table needs our verb rows (demo.rs::demo_floor_table is the
template).

**(b) Lifecycle records** — no inventory surface exists; align via the
WL-7 pattern instead: declarations = keyed-entry manifests, facts =
single positional-record manifests on chains declared via
console-chain:chain-view; sign-offs = AuthorityRecord +
CeremonyAttestation (capstan::provision_binding is the exemplar);
strata/console-command.ndfs (hash back-refs, list-of(hash) optionals,
measured:instant stamps) is the record-shape template. Their friction #7:
positional facts hard-fail on arity change — version from day one.

**(c) Companion management** — Capstan is ~15% of the need: usable NOW =
CapabilityReport::probe (binary --capabilities gap check), Admission +
GateCell::swap (refuse-without-poison mid-run), EpochName::rotate.
Missing (we'd contribute, not consume): config-as-manifests, drift
detection vs declared config, reconcile, placement model.

## The four blockers between our drones and Sextant today

1. **Docking closed over five hard-coded decl types**
   (console-core/src/state.rs:256-269 tries the five ::from_manifest THEN
   gives up — before the matcher runs; PanelState is a closed enum). A
   foreign stratum that would bind Approximate can never dock. Ask:
   PanelState::Generic{manifest, verdict} or match-driven dispatch.
2. **Fact routing ignores chain root/subject** (state.rs:180-246,
   `_root` unused): every TRACK_FIX lands in every Track panel — two
   drones = one smeared panel. THE fleet blocker; the verb arm filters
   correctly, proving the pattern.
3. **Consumer onboarding is compiled in** (sextant-tty/src/main.rs:26-32,
   85-95: groups, writer keys, floor, peers). Need CLI injection.
4. **No vocabulary injection into the adoption DAG**
   (adoption.rs:74-80 pins console_dag(); set_contracts/set_frontier
   exist, insert_vocabulary doesn't) — consumer strata can't enter.

Additional friction: demo gate-flip is a silent no-op on FabricPort (LOG
chain never followed in main.rs); FabricPort::poll re-resolves whole
chains per 120 ms tick (O(chain-length), fleet-scale hazard); Series
history unbounded; REPORT.md test-count nit (19 vs 18+1). They co-signed
our flotilla A-3 ask and hit a real flotilla bench fixed-point bug
(record-nesting-record mis-binding) that shaped WL-7 — affects shapes we
emit too.
