# v2/v3 dashboard convergence — plan and decisions

**Finding (code-level comparison, 2026-08-13):** the two dashboards are one
lineage that drifted. v2 `dashboard.html` carries "(ported from v3)" markers;
v3's `server.rs` calls its asset "ported v2 dashboard.html"; 71/76 v2 top-level
JS functions exist verbatim in v3; CSS tokens are byte-identical; and the core
WS wire kinds (`telemetry`, `telemetry_stale`, `search_status`, `event`,
`video_stats`, `raster_preview`, binary `[idx][jpeg]` frames) are byte-identical
between the two backends. Both files use the same architecture: one
`dispatch()` switch → per-kind handlers → named globals → rAF canvas `draw()` +
imperative panel renderers, with replay re-fed through the same `dispatch()`.

**Decision: one canonical `dashboard.html`, based on v3's (the superior
variant), carrying the v2-only features, capability-gated off `hello`.**
v3 is the better base: bigger feature set (task-queue strip, RC/pilot surface,
network/fabric lens, coverage sweep, display panel with persisted per-layer
opacity, catalog browser, record button, event-log filters), a real display
config store (`DISP`), one zoom-scaling policy (`iconPx()/lw()`), and backend
tripwires (`catalog.rs` scans the HTML's dispatch kinds so contract drift
fails loudly).

## Canonical file + how each stack serves it
- Canonical: `examples/python/v2_flight_services/dashboard.html` on the v2
  working branch (v2 serves it from disk via `--html` → edit-and-refresh).
- v3 adopts by copying into `crates/muas-dashboard/assets/` (compile-time
  `include_str!`). Same file, both backends — the byte-identical wire kinds
  make this a drop-in; everything divergent is gated on `hello`.

## v2-only features ported INTO the v3 base
| Feature | Source (old v2 html) | Gate |
|---|---|---|
| Mission bundle download/import (+ `startReplay(name,text)` split) | :399-403, :1258-1282, :1224 | `hello.bundle` |
| Video transport selector (segmented poll vs NDNSF stream) | :1899-1902, :1916-1922 | `hello.video_transport` |
| Command-routing toggle (targeted vs two-phase) | :410, :1692-1702 | `hello.command_mode` |
| `hello.anomalies` seeding of the sim layer | :1931 (v2 dispatch) | presence |

## Contract reconciliations (in the shared file)
1. **Bug fixed:** v2 frontend sent `{kind:"command_mode"}` but the backend
   keys on `cmd` — the toggle was a silent no-op. Converged file sends `cmd`
   uniformly (matches both backends' dispatch).
2. **`sim_anomalies` payload key:** v2 sends `anomalies`, v3 sends `items`.
   Shared file accepts both (`m.items || m.anomalies`); no backend churn.
3. **Record button** gated on `"recording" in hello` (v3 sets it; v2 doesn't
   have session recording yet → button hidden, not broken).
4. v3-only kinds (`task_queue`, `rc`, `coord`, `net`) simply never arrive on
   v2 — their layers/panels stay empty or are already capability-gated
   (RC via target caps, sim via `hello.sim`).

## v2 backend (`run_dashboard.py`) changes
- `hello` gains capability flags: `bundle: true`, `video_transport: true`,
  `command_mode: true`, `sim: <whether sim anomalies are active>` (v3's HTML
  gates the sim panel on `hello.sim`; v2's old panel was unconditional).
- No other wire changes needed — core kinds already byte-identical.

## Explicitly NOT done here (follow-ups, from the motion/contract comparison)
- Port the 8 hard-won v2 field fixes into v3 (avoid_tier priority tiers +
  pause, goto stall detector, attitude-aware projection, busy stale reclaim,
  AGL pinning, confirm_guided, agl_alarm producer, video fetch cap) — v3-side
  work, tracked separately.
- Language-agnostic extraction (name grammar, wire schemas, closed vocabs,
  safety-limit config, raster fixture vectors, deconflict golden vectors,
  frame-container spec) — the deeper contracts convergence.
- v3 backend implementing `bundle`/`video_transport`/`command_mode` so those
  gates light up there too.
- Inbound-side trait in v3 (`ndn.rs` pollers still call hub/mission directly;
  outbound `providers::Commander` seam already exists).
