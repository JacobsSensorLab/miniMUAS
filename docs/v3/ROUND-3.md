# Round 3 — evaluation feedback, gap-fill, and design directions

Captured 2026-07-10 from the owner's second evaluation flight. This document
is the structured form of that feedback plus the architectural
recommendations they asked for. Companion piece: `UNTRAPPED.md` (the
data-still-thinks-it's-trapped statement). Work is tracked as two
implementation waves (dashboard, agent) and two design tracks (service
strategies/authoring, NDF-native dashboard).

## 1. Defects & UX (implementation waves)

### Dashboard wave
- **GCS anchor bug**: the network layer's GCS node pans with the map — it
  is screen-anchored where it must be world-anchored at the GCS location.
- **Sensor viz semantics were wrong**: rings read as "visual range" while
  the quad already shows it. Redesign per the owner's model:
  - **Forward-facing vision** → cone/sector (top-down projection of the FoV
    volume) with **shaded D/R/I regions** inside it.
  - **Ground-facing vision** → the ground quad: rectangle when level,
    trapezoid as the drone rolls/pitches/yaws (attitude now rides
    telemetry additively; the per-vertex math was already in).
  - **Sensor model** grows: mount position relative to drone center,
    fixed vs single/multi-axis gimbal, and **coarse/fine pose modes** so
    full-pose telemetry is optional when data rate is precious (coarse =
    heading-only projection, fine = full attitude).
- Icon scaling with map zoom; ring/line contrast fixes; a **viz config
  panel** (sizes, background opacity/coloring); **collapsible tool
  sections**; **resizable/collapsible side panels**; **toggleable legend**.
- **Replay semantics**: recording currently runs indefinitely including
  idle, producing an arbitrary-feeling replay list. Change to
  session-scoped recordings: a recording begins at mission start or
  explicit arm, ends at mission complete/RTL-all/explicit stop, is named
  by run + mission, and idle periods don't record. The journal chains are
  unaffected (they are the durable record; recordings are a UI artifact).
- **Planning**: coverage shading from sensor footprint at planned AGL,
  area-per-pixel given sensor resolution, derived stats (GSD, overlap,
  expected frames); **live coverage shading** as the raster progresses.
- **Command feedback**: `command.result … error="smart rtl engaged
  (slot-layered)" ok=true` — the detail rides the error field. Split
  `detail` from `error`; give every command a visible lifecycle in the UI
  (sent → acked/refused with reason → outcome), not just a log line.

### Agent wave
- **Sensor tasking is broken/misleading**: capture-now always captures at
  the current location regardless of mode. Per the v2 contract, `override`
  = fly to the picked point, capture, resume the interrupted task. Fix
  execution AND make the UI say what each mode will do.
- **Second-target dispatch bug**: with two confirmed targets, only the
  first is investigated. The queue must drain: a finishing IUAS takes the
  next serviceable job; an idle capability-matching IUAS takes it
  immediately.
- **Hover altitude oscillation ±0.5 m**: suspect the coordination overlay
  (bias engage/release limit cycle — KNOWN-ISSUES #1 — or hold/goto
  re-target interactions with the effective-AGL floor). Diagnose from
  journals (bias values over time), fix at the source, and add a
  regression check (hover variance bound) to --verify.
- **Post-task idle**: drones hover in place indefinitely after a task.
  Interim: an explicit idle policy hook (hold / return-to-slot / RTL after
  timeout) with a config default; the real answer lands with strategies
  (§2).
- **Acoustic flyover pattern**: new uas-flight pattern — transit at cruise
  AGL, dip to a low AGL over the target (omnidirectional mic needs
  proximity, not orbit), climb out, optionally repeat on a cross-axis —
  and dispatch selects pattern by sensor type (orbit for camera, flyover
  for audio).

## 2. Service strategies & authoring (design)

**The need** (owner's scenario): one camera IUAS, two targets. The
provider must either accept-and-queue the second interrogation while
finishing the first, or deny it — and the requester must then re-ask up to
a policy-bounded time or accept the outcome. With more camera UAS, idle
ones take the job, with idle-hover priority conditioned on reasonable
remaining flight time.

**Recommended placement** (the owner asked for input):

1. **Strategy = data, not code.** A strategy document is a signed,
   manifested record (its own stratum) on a fleet or mission chain:
   provider-side rules (queue depth, deny conditions, priority terms like
   `idle-first`, `flight-time-floor`) and requester-side rules (re-ask
   backoff, give-up horizon, fan-out). Agents *interpret* strategy records;
   they never hardcode them. This keeps strategies auditable, replayable,
   diffable, and associable with outcomes — the same association story as
   run-configs.
2. **Interpreter lives in the fleet plane**: new crate `uas-fleet-strategy`
   — kinds + a small deterministic evaluator consumed by (a) the agent's
   service ack path (accept/queue/deny + queue position in the Ack detail)
   and (b) the dispatcher's candidate ranking (idle-first with flight-time
   condition). Start with exactly the owner's scenario as the reference
   strategy shipped as a record, not a constant.
3. **Authoring frontends are pluggable instruments** that all emit the same
   strategy records: a forms/JSON editor first (cheap), then LLM-assisted
   authoring (draft-from-intent, always producing a reviewable record —
   Pilot-style: drafts, never signs), then node-graph editing. None of
   these live inside the dashboard core; they are surfaces over the
   strategy stratum, discoverable via manifest/render-contract like
   everything else.
4. **Onboard autonomy seam**: objective records (`maximize-coverage`,
   `investigate-all`, metric weights + constraints) interpreted by an
   onboard planner in the agent (drawing on uas-flight primitives) — the
   same record grammar, so external command and onboard autonomy are two
   interpreters of one authored intent, switchable per ceremony-gated
   config.

## 3. The NDF-native dashboard (design)

The owner's read is correct: the dashboard is a v2-shaped surface that
*consumes* NDF data but isn't *made of* NDF yet, and it must not dead-end
into a feature/data silo. Target architecture ("builder mode"):

1. **Minimal core surface** (map + safety verbs + the abort ladder —
   always present, never malleable away).
2. **Everything else is composed**: drone cards expose a namespace; the
   user browses data, capabilities, and semantic manifests from that
   namespace (discovery = chain/manifest enumeration, authorized by the
   viewer's frontier); picking a datum surfaces the render contracts the
   dashboard can Express (full-circle gauge) and alternatives
   (semi-circle dial) — user places the binding on the surface or a
   pop-out. Layouts are themselves records (waterline's P7 LayoutStore
   pattern; pin-vs-re-resolve per our WATERLINE-INPUT position).
3. **Bindings are kind-scoped, never device-scoped**: a placed gauge binds
   to (manifest kind, subject pattern), so pointing it at another drone
   works, and absence degrades gracefully: Express → Approximate (loss
   chips) → Refuse/Unresolved → the render-contract text+value baseline —
   never a broken widget.
3½. **The surface catalog (practicality requirement, round-3½)**: the
   dashboard must publish a catalog of ITSELF — the render contracts it
   can Express, the data kinds it understands, and its **surface-native
   widgets**: ready-to-use, zero-authoring, zero-network-pull building
   blocks (gauge, dial, sparkline, tile, log strip, map layer slots).
   Malleability without a stocked shelf is a blank-page problem; the
   catalog IS a manifested document (the instrument descriptor grown
   inward), so browsing "what can this surface do" uses the same
   machinery as browsing "what does this drone publish".
4. **Manifest-powered help**: every placed element supports inspection —
   what am I, what do I show, from which chain, under which contract,
   with which losses — via element lookup on the manifests. This is how a
   stranger uses someone else's custom surface.
5. **Lifecycle records surface here**: builder-mode drone cards grow the
   build/calibration/maintenance/firmware panels from `uas-fleet-records`'
   stratum + contracts (already authored) — the first real proof of
   cross-vertical browsing.
6. **Migration honesty**: the current WS/canvas dashboard remains the
   operational surface while builder mode grows beside it (a parallel
   route consuming uas-console bindings), so we never regress field
   readiness while escaping the silo shape. The WS schema becomes an
   implementation detail behind bindings, not the product.

## 3¾. Network layer redesign (round-3½)

The owner's critique: phase-1 lines re-commit the unicast fallacy
visually on a broadcast medium; per-line stats are useful but the form is
misleading. Full revision recorded in `NETWORK-VIZ.md` "Revision 2":
lines demoted to an earned, toggleable overlay-bearer sub-layer; broadcast
media render as fields; the data-centric backlog (interest/data heatmaps
by namespace, span-fed named-data traceroute, data-centric ping, namespace
lens, real-radio stats under the never-synthesize rule); GCS position as
pluggable data (sim export / field positioning backend / manual survey),
never chrome.

## 4. Fleet lifecycle surfacing (queued)

`uas-fleet-records` manifests + contracts exist; surfacing = the drone
cards above + a Sextant docking already prepared in uas-fleet-instrument.
Ship after the strategies interpreter so cards can also show
strategy/objective state.

## 5. Kept in view

- Frame the replay/recording rework against the chains (recordings are
  derived artifacts; the chain is truth) — no second source of truth.
- Every new kind/contract in this round follows association-over-hash
  presentation and lands in the waterline feedback loop.
