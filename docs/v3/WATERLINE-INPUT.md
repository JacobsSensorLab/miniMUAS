# Fleet-management UI/UX input → ndn waterline suite (Anchor · Capstan · Sextant)

From the miniMUAS operator/builder perspective, grounded in two field seasons
of the v2 GCS dashboard (feature inventory: `surveys/minimuas-v2.md`) and the
suite docs in `~/Downloads/latest`. Organized as: what fleet ops actually
demands, answers to the suite's flagged open questions, and what we'd need to
adopt the suite's instruments model. Living doc — updated as v3 milestones
produce evidence.

## What running a small UAS fleet actually demands of a console

1. **The map is the home screen, everything else is peripheral vision.** In
   the field the operator watches vehicle markers ~90% of the time. Panels
   must be glanceable from 1 m in sunlight; v2's per-vehicle tiles earn their
   place only because each fact is a color-coded chip (link age
   green/yellow/red, battery graded, ⚠ avoidance bias). Sextant's fleet map
   should assume this posture: map primary, instruments dockable around it.

2. **Latency honesty beats smoothness.** v2 smooths 3–4 Hz fixes to 60 fps
   marker motion (τ≈0.25 s, shortest-arc heading) — good — but the *link tag*
   ("last heard 2.1 s ago") is what the operator actually trusts. A marker
   must visibly age (v2: stale-grey) rather than keep gliding on
   extrapolation. This is the honest-numbers doctrine applied to position:
   an extrapolated pose IS an interval that widens with silence.

3. **Preview must be the exact artifact that executes.** v2's raster preview
   renders the very `RasterPlan` the drone flies (shared geometry code).
   Every "what-if" surface in Capstan/Sextant should make this guarantee
   explicit — a preview computed by different code than the executor is a
   field incident waiting.

4. **The abort ladder must be reachable from anywhere, always.** RC override →
   per-vehicle RTL/Land/Hold → RTL ALL. No modal, no navigation, no more than
   one confirm on the whole ladder (v2 confirms only mission start). Any
   suite chrome that can cover these buttons is a defect. Corollary: disabled
   /degraded states must never remove the safety verbs (v2's disabled vehicles
   still accept RTL/Land/Hold).

5. **Replay is a first-class mode, not an afterthought.** v2 records every
   broadcast and replays through the *same* dispatch handlers with
   deterministic scrub. For a trust console this generalizes: any view should
   be re-derivable at time T. NDF chains make this nearly free — please keep
   "re-resolve at historical head" a supported RenderSession mode.

6. **Multi-vehicle asymmetry is normal.** A camera drone and a mic drone work
   the same target; a disabled vehicle still telemeters. Fleet UI that
   assumes homogeneous members (one template per node) will fight us —
   capability chips per vehicle (v2: 📷/🎙 from CapabilityProfile) should be
   the suite's atom, and dispatch UIs should filter by capability, not type.

## Answers to the suite's flagged open questions

**Anchor's rings-not-tables at fleet scale (40+ grants).** Workable *if* the
fleet is a first-class object. Per-vehicle grants (telemetry read, command
write, sensor tasking, shutdown) × N vehicles as individual beads will be
noise by vehicle five. What we'd want: a **fleet/role bead** (e.g.
"iuas-fleet: investigate+rtl grantees") that expands on focus into its member
chains. The ⌘K summon + tables-one-keystroke-down answer is right; the rings
must aggregate, not enumerate. Ring membership churches every field day
(vehicles added/benched) — make join/leave a C1 gesture inside a standing
fleet context, not a fresh ceremony.

**C0–C4 gravity at fleet scale.** The tier idea maps beautifully onto what v2
hand-rolled: shutdown's type-the-vehicle-id IS a C2-equivalent; mission start's
single confirm IS C1; RTL needs to stay C0/C1 (it's the *safe* direction —
never add friction to the abort ladder). Two requests: (a) **danger tiering
must consider direction of safety** — an irreversible-but-safing action (kill
throttle on a flyaway) needs LESS ceremony than an armful takeoff, and a naive
danger=D3→C4 mapping would get someone hurt; (b) "darken the room" C4 theatre
is fine indoors, but in sunlight at a field table the gesture must be
tactile/typed, not luminance-based.

**Fields/ripples ("draw the medium, not the link") for large fleets.** For our
scale (3–10 airborne nodes, 1–2 ground) this is genuinely better than fake
edges — broadcast reach IS the operational question ("can iuas-02 still hear
the GCS at the far leg?"). Two needs: (a) ripples must be rate-limited /
aggregated above a few Hz per node or a 4 Hz telemetry fleet becomes visual
static — render *coverage* (the field) steady-state and *anomaly* (the missed
emission) as the event; (b) we need predicted-coverage-at-plan-time (shade the
raster area the mesh won't reach) — that's a killer feature no GCS has.

**Honest-numbers bands without clutter.** Progressive disclosure by zoom/
focus: chip shows the value, band appears on hover/focus/alarm. But two
intervals must be ALWAYS visible in a fleet console: position staleness and
clock uncertainty (v2 shows clock skew Δ per vehicle; upgrade it to the
named-time ±interval — "esp32-3 holdover ±41 ms and widening" is exactly the
right rendering). Battery percentage from MAVLink is a lie of precision; a
band there would be honest AND useful (we've landed on "%±10 or voltage").

**Pilot seams.** Observe→Ask is the one we'd use daily ("why did iuas-02
climb at 14:02?" → span citation — this is our OTLP dream query). Drafting
LVS rules in Trust Studio: yes, with the diff-before-apply they already
promise. Keep Pilot OUT of the dispatch path — an operator under stress must
never wonder whether a suggestion or a command is armed. Draft-vs-armed needs
a hard visual wall.

**Re-resolution vs anchoring (pin the layout).** Strong agreement with their
own "load-bearing UX" flag, with field evidence: operators build muscle memory
of glance targets (battery is top-right of tile 2). A re-resolve that reflows
tiles mid-mission would be actively dangerous. Requested semantics: **pinned
during an active mission session, re-resolve allowed at mission boundaries or
on explicit operator action.** Mission-mode = layout freeze is a concept the
suite could own generically ("operational hold" on a RenderSession).

## What we need to ship miniMUAS panels as instruments

The instruments model (separate process joins fabric over SHM face, publishes
namespaces + three-layer manifest, Sextant/Capstan render it) is exactly what
we want — v3 will present the agent and dashboard as instruments. To land it
we'll need, roughly in order:

1. A worked instrument example beyond the suite's own (manifest + edges +
   contract + the native renderer registry glue) — we'll happily be the guinea
   pig and document the friction.
2. Contract vocabulary for **live geospatial tracks** (position+heading+
   uncertainty time series) and **video tiles** — the two intents every fleet
   tool needs; we'd rather adopt suite-standard intents than mint
   `muas:*`-private ones that nothing else can express.
3. A story for **command/control intents** (not just measurement): our RTL-all
   button inside a Sextant panel needs the preflight→receipt grammar +
   danger-tier gating. P11 actuation-safety records seem to be the answer —
   an end-to-end example would unblock us.
4. Guidance on **replay**: can an instrument declare its namespaces
   chain-backed so the suite's time controls scrub it natively? That would
   let us delete our transport-bar code.

## Standing offer

v3 milestones (see ARCHITECTURE.md) will produce: OTLP traces of real
missions (Observe→Ask test fodder), a fleet trust-schema + zero-touch
provisioning exercise (Anchor test fodder), radio-mode comparison telemetry
(Sextant fields test fodder), and instrument-adoption friction reports. Ask
via a file in `docs/v3/requests/` and we'll prioritize.
