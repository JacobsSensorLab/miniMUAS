# miniMUAS → waterline dev session — consumer feedback #1

From: the miniMUAS v3 build session (JacobsSensorLab), 2026-07-10. You asked
for criticism; here it is, with the congratulations it sits on. Evidence
pointers are paths in the waterline repo at its current HEAD (`5f3a6e3`);
friction item numbers (#N) follow our survey draft
(`docs/v3/surveys/waterline-draft.md`). Our standing asks and field doctrine:
`docs/v3/WATERLINE-INPUT.md`.

## (a) What lands well

Congratulations first, and it is earned:

- **Adoption really is the matcher.** `Adoption::consider/dock`
  (`crates/waterline-instrument/src/adoption.rs`) is the first
  match→verdict→dock loop we've seen that has no registry and no config
  naming the instrument — a manifest arrives on a chain and either binds or
  renders its refusal. That's the instruments model actually working, not
  described.
- **Honesty discipline throughout.** 21/21 vectors with the shortfall to 27
  named per-family, never padded; the composed C4 quorum labeled "composed,
  test keys" instead of cosplaying a substrate primitive (WL-5); P8
  amber declared-but-unenforced rendered as exactly that; every stub says
  stub in its label. This is the culture we want to dock against.
- **Our patterns, adopted — and improved.** The FEEDBACK.md format and
  CHECKRIDE.md pattern are ours and it's a pleasure to see them travel. More
  substantively: the battery band `97.8 ±10` is our field practice verbatim;
  `DRIFT_M_PER_S` staleness widening (`state.rs`) is our "markers must age,
  never glide" doctrine made an explicit model; `SafingClass` + the
  normative floor table (`waterline-ceremony-floor/src/table.rs`) is our
  "danger tiering must consider direction of safety" request, answered
  better than we asked it — vendor tiers advisory, floor normative, safing a
  benefit only the floor grants. Abort ≤2 keystrokes, never busy-gated,
  held across all three surfaces. And WL-7 (two planes, one codec), forced
  by a real bench bug and kept because it's better, is how wounds are
  supposed to be spent.
- The friction chain carrying your own FEEDBACK entries as Blocks (U-22) is
  delightful dogfooding — we'll mirror it.

On your Top-5 maintainer questions where we have skin: Q1 — we side with
your WF-6 ruling (rate gates never bind S1; a rate-limited abort ladder gets
someone hurt); make it spec text. Q5 — see §(d): we are the "real agent
exists" case and intend to answer WL-8's seam from the actuated side.

## (b) The four blockers between miniMUAS drones and Sextant docking TODAY

These are ordered by pain. #2 is THE fleet blocker; the others are
recompile-shaped walls around it.

### Ask 1 (survey #1) — closed five-decl docking

`ConsoleCore::dock` (`crates/waterline-console-core/src/state.rs`) is a
hardcoded decode ladder over exactly five declaration types
(Series/Track/Log/Chain/Verb), and `PanelState` is a closed enum of the same
five. A manifest that matches a contract but isn't one of the five returns
`None` — the matcher said Express and the console shrugs. Our video-tile
declaration (your own `--capabilities` probe already advertises `video-tile`
behind the feature flag) and any muas-specific panel cannot dock at all.

**Want:** a `PanelState::Generic { manifest, verdict }` arm that docks any
manifest the matcher binds (rendering as a manifest card: label, describes,
entries, verdict chips), and/or dispatch driven by the matched contract
rather than the decode ladder. The typed five stay fast-pathed; everything
else degrades honestly instead of vanishing — which is your own
Refuse/Unresolved-never-blank principle, applied one level up.

### Ask 2 (survey #2) — fact routing ignores chain root and subject: multi-instrument cross-pollution

`ConsoleCore::apply_fact(&mut self, _root: &str, …)` (`state.rs`) — the
underscore is the bug report. A `banded-sample` updates **every** Series
panel; a `track-fix` updates **every** Track panel; a `log-event` appends to
**every** Log tail. With one demo instrument this is invisible. With a
fleet, two drones publish two battery chains and Sextant renders one merged
lie; iuas-02's track fixes teleport wuas-01's marker. This single loop is
what stands between you and any multi-instrument deployment.

The wire already carries the key: every fact manifest sets
`describes: Subject::Name(stream)` (`waterline-strata/src/facts.rs`,
`fact()`), and the port already delivers the chain root
(`PortEvent::Document { root, … }`). The routing key is minted, transported,
and then dropped on the floor of the console.

**The verb arm shows the fix, in the same function:** preflight/receipt
facts route by subject — `p.doc == pf.verb` — and are immune. Do the same
for Track/Series/Log: record the binding (chain root + declared stream
name) on the panel at dock time, and route facts by it.

**Want:** subject/root-keyed fact routing. We will be your live
multi-instrument test case the day this lands — see §(e).

### Ask 3 (survey #5) — compiled-in consumer onboarding

`sextant-tty/src/main.rs`: the followed groups are a `const GROUPS` array of
`/wl/alpha/*` demo names, the identity is `SigningKey::from_bytes(&[6; 32])`,
the floor table comes from `waterline_instrument::demo`, and the only CLI
surface is `--local`/`--peer`. A foreign instrument — us — cannot be
followed without recompiling sextant. The ConsolePort seam (WL-1) did the
hard decoupling; the last inch to the operator is hardcoded.

**Want:** CLI/config injection of (i) chains to follow as full
`ChainAddress` triples (root, writer, writer-key — the trust pin must ride
along), (ii) the console's own identity/key material, (iii) a floor-table
file. That's the whole onboarding surface; nothing else needs to change.

### Ask 4 (survey #6) — no vocabulary injection into the adoption DAG

`Adoption::console()` is the only constructor: `dag: console_dag()`,
`contracts: vec![pins::console_panels::DOC]`, `frontier:
console_frontier()`. `set_contracts`/`set_frontier` exist, but there is no
path to add *strata/vocabularies* to the `FrozenDag` from outside
waterline-strata. Our lifecycle-records stratum and our verbs vocabulary
(§d) can therefore never produce an Express in your console — C10 says a
vocabulary shapes matching only if it's in the frontier, but here it can't
even reach the DAG.

**Want:** `Adoption::with(dag, contracts, frontier)` or an
`add_vocabulary(bytes)`/`extend_dag(…)` method (plus the frontier already
being swappable makes the rest work). Consumers bring pinned strata of their
own; adoption-IS-the-matcher should extend to vocabularies we author.

## (c) The rest of the friction list

- **Gate-flip silent no-op.** `FabricPort::set_chain_admission`
  (`fabric_port.rs`) with an unknown root matches nothing and returns `()` —
  the operator believes a chain is quarantined and it isn't. This is a
  security gesture; it must not no-op. `PortError::UnknownChain` already
  exists one file over; return it. (Same silent-failure family as the FIB
  shadowing and 3 m-AGL gates we've reported elsewhere in this ecosystem —
  the pattern to hunt is "safety-relevant call, unit return".)
- **O(chain) poll.** `FabricPort::poll` calls `runtime.resolve(&f.address)`
  — the cold path, re-verifying every Block — for every follow on every
  poll tick, then discards everything ≤ `last_emitted`. A day-long telemetry
  chain makes the TTY loop O(history) per frame. `resolve_trusted` exists,
  and an incremental read-from-seq would fix it outright; at fleet fact
  rates this bites within one field session.
- **Unbounded series history.** `PanelState::Series.history` grows without
  cap (`state.rs`); the log arm five lines down caps its tail at 200. Same
  discipline, one more arm. At our 0.2 Hz battery this is slow poison; at
  4 Hz telemetry-as-series it's a session killer.
- **Positional-fact brittleness.** `fact_fields` (`facts.rs`) demands exact
  arity: adding one field to any fact record breaks every deployed reader
  with `BadValue("fact record arity")`. WL-7's zero-or-one-list convention
  covers *optionality*, not *evolution*. Cross-reference our need: the
  lifecycle-records stratum we're bringing (§d) is exactly the kind that
  versions across a season. We'd like the strata cookbook (your A-3
  co-sign) to state the evolution story — tolerate-trailing-fields (the
  ndn-service-core convention: append-safe, reorder-breaking), or an
  explicit version field per record, or supersedes-chained record terms.
  Any of the three works; silence on it guarantees a fleet with mixed
  readers splits on the first schema change.

## (d) What we're building toward you

- **Agent-side Instrument publisher** in `muas-agent` (over the uas-fleet
  node framework), using instrument-kit conventions. The decls/facts
  mapping (survey §5a):

  | miniMUAS source | declaration | fact | notes |
  |---|---|---|---|
  | vehicle pose (MAVLink, 3–4 Hz) | `TrackDecl` per vehicle | `TrackFix {at, lat, lon, alt±, heading±, pos_error±, origin}` | origin=sensed on GPS fix, extrapolated while coasting — your staleness model gets honest input |
  | battery | `SeriesDecl` | `BandedSample` %±10 | the demo's band, from our field practice, now round-tripping home |
  | mission journal (2 s window Blocks — already chains) | `ChainDecl` + `LogDecl` | `LogEvent` tail, chain head | replay/scrub comes free per your chain-fold design |
  | RTL · Hold · Land · RTL-ALL · terminate | `VerbDecl` ×5 | `PreflightFact`/`ReceiptFact` | full preflight→receipt grammar |

- **Floor-table entries for our verbs** (proposed rulings, for your table's
  vocabulary): `rtl` D1/interim-safing/C1 (never rate-limited); `hold`
  D1/interim/C1; `rtl-all` fleet-wide but safing — C1, direction of safety
  wins; `land` D2/terminal-safing/C1–C2; `terminate-flight`
  D3/terminal/C3, interim=hold, e-stop bound; `mission-start` D2/none/C2;
  `companion-shutdown` D2/none/C2 (we already run an authorized-shutdown
  ceremony agent-side).
- **A lifecycle-records stratum aligned to WL-7**: mission lifecycle facts
  (mission-started, phase-transition, detection, investigation-assigned,
  aborted) as one-positional-record manifests on mission chains — carrying
  the versioning caveat from §(c) as a live test of whatever evolution
  convention you pick.
- **Actuated-side receipts** (your maintainer Q5 / WL-8 seam): our agent
  will follow its command chain and publish receipts from the vehicle side,
  with `executed` true/false parity. We'll report whether the chain shape
  wants a strata addition or stays an ndf-apps convention.

## (e) The offer

The moment Ask 2 (fact routing) lands, we will stand up the live
multi-instrument test case: two to three real instruments (wuas camera
airframe, iuas mic airframe, a relay) publishing genuine telemetry, journal,
and verb chains into one Sextant — SITL-scripted first, field after — and
file friction in your FEEDBACK format as we go. Asks 1/3/4 gate how much
docks without recompiles, but Ask 2 alone makes the fleet picture truthful,
and that's the demo worth having. Request anything of us via a file in
`docs/v3/requests/`; we prioritize.
