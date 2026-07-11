# miniMUAS v3 → ndn-workspace / ndf-rs maintainers — field report #2

From: the miniMUAS v3 build session (JacobsSensorLab). Dated 2026-07-11.
Follow-up to field report #1 (2026-07-10). This report has ONE theme the
owner asked us to surface directly:

> **We keep hand-rolling infrastructure that belongs in your layer.** Each
> piece works, but it's a private reimplementation of something every NDF
> consumer will need. If it lived upstream, it would be general, tested
> once, and available to the whole ecosystem instead of buried in one
> drone app.

Below is the running inventory of what we built because the framework
didn't offer it, sorted by how clearly it belongs upstream. Every item is
real code in our tree today; file/crate pointers on request. Treat this as
a "promote these" backlog, not a complaint — you've fixed our asks same-day
before, and this is the higher-leverage version of that.

## Tier 1 — pure infrastructure, obviously yours, we just filled a hole

1. **A "fetch to current head then stop" read.** `ndf-apps` `follow()` is a
   long-lived subscription; a catch-up-then-return reader (artifact
   generation, report tooling, a console loading history) has no "history
   complete" signal, so we drain on `quiet_rounds × step_timeout` of
   silence. We hand-rolled a budgeted quiet-round drainer in TWO separate
   places (artifact resolver, strategy/lifecycle chain folds). Ask:
   `AppRuntime::sync_to_head(addr) -> ResolvedChain` (or a `Follow` mode
   that completes at the current head). This is the single most-duplicated
   thing across our consumers.

2. **A record-chain read/write helper.** We now have THREE crates
   (`uas-fleet-records`, `uas-fleet-strategy`, the agent journal mirror)
   that each implement the same pattern: typed record → versioned
   envelope → publish over `AppRuntime` → fold a chain into "latest valid
   per kind, honoring supersedes." We factored ours into a shared
   `RecordWriter/RecordReader`, but that pattern is the NDF app model —
   it should be an `ndf-apps` (or a thin `ndf-records`) facade so every
   app isn't reinventing the envelope + fold + supersede semantics.
   Concretely: `uas-fleet-records`' `LifecycleRecord` trait is a closed
   6-family taxonomy, so `uas-fleet-strategy` could NOT reuse it and had
   to mirror the whole thing — the generic version should be
   taxonomy-open.

3. **Foreign-flow carriage over a SimLink (ndn-sim).** Reported in #1;
   still true and now bitten twice. Our ndf-spark telemetry lane and our
   RC frames are non-NDN UDP flows; ndn-sim can't carry them over a
   `SimLink`, so we hand-rolled an impairment relay (`nettap.rs` +
   the earlier spark impairment relay) duplicating `LinkConfig` semantics.
   Ask: `RunningSimulation::bridge_udp_flow(link_profile)`.

4. **Per-name / per-prefix traffic accounting (ndn-sim).** For the network
   viz namespace lens we needed per-prefix interest/data counters;
   `RunningSimulation::face_stats` exposes per-face TOTALS only, `SimTracer`
   only face up/down. We hand-rolled `nettap.rs`: a UDP relay interposed at
   the bridge seams that decodes each datagram's L3 name and accounts by
   prefix. This is generic NDN observability every network console wants —
   it belongs in ndn-sim (and, for real deployments, is exactly what
   `ndn-observability` spans should expose). Ask: name-aware counters on
   the sim fabric, and a documented per-prefix stat surface.

5. **The three-silences matcher diagnosis (flotilla).** Reported in #1;
   we built `diagnose_no_selection` (Refused vs Unresolved vs no-offer vs
   floor-filtered precedence) AND the console next door built the same
   thing. Two independent consumers hand-rolling identical logic is the
   textbook promote signal. Ask: `explain(matches, intent, floor)` beside
   `select_best_for`.

6. **Manifest attribute-authoring sugar (flotilla).** Reported in #1; we
   hand-rolled a label-resolving builder (`Semantics`) because attribute
   keys are raw hashes. We've now authored THREE strata this way
   (fleet-data, lifecycle-records, strategy) — every one carried the same
   hand-rolled helper. Ask: `Attribute::text(key_term, value)` +
   label-resolving builder in the `manifest` core.

## Tier 2 — a mechanism exists but has no upstream driver, so we drive it

7. **`PresenceActuator` is a seam with no engine (ndf-policy).** The
   presence *declarations* exist; nothing enforces them, so our MCU
   windowed-follow / replica-tier logic (lifecycle-records replication) is
   entirely hand-rolled against a trait with no implementation. Either ship
   a reference actuator or document that consumers own it (right now it
   reads as "provided" but isn't).

8. **`NdnsfCarrier` has no engine binding (ndn-ndnsf).** Reported in #1;
   `SvsPubSub::join` takes raw channels, so we hand-rolled a datagram pump
   to run the NDNSF comparison carrier at all. Ask: `SvsPubSub::over_face
   (engine, group)`.

9. **No latest-wins producer (ndn-app).** Reported in #1; every one of our
   ~8 latest-wins streams (telemetry, coord, search, capability, video,
   queue, rc/status, net) hand-rolls the freshness-0 + MustBeFresh CS
   dance. Eight instances in one app. Ask:
   `Node::serve_latest(watch::Receiver<Bytes>)`.

## Tier 3 — safety / correctness scaffolding we'd rather not own privately

10. **Trace-context metadata slot (ndn-service-core).** The flagship #1
    item, unchanged: no per-invocation metadata slot, so distributed OTLP
    trace context is smuggled per-op. Every service-based NDF app will
    hand-roll this. Ask: `metadata: BTreeMap<String, Bytes>` on
    `Invocation`/`Response`.

11. **Provider authorization enforcement (ndn-ndnsf).** `ServicePolicy
    .providers` exists but is unenforced on ACK acceptance (doc-comment
    only). We run `.insecure()` and gate authorization ourselves. Security
    scaffolding is the worst kind to hand-roll per-app. Ask: enforce it, or
    make the gap loud at construction.

12. **Stratum pin lifecycle ergonomics (flotilla bench).** We've now taken
    TWO external strata (lifecycle-records, strategy) through
    `bench compile --lock`. It works — genuinely good — but every external
    author hand-vendors the ride-on pins per repo (no shared store
    convention), `--lock` rewrites and strips comments, and there's no
    read-only compile for CI drift-checks. Ask: a shared store convention +
    `bench check` (verify pins without rewriting).

## What this costs the ecosystem (the argument for promoting these)

We are, as far as we know, the most complete external NDF consumer right
now, and the waterline suite is the second. On items 2, 5, and 6 we and
the suite have *independently hand-rolled the same thing* — that's the
proof these aren't app-specific. Every one of these lives in our tree as
working, tested code you're welcome to lift directly; several are ~50-line
facades over machinery you already have. Promoting them turns N private
reimplementations into one tested primitive and makes the next consumer's
first week dramatically shorter.

## Positive, for balance

The core substrate keeps holding: `ndf-apps` publish/fold is genuinely
zero-ceremony once you know the pattern; the flotilla matcher and the
bench pin lifecycle worked for a second external stratum with no drama;
ndf-spark's SP-3 replay judging is doing real safety work in our RC path
(refusing replayed control frames before our ledger sees them) — that one
we did NOT have to hand-roll, and it's exactly the kind of thing that
makes building on NDF worth it. More of the tier-1 list promoted to that
standard and the "hand-rolled" column empties out.

## Standing offer (unchanged)

Every item here is lift-ready code in JacobsSensorLab repos your session
can be granted. Point us at whichever you promote and we'll be the
first-consumer test case, same-day, the way we have been.
