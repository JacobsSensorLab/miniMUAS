# Feedback log → ndn-workspace & NDF (refounding) maintainers

Living log of friction, bugs, ideas, and praise encountered while building
miniMUAS v3 on these frameworks. One entry per item; keep entries dated and
specific enough to act on (repro steps or file/API names). Move items to
"Delivered" once handed to the maintainers.

Format:

```
## [YYYY-MM-DD] <short title>
- **Project:** ndn-workspace | ndf-rs | ndn-sim | ndn-ext | ndn-service | flotilla
- **Type:** bug | friction | missing-feature | idea | praise
- **Context:** what we were building when we hit it
- **Detail:** what happened / what we expected / suggested fix
```

---

## Open

## [2026-07-10] No metadata slot on Invocation/Response — trace context must be smuggled per-op
- **Project:** ndn-workspace (ndn-service-core)
- **Type:** missing-feature (flagship item)
- **Context:** miniMUAS v3 wants W3C trace context to ride every service
  invocation so GCS-dispatch → drone-execution → artifact-publication is one
  distributed OTLP trace — the "next-level opentelemetry" showcase.
- **Detail:** `Invocation { op, request, requester }` / `Response { producer,
  payload }` have no extension slot, so cross-cutting concerns (trace context,
  deadlines, auth beyond the ndnsf user_token) must be appended in-band to
  every op's Frame. Request: a `metadata: BTreeMap<String, Bytes>` (or one
  reserved appended frame field) on both types, uniform across carriers.

## [2026-07-10] Keel matcher from an external consumer — verdicts all work, wants an explain() and per-manifest match
- **Project:** flotilla (render-contract)
- **Type:** friction + praise
- **Context:** uas-console binds its lens intents through the real matcher
  (match → authorize → instantiate). All four verdicts, C9 loss demotion,
  C10 frontier gating, deterministic selection, and the IM0 raw.inspect
  reroute floor validated end-to-end; no_std spec crates path-depped from a
  std workspace with zero friction (F54 held).
- **Detail:** (1) no manifest-scoped match entry point — binding one intent
  for one vehicle's manifest runs the fleet-wide match and filters after;
  wanted `match_for(dag, manifest, ...)` or a cached-match handle, else the
  bind-shaped API nudges consumers into re-matching per bind; (2) every
  consumer will hand-roll the three-silences diagnosis (Refused vs
  Unresolved vs no-offer vs floor-filtered) — an `explain(matches, intent,
  floor)` companion to select would keep consumers honest for free;
  (3) `contract_via -> Option<&Via>` conflates "Express clause with no via
  authored" with "nothing to walk" — a Result with reason would be kinder;
  (4) cosmetic: FrozenDag::insert_document maps internal decode failure to
  EncodeError::NestedAttribute, a lossy name that cost a debugging minute.

## [2026-07-10] ndf-spark as an RC transport — clean core, two notes
- **Project:** ndf-rs (ndf-spark)
- **Type:** friction + praise
- **Context:** uas-rc rides SparkEmitter/SparkAcceptor over UDP for
  1-to-many RC frames (50 Hz, loss-honest). Envelope cost measured at
  ~0.05 ms per frame at this rate — negligible; SP-3 replay judging
  refused an identical replayed datagram before our frame ledger saw it
  (exactly the property RC wants). Sync-std-friendly core (no tokio
  required) was a pleasant surprise.
- **Detail:** (1) the acceptor hashes wire bytes as sent, so honest
  loss-injection testing needs the stamp/carry split exposed — worth a doc
  note since any transport-testing consumer will hit it; (2) on sender
  restart the acceptor resets (monotonic=false) but consumers keeping their
  own seq ledgers must reset them on instance rotation or stale-drop the
  restarted stream — a "carry your ledger across instance rotation"
  cookbook note would help.

## [2026-07-10] First external flotilla manifest authoring — works, needs sugar
- **Project:** flotilla (manifest / manifest-derive)
- **Type:** friction + praise
- **Context:** authored L1 semantic manifests for the v3 fleet kinds
  (telemetry, coord, capability, search, frame container) in
  uas-fleet-data/src/manifests.rs — to our knowledge the first consumer
  outside the suite. All documents pass R13 decode∘encode byte identity.
- **Detail:**
  1. `#[derive(Manifest)]` is effectively unreachable for a consumer that
     only deps `manifest`: it lives in `crates/tools/manifest-derive` and its
     generated code calls a second crate (`manifest-describe`) — a facade
     re-export (manifest-kit?) would fix adoption.
  2. Attribute keys are raw `Hash`es with zero authoring sugar — every
     `unit = "m"` needs a hand-built key Term + `term_hash` threading. An
     `Attribute::text(key_term, "m")` helper or label-resolving builder in
     core would delete most consumer boilerplate (we hand-rolled one).
  3. The recipe for closed string vocabularies (marker terms +
     `narrower-than` edges, since the flat-attribute law rejects lists) is
     correct but undocumented — cookbook entry wanted.
  4. `ListOf(TermOf(record_hash))` vs inline-Record convention for "list of
     records" is only discoverable from manifest-derive's generated code.
  5. Praise: `Decimal::from_canonical`/`normalize` split is exactly right
     for threshold authoring; the encode/decode/encode_decoded trio makes
     R13 conformance a three-line test; zero-dep crate compiles instantly.
  6. Minor: `EncodeError` (3 variants) has no Display/Error impl — consumer
     expect() messages carry the debugging load.

## [2026-07-10] Integration-survey friction batch (12 items)
- **Project:** ndn-workspace / ndf-rs
- **Type:** friction/bug batch
- **Context:** deep API survey of ndn-engine, ndn-service(+rpc/ndnsf),
  ndf-apps, ndf-spark, ndn-sim before wiring the v3 agent. Full detail with
  file:line cites in the survey's §8 (kept in the v3 scratchpad; summarized
  here).
- **Detail (abridged):** (1) the metadata-slot gap above; (2) generated
  service clients store the carrier by value with no `&C` accessor and
  carriers aren't Clone — sharing one carrier across two typed clients needs
  caller-side Arc wrapping; (3) feature-gate discoverability: `NdnsfCarrier`
  vanishes without `--features driver` (and `FaceRpcCarrier` without
  `engine`) with no compile-time pointer at the missing feature; (4) doc
  drift: `RpcCarrier` header still says the `#[ndn_service]` macro is
  "planned" (it shipped, with tests); ndn-sim self-describes as "ndn-lab";
  (5) four distinct "publish" verbs across layers (ndn_app::Node,
  AppRuntime, NamedPublisher, service leaf) — correct locally, rough to
  navigate cold; (6) `add_face` takes a raw CancellationToken with no
  guidance to child it off the shutdown token — faces silently outlive
  shutdown; (7) `InProcFace::new(FaceId, ..)` invites id collisions, nothing
  enforces uniqueness at insert; (8) **ndnsf provider-authorization gap**:
  any trusted group member may serve any service — `ServicePolicy.providers`
  exists but is unenforced on ACK acceptance (doc-comment only); (9) FIB
  silent-failure cluster: `make_reachable` saves app-layer users, raw engine
  consumers get no shadow warning though `explain_route` exists; (10) Spark
  checkpoint retention (FS-7b) explicitly unowned — anchors accumulate;
  (11) service framing is length-prefixed-in-order while trait docs promise
  skippable-TLV — append-only evolution is safe, anything else silently
  isn't; (12) ndn-sim MavlinkReader thread never joins (blocking recv) —
  lingers on teardown in long-lived processes.

## [2026-07-09] Refounding rev pin vs ndn-workspace HEAD — no breakage at scaffold depth
- **Project:** ndf-rs / ndn-workspace
- **Type:** praise (with a watch item)
- **Context:** miniMUAS v3 scaffold taking path deps on flotilla `manifest`,
  `render-contract`, and refounding `ndf-core`.
- **Detail:** refounding README pins ndn-workspace HEAD `5798fa3f`; our ndn-rs
  checkout is at `043d3c15`. All three crates compiled clean and our workspace
  tests pass. Watch item: deeper deps (`ndf-apps`, `ndf-surface`,
  `ndn-engine`-graph) haven't been exercised yet — first M3 build will tell.
  Suggestion for maintainers: a `refounding/COMPAT.md` (or CI matrix) stating
  which ndn-workspace revs the refounding is known-good against would remove
  guesswork for consumers.

## Delivered

(none yet)
