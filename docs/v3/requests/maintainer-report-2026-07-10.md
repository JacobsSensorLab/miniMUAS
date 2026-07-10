# miniMUAS v3 → ndn-workspace / ndf-rs maintainers — field report #1

From: the miniMUAS v3 build session (JacobsSensorLab). Dated 2026-07-10.
Context: over two days we rebuilt a real multi-drone system (miniMUAS) on
your stack — ndn-engine + ndn-service carriers + ndf-apps/ndf-spark +
flotilla manifest/render-contract + ndn-sim — as a four-repo Rust stack
(uas-flight → uas-fleet → uas-console → miniMUAS app). Everything below is
from code that compiles, tests, and runs today; evidence pointers are to
private JacobsSensorLab repos your session can be granted access to, plus
miniMUAS `docs/v3/` (branch v3). Longer per-item detail: `docs/v3/FEEDBACK.md`.

## 0. What your stack survived (the headline)

You have a real consumer now. In production-shaped use, not demos:

- **ndn-service-core**: our vehicle service (`#[ndn_service]`, 9 ops) hosts
  over FaceRpcCarrier on real engines with UDP faces; NdnsfCarrier runs as a
  comparison backend behind a flag. Policy-gated acks, real clients.
- **ndf-apps AppRuntime**: agent journals mirror into signed Block chains
  (one Block per 2 s window) alongside a power-loss fsync JSONL.
- **ndf-spark**: two independent uses — 4 Hz telemetry lane and a 50 Hz
  1-to-many RC stream (26-byte frames). Envelope cost ~0.05 ms/frame;
  SP-3 replay judging refused replayed datagrams before our ledger saw them.
- **flotilla**: to our knowledge the first external consumer of both the
  manifest calculus (L1 manifests for 5 fleet kinds, all R13
  decode∘encode byte-identical) and the Keel matcher (console binds
  lenses through match→authorize→instantiate; all four verdicts, C9 loss
  demotion, C10 frontier gating, and the IM0 raw.inspect reroute floor
  all behaved exactly as documented). F54 held: no_std spec crates
  path-depped from a std workspace with zero dep friction.
- **ndn-sim**: unmodified agents co-simulated via UDP interop bridges over
  lossy SimLinks; our whole v2-parity coordination validation suite runs
  on it, plus a three-profile radio comparison (miniMUAS
  `docs/v3/radio-comparison.md`).

Totals: ~570 tests across the stack consuming your crates, zero clippy
warnings, one SITL-verified MAVLink adapter, one live dashboard.

## 1. P0 — deployment is blocked on hosting/pushes (action needed)

Our NixOS deployment layer pins every source by rev from a remote. Today it
cannot, because:

1. **flotilla**: HEAD `b4ac691` is pushed, but our validated builds compiled
   against *uncommitted working-tree edits* to `crates/core/manifest`
   (`canon.rs`, `kernel.rs`) and conformance vectors. Please commit/push
   (or tell us those edits are abandonable and we'll re-validate).
2. **ndn-rs**: local HEAD `043d3c15` is not on the remote. Please push.
3. **ndn-ext**: local main is 10 commits ahead of origin. Please push.
4. **ndf-rs**: no git remote exists. The refounding is our app SDK — we
   need a hosting decision (org + visibility) to pin it at all.

The staged config (`minidronesys-configurations` branch `mini-muas-v3`,
README-V3.md) has the exact commented input lines waiting for these revs.
Related nice-to-have: a `refounding/COMPAT.md` stating which ndn-workspace
revs the refounding is known-good against — we verified the current HEADs
work (the `5798fa3f` pin note is stale) but had to find out by building.

## 2. P1 — the API gaps that cost us real design compromises

1. **No metadata slot on `Invocation`/`Response`** (ndn-service-core).
   Our flagship goal is one distributed OTLP trace per mission across
   GCS→drone→artifact. With no carrier-uniform extension slot, W3C trace
   context must be smuggled into every op's Frame. Ask: `metadata:
   BTreeMap<String, Bytes>` (or one reserved appended frame field) on both
   types, honored by all carriers.
2. **`Frame` has no `f64`/`f32` impl** (ndn-service-core). Flight
   contracts are number-heavy; we fell back to whole-struct JSON Frames,
   losing per-field length-prefixed evolution. An f64 impl or per-field
   serde fallback in the derive would fix it.
3. **No latest-wins producer primitive** (ndn-app). "Serve the freshest
   telemetry" requires discovering the freshness-0 + MustBeFresh CS
   interaction from `stages/cs.rs` tests. Ask:
   `Node::serve_latest(watch::Receiver<Bytes>)`.
4. **NdnsfCarrier has no engine binding** (ndn-ndnsf). `SvsPubSub::join`
   takes raw channels, so we hand-rolled a datagram pump; an
   `over_face(engine, group)` adapter would make rpc-vs-ndnsf comparisons
   apples-to-apples. Also: the documented provider-authorization gap (any
   trusted group member may serve any service; `ServicePolicy.providers`
   unenforced on ACK) matters for fleets — we'd use the enforcement.
5. **Keel matcher** (flotilla): (a) no manifest-scoped entry point —
   binding one intent for one vehicle runs the fleet-wide match and
   filters after (`match_for(dag, manifest, ...)` wanted); (b) every
   consumer will hand-roll the Refused/Unresolved/no-offer/floor-filtered
   diagnosis — an `explain()` companion to `select_best_for` would keep
   consumers honest for free.
6. **Manifest authoring sugar** (flotilla): attribute keys are raw hashes
   (we hand-rolled a label-resolving builder); `#[derive(Manifest)]` is
   unreachable without wiring two extra path deps (facade re-export
   wanted); the closed-string-vocabulary recipe (marker terms +
   narrower-than) needs a cookbook entry.
7. **ndn-sim**: (a) no way to carry a foreign UDP flow over a SimLink —
   our spark lane needed a hand-rolled impairment relay duplicating
   LinkConfig semantics; (b) UDP bridges are wall-clock-only, so
   VirtualKernel is unusable for external forwarders and regression
   suites run realtime with compressed parameters.

## 3. P2 — friction batch (short list; detail in FEEDBACK.md)

- Feature-gate discoverability: `NdnsfCarrier` without `--features driver`
  (and `FaceRpcCarrier` without `engine`) silently vanish — no compile
  error names the feature.
- `UdpFace::from_shared_socket`: sibling faces on one socket steal-and-drop
  each other's datagrams; docs don't warn against >1 face per socket.
- Generated service clients store the carrier by value (no `&C` accessor,
  carriers not Clone) — sharing one carrier across typed clients needs
  caller-side Arc.
- Carrier `serve()` loops have no cancellation leg (we abort tasks instead
  of draining).
- `add_face` takes a raw CancellationToken with no guidance to child it
  off the shutdown token — faces silently outlive shutdown.
- `InProcFace::new(FaceId, ..)` invites id collisions; nothing enforces
  uniqueness at insert.
- Spark checkpoint retention (FS-7b) is explicitly unowned — consumers
  must plan their own anchor retention; also, the acceptor hashing wire
  bytes as-sent means loss-injection testing needs the stamp/carry split
  (doc note wanted), and consumers keeping their own seq ledgers need a
  "reset on instance rotation" cookbook note.
- Service framing is length-prefixed-in-order while the trait docs promise
  skippable-TLV — append-only evolution is safe, anything else silently
  isn't (the caveat is buried at lib.rs:295).
- Doc drift: `RpcCarrier` header still says the `#[ndn_service]` macro is
  "planned"; ndn-sim self-describes as "ndn-lab"; `EncodeError` (flotilla)
  has no Display impl; `FrozenDag::insert_document` maps internal decode
  failures to `EncodeError::NestedAttribute` (lossy name, cost us a
  debugging session).
- MavlinkReader thread in ndn-sim never joins (blocking recv) — lingers on
  teardown in long-lived processes.

## 4. Numbers you might want (first external benchmarks)

Radio-mode comparison, 2 vehicles, full agent stack over ndn-sim SimLinks
(miniMUAS `docs/v3/radio-comparison.md`):

| profile | telemetry p95 | coop success | service RTT p50 | spark loss |
|---|---|---|---|---|
| apsta (0.1% loss, 2 ms) | 222 ms | 100% | 12 ms | 0.2% |
| ndr-good (1%, 5±2 ms) | 246 ms | 90% | 19 ms | 1.3% |
| ndr-contested (8%, 15±8 ms) | 1168 ms | 30% | 46 ms | 8.1% |

The coop-degradation curve is dominated by our own protocol's grace window
(2.5 s) interacting with loss on the coord fetch path — but the stack's
share of the budget (service RTT, spark loss tracking link loss ~1:1) was
clean and predictable. Happy to re-run any matrix you want.

## 5. Standing offer

We generate OTLP traces of real missions, a fleet trust-schema exercise
(when we take on signed ndnsf + zero-touch provisioning next), and
instrument descriptors for the waterline suite. Requests for specific
experiments: drop a file in miniMUAS `docs/v3/requests/` and we'll run it.
UI/UX input for Anchor/Capstan/Sextant is already at
`docs/v3/WATERLINE-INPUT.md`.
