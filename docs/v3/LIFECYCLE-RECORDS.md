# Lifecycle records — the "UAS as data" vertical

Design for trusted, audited, traceable fleet management where the airframe's
whole life — build, tuning, payloads, deployments, maintenance, firmware —
is named, signed, content-addressed data on NDF chains. Nothing here is a
tool suite: every surface (waterline consoles, our dashboard, a per-UAS MCU
display, report generators) consumes the same chains, because the data is
not siloed.

Scaffolded as crate `uas-fleet-records`
(`/Users/pmle/Documents/Dev/uas-fleet/crates/uas-fleet-records`); this doc
is the design of record. Companion survey inputs:
`surveys/ndn-integration-cheatsheet.md` §3 (AppRuntime chains) and
`surveys/waterline-draft.md` §(b) (the WL-7 alignment pattern).

## 1. Thesis: the airframe is the identity

One airframe = one bundle of append-only, single-writer Block chains:

```
<fleet>/<airframe-serial>/records/build
<fleet>/<airframe-serial>/records/calibration
<fleet>/<airframe-serial>/records/payload
<fleet>/<airframe-serial>/records/deployment
<fleet>/<airframe-serial>/records/maintenance
<fleet>/<airframe-serial>/records/firmware
```

e.g. `/jsl/fleet/AF-001/records/calibration`. The chain root names the
*airframe serial*, never a device: the flight controller, companion
computer, and records MCU are all replaceable parts whose state is captured
*into* the chains. Trust is structural, not positional — every Block is
Ed25519-signed by its writer, content-addressed (SHA-256 of the signed
packet), and parent-linked; readers re-verify against the pinned writer key
on every cold resolve (`AppRuntime::resolve`), so **a fold over a chain is
also an audit pass**. Records integrate with any surface because their
identity is the name + hash, not the database they happen to sit in (the
muas-artifacts posture: transport differs, data identity doesn't).

Three payload layers ride the chains, distinguished by MIME:

| MIME | payload | role |
|---|---|---|
| `application/vnd.uas.lifecycle-record+json` | versioned `RecordEnvelope` (compact sorted-key JSON) | the records themselves |
| `text/x-param-snapshot` | raw autopilot param dump (e.g. `.parm` bytes) | FC state capture; back-ref target |
| `application/vnd.ndf.authority` (and kin) | `AuthorityRecord` (+ `CeremonyAttestation`) | grants and sign-offs; back-ref target |

Readers fold records and *skip-but-surface* everything else
(`ChainHistory::skipped`), so snapshots and sign-offs are present, cited,
and auditable without being schema-coupled.

## 2. Record schemas (wire = compact sorted-key JSON, envelope-versioned)

Every record Block is `{"kind":"uas.<family>","record":{…},"version":N}`.
Typed structs: `uas-fleet-records/src/records.rs`. Units follow the fleet
conventions (`_ns` = Unix-epoch nanoseconds like `gps_time_ns`; `_s`, `_pct`,
`_g`, `_m_s`, `_wh`, `_mah` in field names; the semantic layer carries
unit/datum vocabulary so the wire stays plain). Hash back-refs are
lowercase-hex SHA-256 content hashes of Blocks (`HashHex`).

- **`uas.build`** — parts list, who built it, specs.
  `airframe_serial`, `builder` (principal name), `built_at_ns`, `model`,
  `notes`, `parts: [PartRef{batch?, manufacturer, name, part_no, qty,
  serial}]`, `specs: [SpecEntry{name, unit, value}]`. Genesis of the build
  chain; corrections are further build records (append, never rewrite).

- **`uas.calibration`** — tuning/calibration affecting autopilot parameters,
  tracked over time for diagnosis/visualization.
  `at_ns`, `calibrator`, `deltas: [ParamDelta{name, now, was?}]`, `fc_id`
  (board serial holding the params), `kind` (`accel-cal`, `compass-cal`,
  `pid-tune`, `param-set`, …), `param_snapshot?` → **hash back-ref to a
  param-snapshot Block on the same chain**, `reason`. Diagnosis primitives
  are folds: `fold_params(history)` = current param view; the same fold over
  `ChainHistory::up_to(seq)` = "what was `ATC_ANG_RLL_P` when that flight
  happened"; snapshots re-anchor the fold against delta drift.

- **`uas.payload` / `uas.capability-report`** — payloads, attachments,
  accessories, with capability reporting (automated or manual).
  Payload event: `action` (`attached|detached|updated`), `actor`, `at_ns`,
  `payload_id`, `payload_kind`, `capability?` (embedded report at attach
  time). Capability report (also standalone on the same chain):
  `sensors: [SensorCapability]`, `battery: BatteryCapability{capacity_mah,
  cells, chemistry, cycles?}`, `est_flight_time_s?`, `payload_capacity_g?`,
  `effectors[]`, `max_speed_m_s?`, `extras[]` (mirrors the live
  `CapabilityProfile.extras` vocabulary — `orbit`, `mic`), `source`
  (`automated|manual`), `reporter`. **Battery + flight time "dynamically
  tracked over time" is a derived series, not stored state**:
  `endurance_series(deployments)` folds flight time and battery burn out of
  deployment records into a cumulative airframe odometer plus a per-run
  full-battery endurance estimate — trending that estimate down over months
  IS the battery-aging signal. Same projection thesis as ndf-policy: delete
  the view, replay the chain, get it back.

- **`uas.deployment`** — deployment history linked to flight logs,
  experiments/field tests, reports/observations.
  `started_at_ns`, `ended_at_ns`, `site`, `kind` (`experiment`,
  `field-test`, …), **`run_id?`** — the association with our run-config
  records: the same id `muas-agent` stamps on every journal line of a run
  (`crates/muas-agent/src/journal.rs`), so deployment ⇄ run-config ⇄ journal
  join on one key; `flight_logs: [ChainRef{root, head_hash?}]` — **journal
  chain roots plus the head Block hash at close-out** (tamper-evident even as
  chains grow), e.g. `/muas/v3/iuas-02/journal/companion`;
  `artifacts: [hash]` (reports/observations, muas-artifacts outputs);
  `flight_time_s`, `battery_used_pct?`, `energy_wh?`, `outcome`,
  `observations`.

- **`uas.maintenance`** — parts + person + ceremony sign-off.
  `action`, `at_ns`, `maintainer`, `parts_installed[]`, `parts_removed[]`
  (full `PartRef`s), `next_due_ns?`, `notes`, **`signoff?` → hash back-ref
  to an `AuthorityRecord` Block whose `CeremonyAttestation` satisfied the
  demanded tier** (ndf-core `authority.rs`; waterline's
  `capstan::provision_binding` is the ceremony exemplar). Whether a given
  action *requires* a sign-off is writer-side policy (a floor-table rule),
  not schema — the record stays a plain fact.

- **`uas.firmware`** — firmware with records persisting across reflashes.
  `at_ns`, `flashed_by`, `target` (`fc`, `companion`, `records-mcu`,
  `payload-mcu`), `fc_id` (hardware identity of the flashed device),
  `image_name`, `version`, **`image_sha256`** (image hash),
  **`params_migration?` → hash back-ref to the params-migration snapshot on
  the calibration chain** that re-established parameter state after the
  flash, `notes`.

## 3. Writer identities — who signs what

Each family chain is **single-writer**, and the writer is a *role identity*
(`ndf_apps::Identity`: principal namespace + device + Ed25519 key):

| chain | writer role | typical identity |
|---|---|---|
| build | builder | `/people/<builder>` + bench device |
| calibration | calibrator | `/people/<calibrator>` + field box; or the companion for automated cal events |
| payload | companion (automated probes) or technician | `/muas/v3/<vid>/companion` |
| deployment | companion (mission close-out) | `/muas/v3/<vid>/companion` |
| maintenance | maintainer | `/people/<maintainer>` |
| firmware | maintainer / deployer | `/people/<maintainer>` |

Enforcement is layered:
1. **Mechanically**, `AppRuntime::publish` refuses any chain whose
   `writer`/`writer_key` is not the runtime's own identity
   (`PublishError::NotOurChain`) — role separation is not a convention.
2. **Trust-wise**, readers pin `writer_key` in the `ChainAddress` they
   follow; a Block signed by anyone else never enters the store (ChainGate,
   reject-without-poison).
3. **Authority-wise**, each role holds an `AuthorityRecord` **Grant**
   (`ndf-core/src/authority.rs`: `AuthorityBody::Grant`, `Scope
   { namespace_prefixes: [<the family chain root>], action_classes:
   ["publish"] }`, a `ValidityWindow`, and a `CeremonyAttestation` for the
   granting gesture) published on the fleet's authority chain. Verifiers
   answer "was Kai *authorized* to sign maintenance on AF-001 last March"
   from data, not from an admin database. Grant issuance/rotation rides
   ndf-policy `bootstrap`/`trust` — we consume, never fork.

Key rotation and lost keys are authority-chain events (new Grant, old one
expires via `ValidityWindow`); the record chains themselves never rewrite.

## 4. The reflash-persistence argument

The failure mode being designed away: parameters, tuning history, and
"what firmware is this" living *in* the flight controller, wiped by every
reflash and lost entirely on FC swap.

Here, records live on the **airframe chain**; the FC is replaceable
hardware named *inside* records (`fc_id`), and FC state is captured *onto*
the chain as param-snapshot Blocks:

- A **reflash** is one `uas.firmware` record (new `image_sha256`, same
  `fc_id`) whose `params_migration` back-refs the post-flash re-snapshot on
  the calibration chain. Chain seq advances by one; nothing is lost —
  the pre-flash tuning history and its snapshots remain resolvable forever.
- An **FC swap** is the same record with a new `fc_id`. Chain continuity
  (parent-linked seqs on one root) *is* the audit trail across the swap.
- The chains live wherever the replication policy puts them (§5) — the
  records MCU, companion, server, laptop — so no single hardware failure,
  reflash, or swap can take the history with it.

Test-proven: `records_persist_across_reflash_and_fc_swap` in
`uas-fleet-records/tests/lifecycle.rs` (reflash + FC-swap on one chain;
parent-ref continuity asserted; every flash's cited snapshot still
resolvable).

## 5. Replication topology — NDF policy, by name

Four replica classes, one policy vocabulary
(`/Users/pmle/Documents/Dev/ndf-rs/refounding/ndf-policy/src/presence.rs`;
no README exists — module docs are the reference). Presence is declared,
per principal, as a chain of `PresenceDeclaration`s (D-32: prefix,
`DeviceRef`, `PresenceKind`, `PrefetchPolicy`, `EvictionPolicy`), resolved
most-specific-wins, entering through the same `ChainGate` as everything
else. Declarations for the records prefixes:

| device | prefix | PresenceKind | PrefetchPolicy | EvictionPolicy |
|---|---|---|---|---|
| records MCU (per-UAS, small display) | `<fleet>/<serial>/records/` | `Recent { window }` (head + recent) | `Subscribe` (SVS notify → fetch) | `LruAfter { bytes }` sized to flash |
| records MCU | …`/records/build` + latest capability report | `Persistent` (tiny, identity-critical: show *what this airframe is* offline) | `Eager` | `KeepUntilExpiry` |
| companion computer | `<fleet>/<serial>/records/` (own airframe) | `Persistent` (full replica) | `Subscribe` | `EvictWhenSpaceTight` |
| repo server | `<fleet>/` (every airframe) | `Persistent` (fleet archive) | `Eager` | `KeepUntilExpiry` |
| field laptop | `<fleet>/` | `Persistent` for the airframes on today's manifest, `OnDemand` for the rest (most-specific-wins does the split) | `Eager` / `Lazy` respectively | `EvictWhenSpaceTight` |

Mechanics: replication itself is `AppRuntime::follow` (SVS convergence +
`ChainGate` commit) with the follower's posture derived from
`ndf_policy::presence::{DeviceDimensions, UsageClass, AttachmentIntent}` —
the MCU declares tight capacity (`capacity_tight: true`, attachment
`Fixed`), which drives the Carry-vs-Reach `CarryDecision` in
`ndf_apps::follow`. The actuation seam (declaration → actual Interest
issuance/eviction) is `PresenceActuator` — upstream has no presence
mechanism to ride yet (their ledgered next slice), so today the companion
and server run eager follows and the MCU a windowed follow; the
declarations are still published so posture converges to policy the moment
the actuator lands. Write availability needs no coordination: each chain
has one writer, everyone else replicates; a device offline for a month
folds forward deterministically on reconnect.

## 6. Evolution & versioning (waterline friction #7, addressed day one)

Waterline's finding: positional facts hard-fail on arity change. Our rules:

1. **Envelope-versioned kinds**: every Block carries `(kind, version)`
   outside the body. Readers route on it; unknown kinds and unknown
   versions are **skipped and reported** (`TypedHistory::unspoken`,
   `ChainHistory::skipped`), never fatal — an old MCU keeps rendering a
   chain a newer companion writes. Test:
   `unknown_versions_and_kinds_skip_never_poison`.
2. **Append-only within a version**: new JSON fields are optional-with-
   default (serde `#[serde(default)]`; unknown keys ignored by readers).
   Removing/re-typing/reordering = `version += 1`.
3. **Positional facts (stratum draft)**: `record-version` is the FIRST
   field of every record; optional fields ride as `list-of` zero-or-one
   (the WL-7 shape), so presence changes never shift arity. Arity changes
   are a new record term + `supersedes` — flotilla bench lint **L-07**
   mechanically refuses an edited stratum without a supersedes line
   (verified, §8).
4. **Old data is never migrated** — chains are append-only; readers speak
   version ranges. A "migration" is new records, e.g. a params re-snapshot,
   never a rewrite.

## 7. The UI-agnostic boundary

**Core = kinds + chains + verifiers (+ pure folds). Full stop.**
`uas-fleet-records` knows: record schemas, chain addresses, envelope
versioning, publish/resolve over `AppRuntime`, and derived folds
(`endurance_series`, `fold_params`). It does not know: panels, pages,
colors, layouts, refresh rates, pagination, or which device is drawing.

Surfaces all consume the same chains, each through its own idiom:
- **waterline consoles** — adopt via the `uas-lifecycle` stratum +
  `console-chain:chain-view` declarations; scrub = their chain-fold, which
  is our `up_to(seq)`.
- **our dashboard / muas-artifacts** — resolve + fold, render HTML with
  Block-hash citations (the existing no-data-silos pattern).
- **records MCU display** — follows head+recent, renders
  `ChainHistory::latest::<CapabilityReport>()` and the last
  maintenance/firmware entries; same verifier, 320×240 output.
- **field tools / CI** — the same folds as assertions (airworthiness
  checks: "unsigned maintenance since last flight?").

The one deliberate wire concession to small consumers: hash back-refs are
hex text in JSON (MCU-friendly), 32-byte hashes in the manifest/stratum
layer. No surface concern reaches the record schemas.

## 8. Semantic layer & waterline alignment (what the stratum draft needed)

Two artifacts, one taxonomy:
- **Operative (house style)**: `src/manifests.rs` — flotilla vocabulary
  `uas-lifecycle`, field labels byte-for-byte the wire JSON keys,
  importing `uas-fleet-data`'s `fleet-semantics` attribute keys (`unit`,
  `datum`) **by document hash** — lifecycle records share one semantic
  universe with live telemetry (no silo).
- **Waterline draft**: `strata/uas-lifecycle.ndfs` — the same records in
  WL-7 shape: declarations as keyed-entry manifests (`airframe` +
  `airframe-fleet/serial/model/build-genesis` keys; chains declared via
  their `console-chain:chain-view`), facts as single positional-record
  manifests, `measured:instant` stamps, `list-of(hash)` optionals, hash
  back-refs — `strata/console-command.ndfs` was the template. Nested
  records (parts, sensors) are deliberately hash-refs to their own facts,
  not embedded records: waterline hit a real flotilla bench
  record-nesting-record mis-binding, so we author around that shape.

**The pin lifecycle works from an external repo** (this was the experiment):
flotilla `bench compile` ran from our crate with `--lock`/`--store`
pointing into the crate; `measured` resolved from a vendored copy of the
pinned `.ndf`; compile was clean (0 err/0 warn), deterministic across
reruns, and L-07 correctly refused a drift-edit without `supersedes`. Pin:
`uas-lifecycle = 6770f6dd…` (9452 canonical bytes), recorded tool-only in
`strata/Atelier.lock` + `strata/STRATA.md`, drift-guarded by the
`compiled_stratum_matches_the_pin` test. Until waterline adopts the
stratum, the vocabulary hash (house) and stratum hash (draft) are
*intentionally distinct documents*; adoption means waterline consumers bind
our declared chains through the stratum's terms while our devices keep
emitting the same bytes.

## 9. Crate map & tests

```
uas-fleet/crates/uas-fleet-records/
  src/records.rs     typed records + RecordEnvelope (versioned) + errors
  src/chains.rs      RecordFamily, records_root, records_address
  src/manifests.rs   uas-lifecycle flotilla vocabulary (+ pin drift guard)
  src/writer.rs      RecordWriter (publish, publish_param_snapshot), MIMEs
  src/reader.rs      RecordReader, ChainHistory (fold/latest/up_to)
  src/series.rs      endurance_series, fold_params (derived views)
  strata/            uas-lifecycle.ndfs, Atelier.lock, STRATA.md, store/
  tests/lifecycle.rs in-proc AppRuntime gates (muas-artifacts offline pattern)
```

18 tests green (14 unit + 4 integration), clippy clean: serde round-trips
for all seven kinds; envelope canonical bytes; typed version/kind refusals;
forward-compat decode; manifest R13 canonical round-trip + wire-key/sorted
order checks + semantics-import guard + stratum pin guard; chain
write→read→fold across all six families with real Blocks (snapshot and
sign-off Blocks present-but-skipped and cited by hash); param fold at
head and at scrubbed seq; skip-never-poison; reflash/FC-swap continuity.

## 10. Friction (external things touched; file-cited)

1. **ndf-policy has no README** — the task's "replication story" reference
   doesn't exist as a file; the actual references are the module docs in
   `ndf-policy/src/{presence,replication}.rs` and `src/lib.rs`. Worth
   writing the README from those docs.
2. **Presence actuation is a declared seam, not a mechanism**
   (`ndf-policy/src/presence.rs` header: "no upstream presence mechanism
   exists to ride yet, so the actuator is the boundary"). Our §5 topology
   compiles to declarations today, follows tomorrow — fine, but the gap is
   load-bearing for the MCU (windowed follow must be hand-rolled until the
   D-46 strategy slice lands).
3. **`ndf_apps` errors are Debug-only** — `PublishError`, `ResolveError`,
   `ServeError` (`ndf-apps/src/runtime.rs:27-96`) implement neither
   `Display` nor `std::error::Error`, so every consumer (muas-artifacts
   does the same) renders `{e:?}` into strings and loses `#[source]`
   chains. Ask upstream for `thiserror` on the SDK error surface.
4. **flotilla bench worked externally, with three nits** (tool:
   `flotilla/crates/tools/bench/src/main.rs`):
   (a) `bench compile --lock` **rewrites the lock file and strips
   comments** — `save_lock` (main.rs:58-64) emits only its own header,
   though `load_lock` documents `#` comments as part of the format; our
   explanatory pin comment was silently deleted on first compile.
   (b) The `--store` flag is accepted by `compile` but undocumented in the
   usage block (main.rs:5 lists it only for `doc`/`vectors`); without it,
   `use measured as measured` fails to resolve and nothing hints that a
   store directory is the fix.
   (c) There is no read-only compile: checking a stratum from CI mutates
   `Atelier.lock` (timestamp/ordering churn) unless you copy the lock
   aside; `bench lint` doesn't emit the hash you'd want to assert.
5. **Vendored ride-on pins** — reproducing the compile outside waterline
   required copying `measured`'s canonical bytes
   (`waterline-strata/build/7621340c….ndf`) into our `strata/store/`.
   Content addressing makes this safe, but there's no shared store/registry
   convention yet; every downstream repo will grow its own vendored copies
   (same hash, N locations). A fleet-level `--store` convention (or
   flotilla publishing its build dir) would remove the copies.
6. **`uas-fleet-data` keeps its authoring helpers private**
   (`crates/uas-fleet-data/src/manifests.rs`): `BuiltManifest::build`,
   `semantics()`/`Semantics` (the attribute-key hashes), and
   `state_vocabulary` are all private, so `uas-fleet-records` re-derives
   key hashes by label lookup inside the imported document and re-implements
   the two small helpers (drift-guarded by test
   `lifecycle_imports_fleet_semantics_by_hash`). Export
   `BuiltManifest::build` + the key hashes upstream.
7. **waterline STRATA.md's countersign loop is repo-local** — their C4
   ceremony attests *their* pins on *their* chain
   (`waterline-strata/STRATA.md`); an external stratum like ours has no
   place to dock a countersignature yet. When our stratum is adopted, the
   pin should enter their ride-on table (as `units`/`loss`/`measured` did)
   rather than living only in our lock.
8. **ArduPilot param dumps have no canonical byte form** — two dumps of
   identical state differ (ordering/whitespace/comments), so
   `text/x-param-snapshot` hashes are identity-of-capture, not
   identity-of-state. Fine for back-refs; anyone wanting state-equality
   comparison needs a canonicalizing capture tool (records-side, later).
