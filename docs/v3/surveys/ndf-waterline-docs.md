# Survey: NDF refounding + waterline suite docs (~/Downloads/latest)

Surveyed 2026-07-09. Builder's digest of the 24 docs — the conceptual ground
v3's dashboard, security, and radio work stands on.

## The stable surface to build against

Per the docs themselves, everything else is still settling; these are stable:
- **Five verbs**: identity · publish · grant · subscribe · spark.
- **Three-layer contract**: L1 semantic manifest (what data IS — units,
  precision, thresholds, signed) → L2 relationship graph (typed edges:
  is-measured-by, is-calibrated-by, is-derived-from, downstream-of,
  authorized-by; spatial weights only as hints) → L3 render contract (what a
  lens can express).
- **Four match verdicts**: Express / Approximate / Refuse / Unresolved.
- **Ceremony tiers C0–C4** (standing / glance-confirm / proximity-tap /
  confirm-with-factor / quorum).

## NDF refounding — key decisions

- Boundary litmus D-47: bytes → ndn-workspace; meaning → NDF. Promotion litmus
  D-36: new ideas enter as Content+manifest; substrate only when the verifier
  must dispatch on them.
- **The loop-killer rule**: manifests/contracts reference other definitions
  **only by content hash** → definitional graph is a DAG by physics.
- D-48 (ratified): manifest + render-contract calculi are chain-independent
  spec crates (no_std, zero ndf-* deps). Runtime order fixed:
  **match → authorize → instantiate**. A lying contract buys bad rendering,
  never access.
- D-49 (ratified): frozen 32-term kernel V0.2, published as a quine; term
  identity = hash. Obligations C1–C10 with a 252-vector conformance corpus.
  Three anchors: V0 (kernel), T0 (terminal contract — expresses raw.inspect +
  text.plain, makes refusal safe), IM0 (implicit manifest — every block yields
  {opaque, media-type, name, size, kind}; nothing is ever undescribed).
- Two invariants to tattoo: **no manifest ever names a renderer; no renderer
  ever names an app.** The resolver (Surface Authority) binds intents ×
  contracts × grants into RenderSessions; refusal reroutes to a surface that
  can carry the intent; partial expression across surfaces is normal.
- Pattern Book P1–P11 idioms; **P11 actuation-safety record** (danger tier ·
  reversibility · idempotency · confirmation tier · proximity · rate-limit ·
  e-stop) — directly relevant to drone command authorization.
- C10: a semantic edge shapes *your* matching only if its vocabulary is in
  *your* TrustFrontier. Signature decides who spoke, never whether it shapes
  your matches.
- AuthorityRecord family unifies grant/accept/revoke/session. "Sync" retired
  as vocabulary → presence declarations + strategies.
- **Sparks**: AEAD-encrypted, sequenced, ephemeral streaming beside the block
  plane; rollback windows; derived streams; periodic checkpoint Blocks.
  Doctrine: *Blocks remember; Sparks move.* Honest about loss.
- Keel build spec: `ndn-manifest` + `ndn-render-contract` + `ndn-bench`
  (compiles .ndfs/.ndfm/.ndfc, L-01..L-15 lint, bench doc cards, vector
  corpus). Matcher signature frozen:
  `match(dag, contracts, TrustFrontier, Budget) -> Vec<Match>` + deterministic
  `select(matches, Floor)`. (In the flotilla checkout these exist as
  `manifest`/`render-contract` crates.)
- Builder tagline: "a substrate, not a service — delete your backend."
  Onboarding is a ceremony, not a destination; identity ladder
  did:peer → did:ndn → federation.

## Waterline suite (WIP — explicitly seeking fleet-management UI/UX input)

Shared design language: **draw the medium, not the link** (broadcast media as
fields, emissions as ripples; lines only for true point-to-point); **honest
numbers** (every measured value an interval with provenance — the "band"
motif); names are the interface; authorization before action (preflight →
receipt; ceremony tiers C0–C4 scale the gesture); pictograms first; **Pilot** =
small on-node model appearing in exactly four seams, always drafts, never
signs/applies/sends.

- **Anchor** — trust instrument (identities, keys, DID ladder, certs,
  ceremonies, grants, recovery/succession). Radically *objects-not-views*: one
  map, three rings (custody/membership/extended trust), chain language (beads,
  dashed lent-out with draining time arc), verbs Lend/Join/Onboard/Recover,
  gravity = blast radius (C4 "darkens the room"), three tenses (tide/map/
  strand). ndnsec/DID depth one keystroke down.
- **Capstan** — machinery: RUN (engines, setup) / RIG (Trust Studio, fabric,
  hardware) / GROW (provision). Manages every placement of the one engine
  (daemon/container/router/phone/browser-tab/ndn-lab virtual). Trust Studio:
  English rule → LVS pattern diff, tested live. Hardware claiming (monitor
  radios + own driver).
- **Sextant** — network console: fleet map (fields/lines/overlays/simulated),
  names, behaviors, time panel (holdover ±uncertainty), signals, traces,
  Observe→Ask Pilot ("why did X stall at 14:02" → query plan → spans cited).
- **Sextant Tab** — browser extension, one WASM engine shared by tabs,
  permissioned `window.ndn`, ladder tab→extension→native→peer.
- **Extension mechanism = instruments**: a capability is a separate process
  joining the node fabric over an SHM face, publishing measurement/control
  namespaces + a three-layer manifest. Sextant/Capstan are just built-in
  renderers matching it. Install = drop binary + adopt manifest.
  **miniMUAS v3 should present itself as instruments** so its panels appear in
  Sextant with zero suite changes.
- Two trust tiers, one fabric: local (shm/unix, `/local/…` never leaves) vs
  networked (Anchor identity + trust context). "Joining a network = joining a
  context, not an SSID — radios follow the trust."
- Surface Plane: **RenderSession** is the one new primitive (binding of scene
  region × contract resolution × renderer × grants × device context); sessions
  **re-resolve, not restore**. Authority enforces; Behavior expresses. Input
  plane: Grant = Block, Stream = Sparks, Commit = Block.

### Open questions they want feedback on (→ WATERLINE-INPUT.md)
- Anchor's rings-not-tables model at fleet scale (40+ grants)?
- C0–C4 gravity metaphor applied to fleet ops at scale?
- Field/ripple visualization readability for large fleets?
- Honest-numbers bands everywhere without clutter?
- Where Pilot AI-drafting helps vs intrudes in a trust console?
- **Re-resolution vs anchoring — users must be able to pin a layout**
  (flagged load-bearing UX).
- RenderSession irreducibility; frame-budget vs TCB; ModeProfile placement.

## Named data radio / named time / ceremonies (fleet-relevant)

- Root cause thesis: the medium is broadcast; the commodity stack burns its
  budget simulating unicast. Monitor mode is the only commodity path to the
  broadcast-native primitive. Own the driver, rent the silicon (userspace
  libusb driver; fixed a 34 ms Discovery-Window RX-DMA lag the kernel hides).
- Addressing: no dest MAC (**the name is the filter**), no BSSID; only link
  artifact is an ephemeral per-generation nonce owned by the coding layer
  (RLNC). Rendezvous is a mode selected by power, never the foundation.
- Framing: atomic path + coded/RLNC path; coded Data carries an uncoded
  manifest {generation nonce, name-prefix-hash, k/n} in the clear.
- Admission = cost-monotonic sieve (PHY CRC → name extraction → Bloom test vs
  pending-PIT ∪ FIB ∪ served-CS → lookup → expensive verify). Never keyed on
  identity (that rebuilds the MAC). **Trust-blind admission; trust weights
  strategy after verification.** Trust is not transitive.
- Authority = named LVS schema; onboarding proximate/physical; **revocation is
  expiry-by-default** (no CRL to reach in the field).
- `Measured<T> { value, sigma, prov }` — exposure is a type, not a score;
  combiner requires threat-diversity, not count.
- Named time: every reading is an interval, never a point; agreement ≠
  traceability (GPS-less swarm needs a coherent ensemble timescale, not UTC);
  monitor-wifi radiotap TSFT ~1 µs RX timestamps (latch-to-antenna offset
  varies with MCS — must be modelled); soft→hard uncertainty ratchet for
  bootstrap. **Named drone-specific risk: EWMA staleness windows tuned for
  quasi-static media lag the truth at swarm velocities**; sync+ranging+
  kinematics should fuse in one estimator (observability caveat: stationary or
  collinear formations can be unobservable).
- Capability-grant ceremony: friction scales with stakes; all tiers emit the
  same signed, proximity-bound, anti-replay, revocable grant artifact. Keystone
  table danger→ceremony→hardware: D0 observe → C0–C1; D1 reversible → C1;
  D2 consequential → C2 non-delegable; D3 irreversible → C3–C4, rate-limited,
  proximity re-confirm per use, hard e-stop. Roles make ceremonies disappear
  (fresh ceremony only for first contact, escalation, danger). For the fleet:
  monitor-radio proximity IS the C2 nonce source; expiry-by-default IS field
  revocation; P11 + danger tiers gate what a drone may be commanded to do.
