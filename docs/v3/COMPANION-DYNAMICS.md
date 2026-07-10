# Companion dynamics: NDF-driven node management (design)

Status: DESIGN, 2026-07-10. Companion piece to `MDEPLOY-PLAN.md` and
`REPO-TOPOLOGY.md` §Deployment decoupling. Grounded in: the mini-muas-v3
config repo (`minidronesys-configurations`, flake.nix + nix/), the waterline
campaign survey (`surveys/waterline-draft.md`, esp. Capstan findings),
`surveys/ndf-waterline-docs.md` (ceremony tiers C0–C4, P11), and
`surveys/ndn-integration-cheatsheet.md` §3 (ndf-apps AppRuntime).

## The problem being solved

Today every companion-computer change — which stack runs, what role a node
plays, one agent knob, one radio bearer — is a NixOS deployment:

1. push the source repo, 2. bump `?rev=` in the config repo (twice for the
   flake/`-src` twins), 3. `nix flake lock --update-input …`, 4. rebuild the
   whole system toplevel, 5. ship/switch per host.

Role is baked into per-host directories (`nix/nixos/minidronesys-0{1..4}/`),
app knobs historically leaked into the deployment layer as patches against
app internals (`nix/ndn-packages/minimuas/config_dir.patch` — the v3
inversion already bans this), and radio bearer selection is a branch/module
choice (`wifi-adhoc` / `wifi-mesh` / `morse-micro` / `newracom` / `wfb`).
Nothing is changeable in the field, per node or fleet-wide, without a laptop,
the repo, network to the binary cache, and minutes-to-hours of turnaround.

The owner's intent for v3: some things on companions become **dynamic with
NDF assurances** — switching software stacks, node mode/config, fleet-wide or
per node, in the field, from an onboard simple interface or from networked
authorized nodes. This document is that design.

## 1. What stays static, what becomes dynamic

The split rule: **Nix owns bytes-on-disk; NDF owns which of those bytes are
live.** Everything appliable in the field must already exist in the nix
store; the dynamic layer only ever *selects*, never *builds*.

| Layer | Contents | Changed by |
|---|---|---|
| **Static — NixOS base image** (config repo, declarative, per the v3 flake) | kernel, device trees, radio drivers/firmware (morse-micro, rtl8812eu, …), forwarder/system units, helm itself, the **closure set** (every deployable stack variant, pre-built), helm's verifier floor table, host identity provisioning | git + flake bump + ship, exactly today's flow — but now rarely |
| **Dynamic — NDF node-config chain** (`<fleet>/<host>/config`, e.g. `/muas/minidronesys-02/config`) | which stack closure runs; node role/mode (`wuas` / `iuas` / `relay` / `gcs`); agent config knobs (flight floor, telemetry rate, coop grace, …); radio **bearer selection** among pre-installed bearers; active trust context (which scope/TrustFrontier the node operates under) | signed config records, in the field |

Deliberately NOT dynamic: key custody and grants (Anchor territory), the
floor table itself (baked into the image so authorization can never be
weakened over the same channel it protects), kernel/driver/firmware, nix
store GC policy.

The **closure set** rides the mechanism the flake already uses for offline
rebuilds (`system.extraDependencies`): the image carries one `buildEnv` per
stack variant — e.g. `muas-agent`, `muas-agent-diagnostic`, `relay-only`,
`recovery` (a minimal known-good agent + bearer fallback) — each a normal nix
store closure exported by the repo flakes per MDEPLOY-PLAN §2. Bearer- or
driver-level alternatives that need more than a unit swap are **NixOS
specialisations** of the same toplevel, also pre-built into the image.

## 2. Mechanism: the `helm` supervisor and the node-config chain

`helm` is a small companion daemon (crate lives in **uas-fleet** — the
generic-by-default test passes; nothing miniMUAS-specific in it). It is the
only writer of applied state and the only actuator of switches.

### The chain

One NDF chain per host: `<fleet>/<host>/config`. Records are keel manifests
(one fact-style record per manifest, WL-7 convention) with, at minimum:

- `stack` — closure-set entry name (never a store path; helm maps name →
  pre-installed closure and **refuses names not in the set**: no field
  compilation, ever).
- `mode` — `wuas` | `iuas` | `relay` | `gcs` (+ mission-mode sub-knobs).
- `knobs` — typed key/values consumed by the agent (proper config options
  per REPO-TOPOLOGY: the knob exists in the app repo, the record sets it).
- `bearer` — one of the image's installed bearers.
- `trust-context` — the scope/frontier the node should operate under.
- `basis` — hash of the applied-state receipt the operator was looking at
  when issuing this record (optimistic concurrency: two operators cannot
  silently cross-apply; a stale basis is a typed refusal).
- `authorization` — operator identity + `CeremonyAttestation` reference(s)
  meeting the floor for this change class (below).

Partial records are the norm: a record that only sets `knobs.telemetry_rate`
leaves everything else at the previously applied value.

### The loop

helm = `AppRuntime::follow_gated` on the config chain (ndf-apps, exactly the
cheatsheet §3 surface) with a `ChainKind` admission policy in a `GateCell`,
so the chain can be refused mid-run without poisoning (waterline's AD-14
pattern — a compromised or misbehaving writer is quarantined by a gate flip,
and the refusals stay visible).

For each new record: **verify → apply → receipt.**

1. **Verify**: chain writer-key pin (the follow's `ChainAddress` pins the
   authorized operator identity), then the floor check — the attached
   ceremony attestation must meet helm's *baked-in* floor table for the
   change class. Vendor/operator-declared tiers are advisory; the floor is
   normative (waterline's U-3 posture, adopted wholesale).
2. **Apply**: systemd-level switching between pre-built closures only —
   flip a `/var/lib/helm/current` profile symlink to the named closure's
   `buildEnv` and restart the stack unit (whose `ExecStart` points through
   the symlink); or, for bearer/driver classes, run the pre-built
   specialisation's `switch-to-configuration test`. Mode and knobs
   materialize as the environment/config file the agent unit reads.
3. **Receipt**: publish an **applied-state record** on helm's own chain
   (`<fleet>/<host>/config-applied`): `{desired: Hash, outcome: applied |
   refused | failed, closure applied, unit states, observed {stack, mode,
   bearer, knobs, trust-context}, at, refusal_code}`. Every attempt lands a
   receipt, `executed` true or false — audit parity, and receipts come from
   the **actuated side** (the answer waterline's WL-8 left as a seam for "a
   real agent": helm is that agent).

**Drift surface** = diff(latest desired, latest observed). Because desired
and observed are two chains of signed records, any console can render drift
with zero helm cooperation — and a node that reboots, partially fails a
switch, or gets hand-edited shows up as drift, not as silence.

**Rollback** = publish a record whose content points at the previous
record's selections (chains are append-only; rollback is re-assertion, not
deletion — the audit trail keeps the excursion). helm additionally keeps the
previous profile symlink target for a local last-known-good.

**Partition behavior**: chain unreachable ⇒ keep last applied state
(fail-still, never fail-revert). helm never invents state; everything it
applies traces to a signed record.

### Authorization: change classes → ceremony tiers

Mapped onto C0–C4 (ndf-waterline-docs) with **P11 danger direction** — the
safing direction gets *less* ceremony, never more (our own WATERLINE-INPUT
doctrine: never add friction to the abort ladder):

| Change class | Direction | Ceremony |
|---|---|---|
| Switch TO recovery/safer stack, or to fallback bearer (single node or fleet) | SAFING | C0/C1 — and never rate-limited (waterline's WF-6 ruling: rate gates never bind S1) |
| Single-node stack switch / mode change, on ground, disarmed | normal | C1 (glance-confirm); C2 if the node is mission-assigned |
| Knob changes within the active stack | normal | C1 |
| Fleet-wide stack or bearer switch | consequential | C2+, non-delegable (proximity-tap at the GCS; bearer switches can partition the fleet) |
| Trust-context change | consequential | C2 minimum |
| Any non-safing switch while armed/airborne | dangerous | **deny by default**; where mission doctrine allows it at all, C3 (confirm-with-factor), rate-limited |

Refusals (wrong tier, armed-deny, unknown closure, stale basis) are typed
and receipted — never silent.

## 3. Field interfaces (core stays UI-agnostic)

The records and receipts above are the entire API. Everything below is a
renderer/issuer over them; none of them adds capability the records don't
carry.

- **CYD / onboard button surface** (ESP32-CYD fleet node, ARCHITECTURE.md):
  **pre-authorized single-node recovery actions only.** The CYD holds
  pre-minted standing grants (C0/C1, proximity-bound, expiry-by-default) for
  the safing rows of the table: switch-to-recovery-stack, fallback-bearer,
  safe/hold mode. It issues the corresponding config record over its local
  link; it can never mint a C2+ record because it holds no such grant. A
  glance surface, not a console.
- **Networked authorized nodes** — the dashboard (uas-console) and, later,
  Capstan: full preflight → ceremony → record-issue → receipt-watch flow,
  fleet-wide or per node. Preflight runs against the same floor table
  content helm enforces (console-side copy is convenience; helm's copy is
  normative).
- **Capstan mapping** (per the waterline survey §5c — what exists today
  rides as-is):
  - `CapabilityReport::probe` + gap check: our binaries already answer a
    `--capabilities` flag (the waterline example-instrument convention);
    Capstan probes each closure in the set and diffs against what a config
    record demands — the **pre-run gap check**, caught before first run.
    A record naming a closure that lacks a demanded capability is a
    Features-view gap, and helm refuses it with `closure-absent`/
    `capability-gap`.
  - **Admission/GateCell**: Capstan's Straits gesture is exactly helm's
    quarantine — refusing a config chain mid-run, refusals visible,
    permitted flow continuing.
  - **Proposed contributions to Capstan** (things we'd build and offer, NOT
    dependencies of this design): a config-as-manifests stratum (the record
    shapes above as a published vocabulary), a drift/reconcile view
    (desired vs applied-receipt, per node and fleet-rollup), and a
    closure-set inventory view (what each host's image can become, from
    probe data). This design works from any chain-capable console; Capstan
    adoption is phase 3 polish, not a prerequisite.

## 4. Migration path

- **Phase 0 — status quo.** Everything via the config repo; MDEPLOY-PLAN
  bump flow. No new moving parts.
- **Phase 1 — closure-set image + helm, read-only.** Image ships the
  closure set and helm; helm follows the config chain and publishes
  observed-state receipts but *switches nothing* (records render as drift
  against reality). Buys: fleet-wide config observability, the receipt/
  drift surface, and field validation of chain-following on companions —
  with zero actuation risk. Switching still = nixos deploy.
- **Phase 2 — authorized switching.** Floor table armed; helm applies
  verified records (unit flip + specialisation paths); CYD recovery buttons
  live with pre-minted standing grants. The deployment pipeline is now for
  images and closure-set updates only — the daily/field loop leaves nix
  entirely.
- **Phase 3 — Capstan surfaces adopt the chains.** Probe-based gap checks,
  drift/reconcile and inventory views, Straits-style admission control over
  config chains; contribute the config stratum upstream. No record or helm
  change required — phase 3 is renderers.

## Invariants (tattoo list)

1. No field compilation — helm selects among pre-built closures, only.
2. The floor table is static image content; authorization can't be lowered
   over the channel it protects.
3. Every attempt receipts, `executed` true or false; receipts come from the
   actuated side.
4. Safing direction gets minimal ceremony and is never rate-limited.
5. Non-safing changes while armed are denied by default.
6. Partition ⇒ hold last applied state; helm never invents state.
7. The records are the API; every UI is a replaceable renderer.
