# muas-fabric — runtime-switchable NDN fabric for the miniMUAS fleet

**Goal:** field-reconfigurable A/B testing of `forwarder ∈ {nfd, ndn-fwd}` ×
`link ∈ {wifi, radio}` (and, above it, v2-vs-v3) that is **foolproof**: a bad switch
in the field self-heals; the node is never left without a working fabric, and never
needs a rebuild/redeploy to change cells.

Companion: `named-data-radio-integration-tiers.md` — establishes that radio is a
shared NDNLPv2 medium any forwarder can peer with (bridge at LP, don't patch NFD).

---

## 1. Invariants (what makes it clean)

**I1 — App socket invariant.** Whichever forwarder is active owns
`/run/nfd/nfd.sock`. ndn-fwd's default management socket is already this path, so
NDNSF/ndn-cxx apps are byte-for-byte unchanged in every cell. Exactly one forwarder
binds it at a time.

**I2 — One seam.** `muas-fabric.target` = "a forwarder owns the app socket AND
`/muas` reaches the fleet". Role units (agent/controller/gcs/dashboard/svsgen)
depend **only** on this target (+ identities + time). No role unit names
`nfd.service` again. All forwarder/link knowledge lives in the fabric module.

**I3 — Selection is runtime state, not eval state.** Every backend (nfd, ndn-fwd,
uplink configs, all setup paths) is built into **every** generation. Switching cells
writes a state file and restarts one service — never `nixos-rebuild`.

**I4 — Management plane stays up.** The Wi-Fi mesh (`mesh0`) and wired `end0`
(ssh, chrony time, `muas-fabric` control itself) are **not** part of the fabric and
are never torn down by a switch. Radio cells claim only the spare dongle
(`address = "1-1.4"`, `NDN_GUARD_LIVE_LINK=1` refuses an `operstate=up` device).
Rollback therefore always has a working path home.

**I5 — Radio is a shared LP medium.** The radio branch = bring up the medium, choose
which forwarder peers with it (T2/T3 of the tiers doc). Never "tunnel through
ndn-fwd" as a special case.

---

## 2. Decomposition of today's v2.nix

`muas-v2-setup` currently mixes two jobs; split it:

| New unit | Contents | Forwarder-aware? |
|---|---|---|
| `muas-v2-identities.service` | HOME pinning, `ensureIdentities` keyset import, keychain chmod | no — always runs, unchanged |
| fabric backends (below) | strategy set, faces, routes, self-heal timer | yes — moves into the fabric module |

Role units change from
`requires = [ nfd.service muas-v2-setup.service muas-time-set.target ]`
to
`requires = [ muas-fabric.target muas-v2-identities.service muas-time-set.target ]`
(same for `after`). The ~8 scattered `nfd.service` references collapse to this one
edit site. `muas-v2-routes` (the 30 s self-heal timer) becomes a *backend-owned*
unit: each backend brings its own reconcile timer, because "self-heal the faces"
is exactly the per-backend knowledge.

---

## 3. Runtime state machine

State (persisted, survives reboot):

```
/var/lib/minimuas/fabric/desired    e.g. "ndn-fwd radio"   ← what the operator asked for
/var/lib/minimuas/fabric/good       e.g. "nfd wifi"        ← last cell that passed health
/var/lib/minimuas/fabric/active     e.g. "nfd wifi"        ← what apply last brought up
```

Default (fresh node): `desired = good = nfd wifi` — the field-proven cell.

**`muas-fabric-apply.service`** (oneshot, the only writer of `active`/`good`):

```
1. read desired (fall back to good, then to "nfd wifi")
2. stop the non-selected backend units; free /run/nfd/nfd.sock
3. start the selected backend (forwarder + link plumbing + its reconcile timer)
4. health check (below)
   pass → active := desired; good := desired; reach muas-fabric.target
   fail → log + revert: bring up `good` instead, re-run health
          (good is known-working by induction; if even good fails local health,
           keep retrying good — never oscillate back to a failed desired)
5. nudge app reconnect (§6)
```

**Health check** — two phases:

- **Local (hard, gates promotion):** socket exists and answers
  (`nfdc status` / `ndn-ctl status`), `/muas` route present toward ≥1 face,
  radio cells: medium face mounted (forwarder face list shows `radio://` /
  the uplink peer face up).
- **Fleet (soft by default):** `ndn-ping` the GCS (drones) / any drone (GCS) with a
  generous timeout. Failure logs + marks `status: degraded` but does **not** roll
  back — a node switched before its peers is not broken. `muas-fabric set --strict`
  makes the fleet probe hard (for switches done while the fleet is known-up).

**Watchdog** (`muas-fabric-watchdog.timer`, 60 s): re-runs local health against
`active`; on failure re-triggers apply (which will fall back to `good`). Covers
mid-mission failures (radio dongle drops, forwarder crash-loop) — the node
self-heals to last-known-good without operator action.

**Field CLI** (`muas-fabric`, a small shell wrapper):

```
muas-fabric status                  # active/desired/good + health summary
muas-fabric set <fwd> <link>        # write desired, systemctl restart muas-fabric-apply
muas-fabric set --strict ...        # fleet probe is a hard gate
muas-fabric revert                  # desired := good; re-apply
```

Fleet-wide switching stays operator-driven (ssh loop / GCS script) in v1; a
named-data control channel for it (like muas-v2-svsgen) is a later nicety.

---

## 4. The four cells

Each cell = one systemd unit-set the fabric module can bring up. All share
`muas-fabric.target` as their completion point.

### 4.1 `nfd wifi` — today's path, refactored in place
- `nfd.service` + the existing nfdc plumbing (multicast strategy on `/muas`,
  udp peer faces mtu 1452 reliability on, routes, 30 s self-heal timer) moved
  verbatim into the backend unit. Zero behavior change — this is the migration
  baseline.

### 4.2 `ndn-fwd wifi`
- `ndn-fwd.service` with a generated TOML: `[[face]] kind="udp"` per peer +
  `[[route]] prefix="/muas"` per face, `[management] face_socket="/run/nfd/nfd.sock"`,
  `[security] profile="disabled"` (v2 trust lives in NDNSF/NAC-ABE).
- **Verify items** (bench, before field): ndn-fwd's multicast-strategy equivalent for
  `/muas` across multiple udp faces (the wrong-drone best-route regression must not
  come back), and LP reliability/MTU knobs on its udp faces (`UdpFaceSystemConfig`)
  to match the mtu-1452 lesson.

- **VERIFIED 2026-09-15 — item 1 FAILED; this cell is not field-ready.** First
  test on a live 3-airframe fleet (the bug is invisible with one airframe).
  With the whole fleet switched to this cell, every per-vehicle NDNSF service
  call timed out — `video.control_timeout` on all three airframes,
  `sensor.timeout` on iuas-02, zero video frames — while polled telemetry
  limped at 2.60-3.16/s with gaps to 5.7 s. NFD on the same fleet the same
  minute: 3.29-3.34/s, zero gaps, every service answering.

  `route list` showed `/muas` with a **single nexthop** although all three
  peer faces existed (2 → .11, 3 → .12, 4 → .14) and the multicast strategy
  *was* set — multicast has nothing to fan to with one nexthop. The cause is
  the one already recorded in §4.3 for radio: a config `[[route]]` is a direct
  FIB add that the RIB clobbers once the local app registers the same prefix.
  The TOML emits one `[[route]]` per peer and the RIB overwrites them; radio
  was fixed by registering through the RIB, wifi never was.

  A fix is committed (the wifi parity setup now does `ndn-ctl route add` per
  peer face) but is **UNVALIDATED**: after redeploy the persistent peer UDP
  faces were missing from the face table entirely and `/muas` had no route at
  all — a second, separate problem, cause not established. Note that
  `muas-fabric apply` short-circuits on "already active + healthy" and does
  **not** re-run the setup script, so testing a setup-script change requires
  bouncing the cell or running the script directly.

  The fleet was reverted to `nfd wifi` and re-verified FLIGHT-READY.

### 4.3 `ndn-fwd radio` — native (T3)
- One ndn-fwd owns the app socket AND the medium:
  ```toml
  [[face]]
  kind = "radio"
  [[face.radios]]
  driver  = "rtl8822e"
  channel = 149
  address = "1-1.4"          # spare dongle; mesh0 untouched (I4)
  [[route]]
  prefix = "/muas"
  face = 0
  ```
  Env: `NDN_GUARD_LIVE_LINK=1`. The medium face is inherently broadcast
  (RX-union/TX-fanout), so the multicast-strategy question vanishes on radio.
- Full native cognition + FIB demand keys + Tier-1 gate — the T3 reference cell.

### 4.4 `nfd radio` — NFD peers with the LP medium (T2 bridge)
- `nfd.service` (app socket, strategy) + `ndn-fwd-uplink.service`: a second ndn-fwd
  whose TOML has **only** `[[face]] kind="radio"` (as 4.3) + `[[face]] kind="udp"
  bind="127.0.0.1:6364"` + `/muas` routes between them, management socket at
  `/run/ndn-fwd-uplink/sock` (NOT the app socket).
- nfd side: one extra udp face to `127.0.0.1:6364`, `/muas` route toward it.
- Cognition, name-MAC, link-FEC, time-sync all run in the uplink (self-contained at
  the medium — tiers doc §"effort line"). NFD is unpatched.
- Cost of the extra hop is a *measured quantity*, not a hidden one — it is part of
  what this cell reports. Later upgrade path: replace the uplink with the
  NDN-over-TAP shim (same cell name, fewer moving parts) when/if it exists.

Cell files live as `fabric/<forwarder>-<link>.nix` modules, all imported; the apply
script selects among their unit-sets at runtime (units exist always, are started
selectively — I3).

---

## 5. What a switch does to running missions

A fabric switch restarts the forwarder under live NDNSF apps. Consequences:

- ndn-cxx app Faces on the Unix socket drop → role daemons exit/error → systemd
  restarts them (existing `Restart=` policy). ABE survives controller restarts via
  the persisted universe (`NAC_ABE_PERSIST_DIR` + gen-token hash), so this is a
  reconnect blip, not a re-mint. Apply nudges explicitly
  (`systemctl restart --no-block` of the role units, same proven pattern as
  `muas-v2-resync`) rather than waiting for crash-restart.
- **Rule of use:** switching is a between-sorties action. `muas-fabric set` refuses
  while the vehicle is armed (reuse `mavlinkArmedProbe`, exit-10 = defer), same
  guard as the resync watchdog.

---

## 6. v2 / v3 relationship

v2 and v3 remain **separate boot generations**; both consume the identical fabric
seam (`/run/nfd/nfd.sock` + `muas-fabric.target` semantics). v3 native mode is
effectively cell 4.3 with the v3 app stack. The fabric mechanism is therefore the
shared substrate for the 2×2 *and* the v2-vs-v3 comparison; no runtime stack
switching in v1 (heavier, and the dashboard/motion-primitive convergence is the
actual v2/v3 work — tracked separately).

---

## 7. Migration plan

| Phase | Content | Risk gate |
|---|---|---|
| **0** | Pure refactor: introduce `muas-fabric.target` + `muas-v2-identities` split; `nfd wifi` is the only backend; role deps rewired to the target. | Deploy to one drone; verify zero behavior change (dashboard, mission, bench). |
| **1** | State files + apply/health/rollback + `muas-fabric` CLI + watchdog; add `ndn-fwd wifi`. | Bench nodes: switch back and forth under load; kill the active forwarder and watch the watchdog recover; verify the two §4.2 items. |
| **2** | Radio cells 4.3 + 4.4. Prereq: ndn-workspace commits pinned (post-#83 `FrameIo`, `address` selector) + ndn-fwd/uplink packaged from those revs. | Two bench nodes with spare dongles: rung ladder (ping → svs sync → ABE bootstrap → mission) per radio cell. |
| **3** | Field A/B; optional TAP shim; optional named-data fleet-wide switch channel. | — |

## 8. Open/verify list

1. ~~ndn-fwd multicast-strategy~~ ANSWERED (phase-2 response §3a): default is
   best-route (one nexthop) — the fabric's ndn-fwd setup script sets the
   multicast strategy via `ndn-ctl strategy set /muas` at every cell start.
2. ~~ndn-fwd MTU/LP-reliability~~ ANSWERED (§3b): udp MTU defaults 1400
   (< 1452 — already safe); LP reliability enabled per-face at cell start via
   `ndn-ctl face update <id> --flags 0x2`. Static TOML fields = upstream
   feature request.
3. App prefix re-registration after socket swap: confirm restart-nudge covers every
   registrar (NDNSF runtime + journal publishers + svsgen).
4. ndn-fwd packaging from committed revs (currently local dirty trees; scratch
   derivation proves the build recipe).
5. Health-check probe target selection on the GCS when no drone is up (soft phase
   already covers it; confirm status wording doesn't alarm operators).
6. `nfdc`/`ndn-ctl` command-signing: current nfdc units sign with per-node identity;
   ndn-ctl with `require_signed_commands=false` — decide whether the field posture
   accepts unsigned local mgmt on ndn-fwd cells (localhost-only socket) for v1.
