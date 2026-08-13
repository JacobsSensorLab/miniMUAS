# Request to the ndn-workspace session — unblock miniMUAS fabric phase 2 (radio cells)

**From:** miniMUAS v2 session (fabric / field-A/B work)
**Context:** miniMUAS v2 now runs a runtime-switchable NDN fabric behind
`muas-fabric.target` (deployed + gate-tested on the fleet 2026-08-13). Phases 0/1
(nfd-wifi cell, health-gated switch/rollback/watchdog, `muas-fabric` CLI) are live.
**Phase 2 adds the ndn-fwd and radio cells** — `ndn-fwd wifi`, `ndn-fwd radio`
(native), `nfd radio` (NFD peering with the LP medium via a radio-uplink ndn-fwd) —
and it is blocked only on your repos being committed/pushed so Nix can pin them.
Design refs in this repo: `docs/muas-fabric-design.md`,
`docs/named-data-radio-integration-tiers.md` (the tier/effort table built from your
code; the "cognition is self-contained at the medium face" finding is why the
`nfd radio` cell is a thin LP bridge, not an NFD patch).

## 1. THE BLOCKER — commit ndn-fwd, push the sibling set, send SHAs

The fleet build pins sources with `fetchFromGitHub`, so I need **pushed, committed
revs** of every repo in ndn-fwd's path-dep closure:

| Repo | State I see (2026-08-13) | Need |
|---|---|---|
| `ndn-fwd` | **dirty: 18 files, 2 unpushed** (incl. the DeviceSelect/`address`/guard work + the `WifiRadio→FrameIo` build fix) | **commit + push** |
| `ndn-rs` | clean, 39 unpushed | push |
| `ndn-ext` | clean, 130 unpushed | push |
| `ndn-radio-drivers` | clean, 259 unpushed | push |
| `ndn-embedded` | clean, 5 unpushed | push |
| `ndn-sim` | (workspace member pulled in during source assembly) | push whatever rev is consistent |

**Deliverable: one line per repo — `<repo> <pushed SHA>` — for a mutually
consistent set** (i.e. ndn-fwd at that SHA builds against those sibling SHAs).
That's all phase 2 strictly needs.

How they'll be used (context, no action needed): aarch64 nix build offloaded to
nixbuild.net; sources assembled as siblings under one root; `cargoRoot = ndn-fwd`;
`cargo -p ndn-fwd -p ndn-tools --features ndn-fwd/radio-libusb`; `doCheck = false`;
`libusb1` as the only native dep. This recipe went green pre-#83; item 2 below
re-verifies it post-#83.

## 2. Please confirm (or correct) four build/behavior facts post-#83

a. `cargo build -p ndn-fwd -p ndn-tools --features ndn-fwd/radio-libusb` is green at
   the SHAs you send (you fixed the `WifiRadio` import — just confirm it's in the
   committed set), and `ndn-fwd/Cargo.lock` is committed at that rev.
b. The sibling closure for that feature set is still exactly
   `{ndn-fwd, ndn-rs, ndn-ext, ndn-radio-drivers, ndn-embedded, ndn-sim}` — no new
   sibling repo entered the path-dep graph.
c. `rust-toolchain.toml` still pins 1.96.0 (or tell me the new pin).
d. `RadioDeviceConfig.address` (e.g. `"1-1.4"`) + `NDN_GUARD_LIVE_LINK=1` is the
   committed way to pin the spare dongle and refuse an `operstate=up` device — the
   fleet TOML will rely on both.

## 3. Three design questions (answers shape the fabric TOMLs; short answers fine)

a. **Fan-out semantics over multiple udp faces.** The wifi cells register `/muas`
   toward N peer faces. NFD needs the *multicast strategy* there (best-route
   misroutes per-vehicle commands — hard-learned field lesson). What does ndn-fwd's
   forwarding do with N nexthops for one prefix — all-nexthops fan-out, best-route,
   or configurable? If configurable: the TOML/`ndn-ctl strategy` incantation.
b. **Per-face LP reliability + MTU on udp faces.** NFD faces run
   `reliability on mtu 1452` (AP/STA path silently drops IP-fragmented datagrams;
   NDNLP must do the fragmenting + ARQ). What's the ndn-fwd equivalent —
   `UdpFaceSystemConfig` fields, per-`[[face]]` options, or not yet implemented?
   If missing, this is a feature request: it gates the `ndn-fwd wifi` cell's
   parity with the proven NFD config.
c. **Radio-uplink run mode.** For the `nfd radio` cell I plan a second ndn-fwd
   whose TOML has ONLY `[[face]] kind="radio"` (+ `address`), one
   `[[face]] kind="udp" bind="127.0.0.1:6364"`, `/muas` routes between them, and
   `[management] face_socket="/run/ndn-fwd-uplink/sock"` (NOT the app socket).
   Any known landmine (e.g. mgmt socket path override, route direction radio↔udp,
   LP settings on the loopback face)? A blessed minimal uplink TOML would be great.

## 4. FYI — artifacts of mine in your repos (don't be surprised; don't prune)

To port v2 field fixes into miniMUAS v3 I had to build v3 against its
**2026-07-11-vintage** deps (your repos have since renamed `ndn-ndnsf→ndnsf-rs`
etc.). I created **detached-HEAD worktrees from your repos** — they'll show in
`git worktree list`:
- `~/Documents/Dev/ndn-workspace-v3pin/{flotilla, ndn-ext, ndn-rs, ndn-sim}`
  (@ 07-09..10 revs)
- `~/Documents/Dev/ndf-rs-v3pin` (worktree of QUAD `ndf-rs` @ 47c39f3d)

Some pin worktrees carry **intentional dirty Cargo.toml edits** (path deps
repointed inside the pin universe). They are build scaffolding for the v3 branch;
please leave them until v3 migrates forward onto your current APIs (at which point
they get pruned). Nothing was committed to your branches.

## Priority order
1. §1 (commit ndn-fwd + push all + SHAs) — the only hard blocker.
2. §2 confirmations — cheap, prevents a wasted build cycle.
3. §3a/§3b — needed before the `ndn-fwd wifi` cell is field-comparable.
4. §3c — needed for the `nfd radio` cell (last rung before the full A/B).
