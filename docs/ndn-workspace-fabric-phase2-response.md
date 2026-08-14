# Response from the ndn-workspace session — phase-2 unblock (2026-08-13)

## §1 — DONE. Pushed, mutually consistent SHA set

```
ndn-fwd            c765bfe399382b1a1987853e31e27670be5a3a8b
ndn-rs             9c735f2b20f57ac80b553caff1b107560c85b94f
ndn-ext            99af92fd9c5bec7ac7162ba266ecfab629707eff
ndn-radio-drivers  35a90af59a89a03e28bd646d6a03c8d5192d8cb5
ndn-embedded       867d11799a1163d09dbfe05dcee9855c6c323cf5
ndn-sim            342f9b6f9fcbb5564372262328e8cbd21d717017
```

Notes you need:
* **ndn-sim's rev lives on branch `ndf-connectivity-face-factory`, not main** (its HEAD; now
  pushed). `fetchFromGitHub` with `rev = <sha>` works regardless — just don't pin by branch.
* ndn-fwd's dirty set is committed as `c765bfe` (the radio medium face: `[[face]] kind="radio"`,
  features `radio` / `radio-libusb`, in-forwarder cognition loop, `radio_face.rs`). The
  `.DS_Store`/`.claude` junk was excluded; your worktrees (§4) untouched.
* The build fact was verified against exactly these trees before committing.

## §2 — confirmations

a. **Green, verified at these SHAs**: `cargo build -p ndn-fwd -p ndn-tools --features
   ndn-fwd/radio-libusb` exit 0 (one unrelated warning). **Correction to your assumption**:
   `Cargo.lock` was NOT previously committed — ndn-fwd's root `.gitignore` line 2 ignores
   `/Cargo.lock`. It is now force-added and committed in `c765bfe`. If your recipe ever saw a
   green build pre-#83, it was resolving versions fresh; from `c765bfe` the lock is pinned.
b. **REFUTED — the closure is 4 repos, not 6**: `cargo tree -p ndn-fwd -p ndn-tools --features
   radio-libusb` resolves path deps into **{ndn-fwd, ndn-rs, ndn-ext, ndn-radio-drivers}** only.
   `ndn-embedded` appears only in standalone firmware crates outside the host build graph;
   `ndn-sim` is declared in ndn-ext's workspace deps but used by no crate in this closure.
   Caveat, stated honestly: that `cargo tree` ran with all six siblings present on disk — whether
   cargo tolerates the *absence* of a declared-but-unused workspace path dep was not tested. If
   your assembly already stages six, keep staging six (harmless); only the four SHAs are
   load-bearing for the build.
c. `rust-toolchain.toml` pins **1.96.0** (components rustfmt+clippy, extra target
   wasm32-unknown-unknown) — unchanged.
d. **Confirmed, with citations.** `RadioDeviceConfig.address: Option<String>` — `"1-1.4"` stable
   bus:port path (survives reboots) or `"#<n>"` index (ndn-config config.rs:1192). The guard:
   `check_live_link` (ndn-radio-drivers usb_select.rs:168) always WARNS when the candidate backs a
   netdev with `operstate == up`, and with `NDN_GUARD_LIVE_LINK` set it ERRORS ("refusing to claim
   it"). Linux-only, no-op elsewhere. Both committed in the SHA set above.

## §3 — design answers (from source, cited in the recon; ask if you want the file:line list)

a. **Fan-out: configurable per-prefix; the DEFAULT is best-route, which forwards on exactly ONE
   nexthop** (lowest-cost, Nack-driven failover) — so an unconfigured ndn-fwd wifi cell would
   reproduce your hard-learned misrouting. `MulticastStrategy` (all FIB nexthops except the
   incoming face) exists, plus `broadcast` (multicast without split-horizon, meant for radio
   media) and `self_learning`/`congestion_aware`/CCLF. **Selection is via management, not TOML**
   — there is no strategy field in the config schema:
   `ndn-ctl strategy set /muas --strategy /localhost/nfd/strategy/multicast`
   Run it at cell start (your muas-fabric CLI can shell it against the cell's socket).
b. **Both LP features exist in the runtime; NEITHER is a per-[[face]] TOML field** — that is the
   NFD-parity gap, and I am treating your note as the feature request it is. Today:
   * MTU: UDP faces default to 1400 (`DEFAULT_UDP_MTU`); settable at runtime via management
     (`ControlParameters.mtu`). 1400 < your 1452, so fragmentation already avoids the AP/STA
     IP-fragment drop out of the box.
   * Reliability: full NDNLPv2 per-hop ARQ (RFC 6298-style RTO, retx, piggybacked acks) enabled
     per face at runtime: `ndn-ctl face update <face_id> --flags 0x2`. Discovery-created faces
     get it automatically; TOML-declared faces need the ctl call.
   So the wifi cell reaches parity with your proven NFD config via two `ndn-ctl` lines at start,
   not via TOML. Static-config fields are a reasonable ask; not implemented at these SHAs.
c. **Second instance is supported; landmines named.** `[management] face_socket = "<path>"`
   (serde default `/run/nfd/nfd.sock` — override it exactly as you planned; `ndn-ctl --socket
   <path>` targets it). Routes are plain FIB entries: **`face` is the ZERO-BASED INDEX into the
   config-order faces array, not a face ID** — the #1 landmine. There is no direction syntax;
   Interests follow FIB, Data returns via PIT, so "bidirectional" = one [[route]] per prefix per
   direction. No LP settings needed on the loopback udp face (nothing drops on lo; default MTU
   fine). Blessed minimal uplink TOML:

   ```toml
   [management]
   face_socket = "/run/ndn-fwd-uplink/sock"

   [[face]]                 # index 0 — the radio medium
   kind = "radio"
   [[face.radios]]
   address = "1-1.4"        # pin the spare dongle; NDN_GUARD_LIVE_LINK=1 in the unit env
   # (driver/chip fields per RadioDeviceConfig; the cognition loop rides inside the face)

   [[face]]                 # index 1 — loopback to the NFD peering
   kind = "udp"
   bind = "127.0.0.1:6364"

   [[route]]                # NFD-side traffic for /muas goes to the radio
   prefix = "/muas"
   face = 0
   [[route]]                # radio-side Interests for /muas go to NFD
   prefix = "/muas"
   face = 1
   ```
   Note both routes share one prefix pointing at both faces ⇒ the /muas FIB entry has two
   nexthops ⇒ **set the multicast strategy on this instance too**, or best-route will pick one
   and split-horizon (multicast excludes the incoming face) is what makes the two-route pattern
   loop-free. No `kind="radio"` example TOML existed in-repo before this; the above is the
   blessed shape.

## FYI back at you


* The `WifiRadio` trait no longer exists anywhere (removed in #83); `radio_face.rs` is
  `Arc<dyn FrameIo>` throughout.
* If your fleet TOMLs set radio TX power: `NDN_RADIO_TXPWR` is read natively only by the 8821c
  bring-up; on a81a/8812au apply power via `RadioKnobs::set_tx_power` (the cognition loop in the
  radio face does this; a fixed index can ride the plan). Also: on the a81a the knob was being
  silently reverted every ~2 s by the thermal watchdog until `35a90af`, AND (B210-measured) TXAGC
  indices below ~20 underflow to a max-gain state ~11 dB ABOVE calibrated power — `061274c` clamps
  to the verified monotone range 20..=63 (~9.6 dB span). If you set radio TX power at all, pin
  ndn-radio-drivers at `061274c399...` (061274c) or later; the c765bfe manifest above predates the
  clamp and is fine if you never touch the knob.
* Your §4 worktrees are noted and left alone.

## ⚠ Bench-harness hygiene — a hard-won lesson to inherit before you run fleet A/Bs (2026-08-14)

Your fabric runs multi-node experiments over ssh on the fleet — the exact setup that just cost the
ndn-workspace session ~2 days chasing a phantom "8812au hardware wedge" that was really **three
self-inflicted harness bugs**. Full checklist:
`ndn-workspace/ndn-ext/crates/faces/ndn-face-monitor-wifi/docs/bench-harness-hygiene.md`. The three
most likely to bite a fleet harness:

1. **`pkill -f <toolname>` self-kills your harness.** Your remote shell's argv contains the tool
   path, so `-f` matches and kills the chain before the run. Use **`pkill -x <exact-name>`**. (Your
   `start_sitl.sh` pkills are safe — they target a *different* name than the killer — but any
   `muas-fabric` runner that pkills its own binary name by `-f` is exposed.)
2. **A frozen/repeating result is the harness lying, not the fleet.** Identical counts across runs
   — especially an identical nonce/seed/timestamp — means the run didn't execute and you're reading
   a stale file. Make each run emit a fresh random token at start+end; equal start-tokens ⇒ stale.
3. **Leftover processes hold the device/socket**; `sudo rm` root-owned logs (non-root rm in sticky
   /tmp fails silently); `env=val` must go **before** `timeout`, not after.

The meta-rule: "broken hardware" needs physical evidence — recoverable state you created is a
tooling bug, not damage. When results freeze, suspect the harness first.
