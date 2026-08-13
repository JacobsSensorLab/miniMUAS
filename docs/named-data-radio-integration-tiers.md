# Named-data radio × foreign forwarders — integration tiers and the effort line

**Purpose:** treat named-data radio as an NDN-native L1/L2 (link+phys, data-centric),
find the seam a *foreign* forwarder (NFD) attaches to, and map which radio features
are available at which integration tier — to pick the best-result-per-unit-work line.
Grounded in a code-level read of ndn-workspace (post-#83: `WifiRadio` folded into
`FrameIo`; `DeviceSelect`/`address` shipped).

## The stack and its seams

```
ndn-radio-hal          FrameIo trait (inject / recv_frame / set_rate / mesh_common_view)
                       "no I/O, no NDN" — THE bare L1/L2 seam            (lib.rs:397)
ndn-frame-io           on-air framing: RawNdn ethertype 0x8624, radiotap, AfPacketBackend
ndn-radio-drivers      USB/serial backends impl FrameIo — "does no NDN", standalone crate
ndn-radio-cognition    pure sans-IO SENSE/DECIDE/ACT (names as hashes)
ndn-face-monitor-wifi  RadioMediumFace (RX-union / TX-fanout over N bearers)
                       + LpLinkService  →  **NDNLPv2 on the air**        (medium.rs:806)
ndn-fwd radio_face.rs  mounts medium face + cognition on the native engine
```

Load-bearing facts:

1. **The medium face is NDNLPv2** (`FaceKind::Wfb`, `remote_uri="radio://broadcast"`,
   MTU 2272, LP fragmentation/reassembly). Any LP-speaking forwarder — NFD included —
   interoperates at the wire. Radio is a *shared LP medium*, not an ndn-fwd internal.
2. **Cognition is self-contained at the medium face.** `RadioControl` is a
   `LinkServiceFeature` whose `on_egress`/`on_ingress` peek the LP name off the wire it
   already carries (control.rs:874,889). Demand tracking, rate/power/FEC/channel
   adaptation, and cooperative sensing (named reception reports) need **no push-down
   from the forwarder**.
3. **Time-sync is self-contained too.** `FaceScheduler` gates each TX by
   `(name, clock)` — a pure function, no coordinator, configured from `NDN_SCHED_*` env
   and fed by inbound frame stamps / `FrameIo::mesh_common_view` (µs TSF common-view).
   Off by default; send path byte-for-byte unchanged when unset.
4. Exactly **two** hooks need forwarder internals:
   - **FIB context / demand key** (radio_face.rs:50,211): without it cognition falls
     back to coarse first-component demand aggregation — degraded, not broken.
   - **Tier-1 name gate (BF-PIT/BF-CS)** (lib.rs:845): *must* be fed the forwarder's
     real PIT/CS or it drops Data the node asked for (regression test lib.rs:1043).
     This is the only feature that genuinely requires patching a foreign forwarder.

## Tier × feature × NFD-modification table

| Tier | What you build | Raw TX/RX | LP peering | Name-MAC T0 (prefix-Bloom) | Link-FEC / A-MSDU | Adaptive rate | Cognition (rate·power·FEC·chan) | Coop sensing | Time-sync (slot/FHSS/µs) | Name-gate T1 (BF-PIT/CS) | NFD change |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **T0** bare frames | link `ndn-radio-drivers` / `AfPacketBackend`, drive `FrameIo` | ✅ | — | — | — | — | — | — | — | — | none (but NFD can't reach libusb without a bridge) |
| **T1** LP bridge | wrap `FrameIo` in `RadioMediumFace`+`LpLinkService`; NFD peers with it | ✅ | ✅ | ✅ | ✅ | ✅ | — | — | — | — | **none** |
| **T2** bridge + cognition | T1 + mount `RadioControl` as a feature *in the bridge* | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ (coarse demand key) | ✅ | ✅ | — | **none** |
| **T3** native | forwarder engine hosts the face: FIB context + PIT/CS-fed T1 gate | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ (full FIB-prefix demand) | ✅ | ✅ | ✅ | **source patch** (ndn-fwd has this natively) |

## The effort line

**Best result-per-unit-work = T2: bridge at LP, don't patch NFD.** A foreign
forwarder whose traffic rides the medium inherits ~90% of named-data radio —
name-aware MAC, link-FEC, adaptive rate, full cognition, cooperative sensing, µs
time-sync — with **zero source modification**. Only Tier-1 BF-PIT/CS gating (and
fine-grained FIB demand keys) is T3-native territory: high integration cost, and the
misfed failure mode (dropping wanted Data) makes it dangerous to bolt on.

Two concrete T2 bridge shapes for NFD:

- **radio-uplink (exists today, zero new code):** a second ndn-fwd process configured
  with *only* a radio face + one udp/tcp face (each `[[face]]` is independent;
  `kind="radio"` + `address="1-1.4"` pins the spare dongle). NFD peers over
  `udp://127.0.0.1`. Cost: second forwarder process + one extra hop.
- **NDN-over-TAP shim (moderate new code):** re-point the `ndn-nan/src/ndi.rs`
  TAP↔`FrameIo` pattern from host-MAC/IPv6 to NDN LP at ethertype 0x8624 — presents a
  kernel netdev that NFD binds a standard Ethernet face to; the shim hosts the medium
  face + cognition. No second forwarder, no udp hop, still zero NFD patches. Does not
  exist yet; `ndi.rs` is the template.

ndn-fwd's genuine native edge (the thing the A/B measures) is T3: PIT/CS-fed Tier-1
gating and FIB-prefix demand aggregation.

**Implementation caveat** (recurring in the code): features can be *decided but
unactuated* if a seam isn't wired (control.rs:429, medium.rs:938). Any bridge must
mount the actuator loop (`spawn_control_loop` + `MediumActuator`), not just the face,
or cognition silently no-ops.

## Consequence for the miniMUAS fabric

The fabric's radio branch is **"bring up the shared LP medium, pick which forwarder
peers with it"** — structurally identical for nfd and ndn-fwd — not "tunnel through
ndn-fwd". See `muas-fabric-design.md`.
