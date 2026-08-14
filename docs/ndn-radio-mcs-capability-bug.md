# ndn-workspace: a81a RX capability mis-advertised → one-way radio link (field-diagnosed)

**Symptom (live fleet, 2026-08-13):** ndn-fwd radio cell, drone RTL8812EU
(userspace, usb-addr) <-> GCS MT7612U (af-packet/mt76x2u kernel), ch149 US.
Drone→GCS perfect (85+ MiB RX). GCS→drone: NOTHING decodable except legacy-6M
control frames — no Interest ever arrives, no round trip, endless drone-side LP
retransmissions (acks never return).

**Root cause (evidence-backed):** GCS TX is fine — strace shows sendto()=96 on
wlan0 and the mt76 hw queue drains. tshark on-air capture: GCS data frames go
out at **2-stream HT MCS 9** (radiotap.mcs.index 9); the drone's minimal
userspace RTL8812EU brings up **one RX chain** and cannot decode 2-stream
MCS>=8. Cognition never backs off because `radio_face.rs build_rtl8822e`
advertises the a81a as `FULL_RX_MCS` ("a81a decodes full HT/VHT") — so the
peer's `worst_neighbor_rx_mcs → force_legacy` gate (medium.rs) never engages.
This is the LEGACY_ONLY_RX mis-attribution the comments at radio_face.rs:421-424
flip-flop over. Regulatory was ruled out (US DFS-FCC, 5730-5850 clean, no
no-IR); af-packet injection works.

**Working rates GCS→drone measured:** legacy OFDM 6M and single-stream HT
(MCS 0-7) decode; MCS 8-15 (2-stream) do not.

**Asks:**
1. `build_rtl8822e`: advertise the true single-chain RX (cap rx_mcs at 7, or
   LEGACY_ONLY_RX) so peers' force_legacy/rate gates engage. This alone closes
   the link.
2. Optionally: per-radio `max_mcs`/`max_nss` in `RadioDeviceConfig` so an
   af-packet TX rate can be capped declaratively (we confirmed unknown TOML
   fields are silently ignored today — a validation warn would also help).
3. FYI: every NDN_RADIO_* rate env is libusb-only; af-packet TX rate is
   plan-only. Documenting that would have saved a bench cycle.

Fleet is back on the nfd/wifi cell meanwhile; the radio cell re-tests in one
command per node once a fixed rev is pinned.

---

## Resolution (ndn-workspace, 2026-08-13)

Root cause confirmed and fixed at both layers — the wrong advertisement **and** the gate that
couldn't act on anything but the extreme.

1. **`build_rtl8822e` now advertises the truth: single RX chain.** New constant
   `SINGLE_STREAM_HT_RX_MCS = 7` (report.rs). The a81a/RTL8812EU userspace bearer advertises it
   instead of `FULL_RX_MCS`. Deliberately **not** `LEGACY_ONLY_RX` — you measured MCS 0–7 decode, and
   legacy-6M would throw away ~10× the throughput.

2. **The worst-receiver cap is now graded, not binary.** The old gate only engaged at
   `max_rx_mcs == LEGACY_ONLY_RX (0)`, so advertising `7` alone would *not* have closed the link —
   your "this alone closes the link" assumed a graded gate that didn't exist. `RadioPolicy` now reads
   `MediumView::worst_neighbor_rx_mcs` and, for a neighbour advertising `1..=7`, caps **both** the MCS
   ceiling (≤ that value) **and** the stream count to 1 — because a 1-chain RX cannot decode a
   2-stream frame at any index, which is exactly your MCS-9 (2-stream) failure. `LEGACY_ONLY_RX` still
   routes through the existing legacy-rate gate; `FULL_RX_MCS`/absent leave the radio's own ceiling.
   Unit test `single_stream_neighbor_caps_mcs_and_stream_count` covers it. **This is what actually
   closes GCS→drone** (the GCS caps to single-stream ≤ MCS 7, which the drone decodes).

3. **Declarative static caps (ask #2).** `RadioDeviceConfig` gains `max-mcs` / `max-nss`, clamped onto
   the `RadioCapability` in all three Wi-Fi builders (`RadioCapability::with_wifi_caps`). Cognition
   already reads `cap.max_mcs()`/`max_nss()`, so this bounds the **transmit** rate declaratively —
   including an `af-packet` TX — and closes the brief cold-start window before the first reception
   report arrives:
   ```toml
   [[face.radios]]
   driver  = "af-packet"
   interface = "wlan0"
   channel = 149
   max-mcs = 7     # never TX above single-stream MCS 7 (peer has one RX chain)
   max-nss = 1
   ```
   *Not done:* the validation **warn** for unknown TOML keys — serde can't warn (it either ignores or
   hard-errors, and `deny_unknown_fields` would reject existing configs). Flagged as a follow-up needing
   custom deserialization; the real fields above remove the immediate need.

4. **Doc (ask #3).** `radio_face.rs` module docs now state plainly: `NDN_RADIO_*` rate/knob envs are
   **libusb-only**; an `af-packet` TX rate is **plan-only** (cognition plan → actuator `set_rate` →
   radiotap), capped via the worst-receiver mechanism or `max-mcs`/`max-nss`, never an env.

Touched: `ndn-radio-cognition` (`report.rs` const, `sense.rs` `MediumView::worst_neighbor_rx_mcs`,
`policy.rs` graded cap + test), `ndn-radio-hal` (`RadioCapability::with_wifi_caps`), `ndn-config`
(`max-mcs`/`max-nss`), `ndn-face-monitor-wifi` (re-exports), `ndn-fwd` (`radio_face.rs`: advert fix +
cap wiring + doc). Tests green: cognition 83, monitor-wifi 123, config 17; `ndn-fwd --features
radio-libusb` checks clean.

---

## Retest at the fix SHAs (ndn-fwd 3553372 / ndn-ext 65b0340 / ndn-rs 54f5681e) — 2026-08-14

**The MCS fix WORKS. The link still does not close — the remaining fault is a
different, sharper one.**

### Confirmed fixed
On-air capture (GCS `tshark -i wlan0 -Y llc.type==0x8624`, radiotap):
frames now go out at **`radiotap.mcs.index 7` / 72.2 Mbps = single-stream
MCS 7 SGI** (was 2-stream MCS 9). The graded worst-receiver cap + `max-mcs=7`
/ `max-nss=1` both took effect. Rate is no longer the blocker.

### The remaining fault: Interests never cross, in EITHER direction
Face counters after ~4 min on the radio cell (both nodes `ndn-fwd radio`,
health ok):

| | in interests | in data | out interests | out data |
|---|---|---|---|---|
| GCS   | **0** | 195 (23.0 KiB) | 359 | 0 |
| Drone | **0** | **0 (0 B)**    | 235 | 0 |

- Drone→GCS **Data** crosses (195 pkts). Drone→GCS **Interests** do not (GCS
  in-interests = 0 while drone out-interests = 235).
- GCS→drone: **nothing at all arrives** — the drone's radio face shows
  `in: bytes=0 B`, i.e. it hears literally zero frames, not "hears and drops".

So the failure is no longer symmetric-rate: **Data crosses but Interests never
do**, and one direction is completely silent.

### Hypothesis for you to check (from our earlier seam read)
The Tier-0 **name-addressed MAC** looks like the suspect: `TxAddr::PrefixBloom`
/ `Tier0Addresser` derive the frame's address filter from the *producer's own
Data name*, and the RX side only accepts frames matching **registered served
prefixes** (`with_bloom_relay`). An **Interest** carries the consumer's desired
name, not a served prefix — so if the Bloom RX filter is applied to Interests,
or if the RX filter isn't fed the node's registered prefixes, Interests are
filtered out while Data (whose Bloom matches its own name) passes. That matches
the observed Data-yes/Interests-no asymmetry exactly.

Secondary: whether the GCS's af-packet injection is on the air at all. The
capture showed only two senders, both with locally-administered (ephemeral
rotating nonce) MACs — one at MCS 7 (data), one at legacy 6M (reports),
matching the drone's known dual-rate pattern. We could not positively
attribute either MAC to the GCS (a monitor iface may not capture its own TX),
so "GCS frames never reach the air" is not excluded. A `ta`-to-node mapping
(or disabling nonce rotation for a debug run) would settle it in one capture.

### Fleet state
Reverted to `nfd wifi` on both nodes; telemetry live again (~0.3 s cadence).
Retest is one `muas-fabric set ndn-fwd radio` per node once there's a candidate
fix — repin cost is ~15 min.
