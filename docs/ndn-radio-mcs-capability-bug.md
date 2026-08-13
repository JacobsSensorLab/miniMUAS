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
