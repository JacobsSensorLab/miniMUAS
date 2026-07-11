# RC over named data — from bench to bound control

Status 2026-07-10: transport + failsafe substrate SHIPPED (uas-rc, M6
bench: 26-byte frames, UDP + ndf-spark bindings, silence ladder, pacer,
P11 tiers; uas-mavlink override/manual/release). This doc records the
motivation the owner supplied and the remaining path to flyable,
*bindable* control.

## The friction being killed

ELRS transmitters bind by **binding phrase** — slow, tedious, per-pair,
and it buys only a point-to-point link. The goal: **key control to a UAS
via named data and its security**, inheriting the strengths of a network
— multi-path, multi-radio, observable, revocable — instead of a
dedicated P2P bond per aircraft.

## Why the substrate already fits

- Control is addressed by NAME (`/muas/v3/<vid>/rc`), not by paired
  radio: any authorized sender, any bearer that can carry the name —
  wifi UDP today, monitor-mode named-data radio or an ESP32 bridge
  tomorrow, simultaneously (first-arrival wins by seq; the medium is
  broadcast anyway).
- Spark instance identity + SP-3 replay judging already refuse replayed
  or stale-instance frames before the RC ledger sees them.
- Loss honesty + the silence ladder mean link handover is SAFE by
  construction: worst case is a designed hold/RTL, never stale sticks.

## Binding = ceremony, not phrase (the keying design)

1. **Authorization is an AuthorityRecord grant**: `rc-control` over
   `<vid>` scoped to a pilot identity, minted by a **C2 proximity tap**
   (QR on the CYD/companion, or NFC) at the field table — the same
   grammar as every other fleet grant, ~seconds, no phrase entry.
   Expiry-by-default IS the unbind (a lost transmitter lapses; no CRL,
   no re-bind party).
2. **Frames are attributable**: the RC Spark stream's checkpoint Blocks
   are signed by the pilot identity; the vehicle's RC receiver admits a
   stream only while a live, unexpired grant covers (pilot, vehicle).
   Per-frame signatures stay unnecessary (Spark instance + checkpoint
   anchoring + admission-by-grant), keeping the 26-byte hot path.
3. **1-to-many falls out**: one grant per (pilot, vehicle) pair — or a
   fleet-scoped grant — and the selector chooses One|Broadcast at the
   pacer. Handing off a vehicle between pilots = one C2 tap, visible in
   the audit chain, no radio surgery.
4. **The vehicle side is a strategy consumer**: which grants admit,
   what silence policy applies, whether broadcast is honored — strategy
   records, not constants (ROUND-3 §2).

## Transport correction (2026-07-11) — named data over the fabric, not UDP

R1/R2 shipped over a direct dashboard→agent **UDP socket**. That was
wrong and is being corrected. The socket bypasses the NDN fabric
entirely: no named addressing, no NDN security, no broadcast-native
1-to-many, and — most damning — RC frames never cross the ndn-sim
SimLinks that every other stream (telemetry, coord, services) rides. The
one capability whose entire purpose is to demonstrate **data-centric
control from the network layer to the physical layer** was the one
shortcutting the data-centric stack.

The rule, matching how the network layer already treats bearers:

- **Named data over the engine/faces is the ONLY default.** RC frames are
  published under `/muas/v3/<vid>/rc` and travel the same
  `ForwarderEngine` + faces as telemetry — across the ndn-sim SimLinks in
  sim, across real faces (UDP/AP-STA, and ultimately monitor-mode
  **named-data radio**) in the field. This is what makes the frames
  name-addressed, NDN-secured, cacheable, and broadcast-native for free.
- **UDP is a comparison bearer only**, behind an explicit flag, exactly as
  AP/STA mode is a comparison bearer for the network — never the default,
  never the thing a demo shows first.
- Carriage: ndf-spark over the engine is the intended path (ephemeral,
  sequenced, loss-honest — the RC profile). If the framework's spark does
  not yet offer a turnkey named-data-over-engine carriage (the R1 spark
  binding was spark-over-UDP), that gap is itself a maintainer feedback
  item — file it — and the interim path is RC frames as fast-cycling named
  Data objects served/fetched over the engine like the latest-wins
  telemetry streams. Either way the frames cross the fabric as **named
  data**, not a side socket.

This correction is the immediate RC work; R3–R5 below assume it.

## Remaining path (in order)

- **R1 — agent RC receiver task**: subscribe `<vid>/rc` (spark binding),
  frames → `rc_channels_override`; failsafe intents → hold()/rtl() +
  `release_rc_override`; grant admission stub first (allow-configured),
  real grant check when the trust milestone lands. Journals every
  engage/release/failsafe.
- **R2 — dashboard pilot surface**: Web Gamepad API → WS → the pacer +
  stream sender in the dashboard process; vehicle selector + broadcast
  toggle; an RC status strip (rate, gap %, age, failsafe state, e-stop).
  Keyboard fallback for bench.
- **R3 — SITL end-to-end**: stick → SITL attitude in the virtual
  deployment; measured stick-to-motion latency in --verify; RC-loss
  drill (kill sender, watch the ladder fire on schedule).
- **R4 — ceremony keying**: C2 grant mint + admission on the receiver;
  unbind-by-expiry drill.
- **R5 — bearers**: monitor-mode named-data radio lane (rtl8812eu, M5
  field gear) and the **ESP32 CRSF bridge** (DroneBridge/ELRS-inspired:
  ESP32 speaks named data on one side, CRSF into the FC on the other —
  the drop-in ELRS-receiver replacement; uas-cyd's e-stop fob shares the
  frame). Hardware-gated.
- **MAVLink-over-NDN note**: RC frames deliberately do NOT ride MAVLink
  framing on the wire (26 B < any MAVLink envelope); the adapter speaks
  MAVLink only at the FC boundary. A general MAVLink-over-NDN mapping
  (missions, params, ftp) is a separate, worthwhile spec — queued behind
  R3 evidence.
