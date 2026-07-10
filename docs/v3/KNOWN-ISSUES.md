# Known issues & follow-ups

Live tracker of defects and design debts found during the v3 build,
ordered by risk. Fix in the named layer repo; check off with the commit.

## Coordination protocol (uas-fleet-node) — found by the M3 co-sim harness

1. **Release/adopt limit cycle.** After a maneuver releases, `adopt_remote`
   re-engages from the peer's stale-cached coord entry with no geometric
   gate — a released pair ping-pongs coop maneuvers indefinitely
   (reproduced at 0.1% link loss, >45 s, never settles). Likely inherited
   from the v2 Python (same algorithm). Fix direction: require a fresh CPA
   violation (or entry gps_time_ns newer than our release) before adopting.
2. **Cold-start blindness (~6 s).** `fetch_telemetry` is cache-and-enqueue:
   tick 1 empty (+3 s relaxed recheck), tick 2 may hold the peer's
   pre-takeoff sample (+3 s) — freshly booted fleets cannot coordinate for
   ~6 s even on a perfect link. Fix direction: prime the cache at startup
   and distinguish "no sample" from "stale pre-flight sample".
3. **Confirm is invisible on the wire.** pending→coop only mutates the
   local active table (+`coord.confirmed` event); coord/status republishes
   on engage/release only, so peers/observers see "coop-pending" until
   quiet release. Harmless to the algorithm (adoption carries it) but
   misleading to dashboards; consider republishing on confirm.

## Agent (muas-agent)

4. **`ensure_airborne` holds the backend mutex during the climb** — stalls
   telemetry for the duration on real vehicles (flagged in the M3 build).
5. **Flight execution for raster/investigate/sensor/video is stubbed**
   (policy-gated, busy-occupying, journaled — but no flight). Wiring the
   uas-flight primitives through the runner into the agent handlers is the
   next increment; the dashboard's artifact/video relays are already wired
   for that day.
6. NdnsfCarrier path runs `.insecure()` — signed mode + trust schema is the
   security milestone's work.

## Cleanups

7. Unify `coordination.rs`'s local PeerTelemetry/CoordEntry with
   uas-fleet-data kinds (they match on the wire; one definition should
   win).
8. Add `Send` bounds to uas-fleet-node callback boxes so PeerGuard can ride
   tokio instead of a dedicated OS thread.
9. uas-fleet-data `BuiltManifest` should export record-term hashes
   (uas-console recomputes them by label lookup).
