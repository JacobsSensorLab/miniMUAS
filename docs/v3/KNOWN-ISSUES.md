# Known issues & follow-ups

Live tracker of defects and design debts found during the v3 build,
ordered by risk. Fix in the named layer repo; check off with the commit.

## Coordination protocol (uas-fleet-node) — found by the M3 co-sim harness

1. ~~**Release/adopt limit cycle.**~~ **FIXED (round 3 agent wave).**
   `adopt_remote` now gates on the peer entry's `gps_time_ns` being newer
   than our recorded release for that pair (fresh CPA violations re-engage
   ungated via the detector); plus engage hysteresis — |bias| < 0.75 m
   never engages (sub-hover-noise churn). This was the root cause of the
   post-task hover oscillation ±0.5 m in the 2026-07-10 eval (867
   clear/adopt cycles in `agent-iuas-01-1783708698.jsonl`). Regression
   tests: `released_pair_does_not_readopt_from_stale_coord_entry`,
   `sub_noise_bias_never_engages` (uas-fleet-node).
2. **Cold-start blindness (~6 s).** `fetch_telemetry` is cache-and-enqueue:
   tick 1 empty (+3 s relaxed recheck), tick 2 may hold the peer's
   pre-takeoff sample (+3 s) — freshly booted fleets cannot coordinate for
   ~6 s even on a perfect link. Fix direction: prime the cache at startup
   and distinguish "no sample" from "stale pre-flight sample".
3. ~~**Confirm is invisible on the wire.**~~ **FIXED (round 3 agent
   wave).** pending→coop now republishes coord/status. Note: before the #1
   fix, "coop" only ever appeared on the wire *because of* the limit
   cycle's re-adoptions — fixing #1 exposed this one immediately (the
   agent smoke test pinned the buggy visibility).

## Agent (muas-agent)

4. **`ensure_airborne` holds the backend mutex during the climb** — stalls
   telemetry for the duration on real vehicles (flagged in the M3 build).
5. **Flight execution stubs** — raster/investigate/video/sensor-capture
   `now` executed as of v3.1; round 3 adds `override`
   (fly-capture-resume with raster suspension) and `opportunistic`
   (watchpoints), plus the acoustic flyover pattern for audio-only
   investigations. Remaining stub: capture execution without a sensor
   feed fitted (no hardware), and in-orbit/in-flyover mission captures.
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
