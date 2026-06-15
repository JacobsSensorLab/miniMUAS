# miniMUAS v2 — Field Day Checklist (Phase 3)

The field flip is configuration-only: `bench = false` on all three hosts.
That swaps the kinematic sims for real MAVLink flight and NTP for GPS/FC
time. Everything else — radios, faces, identities, detection, the mission
choreography — runs exactly as validated on the bench. This document is
the run order and the doctrine.

## Before leaving campus (the field has no build network)

- [ ] Both repos committed and pushed; `minimuas-src` rev bumped; all
      three nodes rebuilt with the FIELD configuration. **Rebuilds happen
      on campus** — the field uplink (phone NAT on node 03) is not a
      deploy path.
- [ ] Tile cache covers the site: load the dashboard, pan/zoom the field
      coordinates at z15–19, then verify offline (disable the laptop's
      other networks, reload — imagery must still paint). Bulk option:
      `sudo muas-v2-fetch-tiles --bbox latS,lonW,latN,lonE --zooms 15 19`.
- [ ] `muas-v2-agent --dry-run --role wuas` and `--role iuas` exit clean
      on their nodes (import/launch sanity for the new build).
- [ ] Batteries charged; props inspected; RC transmitters bound, safety
      pilot briefed: **RC mode switch out of GUIDED overrides the stack
      instantly** — that is the primary abort, always.
- [ ] GCS USB GPS in the bag (node 03 will not start NDN services
      without it: `timeSource = "gps"`).

## On-site power-up order

1. **GCS (node 03)** first: AP up, USB GPS connected with sky view.
   NDN services gate on `muas-time-set.target`; GPS lock can take
   minutes cold. Verify: `timedatectl` shows a sane date, then
   `systemctl status muas-time` → "clock synchronized".
2. **Drones**: FCs powered with GPS view. `muas-time` on drones reads
   the FC's SYSTEM_TIME via mavp2p — it is valid only after the FC has
   GPS lock. Agents will hold in "activating" until then; that is
   correct behavior, not a hang.
3. Laptop joins the AP → `http://192.168.1.13:8080`.

## Pre-flight gate (all on the dashboard, no SSH needed)

- [ ] Connection tag green; **both vehicle tiles live with
      `source: mavlink`** (sim = the flip didn't deploy — stop).
- [ ] Link tags `0–1 s` green; **clock Δ near 0** on both (large Δ =
      time subsystem, fix before flight: journals correlate by it).
- [ ] AGL ≈ 0, armed no, sane lat/lon on both markers, sitting on the
      imagery where the airframes actually sit.
- [ ] Video toggle each camera briefly: live picture, then off (save
      radio for the mission).
- [ ] Event log quiet.

## Flight test ladder (do not skip rungs)

1. **Manual hop per airframe** (RC only, stack idle): confirms tune,
   GPS, RTL behavior independent of anything we built.
2. **AGL verification (dashboard Takeoff).** On each vehicle tile there is
   a Takeoff control (AGL field + button, guarded to 0.5..max-agl). With
   a safety pilot on the sticks, command a takeoff to a known height
   (e.g. 5 m), let it stabilize, and CONFIRM the tile's reported AGL
   matches the real height within ~1 m. This is the single most
   important check after the 2026-06-15 altitude bug — do not trust any
   autonomous altitude until this passes. Then dashboard Land.
3. **WUAS-only raster.** Flip the IUAS tile's enable toggle OFF (it goes
   to a grey "disabled" tag; its telemetry/video keep working, but no
   investigation will launch and its Takeoff is blocked — RTL/Land/Hold
   stay live as a safety backstop). Run a small raster (~20x15 m, AGL
   6 m, 2 m/s) with the racquet at a known spot. Watch: continuous
   constant-velocity legs (no stutter), frames/detects counting,
   `detect.candidate` hits accumulating, a confirmed target marker, and
   — critically — read `gcs.detection.projection` in the GCS journal and
   compare its estimate to ground truth. Mission completes with the
   target queued-but-not-flown (IUAS disabled). Then dashboard RTL.
4. **Full mission.** Re-enable the IUAS tile. Two racquet placements.
   Expect the bench-validated choreography with real airframes:
   continuous WUAS raster, IUAS launching to orbit target #0 while the
   search continues, queue draining, `targets=N investigated=N`.

## Mission parameters (field defaults)

| Param | Value | Why |
|---|---|---|
| Search AGL | 6 m | GSD: racquet ≈ 100 px at 6 m, ~40 px at 15 m |
| Leg spacing | 5 m | ~7.5 m footprint width at 6 m AGL, healthy overlap |
| Capture every | 4 m | consecutive-frame overlap for dedup confidence |
| Speed | 2 m/s | motion blur + capture settle |
| Min confidence | 0.3 | bench-calibrated for grass background |
| Target separation | 8–10 m | field geo-estimates scatter more than sim |
| Orbit AGL / radius | 8 m / 6 m | camera framing at standoff |

## Safety rails (built in — know what they look like)

- **Range guard**: agents reject areas/targets >300 m from the vehicle
  at the ack — surfaces in the event log as
  `area NNNm away > 300m guard`. AGL guard rejects >20 m.
- **Busy guard**: a tasked vehicle rejects new tasks (`busy:...`).
- **Abort ladder**: RC override (instant, per safety pilot) → dashboard
  per-vehicle RTL/Land/Hold → RTL ALL. Any flight command also raises
  the agent's abort flag; a running raster/orbit terminates within one
  capture/waypoint cycle.

## Known gaps (accepted for this field day)

- Battery/mode in the vehicle tile may read blank/0 over MAVLink
  (defensive getattr; SYS_STATUS not yet plumbed). **Monitor battery on
  the RC telemetry.**
- Detect round-trip is ~8–16 s of NDNSF transport tax; the raster never
  waits on it, but expect target markers to trail the WUAS by ~3–5
  capture points.
- `mode_as_predicted` for real flight is guided-yaw-path (no native
  orbit on the MAVLink link) — circle-mode in journals is sim-only.

## If things go sideways

- Requests vanish / decrypt errors → ordered restart: controller (03,
  cascades gcs+dashboard) → agent 01 → agent 02. Clocks first if Δ is
  wild.
- Identities gone (`ndnsec list` empty) → wipe ritual:
  `sudo rm -rf ~/.ndn ~/.ndn-muas && sudo systemctl restart
  muas-v2-setup` then the ordered restart.
- Segmented fetches time out, small traffic fine → check faces:
  `nfdc face list | grep mtu` must show 1452 on the udp4 peer faces.
- Dashboard up, tiles gray → cache miss for that zoom; zoom out a level
  (z15–17 cached wider than z19) or live with the grid.

## Field test #1 debrief (2026-06-15) and the fixes it forced

First real outing. Four failures, all now addressed in code — recorded
here so the next outing checks for regressions.

1. **GCS AP didn't come up on first boot; agents wouldn't register until
   `muas-v2-setup` + `muas-v2-agent` were manually restarted.** This is
   the cold-boot ordering problem the restart doctrine already covers,
   but it shouldn't need hands. Pre-flight: confirm the AP is up
   (`iw dev`, clients can associate) BEFORE powering drones; if agents
   sit in activating after the AP is confirmed, the ordered restart
   (controller → agents) is expected, not a fault. Open item: add a
   boot-time health gate so this self-heals.

2. **IUAS thought it was above 3 m and descended into the ground (crash).**
   Root cause: `connect_flight_link` auto-captured home altitude from the
   FIRST position fix. An agent that reconnects to an already-settled FC
   reads a nonzero relative_alt in that fix, baking a false offset into
   every AGL. FIX: `MavlinkFlightBackend` now pins the link to
   `home_alt_m=0` and works purely in AGL — nothing is captured at
   startup, nothing can drift. Same bug corrupted the WUAS's reported
   AGL, which is why (3) happened too.

3. **Detection position completely wrong; thumbnail showed no racquet.**
   Two causes: (a) the geo-projection divides the image offset by the
   frame's tagged AGL, and the WUAS's AGL was wrong per (2) — fixed by
   the AGL fix. (b) A single high-confidence false positive on grass
   texture launched the IUAS. FIX: the dashboard now requires
   `--confirm-count` (default 2) independent detections within
   `target_separation_m` before promoting a candidate to a dispatched
   target. A real object is seen across many frames; texture noise is
   not. Watch the new `detect.candidate` events accumulate hits in the
   log.

4. **WUAS flew stop-and-go, pitching hard at each waypoint — bad for
   nadir capture.** Root cause: the raster issued one goto per capture
   point and waited for arrival at each, so the vehicle braked and
   re-accelerated 24+ times. FIX: the raster now commands one goto per
   LEG (the far endpoint) and captures on the fly when passing within
   range of each pending capture point — continuous constant-velocity
   passes, far better attitude stability. Capture coverage is preserved
   (unreached points are taken at the leg end).

Still open after these fixes: the cold-boot AP/registration self-heal,
and battery telemetry (now wired via `battery_pct()` but unverified on
the real FC — confirm it populates the tile).
