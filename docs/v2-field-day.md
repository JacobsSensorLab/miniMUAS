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

## NDN stack comparison (C++ NFD vs Rust ndn-fwd) — field day 2026-10-13

The objective is quantitative metrics for both stacks. Fly through the
dashboard as usual; the comparison comes from what the fleet records
while you do.

- **Which stack is live:** header tag `stack NFD` / `stack ndn-fwd`
  (yellow while settling, red `MIXED` if nodes disagree) and the
  "Network stack" panel (cell per node).
- **Switching:** only between sorties, every vehicle landed and
  disarmed. Use the `→ nfd wifi` / `→ ndn-fwd wifi` buttons in the
  Network stack panel. The dashboard refuses unless every vehicle has
  fresh telemetry that says disarmed and not busy. Vehicles switch first;
  the GCS switches last, and the page drops for ~25–30 s while the
  dashboard restarts, then reconnects by itself. A whole switch took
  37–57 s on the ground (2026-10-12). If a vehicle does not accept, the
  GCS is not switched and the panel says which vehicle; fix it and switch
  again.
- **Settling:** after a switch, wait until the panel's 150 s countdown
  ends before flying. Data recorded while settling is reported
  separately, not as either stack.
- **Sorties are not identical, so mix them up:** fly each stack more
  than once, with similar video load and distances, so the report can
  compare like with like.
- **Recorded automatically on every node (nothing to start):** radio
  link per peer, forwarder CPU and memory, UDP drops and queues, face
  counters, packet headers (`/var/lib/minimuas/recorder/`); on the GCS
  also GPS position and a 1 Hz NDN ping to every vehicle. The dashboard
  recording (`/var/lib/minimuas/replays/`) holds telemetry with
  per-sample latency, video stats, command latency and the stack of
  every node.
- **End of day:** before pulling batteries, use the dashboard's
  companion shutdown on each vehicle (a clean poweroff keeps the last
  recordings; enabled in the access policy 2026-10-12, not yet exercised
  on the fleet). Shut the GCS down last.
- **Back on campus:** `ndn-fleet flight collect since=<UTC start of the
  field day>` (read-only), then
  `python3 analysis/flight_report.py results/flights/<id>` for the
  side-by-side report (report.md).

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

5. **The real racket was never reported (2026-07-15 frames).** The GCS
   saved 167 analyzed frames; the racket is visible in 21. Stock COCO
   yolov8n labels it from overhead as "kite" (conf 0.38-0.82), never
   "tennis racket", so the `tennis racket` class filter dropped every
   sighting. FIX: the class filter takes aliases — `tennis racket|kite`
   (the module default) reports "kite" as "tennis racket" and keeps the
   model's class as `raw_label` in the journal and frame sidecars. At
   conf 0.3 that hits 13/21 racket frames and none of the other 146.
   YOLOE-26 (open-vocabulary) was tried and scored ≤0.12 on the same
   frames.

Still open after these fixes: the cold-boot AP/registration self-heal,
and battery telemetry (now wired via `battery_pct()` but unverified on
the real FC — confirm it populates the tile).

## Field day 2026-10-13: detection, "Investigate anyway", sim target

1. **0-1 of 2 detections, results late.** The GCS detector takes ~2.2 s a
   frame (yolov8n on the busy GCS CPU); the WUAS captured one every
   ~1.4 s and the dashboard sent each at once. The provider fast-rejected
   61 frames "detector busy" and the dashboard filed each as a miss: in
   the first mission 17 of 31, among them the frames 2.2 m and 4.0 m from
   the racket, so it was seen once (kite 0.35, then 0.54 in the third
   mission) and never confirmed. FIX: the dashboard queues frames and keeps
   at most `--detect-inflight` (2, = the provider's `--detect-admit`)
   outstanding; a busy answer re-sends the frame. Leftover candidates
   settle only after the last frame is answered (and one status poll after
   the raster's answer collected the final captures); frames from the
   previous mission's status are ignored. The provider's slot frees when
   the work ends, and a handler waits `admit x (fetch + infer)`, so a
   15 s fetch can no longer time out the frames queued behind it.
   The GCS also saves each frame raw (`*.raw.jpeg`) beside the annotated
   copy so hits can be re-scored.
2. **On "nfd wifi" every detection frame fetch timed out.** Mission frames
   were named `/muas/v2/mission/<mid>/<vid>/...`, which only the cost-100
   `/muas` peer routes matched; best-route sent them to iuas-02. FIX: mission
   data now lives under the producer, `/muas/v2/<vid>/mission/<mid>/...`.
3. **"Investigate anyway" failed on both IUAS within 2 s.** Both refused
   takeoff on the ground-AGL check (disarmed baro drift 1.61 m and
   2.27 m > 1.5 m); the card only said "camera failed / audio failed".
   FIX: `mavlink_flight.ensure_airborne` checks ground AGL after arming
   (ArduCopter re-zeroes home altitude at arm); the agents now answer
   "could not get airborne: <stage>: <reason>" and the target card shows
   each failed job's reason.
4. **The placed sim target did nothing.** Only the stub detector read the
   placed anomalies; the field runs yolo. FIX: any detector reports a
   placed visual target that projects inside a captured frame as a
   `simulated` detection (object `SIM-<signature>`), flagged SIMULATED on
   the cards, map, events, investigate request and agent journal, and never
   merged with real hits. Place one under the raster to exercise confirm ->
   dispatch -> investigate without a real object.

## Field day 2026-10-13: hover after RTL, raster speed, RTL speeds

1. **New mission after RTL/Land: took off, then hovered.** wuas-01 16:37:31
   and 16:47:05 CDT: armed, then `ensure_airborne` returned False exactly
   3 s later (agl 0.7-0.8 m, note "airborne failed") while the vehicle kept
   climbing to 6/8 m and hovered in GUIDED with nobody commanding it. The
   3 s is the NAV_TAKEOFF COMMAND_ACK timeout: the telemetry, PeerGuard and
   watchpoint threads drain the same pymavlink connection and one of them
   ate the ACK the takeoff was waiting for (reproduced on SITL:
   `flight.ack_recovered` for takeoff and DO_CHANGE_SPEED). The RTL->Loiter
   cycle only re-rolled that race. FIX: `LoggingFlightLink` re-judges a
   timed-out command from pymavlink's per-type message cache, and
   `ensure_airborne` decides arm/takeoff from telemetry (armed flag, climb
   check) instead of ACKs; every failure is journaled as
   `flight.takeoff_refused {stage, reason}` and the raster answers
   "could not reach search altitude: <stage>: <reason>".
2. **Raster flew 7-8 m/s whatever speed was set.** Requested 0.5 m/s, flown
   p90 7.5 / max 8.3 m/s (2 m/s: max 8.5) on every mission started from the
   ground; the one started after a sensor-tasking goto (16:52:21) flew
   0.45 m/s. ArduCopter keeps DO_CHANGE_SPEED only within one GUIDED
   position episode: refused outside GUIDED, reset to WPNAV_SPEED by GUIDED
   entry and by the first position target after NAV_TAKEOFF
   (`ModeGuided::pos_control_start`). The raster sent it before takeoff.
   FIX: the flight backend holds the desired speed and re-sends it right
   after the first position target of every episode (`CruiseSpeed`);
   `SpeedWatch` checks |v| from telemetry each tick, re-sends on a sustained
   overspeed and puts p50/p90/max into `agent.search.finished`. Telemetry
   now carries `groundspeed_m_s` (it was always 0) and `cruise_speed_m_s`.
3. **WUAS crawled home, IUAS fast.** The smart RTL never set a speed:
   wuas-01 returned at its last raster's 0.45 m/s, both IUAS at WPNAV_SPEED
   (peaks 4.1/4.8 m/s). 26 s in, wuas-01 left GUIDED (RTL, then LOITER 1 s
   later: a pilot/GCS mode change, not the agent; battery 12%) and came
   down in LOITER 9.3 m short of home; the agent yielded (`rtl.takeover`).
   FIX: every vehicle returns at `--rtl-speed-m-s` (default 3); task speeds
   are released when their task ends so none leaks into the next one.
   Sensor-tasking flights (fly to a point, and a raster override's detour)
   fly `--task-speed-m-s` (default 3, = RTL and the investigate default);
   the raster takes its own speed back after a detour.
4. **wuas-01 would not arm, 15:38-15:53 (six takeoffs).** Our gates passed
   (GUIDED confirmed, AGL 0.1-0.2 m, no busy refusal); the autopilot itself
   refused within ~1 s, and its reason (a "PreArm:"/"Arm:" STATUSTEXT or a
   failsafe state) was never recorded: the relay drops STATUSTEXT and the
   dashboard printed every takeoff as "ok: true, No error" because it never
   read the agent's answer. The FC was rebooted at 15:43 and Mission Planner
   was on it 15:49-15:51 and 15:58-16:02 (mavrouter journal); it armed at
   16:07. FIX: the agent keeps the FC's STATUSTEXTs (`flight.fc_text` in
   the journal) and appends the latest arming verdict to the refusal
   ("takeoff failed: arm: autopilot did not arm — FC: PreArm: ..."); a
   failsafe state is named; the dashboard's `command.result` now carries
   the agent's status and message.

Flight-test before relying on it: one raster from the ground at 1 m/s
(journal `agent.flight.speed_applied ok=true`, finished event p90 ≈ 1),
one RTL ALL after it (all three `rtl.landing` at ≈3 m/s), and a mission
started right after an RTL landing (no `flight.takeoff_refused`).
