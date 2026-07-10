# Field QoL for v3 — consolidated draft (M7)

Draws on `docs/v2-field-day.md` (the run order, the 2026-06-15 debrief, and
the still-open items), VISION §8, ARCHITECTURE §Field QoL, and the uas-cyd
design (`~/Documents/Dev/uas-cyd/docs/DESIGN.md`). Organizing principle: the
v2 field day worked because of *doctrine* (checklists, ladders, restart
rituals). v3's QoL work converts doctrine into *equipment* — things that
check themselves — while keeping the doctrine as the fallback.

## 1. Pre-flight checklist automation

The v2 pre-flight gate is a human reading a dashboard against a memorized
list. Automate the reads, keep the human on the decisions:

- **A machine-checkable gate manifest.** Every v2 gate item is already a
  predicate on data the stack has: `source == mavlink` on every tile, link
  age 0–1 s, clock Δ ≈ 0, AGL ≈ 0, armed == false, sane lat/lon, event log
  quiet. Encode them as a `preflight` profile the dashboard (and the CYD's
  alerts page, which already implements a subset: armed+sim-source, stale,
  battery-alarm) evaluates continuously and renders as ONE green/red gate
  chip with per-item drill-down. The operator's job becomes "why is it
  red", not "read 9 numbers on 3 tiles".
- **`--dry-run` as a boot stage, not a command.** v2 required remembering
  `muas-v2-agent --dry-run --role ...` before leaving campus. v3: the agent
  service runs its import/launch self-check on every start and publishes
  the result as a fact the gate manifest consumes ("build sanity: pass,
  build rev X"). The pre-departure check becomes "all gate chips green on
  the bench with FIELD config".
- **The 2026-06-15 lessons as permanent gate items:** AGL-vs-known-height
  verification (rung 2 of the ladder) gets a dashboard assistant — command
  5 m, vehicle reports, the gate compares against the operator-confirmed
  eyeball height and records the delta in the journal. Battery telemetry
  populated (not blank/0) is a gate item, not a "known gap".
- **Boot-order self-heal (the open item from field #1).** The cold-boot
  AP/registration failure has a known manual fix (ordered restart). v3:
  a boot-time health gate on the GCS — services watch their own
  registration state and re-kick in dependency order until green, with the
  attempt count surfaced. Hands stay in pockets.

## 2. One-command stack bring-up

- v2's power-up order (GCS first, GPS lock, then drones, then laptop) is
  physics and stays. What changes: each host converges on its own —
  `muas up` per node is just "apply power"; systemd target + health gate
  do the rest, and the dashboard/CYD show per-node convergence state
  ("waiting: GPS lock", "waiting: controller", "up 143s").
- Per-repo flakes + config-repo composition (REPO-TOPOLOGY) already give
  reproducible hosts; add a `field` vs `bench` *profile* switch that is a
  single config-repo commit, and make every node display which profile it
  booted (the v2 "sim = the flip didn't deploy" trap, killed at the root).
  The CYD grid's red SIM sash is the ambient version of this check.
- **Rebuilds happen on campus** stays doctrine: the field uplink is not a
  deploy path. QoL addition: a pre-departure `field-freeze` script that
  verifies all repos pushed, flake inputs pinned to pushed revs, all three
  nodes rebuilt + booted into the FIELD profile, and prints one go/no-go
  line. (This is the memory-file deployment flow, mechanized.)

## 3. Offline tile prefetch

- v2 flow (pan/zoom by hand at z15–19, then verify by unplugging the
  network) worked but was manual and unverifiable until you tried it. v3:
  `muas tiles fetch --site <name>` with named site presets (bbox + zooms
  from a sites.toml), and — the actual QoL — a **coverage report**: which
  zoom rings are complete for the site polygon, rendered as an overlay so
  the "tiles gray at z19" surprise happens on campus.
- Verification becomes a gate item: dashboard queries its own tile cache
  for the site bbox instead of "disable wifi and squint".
- Tile serving stays GCS-local; drones never need imagery.

## 4. CYD roles (the new equipment)

See `uas-cyd/docs/DESIGN.md` for full detail. In field terms, the CYD adds:

- **Ambient fleet glance** — battery/link/AGL-alarm grading in the safety
  pilot's pocket, using uas-console's exact honest-numbers thresholds, so
  palm and dashboard never disagree. Kills the v2 "monitor battery on the
  RC telemetry" workaround once battery telemetry is a gate item.
- **Alerts triage** — the four field dangers (AGL alarm, battery alarm,
  stale link, armed-on-sim) as a one-page list, no laptop needed.
- **E-stop fob** — a fleet-wide SAFING trigger (uas-rc frame, e-stop flag
  only, all channels ignore) in a second person's hand. v2's abort ladder
  was: RC override (safety pilot) → dashboard buttons (operator at the
  laptop). The fob inserts a rung between them that any crew member can
  hold. Drill B5 (props-off e-stop round trip) joins the preflight.
- **C2 enrollment surface** — QR + proximity tap for joining a replacement
  node/CYD to the fleet trust domain in the field, replacing "edit
  policies files over SSH". Expiring keys = losing one costs nothing.
- Crew kit: two CYDs (one is the spare), one optional C5 bridge for the
  named-data lane; all charged off the same power bank pool.

## 5. Spares & cabling (the boring kit list, learned the hard way)

- USB GPS for the GCS is a hard gate (node 03 won't start NDN services
  without it) — carry TWO, they are $12 failure points.
- One spare rtl8812eu per band-pair (below), one spare CYD, one spare FC
  telemetry cable per airframe, props ×2 sets per airframe, a powered USB
  hub for the GCS (GPS + dongles + FC links exceed laptop ports), and a
  labeled bag per node — v2's "which cable is node 02's" time tax is real.
- Batteries: flight packs charged on campus; the field charges nothing.
  Power banks for CYDs/phone-NAT are separate from flight packs.
- A paper copy of the restart rituals (ordered restart, identity wipe,
  MTU check). When the network is the patient, the wiki is unreachable.

## 6. Antenna / radio handling — the rtl8812eu pair

Two rtl8812eu per node (named-data radio, 5 GHz injection lane):

- **Label the pair per node and per role** (inject vs monitor). The
  driver stack binds by interface; a swapped dongle "works" with silently
  wrong antenna placement/gain.
- Antennas: stock dipoles are fine at field ranges, but mount them OFF the
  airframe carbon and away from the FC GPS puck; the 2026 bench showed the
  8812eu TX desensing GPS when co-located. Velcro + 10 cm separation is
  the whole fix.
- Channel doctrine: the injection lane lives on a fixed 5 GHz channel
  (UNII-1, ch 36 family — the ndn-espnow work standardized here); the GCS
  AP stays on 2.4 GHz so the lanes never share a band. Post the channel
  plan in the config repo, not in heads.
- Bring a **sniffer tool for each band**: the C5 promiscuous sniffer
  (ndn-espnow `sniffer.rs`) for the ESP-NOW/5 GHz lane, `iw`/tcpdump on a
  spare dongle for the injection lane. Doctrine from the M2 debug: when a
  lane is dark, FIRST split RF/channel faults from receive-path faults
  with a promiscuous listener, THEN debug software.
- Thermals: the 8812eu dongles run hot injecting at duty; keep them out of
  direct sun (they throttle and the link "gets flaky at noon").

## 7. What changes vs the v2 field doctrine

| v2 doctrine | v3 |
|---|---|
| Human reads 9 gate items off tiles | One continuously-evaluated gate chip; human judges the red ones |
| `--dry-run` by hand before departure | Self-check on every service start, result is a gate fact |
| Ordered restart ritual on cold boot | Boot health gate self-heals; ritual kept as fallback |
| Pan-and-zoom tile warming + unplug test | Named-site prefetch + coverage report as a gate item |
| Battery watched on RC telemetry | Battery telemetry is a gate item; CYD carries it in-pocket |
| Abort ladder: RC pilot → laptop buttons | + CYD e-stop fob rung in between (any crew member) |
| Trust/config edits over SSH | C2 QR/tap enrollment, expiring keys |
| Single-band improvised radio placement | Written channel plan, labeled dongle pairs, band-split lanes |

Unchanged and non-negotiable: the flight-test ladder (manual hop → AGL
verify → single-vehicle raster → full mission), RC mode-switch as the
primary abort, rebuilds on campus only, and the 3.5 m goto floor.
