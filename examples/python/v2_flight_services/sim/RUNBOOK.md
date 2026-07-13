# miniMUAS v2 fleet sim — operator runbook

Stand up the full v2 fleet (4 nodes) on your workstation and "man the dashboard"
before physical deployment. You get: 4 telemetry markers moving, a raster search
→ detection → target localization, an audio interrogation on the mic airframe,
cooperative avoidance between drones, and latency metrics in the JSONL journals.

---

## Topology: why single-host / single-NFD (Option B)

The native NDNSF stack (matianxing1992 ndn-cxx fork, NDNSD, ndn-svs, OpenABE,
NAC-ABE, NFD) is **Linux-only**, so on macOS everything runs inside the Ubuntu
image built by `docker/Dockerfile`. Given that, the least-friction path to a
*manned dashboard* is one NFD with every role as a separate process on it:

- `run_ndnsf_stack.py` already proves the "N NDNSF participants + `ensure
  multicast strategy` on one local NFD" pattern; `sim/launch_fleet.py` just
  extends it to the whole fleet plus the dashboard.
- What the operator watches (the five behaviours) is identical whether the
  fabric is one NFD or four — per-node NFDs and inter-node UDP faces are
  transport realism the dashboard cannot show.
- The strategy layer that actually matters for correctness **is** mirrored from
  the deployment (`nix/nixos/common/minimuas/v2.nix`): multicast on
  `/muas/v2/group` and `/muas/v2/mission`, best-route on `/muas`. On a single
  NFD the inter-node UDP faces and `/muas` routes collapse to loopback.

**Rejected:**
- *Option A — N containers, per-node NFD, full-mesh UDP faces mirroring v2.nix.*
  Most faithful to the wire, but adds docker networking, per-node keychain
  isolation (`NDN_CLIENT_PIB`/`NDN_CLIENT_TPM`), face-create + route-register
  scripting, and controller reachability across containers — days of friction
  for fidelity the dashboard can't display. See "Faithful upgrade (Option A)"
  below for exactly what changes.
- *Option C — nixosTest VM cluster reusing `services.muasV2`.* Heaviest; needs
  the config repo's nix packaging. Overkill for a manned dashboard.

### The fleet (4 nodes)

| node    | process(es)                              | role                                   |
|---------|------------------------------------------|----------------------------------------|
| gcs     | `run_ndnsf_controller.py` + `run_gcs_provider.py` + `run_dashboard.py` | auth controller, detector, dashboard+mission brain |
| wuas-01 | `run_drone_agent.py --role wuas`         | camera, raster search                  |
| iuas-01 | `run_drone_agent.py --role iuas`         | camera, close inspection               |
| iuas-02 | `run_drone_agent.py --role iuas --audio synthetic --sensors audio` | USB-mic airframe, audio interrogation |

By **default all three drones fly on ArduPilot SITL** — one ArduCopter SITL
instance per drone, all at the sim's Memphis home, giving real GPS/attitude
(so the tilt-aware nadir projection is exercised) and a real GUIDED→arm→
NAV_TAKEOFF flight path. See "Fly the fleet on ArduPilot SITL" below. A
lightweight hand-rolled **kinematic bench** (`SimFlightBackend`, no MAVLink)
remains available behind `--kinematic` for when you don't want to run SITL.

---

## Prerequisites

- Docker Desktop running (the image is Linux).
- A local `NDN_Service_Framework` checkout (default `~/Documents/Dev/NDN_Service_Framework`).
  The image **COPYs this tree** at build time (`Dockerfile` layer 8), so local
  and even uncommitted NDNSF changes compile into the wrapper.
- A local `UAS-IPBRC` checkout (default `~/Documents/Dev/UAS-IPBRC`) — provides
  `relay.flight` (IUAS primitive execution) and `relay.flight.deconflict`
  (fleet avoidance). Without deconfliction the agents fly but avoidance is
  disabled (they log `agent.coord.disabled`).
- For the default SITL path: an `ArduPilot` checkout (default
  `~/Documents/Dev/ardupilot`) with the ArduCopter SITL binary built
  (`build/sitl/bin/arducopter`), and `pymavlink` on the host (`pip3 install
  pymavlink`) for the SITL warmup. Build the binary once with
  `cd ~/Documents/Dev/ardupilot && ./waf configure --board sitl && ./waf copter`.

Build the image once (slow — it compiles the whole NDN dependency chain; the
first build can take 20-60+ min):

```sh
cd examples/python/v2_flight_services/docker
./run_fleet_sim.sh build
# Apple Silicon: if OpenABE/relic fails, build amd64 under emulation:
#   DOCKER_PLATFORM=linux/amd64 ./run_fleet_sim.sh build
```

---

## Man the dashboard

Two steps: start the host SITL fleet, then the docker fleet.

```sh
cd examples/python/v2_flight_services

# 1) launch one ArduCopter SITL per drone at the Memphis home + warm them up
#    (boot + GPS lock, ~10 s each). Prints the port map.
./sim/start_sitl.sh start

# 2) bring up the docker fleet (agents connect to SITL SERIAL0 ports)
cd docker && ./run_fleet_sim.sh fleet
```

Then open **http://localhost:8080/**. Within a few seconds you should see the
telemetry markers (wuas-01, iuas-01, iuas-02) at the Memphis home, streaming
**real SITL** GPS/attitude. Ctrl-C in the `fleet` terminal tears the docker
fleet (and the container's NFD) down; `./sim/start_sitl.sh stop` stops the SITL
instances.

**SITL port map** (agent `--mavlink-endpoint` = SITL SERIAL0 over
`tcp:host.docker.internal:<port>`; SERIAL1 `<port>+2` is left free for an
independent MAVLink observer / MAVProxy):

| drone   | SITL instance | SERIAL0 (agent) | SERIAL1 (observer) |
|---------|---------------|-----------------|--------------------|
| wuas-01 | `-I0`         | 5760            | 5762               |
| iuas-01 | `-I1`         | 5770            | 5772               |
| iuas-02 | `-I2`         | 5780            | 5782               |

**Lightweight kinematic bench instead of SITL** (no ArduPilot needed):

```sh
cd examples/python/v2_flight_services/docker
./run_fleet_sim.sh fleet --kinematic
```

Trigger the behaviours from the UI, or from another terminal with the WS driver
(the UI and the driver speak the same `/ws` protocol — `run_dashboard.py` has no
POST endpoints):

```sh
# from the host, driving the published :8080 — needs aiohttp on the host,
# OR run it inside the container:  ./run_fleet_sim.sh shell  then python3 sim/ws_driver.py ...

# 1) raster search on wuas-01 -> detection -> auto-dispatched investigation
python3 ../sim/ws_driver.py mission --listen 20 --show search_status,event,sensor_data

# 2) audio interrogation routed to the mic airframe iuas-02
python3 ../sim/ws_driver.py audio --vehicle iuas-02 --listen 8

# 3) force cooperative avoidance: get two drones airborne and converging.
#    Launch the fleet with a larger horizontal-separation envelope so the
#    conflict trips readily, then take off / run a mission so they close in:
#      ./run_fleet_sim.sh fleet --coord-hsep-m 20
python3 ../sim/ws_driver.py takeoff --all --listen 6
python3 ../sim/ws_driver.py mission --listen 20 --show event
#    Watch avoid_bias_m go non-zero in telemetry and agent.coord.* in the
#    iuas journals as the two IUAS converge on the same investigate target.
```

In the browser instead: draw a search box and **Start mission** (search →
detect → localize + auto-dispatch), use the per-vehicle **sensor/audio** control
targeting iuas-02, and **takeoff** two drones to provoke avoidance.

### Where the journals land

Everything is bind-mounted, so journals appear on the host under
`results/v2_sim/` (fleet) or `results/v2_sim_smoke/` (smoke):

| file                                   | written by            | carries                                              |
|----------------------------------------|-----------------------|------------------------------------------------------|
| `results/v2_sim/replays/dash-*.jsonl`  | dashboard recorder    | telemetry, search_status, sensor_data (audio artifact), mission events |
| `results/v2_sim/log/<vid>-agent.jsonl` | each drone agent      | `agent.coord.*` (avoidance), capture events          |
| `results/v2_sim/log/gcs-provider.jsonl`| GCS detector          | `gcs.detection.*` (localization projection)          |
| `results/v2_sim/log/wuas-01-user.jsonl`| `run_wuas_user.py`    | `metric.service_rtt` / `metric.latency`              |
| `results/v2_sim/log/console/*.log`     | supervisor            | raw stdout+stderr per role (start here when debugging)|

Quick greps:

```sh
grep -h '"type":"telemetry"'      results/v2_sim/replays/dash-*.jsonl | head
grep -h '"kind":"audio/wav"'      results/v2_sim/replays/dash-*.jsonl
grep -h '"event":"agent.coord'    results/v2_sim/log/*-agent.jsonl
grep -h '"event":"metric.'        results/v2_sim/log/*.jsonl
```

(Dashboard-recorder lines nest the payload under `"m"`; agent/provider/user
lines are flat with `"event"`. `sim/smoke.py` handles both shapes.)

---

## Headless smoke (CI / preflight)

Brings the fleet up, drives the scripted mission + audio + a metrics run, polls
the journals until all five signals appear, prints a PASS/FAIL table, and tears
down. Non-zero exit on any failure.

The smoke defaults to the **kinematic bench** so it is self-contained (no host
SITL needed) — good for CI:

```sh
cd examples/python/v2_flight_services/docker
./run_fleet_sim.sh smoke
```

To run it against the **SITL fleet** (start the SITL instances first):

```sh
cd examples/python/v2_flight_services
./sim/start_sitl.sh start
cd docker && ./run_fleet_sim.sh smoke --sitl
```

Asserts: telemetry (≥2 vehicles), search→detection→localization, audio artifact
on iuas-02, cooperative avoidance, and `metric.*` latency events.

---

## ArduPilot SITL details

- **One instance per drone** at the same Memphis home so telemetry markers,
  detection/localization, and cooperative avoidance all share one frame.
  `sim/start_sitl.sh` launches them (`sim_vehicle.py -v ArduCopter -I<N>
  --no-mavproxy -A "--home 35.1208,-89.9347,50,0"`), each in its own working dir.
- **Warmup is required.** A cold SITL only starts its clock when a client first
  connects to SERIAL0, and GPS/EKF lock takes ~30 s — longer than the agent's
  MAVLink backend waits (`agent.mavlink.connect_failed: no GLOBAL_POSITION_INT
  telemetry within 30s`). `start_sitl.sh` therefore boots each instance and
  waits for GPS lock, then disconnects; the fix persists across reconnect, so
  the fleet's agents get position immediately. Keep `pymavlink` on the host.
- **Real attitude / nadir tilt.** Because the vehicles now fly a real
  GUIDED/orbit path, capture frames carry real roll/pitch, exercising the
  tilt-aware nadir geo-projection in the GCS detector (a no-op under the level
  kinematic bench).
- **Observe / debug a vehicle** without disturbing the agent: attach MAVProxy or
  pymavlink to its SERIAL1 port, e.g. `mavproxy.py --master tcp:127.0.0.1:5772`
  for iuas-01. (SERIAL0 is single-client and owned by the agent.)
- **Speedup.** `SPEEDUP=1` (wall clock) keeps SITL telemetry timestamps aligned
  with the NDN data plane; override via `SPEEDUP=… ./sim/start_sitl.sh start`.
- The container reaches host SITL via `host.docker.internal` (auto on Docker
  Desktop; the wrapper also passes `--add-host=host.docker.internal:host-gateway`).
- **Host resource note:** three ArduCopter SITL instances + the 6-process docker
  fleet run comfortably on a modern multi-core laptop. To run fewer, use
  `COUNT=<n> ./sim/start_sitl.sh start` and `--kinematic` for the rest, or drop
  to the kinematic bench entirely.

---

## Notable non-obvious wiring

- **Controller policy.** The stock `config/v2_minimuas.policies` only authorizes
  `iuas-01`/`wuas-01` and does not let the GCS task any IUAS `sensor/capture`,
  so it cannot drive iuas-02 or the audio path. The sim ships a **superset**,
  `sim/fleet.policies`, that adds iuas-02 as a full provider and grants the GCS
  user the investigate/sensor rights for both IUAS. `launch_fleet.py` passes it
  via `run_ndnsf_controller.py --policy` and adds `-b /muas/v2/iuas-02` to the
  bootstrap identities. The deployment policy is untouched.
- **iuas-02 audio range guard.** Launched with `--audio-range-m 0` so a tasked
  audio capture always records (the mic is otherwise only hot within 30 m of a
  target point).
- **Detector.** Defaults to `stub` (deterministic offset-based fake) so the
  search→detect→localize chain completes with synthetic frames and no model.
  Use `--detector yolo:<model.onnx>?...` for real inference (needs the model and
  imagery with the target class).
- **Latency metrics.** Only `run_wuas_user.py` / `run_mock_mission.py` emit
  `metric.*`; the smoke runs the WUAS user once to produce them. The NDNSF
  wrapper's new `ServiceResponse.timing` fields feed the four-point `metric.
  latency` breakdown when present — the harness matches `metric.*` generically
  and does **not** depend on those fields being present or absent.

---

## Faithful upgrade (Option A) — if you later want per-node NFDs + real faces

Mirror `mini-muas-v2:nix/nixos/common/minimuas/v2.nix` + `.../nfd/`:

- One container/NFD per node (IPs .11 iuas-01, .12 wuas-01, .13 gcs, .14 iuas-02
  in the deployment); isolate each node's keychain with `NDN_CLIENT_PIB` /
  `NDN_CLIENT_TPM`.
- Full mesh **unicast UDP faces**: on each node, for every peer,
  `nfdc face create udp://<peer> reliability on mtu 1452`.
- **Strategies** (same as here): `nfdc strategy set prefix /muas/v2/group
  strategy /localhost/nfd/strategy/multicast` and likewise `/muas/v2/mission`.
- **Routes**: `nfdc route add prefix /muas nexthop <faceid> cost 100` toward
  every peer face (equal-cost), NFD UDP on **6363**.
- Point every node's controller/roles at the gcs controller `/muas/v2/controller`.

`launch_fleet.py`'s role/flag wiring is unchanged per node; only NFD setup and
the docker topology change. The strategy calls it already makes are exactly the
two the deployment overrides.

---

## Troubleshooting

- **A role exited early.** `launch_fleet` logs `role.exited_early`; read
  `results/v2_sim/log/console/<role>.log`.
- **Dashboard blank / no markers.** Check `controller` and the agents came up
  (console logs); telemetry is data-plane and needs the multicast strategy on
  `/muas/v2/group` (the supervisor sets it — look for `nfd.strategy ... ok=True`).
- **`agent.coord.disabled`.** UAS-IPBRC `relay.flight.deconflict` wasn't
  importable; check `UAS_IPBRC_ROOT` is mounted. Avoidance won't engage.
- **No detection.** With `--detector yolo` the synthetic frames won't contain
  the target class; use `stub` or feed real imagery via `--camera file:<path>`.
- **Map tiles missing.** The dashboard fetches satellite tiles upstream; without
  internet from the container the basemap is blank but markers/telemetry still
  work.
- **`run_wuas_user` investigate `timeout` / rc=1 in the smoke.** Expected in
  pure kinematic bench: its investigate uses a 25 m approach + full orbit that
  can exceed the user-side 90 s timeout, so `require_success` raises. The
  latency metric is still emitted around the request, and the successful detect
  RTT satisfies the smoke's `metric.*` check — the smoke stays green. Raise
  `run_wuas_user.py --investigate-timeout-ms` if you want the orbit to complete
  cleanly. (The dashboard's own mission uses a lower 8 m orbit and completes.)
- **Apple Silicon build failure in OpenABE/relic.** Rebuild amd64 under
  emulation: `DOCKER_PLATFORM=linux/amd64 ./run_fleet_sim.sh build`.
