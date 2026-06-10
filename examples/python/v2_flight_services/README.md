# miniMUAS v2 Flight Service Prototype

This prototype exercises the v2 service boundary without requiring NFD, NDNSF,
ArduPilot, MAVLink, or cameras.

It models the intended NDNSF shape:

- WUAS publishes a named image object.
- WUAS requests GCS object detection by passing the image data name.
- GCS fetches the object and returns a target estimate.
- WUAS dispatches an investigate-point task to IUAS.
- IUAS returns a simulated result with a named sensor artifact.

Run the in-process mock:

```sh
python3 examples/python/v2_flight_services/run_mock_mission.py
```

The mock bus is now paired with real NDNSF role scripts:

- `ServiceProvider` for GCS detection and IUAS flight task handlers
- `ServiceUser` for WUAS service calls
- `SegmentedObjectProducer` or collaboration `publish_large` / `fetch_large`
  for image and sensor artifacts

`ndnsf_binding_sketch.py` shows the intended handler boundary for the real
NDNSF Python API.

Preview the real NDNSF commands:

```sh
python3 examples/python/v2_flight_services/run_ndnsf_stack.py
```

Check runtime prerequisites:

```sh
python3 examples/python/v2_flight_services/preflight_ndnsf.py
```

Run the local real-NDNSF request/response stack:

```sh
python3 examples/python/v2_flight_services/run_ndnsf_stack.py --run --start-local-nfd
```

This requires the NDNSF Python wrapper to be built or installed. By default the
scripts look for it at `~/Documents/Dev/NDN_Service_Framework/pythonWrapper`.
Override that with `--ndnsf-root`.

If importing `ndnsf` fails because `_ndnsf` is missing, build the wrapper in the
NDNSF repo first:

```sh
cd ~/Documents/Dev/NDN_Service_Framework/pythonWrapper
python3 setup.py build_ext --inplace
```

Run roles manually when debugging service authorization:

```sh
python3 examples/python/v2_flight_services/run_ndnsf_controller.py
python3 examples/python/v2_flight_services/run_gcs_provider.py
python3 examples/python/v2_flight_services/run_iuas_provider.py
python3 examples/python/v2_flight_services/run_wuas_user.py
```

## Real primitive execution (relay.flight)

The IUAS provider no longer fabricates task results when a UAS-IPBRC checkout
is available. `investigate_plan.py` compiles each `InvestigatePointRequest`
into `relay.flight` primitives — climb, approach to a standoff point, orbit
the target, capture — and executes the plan to a terminal status on simulated
time before responding. The orbit step is chosen by `plan_orbit`'s capability
ladder, so the execution mode reported in the result notes (`circle-mode`,
`guided-yaw-path`, `guided-position-only`) reflects what actually ran, and
requests the vehicle cannot satisfy are rejected at the ack stage with the
reason. Constraints map onto the primitive runner: `min_clearance_m` becomes
an altitude-envelope constraint, `max_speed_mps` flows into the motion
targets, and `deadline_gps_ns` becomes a runner deadline that cancels the
task.

The flight library is found via `UAS_IPBRC_ROOT` (default
`~/Documents/Dev/UAS-IPBRC`) or `--uas-ipbrc-root`. Without it, or with
`--no-execute-plan`, the provider falls back to the fabricated v0 response.
`--no-native-orbit` drops the vehicle's advertised circle-mode capability to
exercise the guided waypoint fallback.

Check the execution slice offline (no NFD, no NDNSF, no MAVLink):

```sh
python3 examples/python/v2_flight_services/investigate_plan.py
python3 examples/python/v2_flight_services/investigate_plan.py --no-native-orbit
```

Execution runs against an in-process `SimFlightLink`; flying the identical
plan on SITL or hardware means swapping in a MAVLink-backed link with the
same method surface (`goto`, `orbit`, `takeoff`, `land`, `rtl`).

## Running the real stack on macOS

The NDNSF native stack is Linux-only. `docker/` wraps it in an Ubuntu 22.04
container with miniMUAS and UAS-IPBRC live-mounted from the host:

```sh
cd examples/python/v2_flight_services/docker
./run_v2_stack_container.sh stack
```

See `docker/README.md` for build details and troubleshooting.

The data plane is implemented: WUAS publishes the camera frame as signed
segmented Data under its mission name, GCS fetches and verifies it before
detecting, the IUAS publishes its sensor artifacts the same way, and WUAS
fetches them back after the mission. Payloads are deterministic synthetic
frames until a real camera is integrated; see `dataplane.py`.

The service contract is documented in `docs/v2_flight_services.md`.
