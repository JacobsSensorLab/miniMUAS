# miniMUAS v2 NDNSF Flight Service Contract

This document defines the first v2 architecture slice for replacing the
hard-coded miniMUAS v1 routine with a service-based, data-centric flight stack.
The goal is to keep flight behavior reusable while letting NDNSF handle
identity, authorization, service discovery, data names, and request/response
delivery.

## Design Boundary

The flight primitive library owns local motion intent and execution:

- takeoff, goto, circle, inspect, raster, capture, return-to-launch
- vehicle capability profiles and fallbacks
- command outcomes, task progress, and telemetry snapshots
- autopilot-specific adapters such as ArduPilot Circle mode

NDNSF owns the distributed service layer:

- named service discovery
- authenticated service requests
- provider selection and ACK policy
- segmented object publication and fetch
- collaboration workflows
- mission data naming and access policy

The service API should expose task intent instead of MAVLink commands. For
example, WUAS should ask IUAS to investigate a target with constraints, not send
raw orbit or reposition commands. The IUAS provider decides whether to execute
that request with Circle mode, guided repositioning, yaw-aware paths, or a
fallback primitive sequence.

## Roles

### GCS

The GCS is the mission authority and offboard compute node.

Responsibilities:

- authorize and start the WUAS mission
- provide object detection or analysis services
- consume mission artifacts and telemetry products
- approve high-level mission transitions when required by policy

### WUAS

The WUAS is the mission orchestrator and wide-area sensor platform.

Responsibilities:

- execute the wide-area mission plan
- publish camera frames or other sensor objects under mission-scoped names
- request offboard detection from GCS
- translate detections into IUAS investigation requests
- relay or aggregate IUAS sensor products for GCS
- approve IUAS return-to-launch when the investigation is complete

### IUAS

The IUAS is the close inspection vehicle.

Responsibilities:

- provide flight task services such as investigate-point and return-to-launch
- run local flight primitives against the vehicle/autopilot
- publish close-range sensor data as named objects
- report task status and terminal outcomes

## Service Names

Service names are intentionally task-oriented and versioned.

```text
/muas/v2/<vehicle>/flight/takeoff
/muas/v2/<vehicle>/flight/execute
/muas/v2/<vehicle>/flight/investigate
/muas/v2/<vehicle>/flight/circle
/muas/v2/<vehicle>/flight/rtl
/muas/v2/<vehicle>/telemetry/state
/muas/v2/<vehicle>/sensor/capture
/muas/v2/gcs/perception/detect-object
```

Large sensor products should be passed by data name instead of embedded in
service payloads.

```text
/muas/v2/mission/<mission-id>/<vehicle>/camera/<camera-id>/frame/<gps-time-ns>/<seq>
/muas/v2/mission/<mission-id>/<vehicle>/sensor/<sensor-id>/<kind>/<gps-time-ns>/<seq>
/muas/v2/mission/<mission-id>/evidence/<object-id>/<gps-time-ns>
```

## Core Payloads

Payloads can be encoded as protobuf, CBOR, or JSON. The prototype uses JSON so
the contract can be exercised without code generation.

### DetectionRequest

```json
{
  "mission_id": "mission-001",
  "frame": {
    "data_name": "/muas/v2/mission/mission-001/wuas-01/camera/front/frame/1710000000000000000/1",
    "gps_time_ns": 1710000000000000000,
    "seq": 1,
    "camera_id": "front",
    "pose": {
      "position": {"lat_deg": 35.1208, "lon_deg": -89.9347, "alt_m": 40.0},
      "yaw_deg": 90.0
    },
    "content_type": "image/jpeg"
  },
  "object_query": "test-object"
}
```

### DetectionResponse

```json
{
  "mission_id": "mission-001",
  "object_id": "target-001",
  "confidence": 0.91,
  "estimate": {"lat_deg": 35.1209, "lon_deg": -89.9346, "alt_m": 0.0},
  "evidence_ref": "/muas/v2/mission/mission-001/evidence/target-001/1710000000000000000"
}
```

### InvestigatePointRequest

```json
{
  "mission_id": "mission-001",
  "source_detection_id": "target-001",
  "target": {"lat_deg": 35.1209, "lon_deg": -89.9346, "alt_m": 0.0},
  "approach_alt_m": 25.0,
  "standoff_m": 8.0,
  "circle_radius_m": 6.0,
  "circle_count": 1.5,
  "facing": "target",
  "sensor_plan": ["capture-still", "publish-frame"],
  "constraints": {
    "max_speed_mps": 4.0,
    "min_clearance_m": 3.0,
    "deadline_gps_ns": null,
    "avoidance_mode": "advisory"
  }
}
```

### FlightTaskResult

```json
{
  "task_id": "iuas-01-investigate-target-001",
  "status": "completed",
  "started_at_gps_ns": 1710000001000000000,
  "completed_at_gps_ns": 1710000010000000000,
  "artifacts": [
    {
      "data_name": "/muas/v2/mission/mission-001/iuas-01/sensor/front/frame/1710000009000000000/1",
      "kind": "image/jpeg",
      "gps_time_ns": 1710000009000000000,
      "pose": {
        "position": {"lat_deg": 35.1209, "lon_deg": -89.9346, "alt_m": 25.0},
        "yaw_deg": 180.0
      },
      "metadata": {"target_id": "target-001"}
    }
  ],
  "notes": "circle-mode"
}
```

## v2 Mission Flow

1. GCS sends WUAS a high-level mission execution request.
2. WUAS executes the wide-area primitive plan.
3. WUAS publishes a camera frame as a named object.
4. WUAS requests `/muas/v2/gcs/perception/detect-object`, passing the frame
   data name and camera pose.
5. GCS fetches the frame object, performs detection, and returns an estimated
   target position plus evidence data name.
6. WUAS builds an `InvestigatePointRequest` and sends it to IUAS.
7. IUAS maps the request to local flight primitives.
8. IUAS publishes close-range sensor artifacts and returns a task result.
9. WUAS relays artifact names to GCS and approves IUAS return-to-launch.

## Timing And Metrics

The default latency metric should be requester-side round-trip time. One-way
latency should only be reported when all participating nodes provide a validated
clock quality record.

Every request/response event should record:

- requester identity
- selected provider identity
- service name
- mission id
- local monotonic send time
- local monotonic response time
- optional GPS timestamp
- optional GPS sync status
- optional clock uncertainty
- referenced data names

This avoids the v1 error of subtracting timestamps from unrelated clocks.

## Capability Mapping

`InvestigatePointRequest` should compile to a local primitive plan. The provider
should advertise enough capability to explain how it will execute the task.

Example capability choices:

- `circle-mode`: use ArduPilot Circle mode for the inspection arc
- `guided-yaw-path`: fly a yaw-aware path while facing the target
- `guided-position-only`: fly a conservative point sequence without guaranteed
  target-facing yaw
- `reject`: return a structured failure if safety or capability requirements
  cannot be met

This keeps ArduPilot-specific behavior below the service boundary while still
making vehicle limitations visible to the mission layer.

## First Prototype Slice

The first implementation should be a fake vertical:

1. WUAS publishes one fake image frame object.
2. WUAS calls the GCS detection service.
3. GCS returns a deterministic fake target.
4. WUAS calls the IUAS investigate service.
5. IUAS returns a simulated flight task result with one named sensor artifact.

The prototype proves service names, payload shapes, mission state, object
references, and metrics before any real camera, NFD, NDNSF runtime, or autopilot
is required.

## Real NDNSF Binding Slice

The next slice binds the same contracts to NDNSF's Python runtime:

- GCS runs a `ServiceProvider` for `/muas/v2/gcs/perception/detect-object`.
- IUAS runs a `ServiceProvider` for `/muas/v2/iuas-01/flight/investigate`.
- WUAS runs a `ServiceUser` and performs the two service calls in sequence.
- `config/v2_minimuas.policies` authorizes those provider/user relationships.

Deployment note: NDNSF carries requests, ACKs, selections, and responses over
ndn-svs group sync. The group prefix (`/muas/v2/group`) must use NFD's
multicast strategy on every forwarder in the deployment; with the default
best-route strategy, sync Interests reach only one registrant and requests
time out with no provider-side activity. `run_ndnsf_stack.py` sets this via
`nfdc` automatically (`ensure_multicast_strategy` in `ndnsf_runtime.py`);
standalone deployments must do the equivalent.

This still uses named frame references without fetching the underlying image
object. The following slice should add segmented image publication and fetch so
the detection provider consumes actual sensor data instead of deterministic test
inputs.

## Segmented Data-Plane Slice

This slice is implemented in `examples/python/v2_flight_services/dataplane.py`
and wired into all three roles. Sensor objects now travel as signed segmented
NDN Data under their mission-scoped names instead of remaining name-only
references:

- WUAS publishes the camera frame payload with NDNSF's
  `SegmentedObjectProducer` under the `FrameRef` data name before requesting
  detection.
- The GCS detection handler fetches the frame by name with
  `fetch_segmented_object`, validates and hashes the payload, and fails the
  service response if the fetch fails. Detection consumes transferred bytes,
  not a trusted reference.
- The IUAS capture step produces real artifact payloads stamped with the
  vehicle pose at capture time; the provider publishes each one under its
  `SensorArtifact` data name before returning the task result.
- WUAS fetches the published sensor artifacts after the mission completes
  and verifies their integrity, closing the loop in both directions.

Until a real camera is integrated, payloads are deterministic synthetic
frames (magic header, JSON metadata, multi-segment pseudo-pixel body) so
segmentation, reassembly, and integrity checking are genuinely exercised.
Swapping in real JPEG bytes changes only the payload source.

## Capability Telemetry Slice

The IUAS provider publishes a `CapabilityProfile` (contracts.py) as a
segmented object under `/muas/v2/<vehicle>/telemetry/state` at startup. The
profile mirrors relay.flight's `FlightCapabilityProfile` vocabulary. Before
dispatching the investigation, the WUAS fetches the profile and predicts the
execution mode with `expected_orbit_mode` — a mission-side mirror of the
plan_orbit capability ladder — and `mission.completed` reports whether the
vehicle executed in the predicted mode (`mode_as_predicted`). Dispatch is
no longer blind: the mission layer reasons over the same capability
vocabulary the vehicle compiles against.

## Design Note: Targeted Fast Path Evaluated and Rejected

NDNSF's C++ API includes `RequestServiceTargeted` (one named provider, no
ACK/selection round, ProviderToken-authorized). We prototyped it for the
WUAS→IUAS investigate call and measured no latency benefit on this
deployment (~60 ms targeted vs ~47-59 ms standard): the ACK/selection round
that targeted requests skip costs almost nothing here, while in NDNSF's
multi-node benchmarks it is a meaningful share of latency.

More importantly, targeting a named provider works against the
data-centric design this system is built on: requests name a *service*,
any capable provider may answer, and selection strategies
(first-responding, random, custom) decide whose response is used. The
capability-telemetry slice already gives the mission layer what it needs
— informed dispatch over published capability data — without re-binding
requests to hosts. miniMUAS therefore uses only the standard
ACK/selection request path, and no changes are carried in the NDNSF
library.

## MAVLink / SITL Slice

`mavlink_flight.py` bridges UAS-IPBRC's `MavlinkDroneLink` (pymavlink,
identical against ArduPilot SITL and real autopilots) to the flight
executor's link surface. Plan compilation is untouched by design: the same
`compile_investigation` output executes, the runner ticks on the wall
clock, position truth is GLOBAL_POSITION_INT telemetry, and the link
exposes yaw control but no native orbit, so the capability ladder reports
`guided-yaw-path`. Request altitudes are interpreted as AGL over MAVLink
and rebased onto the ground ASL auto-detected from the first position fix.

Host-side validation (no NDN), against UAS-IPBRC's SITL chain:

```sh
cd ~/Documents/Dev/UAS-IPBRC && scripts/launch_sitl_chain.sh 0
# wait ~20s for EKF convergence, then:
cd ~/Documents/Dev/miniMUAS/examples/python/v2_flight_services
python3 run_sitl_investigation.py          # udp:127.0.0.1:14550
```

The runner arms, takes off, synthesizes a target north of the vehicle,
flies the full investigate plan in real time, and finishes with RTL.
Requires pymavlink on the host (the UAS-IPBRC nix shell provides it).

Full NDN mission with the IUAS flying SITL (the container reaches the
host's SITL through ArduPilot's spare TCP serial port):

```sh
./run_v2_stack_container.sh stack --mavlink-endpoint tcp:host.docker.internal:5762
```

The stack runner raises the WUAS investigate timeout to 300s for real
flight time, and the IUAS capability profile published over telemetry
reflects the MAVLink link (no `orbit` extra), so the WUAS's
`expected_mode` prediction is `guided-yaw-path` and `mode_as_predicted`
still holds. Note the provider serves one vehicle: concurrent
investigations are not arbitrated in this slice.

Validated 2026-06-10: host flight completed in ~12s of SITL time
(5x speedup, 22 link commands, RTL finish); full container mission
completed with an investigate RTT of 89.4s, `mode_as_predicted: true`,
and the IUAS artifact fetched back over NDN at real flight coordinates.
Preflight handles a vehicle parked in RTL/LAND from a previous flight by
forcing GUIDED before arming. Two standalone diagnostics remain in the
tree for transport debugging (`probe_mavlink_stream.py`,
`probe2_adapter_path.py`): the first checks whether an endpoint delivers
position telemetry on request, the second replicates the adapter's
connect sequence stage by stage.

## Camera Frame Sources

Frames travel in a self-describing container (`dataplane.build_frame_bytes`:
magic, length-prefixed JSON header with `kind`/`body_len`/`body_sha256`,
opaque body) so the body can be real pixels instead of the synthetic
pattern without changing the publish/fetch/verify path. The role scripts
take a `--camera <spec>` flag, applied by the stack runner to both the
WUAS published frame and the IUAS capture artifacts:

    synthetic            deterministic pattern (default; no dependencies)
    file:<path>          real bytes from a file, directory, or glob,
                         cycled per capture; works in the container
    opencv:<index|url>   live JPEG capture via opencv-python: a webcam
                         index or a stream URL (RTSP, etc.); for the dev
                         host and the companion computers

`python3 camera.py <spec>...` round-trips one capture per spec through
the container format offline. Examples:

```sh
# full NDN mission, frames sourced from a real image file:
./run_v2_stack_container.sh stack --camera file:/work/miniMUAS/some.jpg

# real SITL flight, capture artifact from the host webcam:
python3 run_sitl_investigation.py --camera opencv:0
```

OpenCV is intentionally not installed in the container image; use the
file source there, or opencv on hosts and companion computers that have
cameras.

The real runtime path requires the NDNSF Python extension module
`ndnsf._ndnsf`. If the source tree has not built that extension yet, the mock
mission and dry-run commands still validate the contract, but the real
controller/provider/user stack will not launch until the wrapper is built.
Run `examples/python/v2_flight_services/preflight_ndnsf.py` before attempting a
real stack launch to check the native libraries, wrapper import, NFD tools, and
configuration files.

## Primitive Execution Slice

This slice is implemented in
`examples/python/v2_flight_services/investigate_plan.py` and wired into the
IUAS provider. It replaces the fabricated `FlightTaskResult` with real
`relay.flight` execution from a UAS-IPBRC checkout (`UAS_IPBRC_ROOT` or
`--uas-ipbrc-root`):

- `compile_investigation` turns an `InvestigatePointRequest` into a
  `Sequence`: climb to `approach_alt_m`, fly to a standoff point, orbit the
  target, then emit the sensor-plan capture command.
- `plan_orbit` performs the capability mapping above. The chosen mode is
  reported in the result `notes` and in the ACK message, and a `reject`
  outcome is returned at the ACK stage with the reason.
- Constraints map onto the primitive runner: `min_clearance_m` becomes an
  altitude-envelope constraint, `max_speed_mps` flows into motion targets,
  and `deadline_gps_ns` becomes a runner deadline that cancels the task with
  status `canceled`.
- Task status comes from the actual run (`completed`, `failed`, `canceled`,
  `blocked`, or `rejected`), and artifacts are produced by an executor
  handler for the capture command, stamped with the vehicle pose at capture
  time.

Execution currently runs against an in-process simulated link on simulated
time. The SITL/hardware step swaps in a MAVLink-backed link exposing the same
method surface (`goto`, `orbit`, `takeoff`, `land`, `rtl`) with no change to
plan compilation.
