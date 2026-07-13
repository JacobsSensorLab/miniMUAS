"""Wire contracts for the miniMUAS v2 NDNSF flight-service prototype."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
import time
from typing import Any


def gps_time_ns() -> int:
    """Placeholder clock source until GPS/PPS time is wired into the stack."""
    return time.time_ns()


def encode_dataclass(value: Any) -> bytes:
    return json.dumps(asdict(value), separators=(",", ":"), sort_keys=True).encode()


def decode_json(payload: bytes) -> dict[str, Any]:
    return json.loads(payload.decode())


def vehicle_flight_service(vehicle_id: str, service: str) -> str:
    return f"/muas/v2/{vehicle_id}/flight/{service}"


def vehicle_telemetry_state_name(vehicle_id: str) -> str:
    """Data name where a vehicle publishes its current capability/state."""
    return f"/muas/v2/{vehicle_id}/telemetry/state"


def gcs_detection_service() -> str:
    return "/muas/v2/gcs/perception/detect-object"


def vehicle_sensor_service(vehicle_id: str) -> str:
    """On-demand sensor tasking (SensorCaptureRequest)."""
    return f"/muas/v2/{vehicle_id}/sensor/capture"


def vehicle_system_service(vehicle_id: str, action: str) -> str:
    """Companion-computer system control (e.g. authorized shutdown)."""
    return f"/muas/v2/{vehicle_id}/system/{action}"


def vehicle_journal_name(node_id: str, session: str) -> str:
    """Named segmented object where a role serves its ``<role>.jsonl`` journal
    (events + metrics + logs) for a mission session.

    Lets the dashboard's mission-bundle sweep pull every node's journal over
    the fabric (no per-node SSH) while the nodes are still up. ``node_id`` is a
    flying vehicle id for drone-agent journals, or a role-node id like ``gcs``
    for the ground providers. ``session`` scopes the object to one mission
    (nodes may also publish under the well-known session ``latest``)."""
    return f"/muas/v2/{node_id}/journal/{session}"


def vehicle_coord_status_name(vehicle_id: str) -> str:
    """Latest-wins list of the vehicle's active avoidance maneuvers.

    Coordination is data-plane only: the pair plan is deterministic and
    symmetric (both vehicles compute identical roles from each other's
    telemetry), so no request/response is needed — publishing your
    entry and OBSERVING the peer's matching one IS the agreement. A peer
    that never publishes within the grace window is handled as
    uncooperative.
    """
    return f"/muas/v2/{vehicle_id}/coord/status"


def tasked_sensor_name(
    vehicle_id: str, sensor_id: str, kind: str, timestamp_ns: int, seq: int
) -> str:
    """Data name for an operator-tasked (non-mission) sensor capture."""
    return (
        f"/muas/v2/{vehicle_id}/tasked/{sensor_id}/{kind}/{timestamp_ns}/{seq}"
    )


def vehicle_sensor_event_name(vehicle_id: str) -> str:
    """Latest SensorCaptureResult (latest-wins; how opportunistic and
    override captures reach the dashboard after the service ack)."""
    return f"/muas/v2/{vehicle_id}/sensor/last"


def mission_frame_name(
    mission_id: str,
    vehicle_id: str,
    camera_id: str,
    timestamp_ns: int,
    seq: int,
) -> str:
    return (
        f"/muas/v2/mission/{mission_id}/{vehicle_id}/camera/"
        f"{camera_id}/frame/{timestamp_ns}/{seq}"
    )


def mission_sensor_name(
    mission_id: str,
    vehicle_id: str,
    sensor_id: str,
    kind: str,
    timestamp_ns: int,
    seq: int,
) -> str:
    return (
        f"/muas/v2/mission/{mission_id}/{vehicle_id}/sensor/"
        f"{sensor_id}/{kind}/{timestamp_ns}/{seq}"
    )


def mission_evidence_name(mission_id: str, object_id: str, timestamp_ns: int) -> str:
    return f"/muas/v2/mission/{mission_id}/evidence/{object_id}/{timestamp_ns}"


# ---------------------------------------------------------------------------
# Sim ground-truth anomalies (operator-placed targets the synthetic detector
# finds) + advertised sensor metadata for the dashboard's coverage layer.
#
# Both are ADDITIVE, backward-compatible extensions:
#   * anomalies ride the DetectionRequest (optional list[dict]); a GCS that
#     predates the field ignores it, a placement-free mission sends none and
#     the stub detector keeps its legacy always-hit behaviour.
#   * sensor_meta rides the CapabilityProfile (optional dict); legacy readers
#     use .get and skip it.
# Mirrors v3 (muas-contracts::anomaly / muas-contracts::sensors): the sim/
# operator owns the ground truth, the synthetic detector queries it, and the
# dashboard renders coverage from ADVERTISED facts (never airframe-hardcoded).
# ---------------------------------------------------------------------------

# Signature colours a placed VISUAL anomaly can carry (renderer + a future
# blob detector agree on these). Kept small and stable.
ANOMALY_SIGNATURES = ("red", "orange", "blue", "magenta", "yellow")


def _ground_dist_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    """Local-flat ground distance in metres (bench scale — sub-km)."""
    dn = (lat_b - lat_a) * 111111.0
    de = (lon_b - lon_a) * 111111.0 * math.cos(math.radians((lat_a + lat_b) / 2.0))
    return math.hypot(dn, de)


def camera_footprint_radius_m(agl_m: float, hfov_deg: float) -> float:
    """Half-width of a nadir camera's ground footprint at ``agl_m`` — the
    radius within which a placed anomaly falls inside the frame."""
    return max(agl_m, 0.0) * math.tan(math.radians(max(hfov_deg, 1e-3)) / 2.0)


def nearest_visual_anomaly(
    cap_lat: float,
    cap_lon: float,
    cap_agl: float,
    hfov_deg: float,
    anomalies: list[dict] | None,
) -> tuple[dict, float] | None:
    """The visual anomaly the nadir camera at this capture pose would find.

    Returns ``(anomaly, ground_offset_m)`` for the nearest ``"visual"`` anomaly
    whose ground position lies within the camera footprint (plus the blob's own
    radius), or ``None`` for a clean miss. This is the v2 synthetic detector's
    read of the sim ground truth: the placed target under the footprint is the
    one that is "found", and it localises AT that target (offset = how far off
    nadir it sat, the dashboard's localisation-quality metric).
    """
    radius = camera_footprint_radius_m(cap_agl, hfov_deg)
    best: tuple[dict, float] | None = None
    for anomaly in anomalies or []:
        if str(anomaly.get("kind", "visual")) != "visual":
            continue
        try:
            a_lat = float(anomaly["lat_deg"])
            a_lon = float(anomaly["lon_deg"])
        except (KeyError, TypeError, ValueError):
            continue
        dist = _ground_dist_m(cap_lat, cap_lon, a_lat, a_lon)
        reach = radius + float(anomaly.get("size_m", 0.0)) / 2.0
        if dist <= reach and (best is None or dist < best[1]):
            best = (anomaly, dist)
    return best


def default_camera_meta(
    hfov_deg: float,
    width_px: int,
    height_px: int,
    *,
    facing: str = "down",
    dri_m: list[float] | None = None,
) -> dict:
    """Camera facts the agent advertises for FoV/footprint rendering."""
    meta: dict[str, Any] = {
        "hfov_deg": float(hfov_deg),
        "width_px": int(width_px),
        "height_px": int(height_px),
        "facing": facing,
    }
    if dri_m:
        meta["dri_m"] = [float(x) for x in dri_m]
    return meta


def default_audio_meta(
    omni_range_m: float,
    *,
    lobes: list[dict] | None = None,
) -> dict:
    """Microphone facts: an omnidirectional confidence radius, and optional
    beamforming lobes ``[{bearing_deg, width_deg, range_m}, ...]`` when the
    array reports them (empty => the renderer draws the omni circle)."""
    meta: dict[str, Any] = {"omni_range_m": float(omni_range_m)}
    if lobes:
        meta["lobes"] = [dict(lobe) for lobe in lobes]
    return meta


@dataclass(frozen=True)
class CapabilityProfile:
    """Flight capabilities one vehicle advertises to the mission layer.

    Field names mirror relay.flight's FlightCapabilityProfile so the mission
    layer and the vehicle's primitive compiler reason over the same
    vocabulary. `extras` carries open-ended capability strings such as
    "orbit" for native circle mode.
    """

    vehicle_id: str
    gps_time_ns: int
    position: bool = False
    velocity: bool = False
    yaw_control: bool = False
    mode_control: bool = False
    gimbal: bool = False
    obstacle_map: bool = False
    signal_sensor: bool = False
    extras: list[str] = field(default_factory=list)
    # Additive sensor metadata for the dashboard's coverage layer: an
    # open-ended dict with optional "camera" / "audio" sub-objects
    # (see default_camera_meta / default_audio_meta). Legacy consumers that
    # predate the key ignore it; the renderer draws only what is advertised.
    sensor_meta: dict = field(default_factory=dict)

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "CapabilityProfile":
        value = decode_json(payload)
        return cls(
            vehicle_id=str(value["vehicle_id"]),
            gps_time_ns=int(value["gps_time_ns"]),
            position=bool(value.get("position", False)),
            velocity=bool(value.get("velocity", False)),
            yaw_control=bool(value.get("yaw_control", False)),
            mode_control=bool(value.get("mode_control", False)),
            gimbal=bool(value.get("gimbal", False)),
            obstacle_map=bool(value.get("obstacle_map", False)),
            signal_sensor=bool(value.get("signal_sensor", False)),
            extras=[str(item) for item in value.get("extras", [])],
            sensor_meta=dict(value.get("sensor_meta") or {}),
        )


def expected_orbit_mode(profile: CapabilityProfile) -> str:
    """Mission-side mirror of relay.flight's plan_orbit capability ladder.

    Lets a dispatcher predict how a vehicle will execute an orbit request
    before sending it: circle-mode, guided-yaw-path, guided-position-only,
    or reject.
    """

    if not (profile.position and profile.mode_control):
        return "reject"
    if "orbit" in profile.extras:
        return "circle-mode"
    if profile.yaw_control:
        return "guided-yaw-path"
    return "guided-position-only"


@dataclass(frozen=True)
class GeoPoint:
    lat_deg: float
    lon_deg: float
    alt_m: float | None = None

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "GeoPoint":
        return cls(
            lat_deg=float(value["lat_deg"]),
            lon_deg=float(value["lon_deg"]),
            alt_m=None if value.get("alt_m") is None else float(value["alt_m"]),
        )


@dataclass(frozen=True)
class Pose:
    position: GeoPoint
    yaw_deg: float | None = None

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Pose":
        return cls(
            position=GeoPoint.from_dict(value["position"]),
            yaw_deg=None if value.get("yaw_deg") is None else float(value["yaw_deg"]),
        )


@dataclass(frozen=True)
class FrameRef:
    data_name: str
    gps_time_ns: int
    seq: int
    camera_id: str
    pose: Pose
    content_type: str = "image/jpeg"

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "FrameRef":
        return cls(
            data_name=str(value["data_name"]),
            gps_time_ns=int(value["gps_time_ns"]),
            seq=int(value["seq"]),
            camera_id=str(value["camera_id"]),
            pose=Pose.from_dict(value["pose"]),
            content_type=str(value.get("content_type", "image/jpeg")),
        )


@dataclass(frozen=True)
class ConstraintSet:
    max_speed_mps: float | None = None
    min_clearance_m: float | None = None
    deadline_gps_ns: int | None = None
    avoidance_mode: str = "advisory"

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ConstraintSet":
        return cls(
            max_speed_mps=(
                None
                if value.get("max_speed_mps") is None
                else float(value["max_speed_mps"])
            ),
            min_clearance_m=(
                None
                if value.get("min_clearance_m") is None
                else float(value["min_clearance_m"])
            ),
            deadline_gps_ns=(
                None
                if value.get("deadline_gps_ns") is None
                else int(value["deadline_gps_ns"])
            ),
            avoidance_mode=str(value.get("avoidance_mode", "advisory")),
        )


@dataclass(frozen=True)
class DetectionRequest:
    mission_id: str
    frame: FrameRef
    object_query: str = "test-object"
    # Sim ground-truth anomalies (operator-placed targets) the synthetic
    # detector should look for in this frame. Optional and additive: absent
    # or empty keeps the stub detector's legacy always-hit behaviour; when
    # present, the stub reports a hit ONLY for a placed target under the
    # frame's footprint (localised at that target). Ignored by real detectors.
    anomalies: list[dict] = field(default_factory=list)

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "DetectionRequest":
        value = decode_json(payload)
        return cls(
            mission_id=str(value["mission_id"]),
            frame=FrameRef.from_dict(value["frame"]),
            object_query=str(value.get("object_query", "test-object")),
            anomalies=[dict(a) for a in value.get("anomalies", []) if isinstance(a, dict)],
        )


@dataclass(frozen=True)
class DetectionResponse:
    mission_id: str
    object_id: str
    confidence: float
    estimate: GeoPoint
    evidence_ref: str | None = None
    # In-frame ground offset (m) from the capture nadir to the detected
    # object. Small => object was near frame center when captured => the
    # nadir geo-projection is reliable (little AGL/heading lever-arm
    # error). The dashboard prefers the SMALLEST-offset sighting of a
    # confirmed target for its position, not the highest-confidence one.
    offset_m: float = 0.0

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "DetectionResponse":
        value = decode_json(payload)
        return cls(
            mission_id=str(value["mission_id"]),
            object_id=str(value["object_id"]),
            confidence=float(value["confidence"]),
            estimate=GeoPoint.from_dict(value["estimate"]),
            evidence_ref=value.get("evidence_ref"),
            offset_m=float(value.get("offset_m", 0.0)),
        )


@dataclass(frozen=True)
class InvestigatePointRequest:
    mission_id: str
    source_detection_id: str
    target: GeoPoint
    approach_alt_m: float
    standoff_m: float
    circle_radius_m: float
    circle_count: float
    facing: str = "target"
    sensor_plan: list[str] = field(default_factory=lambda: ["capture-still"])
    constraints: ConstraintSet = field(default_factory=ConstraintSet)

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "InvestigatePointRequest":
        value = decode_json(payload)
        return cls(
            mission_id=str(value["mission_id"]),
            source_detection_id=str(value["source_detection_id"]),
            target=GeoPoint.from_dict(value["target"]),
            approach_alt_m=float(value["approach_alt_m"]),
            standoff_m=float(value["standoff_m"]),
            circle_radius_m=float(value["circle_radius_m"]),
            circle_count=float(value["circle_count"]),
            facing=str(value.get("facing", "target")),
            sensor_plan=[str(item) for item in value.get("sensor_plan", [])],
            constraints=ConstraintSet.from_dict(value.get("constraints", {})),
        )


@dataclass(frozen=True)
class SensorArtifact:
    data_name: str
    kind: str
    gps_time_ns: int
    pose: Pose
    metadata: dict[str, str] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SensorArtifact":
        return cls(
            data_name=str(value["data_name"]),
            kind=str(value["kind"]),
            gps_time_ns=int(value["gps_time_ns"]),
            pose=Pose.from_dict(value["pose"]),
            metadata={str(k): str(v) for k, v in value.get("metadata", {}).items()},
        )


@dataclass(frozen=True)
class FlightTaskResult:
    task_id: str
    status: str
    started_at_gps_ns: int
    completed_at_gps_ns: int
    artifacts: list[SensorArtifact] = field(default_factory=list)
    notes: str = ""

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "FlightTaskResult":
        value = decode_json(payload)
        return cls(
            task_id=str(value["task_id"]),
            status=str(value["status"]),
            started_at_gps_ns=int(value["started_at_gps_ns"]),
            completed_at_gps_ns=int(value["completed_at_gps_ns"]),
            artifacts=[
                SensorArtifact.from_dict(item)
                for item in value.get("artifacts", [])
            ],
            notes=str(value.get("notes", "")),
        )


# ---------------------------------------------------------------------------
# Phase 2: drone agent (live telemetry, raster search, flight commands,
# video control) and the GCS dashboard orchestrator.
# ---------------------------------------------------------------------------


def vehicle_telemetry_live_name(vehicle_id: str) -> str:
    """Data name where a vehicle publishes its 1 Hz live telemetry sample."""
    return f"/muas/v2/{vehicle_id}/telemetry/live"


def vehicle_search_status_name(vehicle_id: str) -> str:
    """Data name where a searching vehicle publishes raster progress."""
    return f"/muas/v2/{vehicle_id}/search/status"


def vehicle_video_frame_name(vehicle_id: str, seq: int) -> str:
    """Data name of one live MJPEG video frame (latest-wins by seq)."""
    return f"/muas/v2/{vehicle_id}/video/{seq}"


def vehicle_video_live_name(vehicle_id: str) -> str:
    """Well-known live-video name: republished per frame, fetched by base
    name so version discovery always returns the NEWEST frame (live video
    must never queue — latest-wins, not sequence playback). Payload is an
    8-byte big-endian seq header followed by the JPEG."""
    return f"/muas/v2/{vehicle_id}/video/live"


def vehicle_video_status_name(vehicle_id: str) -> str:
    """Data name where a vehicle publishes its video stream status."""
    return f"/muas/v2/{vehicle_id}/video/status"


def vehicle_video_service(vehicle_id: str) -> str:
    return f"/muas/v2/{vehicle_id}/video/control"


@dataclass(frozen=True)
class TelemetrySample:
    """One 1 Hz vehicle state sample for the dashboard.

    `source` records where the numbers came from ("mavlink" for a live FC,
    "sim" for the bench simulated vehicle, "static" when neither is
    available) so the UI can label trust accordingly. `gps_time_ns` is the
    publisher's clock at sample time; the dashboard derives link health
    from its age.
    """

    vehicle_id: str
    gps_time_ns: int
    source: str = "static"
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_m: float = 0.0
    agl_m: float = 0.0
    heading_deg: float = 0.0
    groundspeed_m_s: float = 0.0
    armed: bool = False
    mode: str = ""
    battery_v: float = 0.0
    battery_pct: float = -1.0
    busy: str = ""  # "", "raster-search", "investigate", ...
    # low-altitude confidence: rangefinder AGL when the vehicle carries
    # one (-1 = not fitted / no reading), and an alarm the agent raises
    # when baro-AGL and rangefinder disagree beyond tolerance
    rangefinder_m: float = -1.0
    agl_alarm: bool = False
    # fleet coordination: ground velocity (peers extrapolate each other
    # with physics from these) and the active vertical avoidance bias
    vn_m_s: float = 0.0
    ve_m_s: float = 0.0
    avoid_bias_m: float = 0.0

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "TelemetrySample":
        value = decode_json(payload)
        return cls(
            vehicle_id=str(value["vehicle_id"]),
            gps_time_ns=int(value["gps_time_ns"]),
            source=str(value.get("source", "static")),
            lat_deg=float(value.get("lat_deg", 0.0)),
            lon_deg=float(value.get("lon_deg", 0.0)),
            alt_m=float(value.get("alt_m", 0.0)),
            agl_m=float(value.get("agl_m", 0.0)),
            heading_deg=float(value.get("heading_deg", 0.0)),
            groundspeed_m_s=float(value.get("groundspeed_m_s", 0.0)),
            armed=bool(value.get("armed", False)),
            mode=str(value.get("mode", "")),
            battery_v=float(value.get("battery_v", 0.0)),
            battery_pct=float(value.get("battery_pct", -1.0)),
            busy=str(value.get("busy", "")),
            rangefinder_m=float(value.get("rangefinder_m", -1.0)),
            agl_alarm=bool(value.get("agl_alarm", False)),
            vn_m_s=float(value.get("vn_m_s", 0.0)),
            ve_m_s=float(value.get("ve_m_s", 0.0)),
            avoid_bias_m=float(value.get("avoid_bias_m", 0.0)),
        )


@dataclass(frozen=True)
class SensorCaptureRequest:
    """Operator-tasked sensor capture — sensors are mission-controlled.

    Sensors never run continuously: a capture happens only when a mission
    step asks for it or an operator issues one of these. Modes:

      now            capture immediately at the current position (the
                     "directly requested" case; no flight involved)
      override       fly to `target`, capture, then RESUME whatever task
                     was interrupted (a WUAS mid-raster picks its leg back
                     up). Rejected during an investigate orbit.
      opportunistic  register a watchpoint: whenever the vehicle happens
                     to pass within `radius_m` of `target` (during any
                     flight) the capture fires. Expires after `expires_s`.

    Audio additionally honors the agent's --audio-range-m guard: a
    capture tied to a target only records while the vehicle is within
    that range, so the microphone is never hot outside a tasked window.
    """

    request_id: str
    sensor: str                       # "camera" | "audio"
    mode: str = "now"                 # now | override | opportunistic
    duration_s: float = 6.0           # audio clip length; camera ignores
    target: GeoPoint | None = None
    radius_m: float = 6.0             # trigger / positioning tolerance
    expires_s: float = 600.0          # opportunistic watchpoint lifetime
    note: str = ""

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "SensorCaptureRequest":
        value = decode_json(payload)
        target = value.get("target")
        return cls(
            request_id=str(value["request_id"]),
            sensor=str(value["sensor"]),
            mode=str(value.get("mode", "now")),
            duration_s=float(value.get("duration_s", 6.0)),
            target=None if target is None else GeoPoint.from_dict(target),
            radius_m=float(value.get("radius_m", 6.0)),
            expires_s=float(value.get("expires_s", 600.0)),
            note=str(value.get("note", "")),
        )


@dataclass(frozen=True)
class SensorCaptureResult:
    """Terminal (or queued) record for one SensorCaptureRequest."""

    request_id: str
    vehicle_id: str
    sensor: str
    status: str                       # captured | queued | rejected | failed
    message: str = ""
    artifacts: list[str] = field(default_factory=list)
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    agl_m: float = 0.0
    gps_time_ns: int = 0

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "SensorCaptureResult":
        value = decode_json(payload)
        return cls(
            request_id=str(value["request_id"]),
            vehicle_id=str(value["vehicle_id"]),
            sensor=str(value["sensor"]),
            status=str(value["status"]),
            message=str(value.get("message", "")),
            artifacts=[str(a) for a in value.get("artifacts", [])],
            lat_deg=float(value.get("lat_deg", 0.0)),
            lon_deg=float(value.get("lon_deg", 0.0)),
            agl_m=float(value.get("agl_m", 0.0)),
            gps_time_ns=int(value.get("gps_time_ns", 0)),
        )


@dataclass(frozen=True)
class SearchArea:
    """Rectangular raster area, definable two ways (matching the UI modes).

    mode "center": center_lat/center_lon + width_m/height_m, axis-aligned
    (width = east-west, height = north-south).
    mode "corners": corner_a/corner_b as opposite (lat, lon) corners of an
    axis-aligned rectangle.
    """

    mode: str = "center"
    center_lat: float = 0.0
    center_lon: float = 0.0
    width_m: float = 40.0
    height_m: float = 30.0
    corner_a: list[float] = field(default_factory=list)  # [lat, lon]
    corner_b: list[float] = field(default_factory=list)  # [lat, lon]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SearchArea":
        return cls(
            mode=str(value.get("mode", "center")),
            center_lat=float(value.get("center_lat", 0.0)),
            center_lon=float(value.get("center_lon", 0.0)),
            width_m=float(value.get("width_m", 40.0)),
            height_m=float(value.get("height_m", 30.0)),
            corner_a=[float(v) for v in value.get("corner_a", [])],
            corner_b=[float(v) for v in value.get("corner_b", [])],
        )


@dataclass(frozen=True)
class RasterSearchRequest:
    """Fly a lawnmower over `area`, capture frames, detect via the GCS.

    Detection requests are issued asynchronously by the searching vehicle
    (the NDNSF transport adds a roughly constant per-request latency, so
    the raster never stops to wait); the first hit at or above
    `min_confidence` ends the search with status "target-found". AGL
    defaults low because detection ground-sampling distance demands it
    (70deg HFOV / 1280 px: a racquet is ~100 px at 6 m, ~40 px at 15 m).
    """

    mission_id: str
    area: SearchArea
    agl_m: float = 6.0
    leg_spacing_m: float = 5.0
    speed_m_s: float = 2.0
    capture_every_m: float = 4.0
    object_query: str = "tennis racket"
    min_confidence: float = 0.3
    max_duration_s: float = 600.0

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "RasterSearchRequest":
        value = decode_json(payload)
        return cls(
            mission_id=str(value["mission_id"]),
            area=SearchArea.from_dict(value.get("area", {})),
            agl_m=float(value.get("agl_m", 6.0)),
            leg_spacing_m=float(value.get("leg_spacing_m", 5.0)),
            speed_m_s=float(value.get("speed_m_s", 2.0)),
            capture_every_m=float(value.get("capture_every_m", 4.0)),
            object_query=str(value.get("object_query", "tennis racket")),
            min_confidence=float(value.get("min_confidence", 0.3)),
            max_duration_s=float(value.get("max_duration_s", 600.0)),
        )


@dataclass(frozen=True)
class SearchStatus:
    """Raster progress, published at vehicle_search_status_name (1 Hz)."""

    vehicle_id: str
    mission_id: str
    gps_time_ns: int
    state: str = "idle"  # idle|transit|searching|found|aborted|failed|done
    leg: int = 0
    legs_total: int = 0
    frames_captured: int = 0
    detects_pending: int = 0
    detects_completed: int = 0
    last_frames: list[str] = field(default_factory=list)  # newest first, capped
    last_note: str = ""

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "SearchStatus":
        value = decode_json(payload)
        return cls(
            vehicle_id=str(value["vehicle_id"]),
            mission_id=str(value["mission_id"]),
            gps_time_ns=int(value["gps_time_ns"]),
            state=str(value.get("state", "idle")),
            leg=int(value.get("leg", 0)),
            legs_total=int(value.get("legs_total", 0)),
            frames_captured=int(value.get("frames_captured", 0)),
            detects_pending=int(value.get("detects_pending", 0)),
            detects_completed=int(value.get("detects_completed", 0)),
            last_frames=[str(v) for v in value.get("last_frames", [])],
            last_note=str(value.get("last_note", "")),
        )


@dataclass(frozen=True)
class RasterSearchResult:
    """Terminal response of a raster-search service request."""

    task_id: str
    status: str  # target-found|completed|aborted|failed
    frames_captured: int = 0
    object_id: str = ""
    confidence: float = 0.0
    target: GeoPoint = field(default_factory=lambda: GeoPoint(0.0, 0.0, 0.0))
    evidence_frame: str = ""
    notes: str = ""

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "RasterSearchResult":
        value = decode_json(payload)
        return cls(
            task_id=str(value["task_id"]),
            status=str(value["status"]),
            frames_captured=int(value.get("frames_captured", 0)),
            object_id=str(value.get("object_id", "")),
            confidence=float(value.get("confidence", 0.0)),
            target=GeoPoint.from_dict(
                value.get("target", {"lat_deg": 0, "lon_deg": 0, "alt_m": 0})
            ),
            evidence_frame=str(value.get("evidence_frame", "")),
            notes=str(value.get("notes", "")),
        )


@dataclass(frozen=True)
class FlightCommandResult:
    """Response shape for rtl / land / hold / takeoff service requests."""

    vehicle_id: str
    command: str
    status: str  # accepted|rejected|failed
    message: str = ""

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "FlightCommandResult":
        value = decode_json(payload)
        return cls(
            vehicle_id=str(value["vehicle_id"]),
            command=str(value["command"]),
            status=str(value["status"]),
            message=str(value.get("message", "")),
        )


@dataclass(frozen=True)
class TakeoffRequest:
    """Arm (if needed) and climb to a target AGL, then position-hold.

    Standalone manual control — distinct from the takeoff embedded in a
    raster or investigate. Its main use is the AGL-verification rung: send
    a known altitude, then read it back on the telemetry tile to confirm
    the vehicle's reported AGL matches reality before trusting autonomy.
    """

    target_agl_m: float = 5.0

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "TakeoffRequest":
        value = decode_json(payload)
        return cls(target_agl_m=float(value.get("target_agl_m", 5.0)))


@dataclass(frozen=True)
class VideoControlRequest:
    """Start/stop the vehicle's MJPEG-over-NDN stream and set its knobs."""

    enable: bool
    width: int = 320
    height: int = 240
    fps: float = 5.0
    quality: int = 40

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "VideoControlRequest":
        value = decode_json(payload)
        return cls(
            enable=bool(value.get("enable", False)),
            width=int(value.get("width", 320)),
            height=int(value.get("height", 240)),
            fps=float(value.get("fps", 5.0)),
            quality=int(value.get("quality", 40)),
        )


@dataclass(frozen=True)
class VideoStatus:
    """Published at vehicle_video_status_name whenever the stream changes."""

    vehicle_id: str
    gps_time_ns: int
    enabled: bool = False
    seq: int = 0
    width: int = 320
    height: int = 240
    fps: float = 5.0
    quality: int = 40

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "VideoStatus":
        value = decode_json(payload)
        return cls(
            vehicle_id=str(value["vehicle_id"]),
            gps_time_ns=int(value["gps_time_ns"]),
            enabled=bool(value.get("enabled", False)),
            seq=int(value.get("seq", 0)),
            width=int(value.get("width", 320)),
            height=int(value.get("height", 240)),
            fps=float(value.get("fps", 5.0)),
            quality=int(value.get("quality", 40)),
        )
