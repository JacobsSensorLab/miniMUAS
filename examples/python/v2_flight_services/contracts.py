"""Wire contracts for the miniMUAS v2 NDNSF flight-service prototype."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
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

    def to_bytes(self) -> bytes:
        return encode_dataclass(self)

    @classmethod
    def from_bytes(cls, payload: bytes) -> "DetectionRequest":
        value = decode_json(payload)
        return cls(
            mission_id=str(value["mission_id"]),
            frame=FrameRef.from_dict(value["frame"]),
            object_query=str(value.get("object_query", "test-object")),
        )


@dataclass(frozen=True)
class DetectionResponse:
    mission_id: str
    object_id: str
    confidence: float
    estimate: GeoPoint
    evidence_ref: str | None = None

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
