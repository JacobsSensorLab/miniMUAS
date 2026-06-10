"""Run the first miniMUAS v2 service-flow prototype.

The script is intentionally dependency-free. It proves the service names,
payloads, named data references, and mission choreography before binding the
same contracts to the real NDNSF Python runtime.
"""

from __future__ import annotations

import json

from contracts import (
    ConstraintSet,
    DetectionRequest,
    DetectionResponse,
    FlightTaskResult,
    FrameRef,
    GeoPoint,
    InvestigatePointRequest,
    Pose,
    SensorArtifact,
    gcs_detection_service,
    gps_time_ns,
    mission_evidence_name,
    mission_frame_name,
    mission_sensor_name,
    vehicle_flight_service,
)
from mock_bus import MockNDNSFBus


MISSION_ID = "mission-001"
GCS_ID = "gcs"
WUAS_ID = "wuas-01"
IUAS_ID = "iuas-01"


class GCS:
    def __init__(self, bus: MockNDNSFBus) -> None:
        self._bus = bus
        self._bus.register_service(
            gcs_detection_service(),
            GCS_ID,
            self._detect_object,
        )

    def _detect_object(self, payload: bytes) -> bytes:
        request = DetectionRequest.from_bytes(payload)
        frame = self._bus.fetch_object(request.frame.data_name)
        if not frame.payload:
            raise ValueError("empty frame payload")

        detected_at = gps_time_ns()
        response = DetectionResponse(
            mission_id=request.mission_id,
            object_id="target-001",
            confidence=0.91,
            estimate=GeoPoint(
                lat_deg=request.frame.pose.position.lat_deg + 0.00008,
                lon_deg=request.frame.pose.position.lon_deg + 0.00006,
                alt_m=0.0,
            ),
            evidence_ref=mission_evidence_name(
                request.mission_id,
                "target-001",
                detected_at,
            ),
        )
        self._bus.publish_object(
            response.evidence_ref or "",
            GCS_ID,
            b"fake-detection-overlay",
            "application/octet-stream",
        )
        print_event(
            "gcs.detection.completed",
            frame=request.frame.data_name,
            evidence=response.evidence_ref,
            confidence=response.confidence,
        )
        return response.to_bytes()


class IUAS:
    def __init__(self, bus: MockNDNSFBus) -> None:
        self._bus = bus
        self._bus.register_service(
            vehicle_flight_service(IUAS_ID, "investigate"),
            IUAS_ID,
            self._investigate_point,
        )

    def _investigate_point(self, payload: bytes) -> bytes:
        request = InvestigatePointRequest.from_bytes(payload)
        started = gps_time_ns()

        artifact_time = gps_time_ns()
        artifact = SensorArtifact(
            data_name=mission_sensor_name(
                request.mission_id,
                IUAS_ID,
                "front",
                "frame",
                artifact_time,
                1,
            ),
            kind="image/jpeg",
            gps_time_ns=artifact_time,
            pose=Pose(
                position=GeoPoint(
                    lat_deg=request.target.lat_deg,
                    lon_deg=request.target.lon_deg,
                    alt_m=request.approach_alt_m,
                ),
                yaw_deg=180.0,
            ),
            metadata={"target_id": request.source_detection_id},
        )
        self._bus.publish_object(
            artifact.data_name,
            IUAS_ID,
            b"fake-iuas-close-image",
            artifact.kind,
        )

        result = FlightTaskResult(
            task_id=f"{IUAS_ID}-investigate-{request.source_detection_id}",
            status="completed",
            started_at_gps_ns=started,
            completed_at_gps_ns=gps_time_ns(),
            artifacts=[artifact],
            notes="simulated circle-mode primitive",
        )
        print_event(
            "iuas.investigation.completed",
            task_id=result.task_id,
            artifact=artifact.data_name,
            execution=result.notes,
        )
        return result.to_bytes()


class WUAS:
    def __init__(self, bus: MockNDNSFBus) -> None:
        self._bus = bus

    def run_mission(self) -> FlightTaskResult:
        frame_time = gps_time_ns()
        frame = FrameRef(
            data_name=mission_frame_name(MISSION_ID, WUAS_ID, "front", frame_time, 1),
            gps_time_ns=frame_time,
            seq=1,
            camera_id="front",
            pose=Pose(
                position=GeoPoint(lat_deg=35.1208, lon_deg=-89.9347, alt_m=40.0),
                yaw_deg=90.0,
            ),
        )
        self._bus.publish_object(
            frame.data_name,
            WUAS_ID,
            b"fake-wuas-wide-frame",
            frame.content_type,
        )
        print_event("wuas.frame.published", frame=frame.data_name)

        detection_payload = DetectionRequest(
            mission_id=MISSION_ID,
            frame=frame,
            object_query="test-object",
        ).to_bytes()
        detection = DetectionResponse.from_bytes(
            self._bus.request_service(
                gcs_detection_service(),
                WUAS_ID,
                detection_payload,
            )
        )

        investigation = InvestigatePointRequest(
            mission_id=MISSION_ID,
            source_detection_id=detection.object_id,
            target=detection.estimate,
            approach_alt_m=25.0,
            standoff_m=8.0,
            circle_radius_m=6.0,
            circle_count=1.5,
            sensor_plan=["capture-still", "publish-frame"],
            constraints=ConstraintSet(
                max_speed_mps=4.0,
                min_clearance_m=3.0,
                avoidance_mode="advisory",
            ),
        )
        result = FlightTaskResult.from_bytes(
            self._bus.request_service(
                vehicle_flight_service(IUAS_ID, "investigate"),
                WUAS_ID,
                investigation.to_bytes(),
            )
        )
        print_event(
            "wuas.investigation.result",
            status=result.status,
            artifacts=[artifact.data_name for artifact in result.artifacts],
        )
        return result


def print_event(event: str, **fields: object) -> None:
    print(json.dumps({"event": event, **fields}, sort_keys=True))


def main() -> None:
    bus = MockNDNSFBus()
    GCS(bus)
    IUAS(bus)
    result = WUAS(bus).run_mission()

    print_event("mission.completed", task_id=result.task_id, status=result.status)
    for metric in bus.metrics:
        print_event(
            "metric.service_rtt",
            requester=metric.requester_id,
            provider=metric.provider_id,
            service=metric.service_name,
            rtt_ms=round(metric.rtt_ms, 3),
        )


if __name__ == "__main__":
    main()
