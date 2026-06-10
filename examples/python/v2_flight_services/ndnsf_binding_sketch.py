"""Sketch for binding the v2 contracts to the real NDNSF Python API.

This file is illustrative rather than a complete runnable deployment. It keeps
the service handlers thin so flight behavior can live in a reusable primitive
library while NDNSF handles service discovery, authorization, and data names.
"""

from __future__ import annotations

from contracts import (
    DetectionRequest,
    DetectionResponse,
    FlightTaskResult,
    InvestigatePointRequest,
    gcs_detection_service,
    vehicle_flight_service,
)


def install_gcs_detection_provider(provider: object, detector: object) -> None:
    """Register GCS object detection against an NDNSF ServiceProvider."""

    @provider.handler(gcs_detection_service())
    def detect_object(payload: bytes) -> bytes:
        request = DetectionRequest.from_bytes(payload)
        # Real implementation:
        # 1. Fetch request.frame.data_name through NDNSF segmented object fetch.
        # 2. Run detector against the frame and camera pose.
        # 3. Publish optional evidence artifact under a mission-scoped data name.
        result: DetectionResponse = detector.detect(request)
        return result.to_bytes()


def install_iuas_investigate_provider(
    provider: object,
    vehicle_id: str,
    primitive_runner: object,
) -> None:
    """Register IUAS investigate-point against an NDNSF ServiceProvider."""

    @provider.handler(vehicle_flight_service(vehicle_id, "investigate"))
    def investigate_point(payload: bytes) -> bytes:
        request = InvestigatePointRequest.from_bytes(payload)
        # Real implementation:
        # 1. Compile request to an InspectPoint/CirclePoint primitive.
        # 2. Select Circle mode, yaw-aware guided flight, or a conservative
        #    fallback based on the vehicle capability profile.
        # 3. Publish sensor artifacts by data name.
        # 4. Return a structured terminal result.
        result: FlightTaskResult = primitive_runner.investigate_point(request)
        return result.to_bytes()


def run_wuas_detection_and_dispatch(
    user: object,
    detection_request: DetectionRequest,
    iuas_vehicle_id: str,
    investigation_factory: object,
) -> FlightTaskResult:
    """Use an NDNSF ServiceUser to call GCS then dispatch IUAS."""

    detection_response = DetectionResponse.from_bytes(
        user.request_service(
            gcs_detection_service(),
            detection_request.to_bytes(),
            ack_timeout_ms=300,
            timeout_ms=5000,
        )
    )
    investigation_request = investigation_factory.from_detection(detection_response)
    return FlightTaskResult.from_bytes(
        user.request_service(
            vehicle_flight_service(iuas_vehicle_id, "investigate"),
            investigation_request.to_bytes(),
            ack_timeout_ms=300,
            timeout_ms=15000,
        )
    )
