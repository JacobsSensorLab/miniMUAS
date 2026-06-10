"""Small in-process stand-in for the NDNSF service/data plane."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


ServiceHandler = Callable[[bytes], bytes]


@dataclass(frozen=True)
class ServiceMetric:
    requester_id: str
    provider_id: str
    service_name: str
    send_monotonic_ns: int
    response_monotonic_ns: int

    @property
    def rtt_ms(self) -> float:
        return (self.response_monotonic_ns - self.send_monotonic_ns) / 1_000_000.0


@dataclass(frozen=True)
class StoredObject:
    data_name: str
    payload: bytes
    content_type: str
    publisher_id: str


class MockNDNSFBus:
    """Mimics the subset of NDNSF needed for the first v2 mission slice."""

    def __init__(self) -> None:
        self._services: dict[str, tuple[str, ServiceHandler]] = {}
        self._objects: dict[str, StoredObject] = {}
        self.metrics: list[ServiceMetric] = []

    def register_service(
        self,
        service_name: str,
        provider_id: str,
        handler: ServiceHandler,
    ) -> None:
        if service_name in self._services:
            raise ValueError(f"service already registered: {service_name}")
        self._services[service_name] = (provider_id, handler)

    def request_service(
        self,
        service_name: str,
        requester_id: str,
        payload: bytes,
    ) -> bytes:
        if service_name not in self._services:
            raise KeyError(f"no provider for service: {service_name}")

        provider_id, handler = self._services[service_name]
        sent = time.monotonic_ns()
        response = handler(payload)
        received = time.monotonic_ns()
        self.metrics.append(
            ServiceMetric(
                requester_id=requester_id,
                provider_id=provider_id,
                service_name=service_name,
                send_monotonic_ns=sent,
                response_monotonic_ns=received,
            )
        )
        return response

    def publish_object(
        self,
        data_name: str,
        publisher_id: str,
        payload: bytes,
        content_type: str,
    ) -> None:
        self._objects[data_name] = StoredObject(
            data_name=data_name,
            payload=payload,
            content_type=content_type,
            publisher_id=publisher_id,
        )

    def fetch_object(self, data_name: str) -> StoredObject:
        if data_name not in self._objects:
            raise KeyError(f"object not found: {data_name}")
        return self._objects[data_name]
