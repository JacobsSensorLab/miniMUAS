"""Mission data-plane helpers: segmented publish/fetch and frame payloads.

This module completes the v2 data-centric story: sensor objects actually
travel as signed segmented NDN Data under their mission-scoped names instead
of remaining name-only references. Producers serve payloads with NDNSF's
`SegmentedObjectProducer`; consumers retrieve them by name with
`fetch_segmented_object`.

Frames travel in a small self-describing container: a magic prefix, a JSON
metadata header, and an opaque body. The body can be a real camera capture
(see `camera.py`) or the deterministic synthetic pattern used when no
camera is wired in — either way publication, segmentation, reassembly, and
integrity checking are genuinely exercised.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any


FRAME_MAGIC = b"MUASFRAME1\n"
FRAME_CONTENT_TYPE = "application/x-muas-frame"
DEFAULT_FRAME_WIDTH = 320
DEFAULT_FRAME_HEIGHT = 240


def build_frame_bytes(
    body: bytes,
    *,
    mission_id: str,
    vehicle_id: str,
    sensor_id: str,
    gps_time_ns: int,
    kind: str,
    width: int | None = None,
    height: int | None = None,
    metadata: dict[str, Any] | None = None,
) -> bytes:
    """Wrap an opaque body (camera bytes, synthetic pattern) in the frame
    container: magic, length-prefixed JSON header, body.

    `kind` describes the body ("image/jpeg", "synthetic", ...). The header
    carries `body_len` explicitly so `parse_frame` can validate transfer
    integrity for any body, not just the width*height synthetic pattern.
    """

    fields: dict[str, Any] = {
        "mission_id": mission_id,
        "vehicle_id": vehicle_id,
        "sensor_id": sensor_id,
        "gps_time_ns": gps_time_ns,
        "kind": kind,
        "body_len": len(body),
        "body_sha256": hashlib.sha256(body).hexdigest(),
        "metadata": metadata or {},
    }
    if width is not None:
        fields["width"] = int(width)
    if height is not None:
        fields["height"] = int(height)
    header = json.dumps(fields, separators=(",", ":"), sort_keys=True).encode()
    return FRAME_MAGIC + len(header).to_bytes(4, "big") + header + body


def synthetic_frame_bytes(
    *,
    mission_id: str,
    vehicle_id: str,
    sensor_id: str,
    gps_time_ns: int,
    width: int = DEFAULT_FRAME_WIDTH,
    height: int = DEFAULT_FRAME_HEIGHT,
    metadata: dict[str, Any] | None = None,
) -> bytes:
    """Build a deterministic multi-segment frame payload."""

    seed = gps_time_ns % 251
    body = bytearray(width * height)
    for y in range(height):
        row_base = (y * 7 + seed) & 0xFF
        offset = y * width
        for x in range(width):
            body[offset + x] = (row_base + x * 31) & 0xFF

    return build_frame_bytes(
        bytes(body),
        mission_id=mission_id,
        vehicle_id=vehicle_id,
        sensor_id=sensor_id,
        gps_time_ns=gps_time_ns,
        kind="synthetic",
        width=width,
        height=height,
        metadata=metadata,
    )


def split_frame(payload: bytes) -> tuple[dict[str, Any], bytes]:
    """Split a frame payload into (header dict, body bytes), validating
    the container structure and the body length."""

    if not payload.startswith(FRAME_MAGIC):
        raise ValueError("payload is not a MUAS frame")
    header_start = len(FRAME_MAGIC) + 4
    header_len = int.from_bytes(payload[len(FRAME_MAGIC):header_start], "big")
    header_end = header_start + header_len
    if header_end > len(payload):
        raise ValueError("frame header is truncated")
    header = json.loads(payload[header_start:header_end].decode())
    body = payload[header_end:]
    if "body_len" in header:
        expected_body = int(header["body_len"])
    else:
        # Pre-body_len synthetic frames: body is width*height pseudo-pixels.
        expected_body = int(header["width"]) * int(header["height"])
    if len(body) != expected_body:
        raise ValueError(
            f"frame body is {len(body)} bytes, expected {expected_body}"
        )
    return header, body


def parse_frame(payload: bytes) -> dict[str, Any]:
    """Validate a frame payload and return its metadata header.

    The returned dict gains `body_bytes` and `sha256` (whole payload)
    fields describing what was actually transferred.
    """

    header, body = split_frame(payload)
    header["body_bytes"] = len(body)
    header["sha256"] = hashlib.sha256(payload).hexdigest()
    return header


def sha256_hex(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def publish_segmented(
    base_name: str,
    payload: bytes,
    *,
    freshness_ms: int = 60000,
    signing_identity: str = "",
):
    """Serve one payload as signed segmented Data; returns the live producer.

    The caller must keep the returned producer referenced (and running) for
    as long as consumers may fetch the object.
    """

    from ndnsf import SegmentedObjectProducer

    producer = SegmentedObjectProducer(
        base_name,
        payload,
        signing_identity=signing_identity,
        freshness_ms=freshness_ms,
    ).start()
    if producer.error:
        raise RuntimeError(f"failed to publish {base_name}: {producer.error}")
    return producer


def fetch_segmented(base_name: str, *, timeout_ms: int = 5000) -> bytes:
    """Fetch one segmented object by name, reassembled into bytes."""

    from ndnsf import fetch_segmented_object

    return fetch_segmented_object(base_name, timeout_ms=timeout_ms)
