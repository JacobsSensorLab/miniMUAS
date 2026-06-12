"""Frame sources for the v2 data plane: synthetic, file-backed, or camera.

A frame source produces complete frame payloads (the `dataplane` container:
magic + JSON header + body) for the WUAS's published camera frame and the
IUAS's close-range capture artifacts. The role scripts select one with a
`--camera <spec>` flag:

    synthetic            deterministic pseudo-pixel pattern (default; no deps)
    file:<path>          real bytes from disk — a single image, a directory,
                         or a glob; cycles through matches per capture.
                         Works everywhere, including the container.
    opencv:<dev|index|url>[?k=v&...]
                         live capture via OpenCV: a V4L2 device path
                         ("opencv:/dev/video0"), a device index
                         ("opencv:0"), or a stream URL ("opencv:rtsp://...").
                         Device captures force the V4L2 backend (string
                         paths otherwise fall into OpenCV's GStreamer URI
                         backend and fail) and default to MJPG fourcc —
                         UVC cameras like the Arducam OV9782 only reach
                         full frame rate in MJPG. Optional params:
                         w, h, fps, q (JPEG quality), fourcc.
                         e.g. opencv:/dev/video0?w=1280&h=800&fps=30

All sources share one method:

    capture(mission_id=..., vehicle_id=..., sensor_id=...,
            gps_time_ns=..., metadata=...) -> bytes

so the publish path is identical regardless of where the pixels came from.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dataplane import build_frame_bytes, synthetic_frame_bytes


_IMAGE_KINDS = {
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".png": "image/png",
    ".gif": "image/gif",
    ".bmp": "image/bmp",
    ".webp": "image/webp",
}


class FrameSourceError(RuntimeError):
    """A frame source could not be created or could not capture."""


class SyntheticFrameSource:
    """Deterministic pseudo-pixel frames; the no-dependency default."""

    spec = "synthetic"

    def capture(
        self,
        *,
        mission_id: str,
        vehicle_id: str,
        sensor_id: str,
        gps_time_ns: int,
        metadata: dict[str, Any] | None = None,
    ) -> bytes:
        return synthetic_frame_bytes(
            mission_id=mission_id,
            vehicle_id=vehicle_id,
            sensor_id=sensor_id,
            gps_time_ns=gps_time_ns,
            metadata=metadata,
        )

    def describe(self) -> dict[str, Any]:
        return {"source": "synthetic"}

    def close(self) -> None:
        pass


class FileFrameSource:
    """Frames whose bodies are real files from disk, cycled per capture.

    Accepts a single file, a directory (all image-suffixed files, sorted),
    or a glob pattern. The file's bytes travel unmodified as the frame
    body; the header records the originating filename in metadata.
    """

    def __init__(self, spec: str) -> None:
        self.spec = f"file:{spec}"
        root = Path(spec).expanduser()
        if root.is_dir():
            files = sorted(
                p for p in root.iterdir()
                if p.is_file() and p.suffix.lower() in _IMAGE_KINDS
            )
        elif root.is_file():
            files = [root]
        else:
            files = sorted(
                p for p in root.parent.glob(root.name) if p.is_file()
            )
        if not files:
            raise FrameSourceError(
                f"no frame files found for {spec!r} "
                f"(expected a file, a directory of images, or a glob)"
            )
        self._files = files
        self._index = 0

    def capture(
        self,
        *,
        mission_id: str,
        vehicle_id: str,
        sensor_id: str,
        gps_time_ns: int,
        metadata: dict[str, Any] | None = None,
    ) -> bytes:
        path = self._files[self._index % len(self._files)]
        self._index += 1
        body = path.read_bytes()
        kind = _IMAGE_KINDS.get(path.suffix.lower(), "application/octet-stream")
        return build_frame_bytes(
            body,
            mission_id=mission_id,
            vehicle_id=vehicle_id,
            sensor_id=sensor_id,
            gps_time_ns=gps_time_ns,
            kind=kind,
            metadata={**(metadata or {}), "source_file": path.name},
        )

    def describe(self) -> dict[str, Any]:
        return {
            "source": "file",
            "files": len(self._files),
            "first": self._files[0].name,
        }

    def close(self) -> None:
        pass


class OpenCVFrameSource:
    """Live JPEG frames from an OpenCV capture device or stream URL.

    Device captures (paths and indexes) open with the V4L2 backend
    explicitly: OpenCV routes bare string paths to its GStreamer URI
    backend, which asserts on `uridecodebin` (observed on the IUAS node).
    MJPG fourcc is requested by default — UVC sensors like the OV9782
    cap YUY2 at ~10 fps but run MJPG to 100 fps — and the capture buffer
    is kept shallow + drained per capture so a frame taken at a waypoint
    is from *now*, not from the driver queue.
    """

    def __init__(self, spec: str, *, jpeg_quality: int = 85) -> None:
        self.spec = f"opencv:{spec}"
        try:
            import cv2
        except ImportError as exc:
            raise FrameSourceError(
                "the opencv camera source requires opencv-python "
                "(`pip install opencv-python`)"
            ) from exc
        self._cv2 = cv2

        target, params = spec, {}
        if "?" in spec:
            target, query = spec.split("?", 1)
            for pair in query.split("&"):
                if "=" in pair:
                    key, value = pair.split("=", 1)
                    params[key.strip()] = value.strip()

        self._jpeg_quality = int(params.get("q", jpeg_quality))
        is_url = "://" in target
        device: int | str = int(target) if target.isdigit() else target

        if is_url:
            self._capture = cv2.VideoCapture(device)
        else:
            self._capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not self._capture.isOpened() and isinstance(device, str):
                # Many OpenCV builds' V4L2 backend only accepts indexes.
                # Resolve symlinks first so stable /dev/v4l/by-id/ paths
                # work, then extract the index from the real node name.
                import os
                real = os.path.realpath(device)
                digits = "".join(ch for ch in Path(real).name if ch.isdigit())
                if digits:
                    self._capture = cv2.VideoCapture(int(digits), cv2.CAP_V4L2)
        if not self._capture.isOpened():
            raise FrameSourceError(f"could not open capture device {target!r}")

        if not is_url:
            fourcc = params.get("fourcc", "MJPG")
            self._capture.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc)
            )
            if "w" in params:
                self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(params["w"]))
            if "h" in params:
                self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(params["h"]))
            if "fps" in params:
                self._capture.set(cv2.CAP_PROP_FPS, int(params["fps"]))
            # shallow buffer so per-capture drain is cheap and effective
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def capture(
        self,
        *,
        mission_id: str,
        vehicle_id: str,
        sensor_id: str,
        gps_time_ns: int,
        metadata: dict[str, Any] | None = None,
    ) -> bytes:
        # drain the driver queue so the frame is current, then read
        for _ in range(3):
            self._capture.grab()
        frame = None
        for _ in range(5):  # first reads after open can fail while exposure settles
            ok, candidate = self._capture.read()
            if ok and candidate is not None:
                frame = candidate
                break
        if frame is None:
            raise FrameSourceError(f"capture read failed on {self.spec}")
        ok, encoded = self._cv2.imencode(
            ".jpg",
            frame,
            [int(self._cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            raise FrameSourceError("JPEG encode failed")
        height, width = frame.shape[:2]
        return build_frame_bytes(
            encoded.tobytes(),
            mission_id=mission_id,
            vehicle_id=vehicle_id,
            sensor_id=sensor_id,
            gps_time_ns=gps_time_ns,
            kind="image/jpeg",
            width=width,
            height=height,
            metadata=metadata,
        )

    def describe(self) -> dict[str, Any]:
        return {"source": "opencv", "device": self.spec}

    def close(self) -> None:
        try:
            self._capture.release()
        except Exception:
            pass


def frame_source_from_spec(spec: str | None):
    """Build a frame source from a `--camera` spec string."""

    spec = (spec or "synthetic").strip()
    if spec == "synthetic":
        return SyntheticFrameSource()
    if spec.startswith("file:"):
        return FileFrameSource(spec[len("file:"):])
    if spec.startswith("opencv:"):
        return OpenCVFrameSource(spec[len("opencv:"):])
    raise FrameSourceError(
        f"unknown camera spec {spec!r} "
        f"(expected synthetic, file:<path>, or opencv:<dev|index|url>)"
    )


def _smoke_test(argv: list[str]) -> int:
    """Round-trip one capture per spec through the frame container.

        python3 camera.py                       # synthetic
        python3 camera.py file:~/Pictures/x.jpg opencv:0
    """

    import json
    import time

    from dataplane import parse_frame

    specs = argv or ["synthetic"]
    failures = 0
    for spec in specs:
        try:
            source = frame_source_from_spec(spec)
            payload = source.capture(
                mission_id="camera-smoke",
                vehicle_id="dev",
                sensor_id="front",
                gps_time_ns=time.time_ns(),
                metadata={"smoke": "1"},
            )
            header = parse_frame(payload)
            source.close()
            print(
                json.dumps(
                    {
                        "spec": spec,
                        "ok": True,
                        "kind": header.get("kind"),
                        "payload_bytes": len(payload),
                        "body_bytes": header["body_bytes"],
                        "body_sha256": header.get("body_sha256"),
                        "width": header.get("width"),
                        "height": header.get("height"),
                    },
                    sort_keys=True,
                )
            )
        except Exception as exc:
            failures += 1
            print(json.dumps({"spec": spec, "ok": False, "error": str(exc)}))
    return 1 if failures else 0


if __name__ == "__main__":
    import sys

    raise SystemExit(_smoke_test(sys.argv[1:]))
