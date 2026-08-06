"""Real object detection for the GCS: YOLOv8 ONNX, CPU-optimized for ARM.

Replaces the v2 detection stub with actual vision. A `yolo:` detector spec
selects this path on the GCS provider:

    --detector "yolo:/path/yolov8n.onnx?backend=onnxruntime&imgsz=320&conf=0.35&classes=tennis racket"

The model is a stock COCO-trained YOLOv8 export (no training needed for
the demo target — "tennis racket" is COCO class 38), produced once with:

    pip install ultralytics
    yolo export model=yolov8n.pt format=onnx imgsz=640 opset=12

Speed on the Odroid C4 (Amlogic S905X3: 4×Cortex-A55, Mali-G31 MP2 GPU,
no NPU). The Mali GPU is NOT a useful accelerator here — the Panfrost/
NixOS stack exposes no OpenCL, and a 2-core Mali-G31 is smaller than the
CPU anyway — so every lever is CPU-side:

  * backend=onnxruntime (default when the package is importable): multi-
    threaded MLAS/XNNPACK NEON kernels across all 4 cores, markedly faster
    than backend=opencv (cv2.dnn's single inference path). Add onnxruntime
    to the GCS Python env to enable it; auto-falls back to opencv if it
    is absent, so behaviour degrades gracefully. `threads=` overrides the
    core count.
  * imgsz: inference cost scales ~quadratically, so 640 -> 320 is ~4x
    faster. Needs a model whose input accepts the size: re-export smaller
    (`yolo export ... imgsz=320`) or with dynamic axes (`... dynamic=True`);
    a static-640 ONNX only runs at 640.
  * INT8: quantize the ONNX offline (onnxruntime.quantization; the A55 has
    the ARMv8.2 DotProd extension, so INT8 GEMM is a real speedup) and
    point the spec at the quantized model — ~2x more, small accuracy cost.

Reference: yolov8n via cv2.dnn at 640 is ~1.5-3 s/frame on the A55s;
onnxruntime at 320 lands well under a second. Detection runs per search
frame on its own worker (see run_gcs_provider), not on continuous video.

Also hosts the nadir geo-projection: a pixel detection in a downward
camera frame becomes a ground offset from the capture position via
ground-sampling distance (2·AGL·tan(HFOV/2)/width). Heading, when known,
rotates the camera-frame offset into north/east; without it the camera
is assumed north-up — fine for bench validation, an honest approximation
in the field until pose carries yaw.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]


class DetectorError(RuntimeError):
    """A detector could not be created or could not run."""


@dataclass
class Detection:
    label: str
    confidence: float
    box_xywh: tuple[int, int, int, int]  # top-left x, y, width, height (px)

    @property
    def center_px(self) -> tuple[float, float]:
        x, y, w, h = self.box_xywh
        return (x + w / 2.0, y + h / 2.0)

    def as_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 4),
            "box_xywh": list(self.box_xywh),
        }


class _YoloDetectorBase:
    """Shared COCO YOLOv8 letterbox + decode; a backend supplies _infer().

    Splitting inference from pre/post-processing lets the cv2.dnn and
    onnxruntime backends share the exact same geometry and NMS, so switching
    runtimes for speed never changes detection semantics.
    """

    backend = "yolo"

    def __init__(
        self,
        model_path: str,
        *,
        conf_threshold: float = 0.35,
        iou_threshold: float = 0.45,
        imgsz: int = 640,
        class_filter: list[str] | None = None,
    ) -> None:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise DetectorError(
                "the yolo detector requires opencv-python (cv2 + numpy)"
            ) from exc
        self._cv2 = cv2
        self._np = np
        self.model_path = model_path
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.imgsz = int(imgsz)
        # Per-thread, NOT a shared instance attribute: the provider reads
        # last_all_detections right after its own detect() to draw the annotated
        # frame + fill the response's all_classes. A shared attribute would hand
        # a thread the *other* frame's detections — the stuck/misassociated boxes
        # seen in the saved field frames. Read from the same thread that set it.
        self._tls = threading.local()
        self.class_filter = (
            {name.strip().lower() for name in class_filter}
            if class_filter
            else None
        )

    @property
    def last_all_detections(self) -> list[Detection]:
        return getattr(self._tls, "all_detections", [])

    def _preprocess(self, image_bgr):
        """Letterbox to imgsz×imgsz -> normalized NCHW blob + geometry.

        Returns the same (1,3,imgsz,imgsz) float32 RGB/255 tensor whether the
        backend is cv2.dnn or onnxruntime (both expect exactly this)."""
        cv2, np = self._cv2, self._np
        height, width = image_bgr.shape[:2]
        scale = min(self.imgsz / width, self.imgsz / height)
        new_w, new_h = int(round(width * scale)), int(round(height * scale))
        pad_x, pad_y = (self.imgsz - new_w) / 2.0, (self.imgsz - new_h) / 2.0
        resized = cv2.resize(image_bgr, (new_w, new_h))
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, dtype=np.uint8)
        top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
        canvas[top:top + new_h, left:left + new_w] = resized
        blob = cv2.dnn.blobFromImage(
            canvas, 1.0 / 255.0, (self.imgsz, self.imgsz), swapRB=True
        )
        return blob, scale, pad_x, pad_y

    def _infer(self, blob):
        """Run the model on one NCHW blob; return the raw output array."""
        raise NotImplementedError

    def detect(self, image_bgr) -> list[Detection]:
        cv2, np = self._cv2, self._np
        height, width = image_bgr.shape[:2]
        blob, scale, pad_x, pad_y = self._preprocess(image_bgr)
        output = self._infer(blob)

        # YOLOv8 head: (1, 4 + n_classes, n_anchors) -> (n_anchors, 4 + n)
        predictions = np.squeeze(output)
        if predictions.ndim != 2:
            raise DetectorError(f"unexpected model output shape {output.shape}")
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(class_ids)), class_ids]
        keep = confidences >= self.conf_threshold
        if not np.any(keep):
            self._tls.all_detections = []
            return []
        boxes_cxcywh = predictions[keep, :4]
        class_ids = class_ids[keep]
        confidences = confidences[keep]

        # canvas px -> original image px, as top-left xywh for NMS
        boxes = []
        for cx, cy, bw, bh in boxes_cxcywh:
            x = (cx - bw / 2.0 - pad_x) / scale
            y = (cy - bh / 2.0 - pad_y) / scale
            boxes.append([
                int(round(x)),
                int(round(y)),
                int(round(bw / scale)),
                int(round(bh / scale)),
            ])

        indices = cv2.dnn.NMSBoxes(
            boxes,
            confidences.astype(float).tolist(),
            self.conf_threshold,
            self.iou_threshold,
        )
        detections: list[Detection] = []
        all_detections: list[Detection] = []
        for index in np.array(indices).reshape(-1):
            class_id = int(class_ids[index])
            label = (
                COCO_NAMES[class_id]
                if 0 <= class_id < len(COCO_NAMES)
                else f"class-{class_id}"
            )
            x, y, w, h = boxes[index]
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            detection = Detection(
                label=label,
                confidence=float(confidences[index]),
                box_xywh=(x, y, w, h),
            )
            all_detections.append(detection)
            if self.class_filter and label.lower() not in self.class_filter:
                continue
            detections.append(detection)
        all_detections.sort(key=lambda d: d.confidence, reverse=True)
        detections.sort(key=lambda d: d.confidence, reverse=True)
        # everything the model saw above threshold, pre-class-filter —
        # essential for diagnosing "empty" results (bad frame vs missed
        # target class vs filter mismatch)
        self._tls.all_detections = all_detections
        return detections

    def describe(self) -> dict[str, Any]:
        return {
            "detector": self.backend,
            "model": self.model_path,
            "conf": self.conf_threshold,
            "imgsz": self.imgsz,
            "classes": sorted(self.class_filter) if self.class_filter else "all",
        }


class YoloOnnxDetector(_YoloDetectorBase):
    """YOLOv8 ONNX via OpenCV cv2.dnn (CPU). Portable, no extra dep, but the
    slowest backend on ARM — prefer onnxruntime where it is available."""

    backend = "yolo-opencv"

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        cv2, np = self._cv2, self._np
        try:
            self._net = cv2.dnn.readNetFromONNX(self.model_path)
        except Exception as exc:
            raise DetectorError(
                f"could not load ONNX model {self.model_path!r}: {exc}"
            )
        # cv2.dnn.Net is not thread-safe, and forward() releases the GIL, so
        # concurrent detect requests race inside the net's lazy graph finalize
        # and trip an OpenCV assertion (outputs.size()==scaleFactors.size() in
        # 'finalize') that wedges the net for every subsequent frame. Serialize
        # inference (below) and force the one-time finalize here, single-
        # threaded, with a warm-up so the race cannot happen at request time.
        self._infer_lock = threading.Lock()
        try:
            warm = np.zeros((1, 3, self.imgsz, self.imgsz), dtype=np.float32)
            self._net.setInput(warm)
            self._net.forward()
        except Exception:
            pass

    def _infer(self, blob):
        with self._infer_lock:
            self._net.setInput(blob)
            # forward() returns a VIEW into the net's internal output blob,
            # reused on the next forward(); copy inside the lock so a
            # concurrent detect() can't overwrite this buffer mid-parse.
            return self._net.forward().copy()


class YoloOrtDetector(_YoloDetectorBase):
    """YOLOv8 ONNX via onnxruntime (CPU, multi-threaded MLAS/XNNPACK).

    Markedly faster than cv2.dnn on the Odroid's A55s — MLAS runs NEON GEMM
    across all cores, XNNPACK (when the onnxruntime build ships it) adds
    more. InferenceSession.run is thread-safe, so no per-call lock is needed;
    the offloaded worker serializes calls anyway."""

    backend = "yolo-onnxruntime"

    def __init__(self, *args, threads: int | None = None, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise DetectorError(
                "backend=onnxruntime requires the onnxruntime package "
                "(add python3Packages.onnxruntime to the GCS env)"
            ) from exc
        import os

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = int(threads or os.cpu_count() or 4)
        opts.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        available = set(ort.get_available_providers())
        # XNNPACK first when present (fastest ARM path), else MLAS CPU EP.
        providers = [
            p for p in ("XnnpackExecutionProvider", "CPUExecutionProvider")
            if p in available
        ] or None
        try:
            self._sess = ort.InferenceSession(
                self.model_path, sess_options=opts, providers=providers
            )
        except Exception as exc:
            raise DetectorError(
                f"could not load ONNX model {self.model_path!r}: {exc}"
            )
        self._input_name = self._sess.get_inputs()[0].name
        self._providers = self._sess.get_providers()
        self._threads = opts.intra_op_num_threads
        try:
            warm = self._np.zeros(
                (1, 3, self.imgsz, self.imgsz), dtype=self._np.float32
            )
            self._sess.run(None, {self._input_name: warm})
        except Exception:
            pass

    def _infer(self, blob):
        return self._sess.run(None, {self._input_name: blob})[0]

    def describe(self) -> dict[str, Any]:
        info = super().describe()
        info["providers"] = getattr(self, "_providers", [])
        info["threads"] = getattr(self, "_threads", None)
        return info


def detector_from_spec(spec: str | None):
    """Build a detector from a `--detector` spec; None for the stub.

        stub                          (default; offset-based fake detection)
        yolo:<model.onnx>[?backend=auto&conf=0.35&iou=0.45&imgsz=640&threads=N&classes=a,b]

    backend: auto (default; onnxruntime if importable, else opencv),
    onnxruntime, or opencv. imgsz below 640 needs a model exported at that
    size or with dynamic axes.
    """

    spec = (spec or "stub").strip()
    if spec == "stub":
        return None
    if not spec.startswith("yolo:"):
        raise DetectorError(
            f"unknown detector spec {spec!r} (expected stub or yolo:<model>)"
        )
    target, params = spec[len("yolo:"):], {}
    if "?" in target:
        target, query = target.split("?", 1)
        for pair in query.split("&"):
            if "=" in pair:
                key, value = pair.split("=", 1)
                params[key.strip()] = value.strip()
    class_filter = (
        [name for name in params["classes"].split(",") if name.strip()]
        if "classes" in params
        else None
    )
    common = dict(
        conf_threshold=float(params.get("conf", 0.35)),
        iou_threshold=float(params.get("iou", 0.45)),
        imgsz=int(params.get("imgsz", 640)),
        class_filter=class_filter,
    )
    backend = params.get("backend", "auto").lower()
    if backend == "auto":
        try:
            import onnxruntime  # noqa: F401
            backend = "onnxruntime"
        except ImportError:
            backend = "opencv"
    if backend in ("onnxruntime", "ort"):
        threads = int(params["threads"]) if "threads" in params else None
        return YoloOrtDetector(target, threads=threads, **common)
    if backend in ("opencv", "cv2", "dnn"):
        return YoloOnnxDetector(target, **common)
    raise DetectorError(
        f"unknown detector backend {backend!r} (auto|onnxruntime|opencv)"
    )


def decode_image(body: bytes):
    """JPEG/PNG bytes -> BGR ndarray (None when undecodable)."""

    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise DetectorError("image decode requires opencv-python") from exc
    array = np.frombuffer(body, dtype=np.uint8)
    return cv2.imdecode(array, cv2.IMREAD_COLOR)


def project_ground(
    center_px: tuple[float, float],
    image_size: tuple[int, int],
    *,
    agl_m: float,
    hfov_deg: float = 70.0,
    heading_deg: float | None = None,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    cam_yaw_offset_deg: float = 0.0,
) -> tuple[float, float]:
    """Pixel in a belly camera frame -> ground offset (north_m, east_m).

    Full ray cast, not a flat nadir map: the pixel becomes a direction in
    the CAMERA frame, is rotated into the body frame by the mounting yaw
    offset, then into local NED by the vehicle's roll/pitch/yaw at
    capture, and intersected with the ground plane `agl_m` below. This is
    what accounts for the vehicle NOT being level while translating — a
    multirotor doing 3 m/s pitches several degrees nose-down, which at
    6 m AGL displaces a "nadir" footprint by half a metre or more.

    Conventions (ArduPilot ATTITUDE): pitch positive = nose up, roll
    positive = right wing down, heading = compass yaw. Camera: image top
    = vehicle nose (before cam_yaw_offset_deg), square pixels, boresight
    through the belly. Degrades gracefully: all-zero angles reproduce the
    old flat nadir projection exactly.
    """

    width, height = image_size
    if width <= 0 or agl_m <= 0:
        return (0.0, 0.0)
    # pixel -> tangent-plane direction components in the CAMERA frame
    # (right, up-image), normalized so boresight has unit down component
    tan_half = math.tan(math.radians(hfov_deg) / 2.0)
    cam_right = (center_px[0] - width / 2.0) / (width / 2.0) * tan_half
    cam_fwd = (height / 2.0 - center_px[1]) / (width / 2.0) * tan_half
    # camera -> body: mounting rotation about the boresight (clockwise
    # from "image top = nose")
    mount = math.radians(cam_yaw_offset_deg)
    bx = cam_fwd * math.cos(mount) - cam_right * math.sin(mount)  # forward
    by = cam_fwd * math.sin(mount) + cam_right * math.cos(mount)  # right
    bz = 1.0                                                      # down
    # body -> NED: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    yaw = math.radians(heading_deg or 0.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rx(roll)
    x1, y1, z1 = bx, cr * by - sr * bz, sr * by + cr * bz
    # Ry(pitch)
    x2, y2, z2 = cp * x1 + sp * z1, y1, -sp * x1 + cp * z1
    # Rz(yaw)
    dn, de, dd = cy * x2 - sy * y2, sy * x2 + cy * y2, z2
    if dd < 0.15:
        # ray near/above the horizon (extreme tilt + frame edge): no
        # trustworthy ground intersection — clamp rather than launch the
        # estimate kilometres away
        dd = 0.15
    scale = agl_m / dd
    return (dn * scale, de * scale)


def project_nadir(
    center_px: tuple[float, float],
    image_size: tuple[int, int],
    *,
    agl_m: float,
    hfov_deg: float = 70.0,
    heading_deg: float | None = None,
) -> tuple[float, float]:
    """Legacy flat-nadir projection: project_ground with a level vehicle."""

    return project_ground(
        center_px,
        image_size,
        agl_m=agl_m,
        hfov_deg=hfov_deg,
        heading_deg=heading_deg,
    )


def offset_latlon(
    lat_deg: float, lon_deg: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Small-offset flat-earth shift of a lat/lon by metres."""

    dlat = north_m / 111_111.0
    denom = 111_111.0 * max(math.cos(math.radians(lat_deg)), 1e-6)
    return (lat_deg + dlat, lon_deg + east_m / denom)
