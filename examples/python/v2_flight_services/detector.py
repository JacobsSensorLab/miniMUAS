"""Real object detection for the GCS: YOLOv8 ONNX, CPU-optimized for ARM.

Replaces the v2 detection stub with actual vision. A `yolo:` detector spec
selects this path on the GCS provider:

    --detector "yolo:/path/yolov8n.onnx?backend=onnxruntime&conf=0.3&classes=tennis racket|kite"

The model is a stock COCO-trained YOLOv8 export (no training needed for
the demo target — "tennis racket" is COCO class 38), produced once with:

    pip install ultralytics
    yolo export model=yolov8n.pt format=onnx imgsz=640 opset=12

Seen from overhead at 5-8 m AGL, that model labels the racket "kite", never
"tennis racket": on the 167 frames the GCS saved on 2026-07-15, the racket
is in 21; "tennis racket" fires on 0 of them, "kite" (conf >= 0.3) on 13,
and on 0 of the 146 without it. Hence the `tennis racket|kite` alias,
reported as "tennis racket" with the model's class kept in raw_label.

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
  * input size: inference cost scales with input pixels. The model's own
    static input defines the letterbox (onnxruntime reads it; the opencv
    backend takes imgsz=WxH). A rectangular 640x416 export of the same
    weights fits the 640x400 search frames with 35% fewer pixels, but on
    the field frames above it found the racket in 10/21 (vs 13/21) with 2
    false frames (vs 0), so the 640x640 export stays.
  * INT8: measured, not used. Static QDQ quantization (onnxruntime,
    per-channel, calibrated on the field frames) loses the racket: 0/21
    hits, or 9/21 with the detection head kept fp32, vs 13/21 in fp32.

Reference (GCS, 2026-10-12): yolov8n 640 via onnxruntime 1.21 (the nixpkgs
build ships only the CPU EP, no XNNPACK) is ~1.0 s/frame with the services
idle and 1.4-2.0 s with one or two cores busy elsewhere; JPEG decode and
letterbox add 5-20 ms. Detection runs per search frame on its own worker
(see run_gcs_provider), not on continuous video.

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
    label: str  # reported label: the target name when raw_label is an alias
    confidence: float
    box_xywh: tuple[int, int, int, int]  # top-left x, y, width, height (px)
    raw_label: str = ""  # the model's own class name (== label unless aliased)

    @property
    def center_px(self) -> tuple[float, float]:
        x, y, w, h = self.box_xywh
        return (x + w / 2.0, y + h / 2.0)

    def as_dict(self) -> dict[str, Any]:
        out = {
            "label": self.label,
            "confidence": round(self.confidence, 4),
            "box_xywh": list(self.box_xywh),
        }
        if self.raw_label and self.raw_label != self.label:
            out["raw_label"] = self.raw_label
        return out


def parse_class_filter(spec: str | None) -> dict[str, str] | None:
    """`classes=` value -> {model class name (lowercase): reported label}.

    Comma separates targets; within a target, `|` lists the model classes
    that count as it, the first name being the label reported. The stock
    COCO yolov8n sees a tennis racket from overhead as "kite", so
    `tennis racket|kite` accepts either class and reports "tennis racket".
    None (no filter) when the spec is empty.
    """

    mapping: dict[str, str] = {}
    for target in (spec or "").split(","):
        names = [name.strip() for name in target.split("|") if name.strip()]
        for name in names:
            mapping[name.lower()] = names[0]
    return mapping or None


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
        imgsz: int | tuple[int, int] | None = None,
        class_filter: dict[str, str] | None = None,
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
        # network input (width, height). None = not configured: 640x640, or
        # whatever static shape a backend reads from the model itself (see
        # YoloOrtDetector); an explicit value is checked against that shape.
        self._imgsz_explicit = imgsz is not None
        if imgsz is None:
            imgsz = 640
        self.input_wh = (
            (int(imgsz), int(imgsz)) if isinstance(imgsz, int)
            else (int(imgsz[0]), int(imgsz[1]))
        )
        # Per-thread, NOT a shared instance attribute: the provider reads
        # last_all_detections right after its own detect() to draw the annotated
        # frame + fill the response's all_classes. A shared attribute would hand
        # a thread the *other* frame's detections — the stuck/misassociated boxes
        # seen in the saved field frames. Read from the same thread that set it.
        self._tls = threading.local()
        # {model class (lowercase): reported label}; see parse_class_filter
        self.class_filter = class_filter or None

    @property
    def last_all_detections(self) -> list[Detection]:
        return getattr(self._tls, "all_detections", [])

    def _preprocess(self, image_bgr):
        """Letterbox into the network input (W×H, not necessarily square)
        -> normalized NCHW blob + geometry.

        Returns the same (1,3,H,W) float32 RGB/255 tensor whether the
        backend is cv2.dnn or onnxruntime (both expect exactly this)."""
        cv2, np = self._cv2, self._np
        in_w, in_h = self.input_wh
        height, width = image_bgr.shape[:2]
        scale = min(in_w / width, in_h / height)
        new_w, new_h = int(round(width * scale)), int(round(height * scale))
        pad_x, pad_y = (in_w - new_w) / 2.0, (in_h - new_h) / 2.0
        resized = (
            image_bgr if (new_w, new_h) == (width, height)
            else cv2.resize(image_bgr, (new_w, new_h))
        )
        canvas = np.full((in_h, in_w, 3), 114, dtype=np.uint8)
        top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
        canvas[top:top + new_h, left:left + new_w] = resized
        blob = cv2.dnn.blobFromImage(
            canvas, 1.0 / 255.0, (in_w, in_h), swapRB=True
        )
        # decode maps canvas px back with the offsets actually used
        return blob, scale, float(left), float(top)

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
            raw_label = (
                COCO_NAMES[class_id]
                if 0 <= class_id < len(COCO_NAMES)
                else f"class-{class_id}"
            )
            target = (
                self.class_filter.get(raw_label.lower())
                if self.class_filter
                else raw_label
            )
            x, y, w, h = boxes[index]
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            detection = Detection(
                label=target or raw_label,
                confidence=float(confidences[index]),
                box_xywh=(x, y, w, h),
                raw_label=raw_label,
            )
            all_detections.append(detection)
            if target is None:
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
        targets: dict[str, list[str]] = {}
        for raw, label in (self.class_filter or {}).items():
            targets.setdefault(label, []).append(raw)
        return {
            "detector": self.backend,
            "model": self.model_path,
            "conf": self.conf_threshold,
            "input_wh": list(self.input_wh),
            # reported label -> model classes accepted as it
            "classes": targets or "all",
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
            in_w, in_h = self.input_wh
            warm = np.zeros((1, 3, in_h, in_w), dtype=np.float32)
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
        # No busy-wait between operators. The GCS shares its 4 cores with
        # the forwarder, dashboard and recorder; a spinning pool burns cores
        # those need, and every parallel op then waits on whichever worker
        # got preempted. Measured on the GCS (yolov8n 640, 4 threads):
        # 1170 -> 971 ms with the services idle, 1816 -> 1436 ms with one
        # core busy elsewhere, 2508 -> 1955 ms with two. 3 threads measured
        # the same as 4 without spinning, so threads stays at core count.
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
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
        model_input = self._sess.get_inputs()[0]
        self._input_name = model_input.name
        self._providers = self._sess.get_providers()
        self._threads = opts.intra_op_num_threads
        # A static model input (a rectangular 640x416 export, say) defines
        # the letterbox; an explicit imgsz= that contradicts it is a config
        # error worth failing on, not a silent mis-scaled decode.
        dims = list(model_input.shape)[-2:]
        if len(dims) == 2 and all(isinstance(d, int) and d > 0 for d in dims):
            model_wh = (dims[1], dims[0])
            if self._imgsz_explicit and self.input_wh != model_wh:
                raise DetectorError(
                    f"imgsz {self.input_wh[0]}x{self.input_wh[1]} contradicts "
                    f"the model's static input {model_wh[0]}x{model_wh[1]}"
                )
            self.input_wh = model_wh
        try:
            in_w, in_h = self.input_wh
            warm = self._np.zeros((1, 3, in_h, in_w), dtype=self._np.float32)
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
        yolo:<model.onnx>[?backend=auto&conf=0.35&iou=0.45&imgsz=640&threads=N&classes=a|alias,b]

    backend: auto (default; onnxruntime if importable, else opencv),
    onnxruntime, or opencv. imgsz: N (square) or WxH (e.g. 640x416).
    onnxruntime reads a static input size from the model itself; the opencv
    backend needs imgsz= for anything but a 640x640 model. classes: see
    parse_class_filter (`tennis racket|kite` reports kite as tennis racket).
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
    imgsz = None
    if "imgsz" in params:
        size = params["imgsz"].lower()
        imgsz = (
            tuple(int(v) for v in size.split("x", 1)) if "x" in size
            else int(size)
        )
    common = dict(
        conf_threshold=float(params.get("conf", 0.35)),
        iou_threshold=float(params.get("iou", 0.45)),
        imgsz=imgsz,
        class_filter=parse_class_filter(params.get("classes")),
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


def ground_to_pixel(
    north_m: float,
    east_m: float,
    image_size: tuple[int, int],
    *,
    agl_m: float,
    hfov_deg: float = 70.0,
    heading_deg: float | None = None,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    cam_yaw_offset_deg: float = 0.0,
) -> tuple[float, float] | None:
    """Inverse of project_ground: a ground offset from the capture position
    (north_m, east_m, `agl_m` below) -> the pixel it images at.

    The pixel may lie outside the image (caller checks bounds); None when
    the point is not below the camera's image plane at all. Same
    conventions and rotation chain as project_ground, inverted step by step,
    so a round trip reproduces the pixel wherever project_ground did not
    clamp a near-horizon ray.
    """

    width, height = image_size
    if width <= 0 or agl_m <= 0:
        return None
    tan_half = math.tan(math.radians(hfov_deg) / 2.0)
    yaw = math.radians(heading_deg or 0.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    # NED -> body: Rx(roll)^T @ Ry(pitch)^T @ Rz(yaw)^T
    x2, y2, z2 = cy * north_m + sy * east_m, -sy * north_m + cy * east_m, agl_m
    x1, y1, z1 = cp * x2 - sp * z2, y2, sp * x2 + cp * z2
    bx, by, bz = x1, cr * y1 + sr * z1, -sr * y1 + cr * z1
    if bz <= 1e-6:
        return None
    bx, by = bx / bz, by / bz
    # body -> camera: undo the mounting rotation about the boresight
    mount = math.radians(cam_yaw_offset_deg)
    cam_fwd = bx * math.cos(mount) + by * math.sin(mount)
    cam_right = -bx * math.sin(mount) + by * math.cos(mount)
    return (
        width / 2.0 + cam_right / tan_half * (width / 2.0),
        height / 2.0 - cam_fwd / tan_half * (width / 2.0),
    )


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


def simulated_target_in_frame(
    anomalies: list[dict] | None,
    *,
    cap_lat: float,
    cap_lon: float,
    agl_m: float,
    image_size: tuple[int, int],
    hfov_deg: float = 70.0,
    heading_deg: float | None = None,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    cam_yaw_offset_deg: float = 0.0,
) -> tuple[dict, tuple[float, float], float] | None:
    """The operator-placed sim target this captured frame would show.

    The dashboard's placed anomalies are ground truth; a VISUAL one counts as
    seen when its position projects inside this frame's image, using the
    frame's own capture pose and the same camera model the real detections
    are geo-projected with (so the footprint is the true 640x400 rectangle
    rotated by heading, not a circle). Returns (anomaly, pixel, ground offset
    from the capture nadir in metres) for the one nearest the image centre,
    or None. Audio anomalies are never "seen" by a camera.
    """

    width, height = image_size
    best = None
    for anomaly in anomalies or []:
        if str(anomaly.get("kind", "visual")) != "visual":
            continue
        try:
            a_lat = float(anomaly["lat_deg"])
            a_lon = float(anomaly["lon_deg"])
        except (KeyError, TypeError, ValueError):
            continue
        north_m = (a_lat - cap_lat) * 111_111.0
        east_m = (a_lon - cap_lon) * 111_111.0 * max(
            math.cos(math.radians(cap_lat)), 1e-6
        )
        pixel = ground_to_pixel(
            north_m, east_m, (width, height),
            agl_m=agl_m, hfov_deg=hfov_deg, heading_deg=heading_deg,
            pitch_deg=pitch_deg, roll_deg=roll_deg,
            cam_yaw_offset_deg=cam_yaw_offset_deg,
        )
        if pixel is None or not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            continue
        off_center = math.hypot(pixel[0] - width / 2.0, pixel[1] - height / 2.0)
        if best is None or off_center < best[0]:
            best = (off_center, anomaly, pixel, math.hypot(north_m, east_m))
    if best is None:
        return None
    return best[1], best[2], best[3]
