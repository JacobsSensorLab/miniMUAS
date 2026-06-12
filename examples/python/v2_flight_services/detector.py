"""Real object detection for the GCS: YOLOv8 ONNX via OpenCV's DNN module.

Replaces the v2 detection stub with actual vision. A `yolo:` detector spec
selects this path on the GCS provider:

    --detector "yolo:/path/yolov8n.onnx?conf=0.35&classes=tennis racket"

The model is a stock COCO-trained YOLOv8 export (no training needed for
the demo target — "tennis racket" is COCO class 38), produced once with:

    pip install ultralytics
    yolo export model=yolov8n.pt format=onnx imgsz=640 opset=12

On an Odroid C4 (4×A55), yolov8n at 640px runs in roughly 1.5–3 s per
frame on CPU through cv2.dnn — irrelevant for detect-per-search-frame,
unsuitable for continuous video (which the design doesn't ask of it).

Also hosts the nadir geo-projection: a pixel detection in a downward
camera frame becomes a ground offset from the capture position via
ground-sampling distance (2·AGL·tan(HFOV/2)/width). Heading, when known,
rotates the camera-frame offset into north/east; without it the camera
is assumed north-up — fine for bench validation, an honest approximation
in the field until pose carries yaw.
"""

from __future__ import annotations

import math
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


class YoloOnnxDetector:
    """COCO YOLOv8 ONNX inference through cv2.dnn (CPU)."""

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
        self.last_all_detections: list[Detection] = []
        self.class_filter = (
            {name.strip().lower() for name in class_filter}
            if class_filter
            else None
        )
        try:
            self._net = cv2.dnn.readNetFromONNX(model_path)
        except Exception as exc:
            raise DetectorError(f"could not load ONNX model {model_path!r}: {exc}")

    def detect(self, image_bgr) -> list[Detection]:
        cv2, np = self._cv2, self._np
        height, width = image_bgr.shape[:2]

        # letterbox to imgsz×imgsz, preserving aspect
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
        self._net.setInput(blob)
        output = self._net.forward()

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
        self.last_all_detections = all_detections
        return detections

    def describe(self) -> dict[str, Any]:
        return {
            "detector": "yolo-onnx",
            "model": self.model_path,
            "conf": self.conf_threshold,
            "imgsz": self.imgsz,
            "classes": sorted(self.class_filter) if self.class_filter else "all",
        }


def detector_from_spec(spec: str | None):
    """Build a detector from a `--detector` spec; None for the stub.

        stub                          (default; offset-based fake detection)
        yolo:<model.onnx>[?conf=0.35&iou=0.45&imgsz=640&classes=a,b]
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
    return YoloOnnxDetector(
        target,
        conf_threshold=float(params.get("conf", 0.35)),
        iou_threshold=float(params.get("iou", 0.45)),
        imgsz=int(params.get("imgsz", 640)),
        class_filter=class_filter,
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


def project_nadir(
    center_px: tuple[float, float],
    image_size: tuple[int, int],
    *,
    agl_m: float,
    hfov_deg: float = 70.0,
    heading_deg: float | None = None,
) -> tuple[float, float]:
    """Pixel offset in a downward-facing frame -> (north_m, east_m).

    Ground sampling distance from HFOV and width; square pixels assumed.
    Camera-frame +x (right) maps to east and -y (up) to north when the
    camera's top edge faces the vehicle's nose and heading is 0/unknown;
    a known heading rotates the offset into the world frame.
    """

    width, height = image_size
    if width <= 0 or agl_m <= 0:
        return (0.0, 0.0)
    ground_width_m = 2.0 * agl_m * math.tan(math.radians(hfov_deg) / 2.0)
    meters_per_px = ground_width_m / float(width)
    right_m = (center_px[0] - width / 2.0) * meters_per_px
    forward_m = (height / 2.0 - center_px[1]) * meters_per_px
    if heading_deg is None:
        return (forward_m, right_m)
    heading = math.radians(heading_deg)
    north = forward_m * math.cos(heading) - right_m * math.sin(heading)
    east = forward_m * math.sin(heading) + right_m * math.cos(heading)
    return (north, east)


def offset_latlon(
    lat_deg: float, lon_deg: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Small-offset flat-earth shift of a lat/lon by metres."""

    dlat = north_m / 111_111.0
    denom = 111_111.0 * max(math.cos(math.radians(lat_deg)), 1e-6)
    return (lat_deg + dlat, lon_deg + east_m / denom)
