"""Detector decode geometry and class-alias reporting.

Run: python3 -m unittest test_detector  (from this directory)
"""

from __future__ import annotations

import os
import unittest

import numpy as np

import detector as D

MODEL = os.path.join(os.path.dirname(__file__), "models", "yolov8n.onnx")
KITE = D.COCO_NAMES.index("kite")
FRISBEE = D.COCO_NAMES.index("frisbee")


class FakeYolo(D._YoloDetectorBase):
    """Backend returning a canned YOLOv8 head output (1, 84, anchors)."""

    def __init__(self, rows, **kwargs):
        super().__init__("fake.onnx", **kwargs)
        self.rows = rows  # [(cx, cy, w, h, class_id, score)] in network px

    def _infer(self, blob):
        assert blob.shape == (1, 3, self.input_wh[1], self.input_wh[0])
        # decode tells (channels, anchors) apart by size: anchors > 84
        out = np.zeros((1, 4 + len(D.COCO_NAMES), 128), dtype=np.float32)
        for anchor, (cx, cy, w, h, cls, score) in enumerate(self.rows):
            out[0, :4, anchor] = (cx, cy, w, h)
            out[0, 4 + cls, anchor] = score
        return out


class NonSquareDecodeTest(unittest.TestCase):
    # a 40x60 box at (100, 50) in the ORIGINAL frame
    BOX = (100, 50, 40, 60)

    def check(self, frame_wh, input_wh, net_box):
        det = FakeYolo([(*net_box, KITE, 0.8)], imgsz=input_wh, conf_threshold=0.3)
        frame = np.zeros((frame_wh[1], frame_wh[0], 3), dtype=np.uint8)
        hits = det.detect(frame)
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0].box_xywh, self.BOX)

    def test_640x400_frame_in_640x416_input(self):
        # scale 1, 8 px letterbox band on top
        self.check((640, 400), (640, 416), (120, 50 + 30 + 8, 40, 60))

    def test_1280x800_frame_in_640x416_input(self):
        # scale 0.5, 8 px band on top
        self.check((1280, 800), (640, 416), (60, 25 + 15 + 8, 20, 30))

    def test_640x400_frame_in_square_input(self):
        # scale 1, 120 px band on top (the deployed 640x640 model)
        self.check((640, 400), (640, 640), (120, 50 + 30 + 120, 40, 60))

    def test_letterbox_offsets_match_the_canvas(self):
        det = FakeYolo([], imgsz=(640, 416))
        frame = np.full((400, 640, 3), 255, dtype=np.uint8)
        blob, scale, left, top = det._preprocess(frame)
        self.assertEqual((scale, left, top), (1.0, 0.0, 8.0))
        # the image starts exactly at row `top`; above it is padding (114)
        self.assertAlmostEqual(float(blob[0, 0, 7, 0]), 114 / 255, places=5)
        self.assertAlmostEqual(float(blob[0, 0, 8, 0]), 1.0, places=5)


class AliasReportingTest(unittest.TestCase):
    def detect(self, classes):
        det = FakeYolo(
            [(120, 88, 40, 60, KITE, 0.8), (400, 200, 30, 30, FRISBEE, 0.6)],
            imgsz=(640, 416),
            conf_threshold=0.3,
            class_filter=D.parse_class_filter(classes),
        )
        hits = det.detect(np.zeros((400, 640, 3), dtype=np.uint8))
        return hits, det.last_all_detections

    def test_alias_reports_target_and_keeps_raw_label(self):
        hits, seen = self.detect("tennis racket|kite")
        self.assertEqual([h.label for h in hits], ["tennis racket"])
        self.assertEqual(hits[0].raw_label, "kite")
        self.assertEqual(
            hits[0].as_dict()["raw_label"], "kite",
        )
        # every above-threshold detection stays visible with its own class
        self.assertEqual(
            sorted((d.label, d.raw_label) for d in seen),
            [("frisbee", "frisbee"), ("tennis racket", "kite")],
        )
        self.assertIn(hits[0], seen)  # the provider colours hits by identity

    def test_without_alias_kite_is_not_the_target(self):
        hits, seen = self.detect("tennis racket")
        self.assertEqual(hits, [])
        self.assertEqual(len(seen), 2)

    def test_no_filter_reports_raw_classes(self):
        hits, _ = self.detect("")
        self.assertEqual([h.label for h in hits], ["kite", "frisbee"])
        self.assertNotIn("raw_label", hits[0].as_dict())

    def test_parse_class_filter(self):
        self.assertEqual(
            D.parse_class_filter(" Tennis Racket | kite , person"),
            {"tennis racket": "Tennis Racket", "kite": "Tennis Racket",
             "person": "person"},
        )
        self.assertIsNone(D.parse_class_filter(None))
        self.assertIsNone(D.parse_class_filter(" , "))


@unittest.skipUnless(os.path.exists(MODEL), "packaged model missing")
class OrtInputShapeTest(unittest.TestCase):
    def setUp(self):
        try:
            import onnxruntime  # noqa: F401
        except ImportError:
            self.skipTest("onnxruntime not installed")

    def test_static_model_input_defines_letterbox(self):
        det = D.detector_from_spec(f"yolo:{MODEL}?backend=onnxruntime")
        self.assertEqual(det.input_wh, (640, 640))

    def test_contradicting_imgsz_fails_loudly(self):
        with self.assertRaises(D.DetectorError):
            D.detector_from_spec(f"yolo:{MODEL}?backend=onnxruntime&imgsz=640x416")


if __name__ == "__main__":
    unittest.main()
