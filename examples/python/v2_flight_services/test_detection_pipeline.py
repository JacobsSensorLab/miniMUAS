"""Search-frame detection pipeline: pacing, settlement, sim targets, naming.

Each case pins a failure seen on the 2026-10-13 field day (see the comments
at the code under test).

Run: python3 -m unittest test_detection_pipeline  (from this directory)
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
import types
import unittest
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeoutError
from types import SimpleNamespace
from unittest import mock

import contracts
import detector as D
import run_dashboard
import run_gcs_provider
from contracts import (
    DetectionRequest,
    DetectionResponse,
    GeoPoint,
    InvestigatePointRequest,
    SearchStatus,
    mission_frame_name,
)
from dataplane import build_frame_bytes

MODEL = os.path.join(os.path.dirname(__file__), "models", "yolov8n.onnx")
LAT, LON = 35.1210834, -89.9349954


def offset(lat, lon, north_m, east_m):
    return D.offset_latlon(lat, lon, north_m, east_m)


# ---------------------------------------------------------------- geometry --

class GroundToPixelTest(unittest.TestCase):
    def test_inverts_project_ground(self):
        size = (640, 400)
        poses = [
            dict(heading_deg=0.0),
            dict(heading_deg=181.6, pitch_deg=-8.0, roll_deg=3.0),
            dict(heading_deg=37.0, pitch_deg=5.0, roll_deg=-4.0,
                 cam_yaw_offset_deg=12.0),
        ]
        for pose in poses:
            for px in [(320, 200), (10, 10), (600, 182), (100, 390)]:
                ne = D.project_ground(px, size, agl_m=6.7, hfov_deg=70.0, **pose)
                back = D.ground_to_pixel(*ne, size, agl_m=6.7, hfov_deg=70.0, **pose)
                self.assertAlmostEqual(back[0], px[0], places=6, msg=pose)
                self.assertAlmostEqual(back[1], px[1], places=6, msg=pose)


class SimulatedTargetInFrameTest(unittest.TestCase):
    """The footprint is the frame's real rectangle, rotated by heading."""

    def find(self, north_m, east_m, *, heading=0.0, kind="visual", extra=()):
        lat, lon = offset(LAT, LON, north_m, east_m)
        anomalies = [{"id": "a", "kind": kind, "lat_deg": lat, "lon_deg": lon,
                      "signature": "red"}, *extra]
        return D.simulated_target_in_frame(
            anomalies, cap_lat=LAT, cap_lon=LON, agl_m=7.0,
            image_size=(640, 400), hfov_deg=70.0, heading_deg=heading,
        )

    def test_wide_axis_in_frame_short_axis_not(self):
        # 7 m AGL, 70 deg HFOV: half-width 4.9 m, half-height 3.06 m
        self.assertIsNotNone(self.find(0.0, 4.0))
        # 4 m ahead is outside a 640x400 frame (a half-width circle took it)
        self.assertIsNone(self.find(4.0, 0.0))
        # turned 90 deg, the long image axis points north
        self.assertIsNotNone(self.find(4.0, 0.0, heading=90.0))

    def test_pixel_and_offset(self):
        anomaly, (px, py), off = self.find(0.0, 2.0)  # 2 m right of nadir
        self.assertEqual(anomaly["id"], "a")
        self.assertGreater(px, 320)
        self.assertAlmostEqual(py, 200, delta=0.5)
        self.assertAlmostEqual(off, 2.0, delta=0.01)

    def test_audio_target_is_never_seen(self):
        self.assertIsNone(self.find(0.0, 1.0, kind="audio"))

    def test_nearest_to_center_wins(self):
        lat, lon = offset(LAT, LON, 0.0, 0.5)
        near = {"id": "near", "kind": "visual", "lat_deg": lat, "lon_deg": lon}
        anomaly, _, _ = self.find(0.0, 3.0, extra=[near])
        self.assertEqual(anomaly["id"], "near")


# --------------------------------------------------------------- contracts --

class ContractsTest(unittest.TestCase):
    def test_mission_data_lives_under_the_producer(self):
        # routable by the per-node /muas/v2/<id> route on both wifi cells
        name = mission_frame_name("mission-1", "wuas-01", "bottom", 5, 3)
        self.assertTrue(name.startswith("/muas/v2/wuas-01/mission/mission-1/"))
        self.assertTrue(contracts.mission_sensor_name(
            "mission-1", "iuas-02", "mic", "audio", 5, 1,
        ).startswith("/muas/v2/iuas-02/mission/mission-1/"))

    def test_simulated_flag_round_trips_and_defaults_false(self):
        det = DetectionResponse("m", "SIM-red", 0.93, GeoPoint(1.0, 2.0, 0.0),
                                simulated=True)
        self.assertTrue(DetectionResponse.from_bytes(det.to_bytes()).simulated)
        legacy = json.loads(det.to_bytes())
        del legacy["simulated"]
        self.assertFalse(
            DetectionResponse.from_bytes(json.dumps(legacy).encode()).simulated
        )
        inv = InvestigatePointRequest("m", "x", GeoPoint(1, 2, 0), 8, 6, 6, 1,
                                      simulated=True)
        self.assertTrue(InvestigatePointRequest.from_bytes(inv.to_bytes()).simulated)


# ------------------------------------------------------- provider admission --

class SubmitAdmittedTest(unittest.TestCase):
    def test_slot_held_until_work_ends_not_until_waiter_gives_up(self):
        pool = ThreadPoolExecutor(max_workers=1)
        gate = threading.BoundedSemaphore(1)
        release = threading.Event()
        try:
            first = run_gcs_provider.submit_admitted(pool, gate, release.wait, 5)
            self.assertIsNotNone(first)
            with self.assertRaises(FuturesTimeoutError):
                first.result(timeout=0.05)          # the handler gives up
            # ... but the worker is still busy: no second frame behind it
            self.assertIsNone(
                run_gcs_provider.submit_admitted(pool, gate, lambda: 1)
            )
            release.set()
            first.result(timeout=2)
            deadline = time.monotonic() + 2
            second = None
            while second is None and time.monotonic() < deadline:
                second = run_gcs_provider.submit_admitted(pool, gate, lambda: 1)
            self.assertEqual(second.result(timeout=2), 1)
        finally:
            release.set()
            pool.shutdown(wait=True)


# ------------------------------------------------------- provider (handler) --

class FakeServiceResponse:
    def __init__(self, status=True, error="", payload=b""):
        self.status, self.error, self.payload = status, error, payload


class FakeProvider:
    def __init__(self, **_kw):
        self.handlers = {}

    def ack_handler(self, service):
        return lambda fn: fn

    def handler(self, service):
        def register(fn):
            self.handlers[service] = fn
            return fn
        return register

    def set_use_tokens(self, _on):
        pass

    def run(self, _service):
        return 0


def run_provider(argv):
    """main() of the GCS provider against a fake NDNSF; returns
    (detect handler, captured events)."""
    made = {}

    def make_provider(**kw):
        made["p"] = FakeProvider(**kw)
        return made["p"]

    fake = types.ModuleType("ndnsf")
    fake.AckDecision = lambda **kw: SimpleNamespace(**kw)
    fake.ServiceProvider = make_provider
    fake.ServiceResponse = FakeServiceResponse
    events = []
    with mock.patch.dict(sys.modules, {"ndnsf": fake}), \
            mock.patch.object(run_gcs_provider, "add_ndnsf_path"), \
            mock.patch.object(run_gcs_provider, "start_journal_publisher"), \
            mock.patch.object(run_gcs_provider, "set_runtime"), \
            mock.patch.object(run_gcs_provider, "print_json",
                              lambda e, **f: events.append((e, f))), \
            mock.patch.object(sys, "argv", ["gcs", *argv,
                                            "--log-dir", "", "--frames-dir", "",
                                            "--no-nfd-metrics"]):
        run_gcs_provider.main()
    return made["p"].handlers[contracts.gcs_detection_service()], events


def frame_payload(body: bytes, *, width=640, height=400, heading=0.0, agl=7.0,
                  kind="image/jpeg"):
    return build_frame_bytes(
        body, mission_id="m", vehicle_id="wuas-01", sensor_id="bottom",
        gps_time_ns=time.time_ns(), kind=kind, width=width, height=height,
        metadata={"lat_deg": f"{LAT:.7f}", "lon_deg": f"{LON:.7f}",
                  "agl_m": f"{agl:.2f}", "heading_deg": f"{heading:.1f}"},
    )


def detect_payload(anomalies):
    frame = contracts.FrameRef(
        data_name=mission_frame_name("m", "wuas-01", "bottom", 1, 1),
        gps_time_ns=1, seq=1, camera_id="bottom",
        pose=contracts.Pose(position=GeoPoint(0.0, 0.0, 0.0), yaw_deg=0.0),
    )
    return DetectionRequest("m", frame, anomalies=anomalies).to_bytes()


def sim_anomaly(north_m, east_m):
    lat, lon = offset(LAT, LON, north_m, east_m)
    return {"id": "anom-1", "kind": "visual", "lat_deg": lat, "lon_deg": lon,
            "signature": "red", "size_m": 4.0}


@unittest.skipUnless(os.path.exists(MODEL), "packaged model missing")
class YoloProviderSimTargetTest(unittest.TestCase):
    """The field detector (yolo) honours placed sim targets. It ignored them
    on 2026-10-13: only the stub branch read request.anomalies."""

    @classmethod
    def setUpClass(cls):
        try:
            import cv2  # noqa: F401
            import onnxruntime  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest(f"needs cv2 + onnxruntime: {exc}")
        handler, cls.events = run_provider(
            ["--detector", f"yolo:{MODEL}?conf=0.35&classes=tennis racket|kite"]
        )
        cls.handler = staticmethod(handler)
        import cv2
        import numpy as np
        grass = np.full((400, 640, 3), (60, 120, 70), dtype=np.uint8)
        cls.jpeg = cv2.imencode(".jpg", grass)[1].tobytes()

    def detect(self, anomalies):
        with mock.patch.object(run_gcs_provider, "fetch_segmented",
                               lambda *a, **k: frame_payload(self.jpeg)):
            return self.handler(detect_payload(anomalies))

    def test_placed_target_in_frame_is_a_simulated_detection(self):
        raw = self.detect([sim_anomaly(0.0, 2.0)])
        det = DetectionResponse.from_bytes(raw)
        self.assertTrue(det.simulated)
        self.assertEqual(det.object_id, "SIM-red")
        self.assertAlmostEqual(det.estimate.lat_deg, sim_anomaly(0, 2)["lat_deg"])

    def test_placed_target_outside_frame_is_a_miss(self):
        resp = self.detect([sim_anomaly(10.0, 0.0)])
        self.assertFalse(resp.status)
        self.assertEqual(resp.error, "no-detection")


class StubProviderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        handler, cls.events = run_provider([])
        cls.handler = staticmethod(handler)
        cls.payload = frame_payload(bytes(640 * 400), kind="synthetic")

    def detect(self, anomalies):
        with mock.patch.object(run_gcs_provider, "fetch_segmented",
                               lambda *a, **k: self.payload):
            return self.handler(detect_payload(anomalies))

    def test_every_stub_answer_is_flagged_simulated(self):
        self.assertTrue(DetectionResponse.from_bytes(self.detect([])).simulated)
        self.assertTrue(
            DetectionResponse.from_bytes(self.detect([sim_anomaly(1, 1)])).simulated
        )
        self.assertFalse(self.detect([sim_anomaly(0, 30)]).status)

    def test_fetch_failure_is_an_answer(self):
        def fail(*_a, **_k):
            raise RuntimeError("Timeout exceeded")
        with mock.patch.object(run_gcs_provider, "fetch_segmented", fail):
            resp = self.handler(detect_payload([]))
        self.assertFalse(resp.status)
        self.assertIn("frame fetch failed", resp.error)


# --------------------------------------------------------------- dashboard --

class FakeUser:
    """Holds every request until the test answers it."""

    def __init__(self):
        self.requests = []
        self.lock = threading.Lock()

    def request_service_targeted_async(
        self, provider, service, payload, *, on_response, on_timeout, timeout_ms
    ):
        with self.lock:
            self.requests.append(SimpleNamespace(
                service=service, payload=json.loads(payload),
                on_response=on_response, on_timeout=on_timeout,
            ))

    request_service_async = None  # targeted mode only

    def detects(self):
        with self.lock:
            return [r for r in self.requests if r.service.endswith("detect-object")]

    def investigations(self):
        with self.lock:
            return [r for r in self.requests if r.service.endswith("/investigate")]


def frames(n, mission="m1"):
    return [mission_frame_name(mission, "wuas-01", "bottom", 1_000 + i, i)
            for i in range(1, n + 1)]


def answer(req, *, status=True, error="", det: DetectionResponse | None = None):
    req.on_response(SimpleNamespace(
        status=status, error=error,
        payload=det.to_bytes() if det else b"",
    ))


class DashboardDetectionTest(unittest.TestCase):
    def setUp(self):
        args = run_dashboard.build_parser().parse_args([
            "--iuas-ids", "iuas-01,iuas-02", "--record-dir", "",
            "--detect-inflight", "2",
        ])
        self.user = FakeUser()
        self.dash = run_dashboard.Dashboard(args, self.user)
        self.dash.COMMAND_RETRY_S = 60.0       # no re-issues mid-test
        self.dash.DETECT_BUSY_RETRY_S = 0.01
        self.events = []
        self.dash.event = lambda kind, **f: self.events.append((kind, f))
        self.dash.mission.update(state="searching", mission_id="m1",
                                 params={"investigate_sensors": ["camera"]})

    def kinds(self):
        return [k for k, _ in self.events]

    def poll(self, names, mission="m1"):
        status = SearchStatus(vehicle_id="wuas-01", mission_id=mission,
                              gps_time_ns=1, last_frames=list(reversed(names)))
        with mock.patch.object(run_dashboard, "fetch_segmented",
                               lambda *a, **k: status.to_bytes()):
            self.dash._poll_search("wuas-01")

    def wait_for(self, pred, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not pred():
            if time.monotonic() > deadline:
                self.fail("condition not reached")
            time.sleep(0.005)

    def hit(self, name, *, simulated=False, east_m=0.0):
        lat, lon = offset(LAT, LON, 0.0, east_m)
        return DetectionResponse("m1", "SIM-red" if simulated else "tennis-racket",
                                 0.9, GeoPoint(lat, lon, 0.0), offset_m=1.0,
                                 simulated=simulated)

    def test_paced_to_inflight_in_capture_order(self):
        names = frames(5)
        self.poll(names)
        sent = [r.payload["frame"]["data_name"] for r in self.user.detects()]
        self.assertEqual(sent, names[:2])
        answer(self.user.detects()[0], status=False, error="no-detection")
        sent = [r.payload["frame"]["data_name"] for r in self.user.detects()]
        self.assertEqual(sent, names[:3])

    def test_busy_is_resent_not_a_miss(self):
        names = frames(1)
        self.poll(names)
        answer(self.user.detects()[0], status=False, error="detector busy")
        self.wait_for(lambda: len(self.user.detects()) == 2)
        self.assertEqual(self.user.detects()[1].payload["frame"]["data_name"],
                         names[0])
        self.assertNotIn("detect.miss", self.kinds())

    def test_previous_missions_frames_are_ignored(self):
        # the agent's status still lists the last raster's frames
        self.poll(frames(6, mission="m0"), mission="m0")
        self.assertEqual(self.user.detects(), [])

    def test_late_hit_after_search_answer_still_confirms(self):
        names = frames(2)
        self.poll(names)
        first, second = self.user.detects()
        answer(first, det=self.hit(names[0]))
        self.dash._on_search_answered("m1")
        with self.dash.targets_lock:   # what the poll loop does post-search
            self.dash.mission["final_polled"] = True
            self.dash._settle_search_locked()
        self.assertNotIn("target.unconfirmed", self.kinds())
        self.assertEqual(self.dash.mission["state"], "searching")
        answer(second, det=self.hit(names[1], east_m=1.0))
        self.assertIn("mission.target_found", self.kinds())
        self.assertNotIn("target.unconfirmed", self.kinds())

    def test_leftover_candidate_settles_once_backlog_drains(self):
        names = frames(2)
        self.poll(names)
        first, second = self.user.detects()
        answer(first, det=self.hit(names[0]))
        self.dash._on_search_answered("m1")
        with self.dash.targets_lock:
            self.dash.mission["final_polled"] = True
            self.dash._settle_search_locked()
        answer(second, status=False, error="no-detection")
        self.assertEqual(self.kinds().count("target.unconfirmed"), 1)
        self.assertEqual(self.dash.mission["state"], "done")

    def test_simulated_hits_never_merge_with_real_ones(self):
        names = frames(3)
        self.dash._on_detect_hit(self.hit(names[0], simulated=True), names[0])
        self.dash._on_detect_hit(self.hit(names[1]), names[1])
        self.dash._on_detect_hit(self.hit(names[2], simulated=True), names[2])
        targets = self.dash.mission["targets"]
        self.assertEqual(len(targets), 1)
        self.assertTrue(targets[0]["simulated"])
        self.assertEqual([c["simulated"] for c in self.dash.candidates], [False])
        found = [f for k, f in self.events if k == "mission.target_found"]
        self.assertTrue(found[0]["simulated"])
        # ... and the IUAS is told the object is simulated
        inv = self.user.investigations()
        self.assertEqual(len(inv), 1)
        self.assertTrue(inv[0].payload["simulated"])


if __name__ == "__main__":
    unittest.main()
