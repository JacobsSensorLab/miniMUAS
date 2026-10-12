"""Stack-switch guards: fail closed, and never move the GCS off its vehicles.

Run: python3 -m unittest test_fabric_switch  (from this directory)
"""

from __future__ import annotations

import json
import os
import stat
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import fabric
import run_dashboard
from contracts import FlightCommandResult

VEHICLES = ["wuas-01", "iuas-01", "iuas-02"]
IDLE = {"armed": False, "busy": ""}


class SwitchBlockersTest(unittest.TestCase):
    def blockers(self, samples, ages):
        return fabric.switch_blockers(VEHICLES, samples, ages, stale_s=2.5)

    def test_all_fresh_disarmed_idle_may_switch(self):
        samples = {v: dict(IDLE) for v in VEHICLES}
        self.assertEqual(self.blockers(samples, {v: 0.3 for v in VEHICLES}), {})

    def test_unknown_armed_busy_each_refuse(self):
        samples = {
            "wuas-01": {"armed": True, "busy": ""},
            "iuas-01": {"armed": False, "busy": "investigate"},
            # iuas-02: never heard from
        }
        ages = {"wuas-01": 0.2, "iuas-01": 0.2}
        got = self.blockers(samples, ages)
        self.assertEqual(set(got), set(VEHICLES))
        self.assertIn("ARMED", got["wuas-01"])
        self.assertIn("busy:investigate", got["iuas-01"])
        self.assertIn("no telemetry", got["iuas-02"])

    def test_stale_disarmed_sample_refuses(self):
        samples = {v: dict(IDLE) for v in VEHICLES}
        ages = {v: 0.2 for v in VEHICLES} | {"iuas-02": 2.6}
        self.assertEqual(list(self.blockers(samples, ages)), ["iuas-02"])

    def test_missing_armed_field_refuses(self):
        # an absent or non-boolean armed flag is not "disarmed"
        samples = {v: dict(IDLE) for v in VEHICLES}
        samples["iuas-01"] = {"busy": ""}
        samples["iuas-02"] = {"armed": None, "busy": ""}
        got = self.blockers(samples, {v: 0.2 for v in VEHICLES})
        self.assertEqual(set(got), {"iuas-01", "iuas-02"})


class VehicleRoundTest(unittest.TestCase):
    def test_gcs_only_after_every_vehicle_accepted(self):
        rnd = fabric.VehicleRound(VEHICLES)
        self.assertIsNone(rnd.record("wuas-01", "accepted"))
        self.assertIsNone(rnd.record("wuas-01", "accepted"))  # duplicate
        self.assertIsNone(rnd.record("iuas-01", "accepted"))
        self.assertEqual(rnd.record("iuas-02", "accepted"), "switch_gcs")

    def test_one_refusal_or_timeout_fails_the_round(self):
        for bad in ("rejected", "timeout", "error"):
            rnd = fabric.VehicleRound(VEHICLES)
            rnd.record("wuas-01", "accepted")
            rnd.record("iuas-01", bad, "why")
            self.assertEqual(rnd.record("iuas-02", "accepted"), "failed", bad)
            self.assertEqual(rnd.refusals(), {"iuas-01": f"{bad}: why"})


class FakeUser:
    """Answers each targeted fabric request from `answers` (vid -> status, or
    None = never answers: the attempt times out after timeout_ms, as NDNSF's
    does)."""

    def __init__(self, answers):
        self.answers = answers
        self.requests = []

    def request_service_targeted_async(
        self, provider, service, payload, *, on_response, on_timeout, timeout_ms
    ):
        vid = provider.rsplit("/", 1)[-1]
        self.requests.append((vid, service, json.loads(payload)))
        status = self.answers[vid]
        if status is None:
            threading.Timer(timeout_ms / 1000, on_timeout, ("rid",)).start()
            return
        on_response(SimpleNamespace(
            status=True, error="",
            payload=FlightCommandResult(
                vehicle_id=vid, command="fabric", status=status, message="m",
            ).to_bytes(),
        ))


class DashboardSwitchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        (root / "fabric").mkdir()
        (root / "fabric" / "active").write_text("nfd wifi")
        cli = root / "muas-fabric"
        cli.write_text("#!/bin/sh\n")
        cli.chmod(cli.stat().st_mode | stat.S_IXUSR)
        self.argv = [
            "--iuas-ids", "iuas-01,iuas-02",
            "--fabric-dir", str(root / "fabric"),
            "--fabric-cli", str(cli),
            "--fabric-switch-file", str(root / "switch.json"),
            "--record-dir", "",
        ]
        self.switch_file = root / "switch.json"
        self.events = []

    def tearDown(self):
        self.tmp.cleanup()

    def dashboard(self, answers):
        args = run_dashboard.build_parser().parse_args(self.argv)
        dash = run_dashboard.Dashboard(args, FakeUser(answers))
        dash.event = lambda kind, **f: self.events.append((kind, f))
        now = time.monotonic()
        for vid in VEHICLES:
            dash.last_sample[vid] = dict(IDLE)
            dash.sample_state[vid] = {"changed_mono": now}
        return dash

    def kinds(self):
        return [k for k, _ in self.events]

    def test_vehicle_refusal_leaves_gcs_alone(self):
        dash = self.dashboard(
            {"wuas-01": "accepted", "iuas-01": "rejected", "iuas-02": "accepted"}
        )
        with mock.patch.object(fabric, "launch_switch") as launch:
            dash.request_fabric_switch("ndn-fwd wifi")
        launch.assert_not_called()
        self.assertEqual(len(dash.user.requests), 3)
        self.assertIn("fabric.switch.failed", self.kinds())
        self.assertNotIn("fabric.switch.gcs", self.kinds())
        self.assertIsNone(dash.fabric_switch)
        self.assertFalse(self.switch_file.exists())

    def test_vehicle_timeout_leaves_gcs_alone(self):
        dash = self.dashboard(
            {"wuas-01": "accepted", "iuas-01": "accepted", "iuas-02": None}
        )
        dash.FABRIC_VEHICLE_TIMEOUT_MS = 700  # one attempt, then the deadline
        with mock.patch.object(fabric, "launch_switch") as launch:
            dash.request_fabric_switch("ndn-fwd wifi")
            deadline = time.monotonic() + 5
            while ("fabric.switch.failed" not in self.kinds()
                   and time.monotonic() < deadline):
                time.sleep(0.05)
        launch.assert_not_called()
        self.assertIn("fabric.switch.failed", self.kinds())
        self.assertIsNone(dash.fabric_switch)

    def test_all_accept_switches_gcs_last(self):
        dash = self.dashboard({v: "accepted" for v in VEHICLES})
        order = []
        answer = dash.user.request_service_targeted_async

        def vehicle_request(*a, **k):
            order.append("vehicle")
            answer(*a, **k)

        dash.user.request_service_targeted_async = vehicle_request
        with mock.patch.object(
            fabric, "launch_switch",
            side_effect=lambda *a: (order.append("gcs"), ("unit", ""))[1],
        ) as launch:
            dash.request_fabric_switch("ndn-fwd wifi")
        launch.assert_called_once_with("ndn-fwd wifi", dash.args.fabric_cli)
        self.assertEqual(order, ["vehicle"] * 3 + ["gcs"])
        # the dashboard the GCS switch restarts must find the switch to judge
        saved = json.loads(self.switch_file.read_text())
        self.assertEqual(
            (saved["cell"], saved["phase"]), ("ndn-fwd wifi", "converging")
        )

    def test_armed_stale_unknown_busy_refuse_before_any_request(self):
        for spoil in (
            lambda d: d.last_sample["iuas-02"].update(armed=True),
            lambda d: d.sample_state["iuas-02"].update(
                changed_mono=time.monotonic() - 10),
            lambda d: d.last_sample.pop("iuas-02"),
            lambda d: d.last_sample["wuas-01"].update(busy="raster-search"),
        ):
            self.events.clear()
            dash = self.dashboard({v: "accepted" for v in VEHICLES})
            spoil(dash)
            with mock.patch.object(fabric, "launch_switch") as launch:
                dash.request_fabric_switch("ndn-fwd wifi")
            launch.assert_not_called()
            self.assertEqual(dash.user.requests, [])
            self.assertIn("fabric.switch.refused", self.kinds())

    def test_gcs_without_muas_fabric_refuses_before_any_request(self):
        dash = self.dashboard({v: "accepted" for v in VEHICLES})
        os.chmod(dash.args.fabric_cli, 0o644)
        dash.request_fabric_switch("ndn-fwd wifi")
        self.assertEqual(dash.user.requests, [])
        self.assertIn("fabric.switch.refused", self.kinds())

    def test_restart_mid_vehicle_round_is_failed_not_resumed(self):
        self.switch_file.write_text(json.dumps({
            "id": "x", "cell": "ndn-fwd wifi", "started_unix": time.time(),
            "phase": "vehicles", "vehicles": {},
        }))
        dash = self.dashboard({v: "accepted" for v in VEHICLES})
        with mock.patch.object(fabric, "launch_switch") as launch:
            dash._judge_fabric_switch()
        launch.assert_not_called()
        self.assertEqual(self.kinds(), ["fabric.switch.failed"])
        self.assertFalse(self.switch_file.exists())


if __name__ == "__main__":
    unittest.main()
