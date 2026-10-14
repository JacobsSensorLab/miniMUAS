"""Flight-path state bugs from the 2026-10-13 field day (issues C, D, G).

Run: python3 -m unittest test_flight_control  (from this directory)

  C  NAV_TAKEOFF "failed" on a lost COMMAND_ACK while the FC climbed, and a
     disarmed baro drift refused good takeoffs   -> mavlink_flight.ensure_airborne
  D  the raster speed was sent before takeoff and ArduCopter dropped it
                                                 -> CruiseSpeed / fly_raster
  G  the return inherited the last raster's speed -> smart_rtl speed_m_s
"""

from __future__ import annotations

import math
import threading
import time
import unittest
from types import SimpleNamespace

import mavlink_flight
import run_drone_agent as agent
from contracts import SearchArea
from raster import build_raster

LAT, LON = 35.1208, -89.9347
M_LAT = 1.0 / 111320.0


def north(m: float) -> float:
    return LAT + m * M_LAT


def wait_until(pred, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


def speed_of(sim) -> float:
    return math.hypot(*sim.velocity())


# -- C: ensure_airborne judges from telemetry, not COMMAND_ACKs -------------


class FakeVehicle:
    """Grounded copter whose AGL rises once the FC accepted a takeoff."""

    def __init__(self, *, disarmed_agl=0.0, armed_agl=0.0, climbs=True):
        self.armed = False
        self.disarmed_agl = disarmed_agl
        self.armed_agl = armed_agl
        self.climbs = climbs
        self.climbing = False
        self.agl = disarmed_agl

    @property
    def position(self):
        if self.climbing:
            self.agl += 1.0
        return SimpleNamespace(lat=LAT, lon=LON, alt=self.agl)


class FakeLink:
    """LoggingFlightLink surface; `*_ack` False = the ACK was lost (the FC
    still executed the command)."""

    def __init__(self, vehicle, *, arm_ack=True, takeoff_ack=True):
        self.v = vehicle
        self.arm_ack = arm_ack
        self.takeoff_ack = takeoff_ack
        self.command_log = []
        self.last_failure = ""
        self.disarmed = False

    def set_mode_guided(self):
        return True

    def arm(self):
        self.v.armed = True
        self.v.agl = self.v.armed_agl  # ArduCopter re-zeroes home at arming
        return self.arm_ack

    def disarm(self):
        self.v.armed = False
        self.disarmed = True
        return True

    def takeoff(self, alt_m):
        self.v.climbing = self.v.climbs
        return self.takeoff_ack


def airborne(link, vehicle, **kw):
    kw.setdefault("confirm_s", 0.3)
    kw.setdefault("climb_check_s", 0.4)
    kw.setdefault("timeout_s", 5.0)
    return mavlink_flight.ensure_airborne(
        link, vehicle, target_agl_m=6.0, home_alt_m=0.0, **kw
    )


class EnsureAirborneTest(unittest.TestCase):
    def test_lost_takeoff_ack_is_not_a_failure(self):
        # wuas-01 16:37:35/16:47:10: False 3 s after arming, then it climbed
        v = FakeVehicle()
        link = FakeLink(v, takeoff_ack=False)
        self.assertTrue(airborne(link, v))
        self.assertEqual(link.last_failure, "")

    def test_lost_arm_ack_is_rechecked_against_heartbeat(self):
        v = FakeVehicle()
        link = FakeLink(v, arm_ack=False)
        self.assertTrue(airborne(link, v))

    def test_disarmed_baro_drift_does_not_refuse(self):
        # iuas-02 16:55:32: 1.53 m disarmed -> "could not get airborne"
        v = FakeVehicle(disarmed_agl=2.27, armed_agl=0.0)
        link = FakeLink(v)
        self.assertTrue(airborne(link, v))

    def test_agl_still_wrong_after_arming_refuses_and_disarms(self):
        v = FakeVehicle(disarmed_agl=2.3, armed_agl=2.3)  # home locked
        link = FakeLink(v)
        self.assertFalse(airborne(link, v))
        self.assertTrue(link.disarmed)
        self.assertTrue(link.last_failure.startswith("ground_agl"))
        self.assertEqual(link.command_log[-1][0], "takeoff_refused")

    def test_refused_takeoff_fails_on_the_climb_check_and_disarms(self):
        v = FakeVehicle(climbs=False)
        link = FakeLink(v, takeoff_ack=False)
        self.assertFalse(airborne(link, v))
        self.assertTrue(link.last_failure.startswith("climb"))
        self.assertTrue(link.disarmed)


class AckRecoveryTest(unittest.TestCase):
    """A COMMAND_ACK read by another thread is still in conn.messages."""

    def link(self, ack):
        conn = SimpleNamespace(messages={} if ack is None else {"COMMAND_ACK": ack})
        inner = SimpleNamespace(
            _conn=conn, takeoff=lambda alt: False, arm=lambda: False,
        )
        return mavlink_flight.LoggingFlightLink(inner)

    def ack(self, command, result=0, age_s=0.0):
        return SimpleNamespace(
            command=command, result=result, _timestamp=time.time() - age_s
        )

    def test_takeoff_ack_consumed_elsewhere_counts(self):
        link = self.link(None)
        link._inner.takeoff = lambda alt: (
            link._inner._conn.messages.__setitem__(
                "COMMAND_ACK", self.ack(22)
            ) or False
        )
        self.assertTrue(link.takeoff(6.0))
        self.assertEqual(link.command_log[-1][1]["ack"], "recovered-from-cache")

    def test_stale_rejected_or_foreign_acks_do_not_count(self):
        self.assertFalse(self.link(self.ack(22, age_s=60)).takeoff(6.0))
        self.assertFalse(self.link(self.ack(22, result=4)).takeoff(6.0))
        self.assertFalse(self.link(self.ack(178)).takeoff(6.0))


class FcReasonTest(unittest.TestCase):
    """wuas-01 refused six takeoffs (15:38-15:53) and no log said why: the
    FC's STATUSTEXT must reach the refusal reason."""

    def setUp(self):
        self.conn = SimpleNamespace(messages={}, message_hooks=[],
                                    target_system=32)
        inner = SimpleNamespace(
            _conn=self.conn, arm=lambda: False, takeoff=lambda alt: False,
            _set_mode=lambda name: True,
        )
        self.link = mavlink_flight.LoggingFlightLink(inner)
        self.vehicle = FakeVehicle()  # never arms: inner.arm only refuses

    def feed(self, kind, src=32, comp=1, **fields):
        msg = SimpleNamespace(
            get_type=lambda: kind, get_srcSystem=lambda: src,
            get_srcComponent=lambda: comp, **fields,
        )
        for hook in self.conn.message_hooks:  # what pymavlink does
            hook(self.conn, msg)

    def test_prearm_text_is_the_refusal_reason(self):
        self.feed("STATUSTEXT", severity=6, text="Barometer 1 calibration complete")
        self.feed("STATUSTEXT", src=255, severity=2, text="not the FC")
        self.feed("STATUSTEXT", severity=3, text=b"PreArm: Compass not calibrated\x00")
        self.assertFalse(airborne(self.link, self.vehicle))
        self.assertTrue(self.link.last_failure.startswith("arm: "))
        self.assertIn("FC: PreArm: Compass not calibrated",
                      self.link.last_failure)

    def test_failsafe_state_is_named(self):
        self.feed("HEARTBEAT", system_status=5)  # MAV_STATE_CRITICAL
        self.assertFalse(airborne(self.link, self.vehicle))
        self.assertIn("failsafe", self.link.last_failure)


# -- D: the cruise speed is (re)asserted when ArduCopter can keep it --------


class CruiseSpeedTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.mode = "LAND"
        self.cs = agent.CruiseSpeed(
            lambda s: self.sent.append(s) or True, lambda: self.mode
        )

    def test_only_sent_after_a_guided_position_target(self):
        self.cs.set(0.5)  # HEAD raster_search sent it here: refused on the FC
        self.assertEqual(self.sent, [])
        self.cs.target_sent()  # target while still in LAND: ignored by FC
        self.assertEqual(self.sent, [])
        self.mode = "GUIDED"
        self.cs.target_sent()
        self.assertEqual(self.sent, [0.5])
        self.cs.target_sent()
        self.assertEqual(self.sent, [0.5])  # held: not re-sent every goto

    def test_resent_after_takeoff_or_mode_change(self):
        self.mode = "GUIDED"
        self.cs.set(0.5)
        self.cs.target_sent()
        self.cs.invalidate()  # NAV_TAKEOFF / GUIDED re-entry
        self.cs.target_sent()
        self.assertEqual(self.sent, [0.5, 0.5])
        self.mode = "LOITER"  # pilot took it: episode over
        self.cs.target_sent()
        self.mode = "GUIDED"
        self.cs.target_sent()
        self.assertEqual(self.sent, [0.5, 0.5, 0.5])

    def test_late_release_never_clears_a_successor(self):
        raster, rtl = object(), object()
        self.cs.set(0.5, raster)
        self.cs.set(3.0, rtl)  # RTL starts while the raster unwinds
        self.cs.release(raster)
        self.assertEqual(self.cs.desired, 3.0)
        self.cs.release(rtl)
        self.assertIsNone(self.cs.desired)


class SimSpeedTest(unittest.TestCase):
    """SimFlightBackend mimics ArduCopter's speed resets (WPNAV default 8)."""

    def setUp(self):
        self.sim = agent.SimFlightBackend(LAT, LON, default_speed_m_s=8.0)
        self.addCleanup(self.sim._stop.set)

    def cruise_speed_after(self, s: float) -> float:
        self.sim.goto(north(60), LON, 3.0)
        time.sleep(s)
        return speed_of(self.sim)

    def test_speed_requested_before_takeoff_is_flown(self):
        # the 2026-10-13 raster order: speed, then takeoff from the ground
        self.sim.set_cruise_speed(1.0)
        self.assertTrue(self.sim.ensure_airborne(3.0))
        self.assertTrue(self.sim.confirm_guided())
        self.assertAlmostEqual(self.cruise_speed_after(1.0), 1.0, delta=0.15)

    def test_speed_watch_recovers_a_reset_it_did_not_see(self):
        self.assertTrue(self.sim.ensure_airborne(3.0))
        self.sim.set_cruise_speed(1.0)
        self.sim.goto(north(60), LON, 3.0)
        with self.sim._lock:  # pilot flicks LOITER->GUIDED between gotos
            self.sim._set_mode_locked("LOITER")
            self.sim._set_mode_locked("GUIDED")
        watch = agent.SpeedWatch(self.sim, 1.0, task="t")
        self.sim.goto(north(60), LON, 3.0)
        self.assertTrue(wait_until(lambda: speed_of(self.sim) > 4.0, 2.0))
        self.assertTrue(wait_until(
            lambda: (watch.tick(), speed_of(self.sim) < 1.2)[1], 5.0
        ))
        self.assertEqual(watch.reasserts, 1)


class RasterAndRtlTest(unittest.TestCase):
    def setUp(self):
        self.sim = agent.SimFlightBackend(LAT, LON, default_speed_m_s=6.0)
        self.addCleanup(self.sim._stop.set)
        self.slots = SimpleNamespace(
            rtl_altitude_slots=lambda ids, base_agl_m, separation_m: {
                v: base_agl_m + i * separation_m
                for i, v in enumerate(sorted(ids))
            }
        )

    def test_raster_from_ground_flies_its_speed_and_rtl_its_own(self):
        plan = build_raster(
            SearchArea(center_lat=north(6), center_lon=LON,
                       width_m=8.0, height_m=4.0),
            leg_spacing_m=4.0, capture_every_m=4.0,
        )
        self.assertTrue(self.sim.ensure_airborne(3.0))
        self.assertTrue(self.sim.confirm_guided())
        watch = agent.SpeedWatch(self.sim, 1.5, task="raster")
        outcome = agent.fly_raster(
            self.sim, plan, agl_m=3.0, speed_m_s=1.5,
            abort=threading.Event(), deadline_mono=time.monotonic() + 60,
            on_capture=lambda cp, here: None, on_leg=lambda i: None,
            speed_watch=watch,
        )
        self.assertEqual(outcome, "completed")
        s = watch.summary()
        self.assertLessEqual(s["speed_max_m_s"], 1.6, s)
        self.assertIsNone(self.sim.cruise_speed())  # released by the sweep

        events = []
        threading.Thread(target=lambda: events.append(agent.smart_rtl(
            self.sim, "wuas-01", ["wuas-01"], deconflict_module=self.slots,
            cancel=threading.Event(), base_agl_m=4.0, speed_m_s=3.0,
            on_event=lambda **kw: events.append(kw),
        )), daemon=True).start()
        self.assertTrue(wait_until(lambda: "landing" in events, 40.0), events)
        landing = next(e for e in events
                       if isinstance(e, dict) and e["kind"] == "rtl.landing")
        self.assertGreater(landing["speed_max_m_s"], 2.5, landing)
        self.assertLessEqual(landing["speed_max_m_s"], 3.1, landing)

    def test_override_detour_flies_task_speed_and_sweep_resumes_its_own(self):
        # the agent's service_override pattern: own speed for the detour,
        # released after; the sweep must take its 1.5 m/s back
        plan = build_raster(
            SearchArea(center_lat=north(8), center_lon=LON,
                       width_m=12.0, height_m=4.0),
            leg_spacing_m=4.0, capture_every_m=4.0,
        )
        self.assertTrue(self.sim.ensure_airborne(3.0))
        self.assertTrue(self.sim.confirm_guided())
        detour = {"done": False, "speeds": []}

        def service_interrupt() -> bool:
            if detour["done"] or self.sim.cruise_speed() is None:
                return False
            detour["done"] = True
            back = self.sim.position()
            owner = object()
            self.sim.set_cruise_speed(3.0, owner=owner)
            try:
                for lat in (north(-10), back[0]):
                    self.sim.goto(lat, back[1], 3.0)
                    while not self.sim.at_target(lat, back[1], 3.0, tol_m=1.0):
                        detour["speeds"].append(speed_of(self.sim))
                        time.sleep(0.05)
                time.sleep(0.6)  # stop at the point (the capture's dwell)
            finally:
                self.sim.release_cruise_speed(owner)
            return True

        watch = agent.SpeedWatch(self.sim, 1.5, task="raster")
        outcome = agent.fly_raster(
            self.sim, plan, agl_m=3.0, speed_m_s=1.5,
            abort=threading.Event(), deadline_mono=time.monotonic() + 90,
            on_capture=lambda cp, here: None, on_leg=lambda i: None,
            service_interrupt=service_interrupt, speed_watch=watch,
        )
        self.assertEqual(outcome, "completed")
        self.assertGreater(max(detour["speeds"]), 2.5)
        self.assertLessEqual(max(detour["speeds"]), 3.1)
        s = watch.summary()
        self.assertLessEqual(s["speed_max_m_s"], 1.6, s)


if __name__ == "__main__":
    unittest.main()
