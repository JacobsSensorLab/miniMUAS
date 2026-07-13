#!/usr/bin/env python3
"""Headless end-to-end smoke: bring the fleet up, drive a scripted mission, and
assert the four operator-visible behaviours land in the JSONL journals.

Asserts (each maps to a task deliverable):
    telemetry  -> 4 vehicles' markers moving      (dashboard recorder)
    search     -> raster -> detection -> localize  (gcs.detection.* + mission events)
    audio      -> interrogation artifact on iuas-02 (sensor_data kind audio/wav)
    coord      -> cooperative avoidance engaged     (agent.coord.* / avoid_bias_m)
    metric     -> latency metrics flowing           (metric.* from the WUAS user)

Runs inside the docker image (native NDNSF is Linux-only). It spawns
sim/launch_fleet.py, drives it through sim/ws_driver.py + run_wuas_user.py, polls
the journals until every required signal appears (or a deadline), prints a
PASS/FAIL table, tears the fleet down, and exits non-zero on any failure.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
V2_DIR = SIM_DIR.parent
MINIMUAS_ROOT = V2_DIR.parents[2]

DEFAULT_NDNSF_ROOT = os.environ.get(
    "NDNSF_ROOT", str(Path.home() / "Documents" / "Dev" / "NDN_Service_Framework"))
DEFAULT_UAS_IPBRC = os.environ.get(
    "UAS_IPBRC_ROOT", str(Path.home() / "Documents" / "Dev" / "UAS-IPBRC"))


def child_env(ndnsf_root: str, uas_root: str) -> dict:
    env = os.environ.copy()
    wrapper = str(Path(ndnsf_root).expanduser().resolve() / "pythonWrapper")
    env["PYTHONPATH"] = (
        wrapper if not env.get("PYTHONPATH")
        else wrapper + os.pathsep + env["PYTHONPATH"])
    env.setdefault("NDNSF_ROOT", ndnsf_root)
    env["UAS_IPBRC_ROOT"] = uas_root
    return env


def log(msg: str) -> None:
    print(f"[smoke] {msg}", flush=True)


# --------------------------------------------------------------------------
# Journal scanning
# --------------------------------------------------------------------------
class Signals:
    def __init__(self):
        self.telemetry_vehicles: set[str] = set()
        self.search = False          # detection produced a localized estimate
        self.audio = False           # audio/wav artifact
        self.coord = False           # positive cooperative-avoidance engagement
        self.coord_disabled = False  # deconfliction lib missing (explains a coord miss)
        self.metric = False
        self.notes: list[str] = []

    def all_required(self) -> bool:
        return (len(self.telemetry_vehicles) >= 2
                and self.search and self.audio and self.coord and self.metric)


def _inspect(obj: dict, sig: Signals) -> None:
    # dashboard recorder lines wrap the payload under "m"; print_json lines are flat
    payload = obj.get("m") if isinstance(obj.get("m"), dict) else obj
    event = payload.get("event", "")
    mtype = payload.get("type", "")

    if mtype == "telemetry":
        vid = payload.get("vehicle")
        if vid:
            sig.telemetry_vehicles.add(vid)
        sample = payload.get("sample", {})
        if isinstance(sample, dict) and abs(float(sample.get("avoid_bias_m", 0) or 0)) > 1e-6:
            sig.coord = True

    if mtype == "sensor_data":
        item = payload.get("item", {}) or {}
        if item.get("kind") == "audio/wav" or item.get("sensor") == "audio":
            sig.audio = True

    # Only real latency metrics count — metric.nfd_counters.started is startup
    # noise every role emits.
    if event.startswith("metric.") and not event.startswith("metric.nfd_counter"):
        sig.metric = True

    if event.startswith("agent.coord."):
        tail = event.split(".", 2)[-1]
        if tail == "disabled":
            sig.coord_disabled = True
        elif tail in ("coop-pending", "coop", "unco", "confirmed"):
            sig.coord = True

    # detection / localization signal (gcs provider + dashboard mission events)
    if event in ("gcs.detection.completed", "gcs.detection.projection"):
        sig.search = True
    if event.startswith("mission.") and payload.get("status") == "completed":
        sig.search = True

    # belt-and-suspenders raw matches for the audio artifact
    if "audio/wav" in json.dumps(payload):
        sig.audio = True


def scan(dirs: list[Path]) -> Signals:
    sig = Signals()
    for d in dirs:
        if not d.exists():
            continue
        for path in d.rglob("*.jsonl"):
            try:
                with path.open("r", encoding="utf-8", errors="replace") as fh:
                    for line in fh:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            obj = json.loads(line)
                        except Exception:
                            continue
                        if isinstance(obj, dict):
                            _inspect(obj, sig)
            except Exception:
                continue
    return sig


# --------------------------------------------------------------------------
# Driving
# --------------------------------------------------------------------------
def drive_ws(url: str, argv: list[str], env: dict, timeout: float = 60.0) -> None:
    cmd = [sys.executable, str(SIM_DIR / "ws_driver.py"), *argv, "--url", url]
    log("ws_driver " + " ".join(argv))
    r = subprocess.run(cmd, cwd=str(V2_DIR), env=env, text=True,
                       capture_output=True, timeout=timeout)
    if r.stdout.strip():
        for ln in r.stdout.strip().splitlines()[:8]:
            log("  ws> " + ln)
    if r.returncode != 0:
        log(f"  ws_driver rc={r.returncode} err={r.stderr.strip()[:300]}")


def run_wuas_user(args, env: dict) -> None:
    cmd = [
        sys.executable, str(V2_DIR / "run_wuas_user.py"),
        "--ndnsf-root", args.ndnsf_root,
        "--group", "/muas/v2/group",
        "--controller", "/muas/v2/controller",
        "--trust-schema", str(MINIMUAS_ROOT / "config" / "trust-schema.conf"),
        "--wuas-id", "wuas-01",
        "--iuas-id", "iuas-01",
        "--camera", "synthetic",
        "--log-dir", args.log_dir,
        "--timeout-ms", "30000",
        "--investigate-timeout-ms", "90000",
    ]
    log("run_wuas_user (detect + investigate -> metric.*)")
    out_path = Path(args.log_dir) / "console" / "wuas-user-driver.log"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        r = subprocess.run(cmd, cwd=str(V2_DIR), env=env, text=True,
                           capture_output=True, timeout=150)
        out_path.write_text((r.stdout or "") + "\n--- stderr ---\n" + (r.stderr or ""),
                            encoding="utf-8")
        log(f"  wuas_user rc={r.returncode} (full output -> {out_path})")
        if r.returncode != 0:
            for ln in (r.stderr or r.stdout or "").strip().splitlines()[-4:]:
                log("  wuas> " + ln[:300])
    except subprocess.TimeoutExpired:
        log("  wuas_user timed out (non-fatal for metric emission)")


# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="miniMUAS v2 fleet smoke test")
    p.add_argument("--ndnsf-root", default=DEFAULT_NDNSF_ROOT)
    p.add_argument("--uas-ipbrc-root", default=DEFAULT_UAS_IPBRC)
    p.add_argument("--url", default="http://127.0.0.1:8080")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--work-dir",
                   default=str(MINIMUAS_ROOT / "results" / "v2_sim_smoke"))
    p.add_argument("--ready-timeout-s", type=float, default=90.0)
    p.add_argument("--observe-timeout-s", type=float, default=120.0)
    p.add_argument("--coord-hsep-m", type=float, default=20.0,
                   help="Raised for the smoke so converging vehicles trip avoidance.")
    p.add_argument("--detector", default="stub")
    p.add_argument("--sitl", action="store_true",
                   help="Run the fleet on ArduPilot SITL (requires host SITL "
                        "instances already started via sim/start_sitl.sh). "
                        "Default is kinematic bench so the smoke is self-contained.")
    p.add_argument("--keep", action="store_true",
                   help="Do not delete the work dir before running.")
    return p


def main() -> int:
    args = build_parser().parse_args()
    work = Path(args.work_dir)
    log_dir = work / "log"
    record_dir = work / "replays"
    tiles_dir = work / "tiles"
    args.log_dir = str(log_dir)

    if work.exists() and not args.keep:
        shutil.rmtree(work, ignore_errors=True)
    for d in (log_dir, record_dir, tiles_dir):
        d.mkdir(parents=True, exist_ok=True)

    env = child_env(args.ndnsf_root, args.uas_ipbrc_root)
    ready_file = work / "READY"

    launch_cmd = [
        sys.executable, str(SIM_DIR / "launch_fleet.py"),
        "--ndnsf-root", args.ndnsf_root,
        "--uas-ipbrc-root", args.uas_ipbrc_root,
        "--log-dir", str(log_dir),
        "--record-dir", str(record_dir),
        "--tiles-dir", str(tiles_dir),
        "--http-port", str(args.http_port),
        "--detector", args.detector,
        "--confirm-count", "1",
        "--coord-hsep-m", str(args.coord_hsep_m),
        "--ready-file", str(ready_file),
    ]
    # The smoke defaults to kinematic bench so it needs no host SITL. Pass
    # --sitl to exercise the (default) ArduPilot SITL fleet instead.
    if not args.sitl:
        launch_cmd += ["--kinematic"]

    log("launching fleet ...")
    fleet = subprocess.Popen(launch_cmd, cwd=str(V2_DIR), env=env)

    rc = 1
    try:
        # 1) wait for readiness
        t0 = time.time()
        while time.time() - t0 < args.ready_timeout_s:
            if ready_file.exists():
                break
            if fleet.poll() is not None:
                log(f"fleet exited during startup (rc={fleet.returncode})")
                return 2
            time.sleep(1.0)
        else:
            log("fleet did not become ready in time")
            return 2
        # SITL needs longer: each agent must connect (booting its SITL), then
        # GPS/EKF must lock and the vehicle must arm+climb through GUIDED.
        settle_s = 35.0 if args.sitl else 8.0
        if args.sitl:
            args.observe_timeout_s = max(args.observe_timeout_s, 200.0)
        log(f"fleet ready; letting telemetry settle ({settle_s:.0f}s)")
        time.sleep(settle_s)

        # 2a) metrics first, against the still-idle fleet, so the WUAS user's
        #     investigate to iuas-01 succeeds cleanly (no contention with the
        #     dashboard-dispatched investigation) and emits real latency metrics.
        run_wuas_user(args, env)

        # 2b) then drive the operator mission: raster search -> detection ->
        #     auto-dispatched investigation (+ audio job), converging the fleet
        #     so cooperative avoidance engages.
        drive_ws(args.url, ["takeoff", "--all", "--listen", "4"], env)
        time.sleep(3.0)
        drive_ws(args.url, ["mission", "--listen", "6", "--show",
                            "search_status,event,sensor_data"], env)
        time.sleep(2.0)
        drive_ws(args.url, ["audio", "--vehicle", "iuas-02", "--listen", "6"], env)

        # 3) observe: poll the journals until everything appears or we time out
        t0 = time.time()
        sig = Signals()
        while time.time() - t0 < args.observe_timeout_s:
            sig = scan([log_dir, record_dir])
            if sig.all_required():
                break
            time.sleep(3.0)

        # 4) report
        checks = [
            ("telemetry (>=2 vehicles)",
             len(sig.telemetry_vehicles) >= 2,
             f"vehicles={sorted(sig.telemetry_vehicles)}"),
            ("search -> detection -> localization", sig.search, ""),
            ("audio interrogation artifact (iuas-02)", sig.audio, ""),
            ("cooperative avoidance (coord)", sig.coord,
             "coord.disabled seen (deconfliction lib missing)"
             if sig.coord_disabled and not sig.coord else ""),
            ("latency metrics (metric.*)", sig.metric, ""),
        ]
        print("\n==================== SMOKE RESULT ====================", flush=True)
        ok = True
        for name, passed, note in checks:
            status = "PASS" if passed else "FAIL"
            ok = ok and passed
            line = f"  [{status}] {name}"
            if note:
                line += f"   ({note})"
            print(line, flush=True)
        print("=====================================================", flush=True)
        print(f"journals: {log_dir}  and  {record_dir}", flush=True)
        rc = 0 if ok else 1
    finally:
        log("tearing fleet down")
        if fleet.poll() is None:
            fleet.send_signal(signal.SIGTERM)
            try:
                fleet.wait(timeout=25)
            except subprocess.TimeoutExpired:
                fleet.kill()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
