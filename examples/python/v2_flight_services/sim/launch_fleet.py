#!/usr/bin/env python3
"""Bring up the full miniMUAS v2 fleet on ONE local NFD for a manned dashboard.

Topology (Option B — single-host, single-NFD, N processes): everything runs as
separate OS processes sharing one NFD, exactly the way run_ndnsf_stack.py already
proves for the controller+providers, extended to the whole 4-node fleet plus the
dashboard. The native NDNSF stack is Linux-only, so this is meant to run INSIDE
the docker image built by docker/Dockerfile (which compiles NDNSF + the pybind
wrapper from the local checkout). See sim/RUNBOOK.md.

Nodes launched:
    controller   run_ndnsf_controller.py   (mints the service-authorization universe)
    gcs          run_gcs_provider.py       (object detector + nadir geo-projection)
    gcs          run_dashboard.py          (operator web UI on :8080 + mission brain)
    wuas-01      run_drone_agent.py --role wuas   (camera, raster search)
    iuas-01      run_drone_agent.py --role iuas   (camera, close inspection)
    iuas-02      run_drone_agent.py --role iuas   (USB-mic airframe: --audio synthetic)

Fabric wiring mirrors the deployment's STRATEGY layer from
minidronesys-configurations@mini-muas-v2 nix/nixos/common/minimuas/v2.nix:
multicast strategy on /muas/v2/group and /muas/v2/mission, best-route (the NFD
default) on /muas. On a single NFD there are no inter-node UDP faces/routes to
create — those collapse to loopback. The multi-container faithful variant
(Option A) is documented in the RUNBOOK.

This supervisor runs in the foreground until SIGINT/SIGTERM, then tears every
child down and stops the NFD it started. It writes a readiness file once all
roles are launched so headless drivers (sim/smoke.py) can wait on it.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
V2_DIR = SIM_DIR.parent
# miniMUAS repo root: .../examples/python/v2_flight_services -> up 3
MINIMUAS_ROOT = V2_DIR.parents[2]

DEFAULT_NDNSF_ROOT = os.environ.get(
    "NDNSF_ROOT", str(Path.home() / "Documents" / "Dev" / "NDN_Service_Framework")
)
DEFAULT_UAS_IPBRC = os.environ.get(
    "UAS_IPBRC_ROOT", str(Path.home() / "Documents" / "Dev" / "UAS-IPBRC")
)

GROUP = "/muas/v2/group"
MISSION_PREFIX = "/muas/v2/mission"
CONTROLLER = "/muas/v2/controller"

# Fleet layout: distinct start points ~10-12 m apart around a common center so
# four markers don't overprint, yet close enough that a converging mission trips
# cooperative avoidance.
CENTER_LAT = 35.1208
CENTER_LON = -89.9347
# Fleet order fixes each vehicle's SITL instance index: wuas-01=I0, iuas-01=I1,
# iuas-02=I2 -> SERIAL0 tcp ports 5760, 5770, 5780. Keep this in sync with
# sim/start_sitl.sh (VEHICLES) and the deployment fleetIds.
FLEET_ORDER = ["wuas-01", "iuas-01", "iuas-02"]
FLEET_IDS = ",".join(FLEET_ORDER)
FLEET_ROLE = {"wuas-01": "wuas", "iuas-01": "iuas", "iuas-02": "iuas"}
# Kinematic-bench start points: distinct so markers don't overprint. In SITL
# mode every instance shares the Memphis home (so coordination/localization line
# up) and the agents ignore these.
VEHICLE_STARTS = {
    "wuas-01": (CENTER_LAT, CENTER_LON),
    "iuas-01": (CENTER_LAT, CENTER_LON + 0.00011),   # ~10 m east
    "iuas-02": (CENTER_LAT + 0.00010, CENTER_LON),   # ~11 m north
}


def log(event: str, **fields) -> None:
    parts = " ".join(f"{k}={v}" for k, v in fields.items())
    print(f"[launch_fleet] {event} {parts}".rstrip(), flush=True)


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, text=True, capture_output=True, **kw)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Launch the miniMUAS v2 sim fleet")
    p.add_argument("--ndnsf-root", default=DEFAULT_NDNSF_ROOT)
    p.add_argument("--uas-ipbrc-root", default=DEFAULT_UAS_IPBRC)
    p.add_argument("--group", default=GROUP)
    p.add_argument("--controller", default=CONTROLLER)
    p.add_argument(
        "--trust-schema",
        default=str(MINIMUAS_ROOT / "config" / "trust-schema.conf"),
    )
    p.add_argument(
        "--policy",
        default=str(SIM_DIR / "fleet.policies"),
        help="Controller policy (default: the sim superset that authorizes iuas-02).",
    )
    p.add_argument("--detector", default="stub",
                   help="GCS detector spec: stub (default, deterministic fake) "
                        "or yolo:<model.onnx>[?conf=&classes=].")
    p.add_argument("--confirm-count", type=int, default=1,
                   help="Detections needed to confirm a target (1 = fast/deterministic "
                        "for the stub detector; the field default is 2).")
    p.add_argument("--log-dir",
                   default=str(MINIMUAS_ROOT / "results" / "v2_sim" / "log"))
    p.add_argument("--record-dir",
                   default=str(MINIMUAS_ROOT / "results" / "v2_sim" / "replays"))
    p.add_argument("--tiles-dir",
                   default=str(MINIMUAS_ROOT / "results" / "v2_sim" / "tiles"))
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--http-host", default="0.0.0.0")
    p.add_argument("--fleet-ids", default=FLEET_IDS)
    p.add_argument("--coord-hsep-m", type=float, default=8.0,
                   help="Horizontal separation minimum for conflict prediction "
                        "(raise, e.g. 20, to make avoidance trip readily in a demo).")
    p.add_argument("--coord-vsep-m", type=float, default=4.0)
    p.add_argument("--kinematic", action="store_true",
                   help="Fly the hand-rolled kinematic bench instead of ArduPilot "
                        "SITL (lightweight fallback; needs no host SITL instances). "
                        "Default is SITL: every drone flies its own ArduCopter SITL.")
    p.add_argument("--sitl-host", default="host.docker.internal",
                   help="Host running the ArduPilot SITL instances, reachable from "
                        "the container (default host.docker.internal on Docker Desktop).")
    p.add_argument("--sitl-base-port", type=int, default=5760,
                   help="SERIAL0 TCP port of SITL instance 0; instance I = base + step*I.")
    p.add_argument("--sitl-port-step", type=int, default=10)
    p.add_argument("--telemetry-hz", type=float, default=4.0)
    p.add_argument("--ready-file", default=None,
                   help="Path to touch once all roles are launched.")
    p.add_argument("--no-nfd", action="store_true",
                   help="Assume an NFD is already running; do not start/stop it.")
    p.add_argument("--controller-wait-s", type=float, default=3.0)
    p.add_argument("--provider-wait-s", type=float, default=2.5)
    p.add_argument("--agent-wait-s", type=float, default=1.5)
    return p


class Fleet:
    def __init__(self, args):
        self.args = args
        self.procs: list[tuple[str, subprocess.Popen]] = []
        self.logs: list = []
        self.started_nfd = False
        self.log_dir = Path(args.log_dir)
        self.console_dir = self.log_dir / "console"

    # -- environment -------------------------------------------------------
    def child_env(self) -> dict:
        env = os.environ.copy()
        wrapper = str(Path(self.args.ndnsf_root).expanduser().resolve() / "pythonWrapper")
        env["PYTHONPATH"] = (
            wrapper if not env.get("PYTHONPATH")
            else wrapper + os.pathsep + env["PYTHONPATH"]
        )
        env.setdefault("NDNSF_ROOT", str(Path(self.args.ndnsf_root)))
        env["UAS_IPBRC_ROOT"] = str(self.args.uas_ipbrc_root)
        return env

    def common(self) -> list[str]:
        return [
            "--ndnsf-root", str(self.args.ndnsf_root),
            "--group", self.args.group,
            "--controller", self.args.controller,
            "--trust-schema", str(self.args.trust_schema),
        ]

    # -- NFD ---------------------------------------------------------------
    def start_nfd(self) -> None:
        if self.args.no_nfd:
            log("nfd.skip", reason="--no-nfd")
        else:
            r = run(["nfd-start"])
            if r.returncode != 0:
                sys.stderr.write(r.stdout + r.stderr)
                raise RuntimeError("nfd-start failed")
            self.started_nfd = True
            log("nfd.started")
        # Strategy layer mirrors v2.nix: multicast on group + mission prefixes.
        for prefix in (self.args.group, MISSION_PREFIX):
            r = run(["nfdc", "strategy", "set", "prefix", prefix,
                     "strategy", "/localhost/nfd/strategy/multicast"])
            log("nfd.strategy", prefix=prefix, ok=(r.returncode == 0))
            if r.returncode != 0:
                sys.stderr.write(r.stdout + r.stderr)

    def stop_nfd(self) -> None:
        if self.started_nfd:
            run(["nfd-stop"])
            log("nfd.stopped")

    # -- process spawning --------------------------------------------------
    def spawn(self, name: str, cmd: list[str], wait_s: float) -> None:
        self.console_dir.mkdir(parents=True, exist_ok=True)
        logf = (self.console_dir / f"{name}.log").open("w", encoding="utf-8")
        self.logs.append(logf)
        log("spawn", role=name, cmd=" ".join(cmd))
        proc = subprocess.Popen(
            cmd, cwd=str(V2_DIR), env=self.child_env(),
            stdout=logf, stderr=subprocess.STDOUT, text=True,
        )
        self.procs.append((name, proc))
        time.sleep(wait_s)
        if proc.poll() is not None:
            log("role.exited_early", role=name, code=proc.returncode,
                console=str(self.console_dir / f"{name}.log"))

    def sitl_endpoint_for(self, vid: str) -> str:
        idx = FLEET_ORDER.index(vid)
        port = self.args.sitl_base_port + self.args.sitl_port_step * idx
        return f"tcp:{self.args.sitl_host}:{port}"

    def agent_cmd(self, vid: str, role: str, extra: list[str]) -> list[str]:
        lat, lon = VEHICLE_STARTS[vid]
        cmd = [
            sys.executable, str(V2_DIR / "run_drone_agent.py"),
            *self.common(),
            "--role", role,
            "--vehicle-id", vid,
            "--camera", "synthetic",
            "--fleet-ids", self.args.fleet_ids,
            "--coord-hsep-m", str(self.args.coord_hsep_m),
            "--coord-vsep-m", str(self.args.coord_vsep_m),
            "--telemetry-hz", str(self.args.telemetry_hz),
            "--uas-ipbrc-root", str(self.args.uas_ipbrc_root),
            "--log-dir", str(self.log_dir),
        ]
        if self.args.kinematic:
            # hand-rolled bench: distinct start points, no MAVLink
            cmd += ["--sim-lat", str(lat), "--sim-lon", str(lon)]
        else:
            # DEFAULT: fly this drone on its own ArduPilot SITL instance. The
            # agent connecting to SERIAL0 also boots that SITL. All share the
            # Memphis home so markers/coordination/localization line up.
            endpoint = self.sitl_endpoint_for(vid)
            cmd += ["--mavlink-endpoint", endpoint]
            log("sitl.attach", vehicle=vid, endpoint=endpoint)
        cmd += extra
        return cmd

    def launch_all(self) -> None:
        for d in (self.log_dir, Path(self.args.record_dir), Path(self.args.tiles_dir)):
            d.mkdir(parents=True, exist_ok=True)

        # 1) controller. Its --bootstrap-identity default already covers all
        # four fleet identities (gcs, wuas-01, iuas-01, iuas-02), so we only
        # override --policy: the sim superset that authorizes iuas-02 services
        # and the GCS's right to task IUAS sensor/capture (the stock
        # config/v2_minimuas.policies does not).
        self.spawn("controller", [
            sys.executable, str(V2_DIR / "run_ndnsf_controller.py"),
            *self.common(),
            "--policy", str(self.args.policy),
        ], self.args.controller_wait_s)

        # 2) GCS detector provider
        self.spawn("gcs-provider", [
            sys.executable, str(V2_DIR / "run_gcs_provider.py"),
            *self.common(),
            "--detector", self.args.detector,
            "--log-dir", str(self.log_dir),
        ], self.args.provider_wait_s)

        # 3) three drone agents
        self.spawn("wuas-01", self.agent_cmd("wuas-01", "wuas", []),
                   self.args.agent_wait_s)
        self.spawn("iuas-01", self.agent_cmd("iuas-01", "iuas", []),
                   self.args.agent_wait_s)
        self.spawn("iuas-02", self.agent_cmd("iuas-02", "iuas", [
            "--audio", "synthetic",
            "--sensors", "audio",
            "--audio-range-m", "0",   # never-out-of-range so tasked capture always records
        ]), self.args.agent_wait_s)

        # 4) operator dashboard (last: it is a user that consumes the fabric)
        self.spawn("dashboard", [
            sys.executable, str(V2_DIR / "run_dashboard.py"),
            *self.common(),
            "--http-host", self.args.http_host,
            "--http-port", str(self.args.http_port),
            "--wuas-id", "wuas-01",
            "--iuas-id", "iuas-01",
            "--iuas-ids", "iuas-01,iuas-02",
            "--confirm-count", str(self.args.confirm_count),
            "--record-dir", str(self.args.record_dir),
            "--tiles-dir", str(self.args.tiles_dir),
        ], self.args.provider_wait_s)

    def alive(self) -> list[str]:
        return [n for n, p in self.procs if p.poll() is None]

    def teardown(self) -> None:
        log("teardown.begin", alive=",".join(self.alive()))
        for _, p in self.procs:
            if p.poll() is None:
                p.terminate()
        for _, p in self.procs:
            if p.poll() is None:
                try:
                    p.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    p.kill()
        for f in self.logs:
            try:
                f.close()
            except Exception:
                pass
        self.stop_nfd()
        log("teardown.done")


def main() -> int:
    args = build_parser().parse_args()
    fleet = Fleet(args)

    stop = {"flag": False}

    def handle(signum, _frame):
        stop["flag"] = True
        log("signal", signum=signum)

    signal.signal(signal.SIGINT, handle)
    signal.signal(signal.SIGTERM, handle)

    mode = "kinematic-bench" if args.kinematic else "ardupilot-sitl"
    log("mavlink.mode", mode=mode)
    if not args.kinematic:
        log("sitl.reminder",
            note=("expects host SITL instances on "
                  f"{args.sitl_host}:{args.sitl_base_port}+{args.sitl_port_step}*I "
                  "(run sim/start_sitl.sh start). Agents boot each SITL on connect."))
    try:
        fleet.start_nfd()
        fleet.launch_all()
    except Exception as exc:  # noqa: BLE001
        log("startup.error", error=str(exc))
        fleet.teardown()
        return 2

    log("ready",
        dashboard=f"http://localhost:{args.http_port}/",
        roles=len(fleet.procs),
        log_dir=str(fleet.log_dir),
        record_dir=str(args.record_dir))
    if args.ready_file:
        Path(args.ready_file).parent.mkdir(parents=True, exist_ok=True)
        Path(args.ready_file).write_text("ready\n", encoding="utf-8")

    try:
        while not stop["flag"]:
            time.sleep(1.0)
            alive = fleet.alive()
            # controller/provider/dashboard dying is fatal; a drone agent
            # crashing is worth noting but the demo can continue.
            for critical in ("controller", "gcs-provider", "dashboard"):
                proc = dict(fleet.procs).get(critical)
                if proc is not None and proc.poll() is not None:
                    log("critical.exited", role=critical, code=proc.returncode)
                    stop["flag"] = True
    finally:
        fleet.teardown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
