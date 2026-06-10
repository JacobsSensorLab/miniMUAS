#!/usr/bin/env python3
"""Preview or run the miniMUAS v2 NDNSF request/response stack."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

from ndnsf_runtime import (
    DEFAULT_POLICY,
    MINIMUAS_ROOT,
    add_common_arguments,
    ensure_multicast_strategy,
    optional_local_nfd,
    print_json,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the miniMUAS v2 NDNSF stack")
    add_common_arguments(parser)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--log-dir", type=Path, default=MINIMUAS_ROOT / "results" / "v2_ndnsf")
    parser.add_argument("--controller-startup-wait-s", type=float, default=1.0)
    parser.add_argument("--provider-startup-wait-s", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--investigate-timeout-ms",
        type=int,
        default=None,
        help="Default 15000, or 300000 when --mavlink-endpoint is set (real flight time)",
    )
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument(
        "--native-orbit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Advertise native circle-mode capability on the IUAS provider "
            "(--no-native-orbit exercises the guided fallback path end-to-end)"
        ),
    )
    parser.add_argument(
        "--mavlink-endpoint",
        default=None,
        help=(
            "Fly IUAS investigations on ArduPilot SITL / a real autopilot "
            "(from inside the container: tcp:host.docker.internal:5762)"
        ),
    )
    parser.add_argument(
        "--camera",
        default="synthetic",
        help=(
            "Frame source for WUAS published frames and IUAS capture "
            "artifacts: synthetic, file:<path>, or opencv:<index|url>"
        ),
    )
    parser.add_argument(
        "--library-dir",
        action="append",
        default=[],
        help="Optional dynamic library directory for the NDNSF Python extension",
    )
    return parser


def common_args(args) -> list[str]:
    return [
        "--ndnsf-root", str(args.ndnsf_root),
        "--group", args.group,
        "--controller", args.controller,
        "--trust-schema", str(args.trust_schema),
    ]


def planned_commands(args) -> list[list[str]]:
    common = common_args(args)
    iuas_extra: list[str] = [
        "--native-orbit" if args.native_orbit else "--no-native-orbit",
        "--camera", args.camera,
    ]
    if args.mavlink_endpoint:
        iuas_extra += ["--mavlink-endpoint", args.mavlink_endpoint]
    return [
        [
            sys.executable,
            str(SCRIPT_DIR / "run_ndnsf_controller.py"),
            *common,
            "--policy", str(args.policy),
        ],
        [
            sys.executable,
            str(SCRIPT_DIR / "run_gcs_provider.py"),
            *common,
        ],
        [
            sys.executable,
            str(SCRIPT_DIR / "run_iuas_provider.py"),
            *common,
            *iuas_extra,
        ],
        [
            sys.executable,
            str(SCRIPT_DIR / "run_wuas_user.py"),
            *common,
            "--timeout-ms", str(args.timeout_ms),
            "--investigate-timeout-ms", str(args.investigate_timeout_ms),
            "--camera", args.camera,
        ],
    ]


def process_env(args) -> dict[str, str]:
    env = os.environ.copy()
    wrapper = str(args.ndnsf_root.expanduser().resolve() / "pythonWrapper")
    env["PYTHONPATH"] = (
        wrapper
        if not env.get("PYTHONPATH")
        else wrapper + os.pathsep + env["PYTHONPATH"]
    )
    if args.library_dir:
        lib_path = os.pathsep.join(str(Path(value).expanduser()) for value in args.library_dir)
        for key in ("LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH"):
            env[key] = lib_path if not env.get(key) else lib_path + os.pathsep + env[key]
    return env


def terminate(processes: list[subprocess.Popen]) -> None:
    for process in processes:
        if process.poll() is None:
            process.terminate()
    for process in processes:
        if process.poll() is None:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


def main() -> int:
    args = build_parser().parse_args()
    if args.investigate_timeout_ms is None:
        args.investigate_timeout_ms = 300000 if args.mavlink_endpoint else 15000
    commands = planned_commands(args)
    if args.dry_run or not args.run:
        for command in commands:
            print_json("ndnsf.stack.command", command=command)
        if not args.run:
            print_json("ndnsf.stack.hint", message="pass --run to launch the stack")
        return 0

    args.log_dir.mkdir(parents=True, exist_ok=True)
    env = process_env(args)
    processes: list[subprocess.Popen] = []
    log_files = []

    if not args.skip_preflight:
        preflight = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "preflight_ndnsf.py"),
                "--ndnsf-root", str(args.ndnsf_root),
                "--trust-schema", str(args.trust_schema),
                "--policy", str(args.policy),
                *(["--require-nfd"] if args.start_local_nfd else []),
            ],
            cwd=str(MINIMUAS_ROOT),
            env=env,
            text=True,
            check=False,
        )
        if preflight.returncode != 0:
            print_json("ndnsf.stack.preflight_failed", returncode=preflight.returncode)
            return preflight.returncode

    with optional_local_nfd(args.start_local_nfd):
        ensure_multicast_strategy(args.group)
        try:
            role_names = ["controller", "gcs-provider", "iuas-provider"]
            for role_name, command in zip(role_names, commands[:3]):
                log = (args.log_dir / f"{role_name}.log").open("w", encoding="utf-8")
                log_files.append(log)
                processes.append(
                    subprocess.Popen(
                        command,
                        cwd=str(MINIMUAS_ROOT),
                        env=env,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        text=True,
                    )
                )
                wait_s = (
                    args.controller_startup_wait_s
                    if role_name == "controller"
                    else args.provider_startup_wait_s
                )
                time.sleep(wait_s)

            print_json("ndnsf.stack.user.starting")
            result = subprocess.run(
                commands[3],
                cwd=str(MINIMUAS_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=(args.investigate_timeout_ms / 1000.0) + 10.0,
                check=False,
            )
            if result.stdout:
                print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
            if result.stderr:
                print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
            print_json("ndnsf.stack.completed", returncode=result.returncode)
            return result.returncode
        finally:
            terminate(processes)
            for log in log_files:
                log.close()


if __name__ == "__main__":
    raise SystemExit(main())
