#!/usr/bin/env python3
"""Preflight checks for the miniMUAS v2 NDNSF prototype."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys

from ndnsf_runtime import (
    DEFAULT_NDNSF_ROOT,
    DEFAULT_POLICY,
    DEFAULT_TRUST_SCHEMA,
    add_common_arguments,
)


REQUIRED_PKG_CONFIG = (
    "libndn-cxx",
    "libndn-svs",
    "libnac-abe",
    "ndnsd",
)


def print_check(name: str, status: str, **fields: object) -> None:
    print(json.dumps({"check": name, "status": status, **fields}, sort_keys=True))


def run_pkg_config(package: str) -> bool:
    return subprocess.run(
        ["pkg-config", "--exists", package],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode == 0


def check_file(path: Path, label: str) -> bool:
    if path.exists():
        print_check(label, "ok", path=str(path))
        return True
    print_check(label, "fail", path=str(path), reason="missing")
    return False


def check_binary(binary: str, required: bool = True) -> bool:
    resolved = shutil.which(binary)
    if resolved:
        print_check(f"binary:{binary}", "ok", path=resolved)
        return True
    print_check(
        f"binary:{binary}",
        "fail" if required else "warn",
        reason="not on PATH",
    )
    return not required


def check_ndnsf_import(ndnsf_root: Path) -> bool:
    wrapper = ndnsf_root.expanduser().resolve() / "pythonWrapper"
    if not wrapper.exists():
        print_check("ndnsf.pythonWrapper", "fail", path=str(wrapper), reason="missing")
        return False
    sys.path.insert(0, str(wrapper))

    try:
        importlib.import_module("ndnsf")
    except Exception as exc:
        print_check(
            "ndnsf.import",
            "fail",
            path=str(wrapper),
            reason=f"{type(exc).__name__}: {exc}",
        )
        return False

    print_check("ndnsf.import", "ok", path=str(wrapper))
    return True


def check_pkg_config_packages() -> bool:
    ok = True
    if not check_binary("pkg-config"):
        return False
    for package in REQUIRED_PKG_CONFIG:
        if run_pkg_config(package):
            print_check(f"pkg-config:{package}", "ok")
        else:
            print_check(
                f"pkg-config:{package}",
                "fail",
                reason="package not found",
            )
            ok = False
    return ok


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Preflight miniMUAS v2 NDNSF runtime")
    add_common_arguments(parser)
    parser.set_defaults(ndnsf_root=DEFAULT_NDNSF_ROOT)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--require-nfd", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    checks = []

    print_check(
        "host",
        "info",
        system=platform.system(),
        machine=platform.machine(),
        python=sys.executable,
    )
    if platform.system() == "Darwin":
        print_check(
            "host.support",
            "warn",
            reason="NDNSF installer is documented primarily for Ubuntu/Linux",
        )

    checks.append(check_file(args.policy, "policy"))
    checks.append(check_file(args.trust_schema, "trust_schema"))
    checks.append(check_pkg_config_packages())
    checks.append(check_ndnsf_import(args.ndnsf_root))

    nfd_required = bool(args.require_nfd or args.start_local_nfd)
    checks.append(check_binary("nfd-start", required=nfd_required))
    checks.append(check_binary("nfd-stop", required=nfd_required))
    checks.append(check_binary("nfd-status", required=False))

    if all(checks):
        print_check("summary", "ok")
        return 0

    print_check(
        "summary",
        "fail",
        reason=(
            "Build/install NDNSF native dependencies and the ndnsf._ndnsf "
            "Python extension before running the real stack"
        ),
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
