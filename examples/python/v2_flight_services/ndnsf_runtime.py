"""Shared runtime helpers for miniMUAS v2 NDNSF role scripts."""

from __future__ import annotations

from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Iterator


MINIMUAS_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_NDNSF_ROOT = Path(
    os.environ.get("NDNSF_ROOT", "~/Documents/Dev/NDN_Service_Framework")
).expanduser()
DEFAULT_GROUP = "/muas/v2/group"
DEFAULT_CONTROLLER = "/muas/v2/controller"
DEFAULT_TRUST_SCHEMA = MINIMUAS_ROOT / "config" / "trust-schema.conf"
DEFAULT_POLICY = MINIMUAS_ROOT / "config" / "v2_minimuas.policies"


def add_ndnsf_path(ndnsf_root: Path) -> None:
    wrapper = ndnsf_root.expanduser().resolve() / "pythonWrapper"
    if not wrapper.exists():
        raise RuntimeError(f"NDNSF pythonWrapper not found: {wrapper}")
    wrapper_str = str(wrapper)
    if wrapper_str not in sys.path:
        sys.path.insert(0, wrapper_str)


def add_common_arguments(parser) -> None:
    parser.add_argument("--ndnsf-root", type=Path, default=DEFAULT_NDNSF_ROOT)
    parser.add_argument("--group", default=DEFAULT_GROUP)
    parser.add_argument("--controller", default=DEFAULT_CONTROLLER)
    parser.add_argument("--trust-schema", type=Path, default=DEFAULT_TRUST_SCHEMA)
    parser.add_argument("--start-local-nfd", action="store_true")
    parser.add_argument("--dry-run", action="store_true")


def provider_kwargs(args, provider_prefix: str, provider_id: str = "") -> dict:
    return {
        "provider_id": provider_id,
        "group": args.group,
        "controller": args.controller,
        "provider_prefix": provider_prefix,
        "trust_schema": str(args.trust_schema),
    }


def user_kwargs(args, user: str) -> dict:
    return {
        "group": args.group,
        "controller": args.controller,
        "user": user,
        "trust_schema": str(args.trust_schema),
    }


def controller_kwargs(args) -> dict:
    return {
        "controller_prefix": args.controller,
        "policy_file": str(args.policy),
        "trust_schema": str(args.trust_schema),
        "bootstrap_identities": list(args.bootstrap_identity),
    }


def print_json(event: str, **fields: object) -> None:
    print(json.dumps({"event": event, **fields}, sort_keys=True), flush=True)


def ensure_multicast_strategy(prefix: str) -> None:
    """Set NFD multicast strategy for an SVS group sync prefix.

    NDNSF transports requests/ACKs/selections/responses over ndn-svs group
    sync. When several participants share one NFD, the default best-route
    strategy delivers each sync Interest to only one registrant, which
    silently breaks sync; the group prefix must use multicast strategy.
    """

    nfdc = shutil.which("nfdc")
    if nfdc is None:
        print_json(
            "ndnsf.nfd.strategy.skipped",
            prefix=prefix,
            reason="nfdc not on PATH",
        )
        return
    result = subprocess.run(
        [
            nfdc,
            "strategy",
            "set",
            "prefix",
            prefix,
            "strategy",
            "/localhost/nfd/strategy/multicast",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    detail = (result.stdout or result.stderr or "").strip()
    print_json(
        "ndnsf.nfd.strategy",
        prefix=prefix,
        ok=result.returncode == 0,
        detail=detail[:200],
    )


@contextmanager
def optional_local_nfd(enabled: bool) -> Iterator[None]:
    """Start a local NFD only when requested and only stop what we started."""

    started_here = False
    if enabled:
        if shutil.which("nfd-start") is None or shutil.which("nfd-stop") is None:
            raise RuntimeError("nfd-start/nfd-stop are required for --start-local-nfd")
        running = subprocess.run(
            ["pgrep", "-x", "nfd"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode == 0
        if not running:
            subprocess.run(["nfd-start"], check=True)
            started_here = True
    try:
        yield
    finally:
        if started_here:
            subprocess.run(
                ["nfd-stop"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )


def require_success(response, service: str) -> bytes:
    if response.status:
        return bytes(response.payload)
    raise RuntimeError(f"{service} failed: {response.error}")
