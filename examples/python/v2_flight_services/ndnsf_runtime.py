"""Shared runtime helpers for miniMUAS v2 NDNSF role scripts."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
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
    # Periodic NFD counter scrape into the metrics journal. On by default for
    # the deployed roles; a no-op on any host without nfdc/nfd-status (dev
    # boxes), so it stays harmless there. The pure-mock entrypoints simply do
    # not call start_nfd_counter_scrape, so it is effectively off for them.
    parser.add_argument(
        "--nfd-metrics",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Periodically scrape nfdc counters into the metrics journal.",
    )
    parser.add_argument(
        "--nfd-metrics-interval",
        type=float,
        default=30.0,
        help="Seconds between NFD counter scrapes (--nfd-metrics).",
    )
    parser.add_argument(
        "--session",
        default="latest",
        help="Mission session id this role serves its journal under "
        "(/muas/v2/<node>/journal/<session>), so the dashboard's mission "
        "bundle sweep can pull it. Defaults to the well-known 'latest'.",
    )


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


_JSON_LOG = {"file": None}


def enable_json_log(path: Path | str) -> Path:
    """Tee every print_json line to `path` with per-line flush + fsync.

    Best-effort persistence for the companion computers: journald and the
    page cache do not survive a pulled battery, and the SD filesystem
    rewinds to its last real flush. Event lines are low-rate (telemetry
    samples are NOT print_json'd), so an fsync per line is cheap and buys
    the strongest guarantee the hardware can give. Failures to open or
    write never take the role process down — the log is a debugging aid,
    not a dependency.
    """

    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    _JSON_LOG["file"] = open(path, "a", buffering=1)
    return path


def print_json(event: str, **fields: object) -> None:
    line = json.dumps({"event": event, **fields}, sort_keys=True)
    print(line, flush=True)
    log = _JSON_LOG["file"]
    if log is not None:
        try:
            import time as _time

            log.write(json.dumps(
                {"ts": _time.time(), "event": event, **fields},
                sort_keys=True,
            ) + "\n")
            log.flush()
            os.fsync(log.fileno())
        except Exception:
            pass


def flush_json_log() -> None:
    """Flush + fsync the tee'd event log (pre-shutdown assurance)."""
    log = _JSON_LOG["file"]
    if log is not None:
        try:
            log.flush()
            os.fsync(log.fileno())
        except Exception:
            pass


def start_role_journal(role_id: str, log_dir) -> None:
    """Enable the fsync-per-line metrics journal for a role entrypoint.

    Mirrors what the drone agent does at startup so metrics from EVERY role
    (providers, user, dashboard) are persisted power-loss-safely under a
    per-role file. An unwritable/empty dir just disables it — the journal is a
    diagnostic aid, never a hard dependency.
    """
    if not log_dir:
        return
    try:
        path = enable_json_log(Path(log_dir) / f"{role_id}.jsonl")
        print_json("role.journal.ready", role=role_id, path=str(path))
    except Exception as exc:
        print_json("role.journal.disabled", role=role_id, dir=str(log_dir), error=str(exc))


def current_journal_path() -> Path | None:
    """Filesystem path of the journal `print_json` is currently tee'ing to."""
    log = _JSON_LOG["file"]
    if log is None:
        return None
    try:
        return Path(log.name)
    except Exception:
        return None


# Keep journal producers referenced so their served Data stays reachable for
# the lifetime of the role; keyed by NDN name so a republish replaces cleanly.
_JOURNAL_PRODUCERS: dict = {}


def publish_journal(
    node_id: str,
    session: str,
    *,
    journal_path=None,
    signing_identity: str = "",
    freshness_ms: int = 60000,
):
    """Serve this role's journal as a named segmented object over NDN.

    Publishes the current `<role>.jsonl` under
    `/muas/v2/<node_id>/journal/<session>` (see contracts.vehicle_journal_name)
    so the dashboard's mission-bundle sweep can fetch every node's
    events+metrics+logs without SSH. Snapshots the journal at call time;
    call again (or use `start_journal_publisher`) to refresh a growing file.

    Best-effort: no journal, no NDN stack, or a producer error is a logged
    no-op returning None — the journal producer is never a flight dependency.
    Returns the live producer (keep referenced) on success.
    """
    from contracts import vehicle_journal_name

    path = Path(journal_path) if journal_path else current_journal_path()
    if path is None or not path.exists():
        print_json("journal.publish.skipped", node=node_id, session=session,
                   reason="no journal file")
        return None
    flush_json_log()
    try:
        payload = path.read_bytes()
    except Exception as exc:
        print_json("journal.publish.error", node=node_id, error=str(exc))
        return None
    name = vehicle_journal_name(node_id, session)
    try:
        from dataplane import publish_segmented

        producer = publish_segmented(
            name, payload,
            freshness_ms=freshness_ms, signing_identity=signing_identity,
        )
    except Exception as exc:
        print_json("journal.publish.error", node=node_id, name=name,
                   error=str(exc))
        return None
    old = _JOURNAL_PRODUCERS.pop(name, None)
    if old is not None:
        try:
            old.stop()
        except Exception:
            pass
    _JOURNAL_PRODUCERS[name] = producer
    print_json("journal.publish.ready", node=node_id, name=name,
               bytes=len(payload))
    return producer


def start_journal_publisher(
    node_id: str,
    session: str = "latest",
    *,
    interval_s: float = 30.0,
    signing_identity: str = "",
):
    """Republish this role's journal on an interval so the dashboard sweep can
    pull a fresh copy at any time while the node is up.

    A daemon thread re-snapshots + re-serves the journal every `interval_s`.
    No-op (returns None) if the journal is disabled. The republisher stops
    the previous producer before serving the new snapshot, so exactly one
    object is live per name. Safe to leave running for the whole mission.
    """
    if current_journal_path() is None:
        return None
    stop = threading.Event()

    def loop() -> None:
        # publish once immediately so the object exists as soon as the node
        # is up, then refresh on the interval
        publish_journal(node_id, session, signing_identity=signing_identity)
        while not stop.wait(interval_s):
            publish_journal(node_id, session, signing_identity=signing_identity)

    threading.Thread(
        target=loop, name=f"journal-publisher-{node_id}", daemon=True
    ).start()
    print_json("journal.publisher.started", node=node_id, session=session,
               interval_s=interval_s)
    return stop


def _nfd_status_command() -> list | None:
    """Return an argv for a machine-readable NFD status dump, or None."""
    if shutil.which("nfdc") is not None:
        return ["nfdc", "status", "report", "json"]
    if shutil.which("nfd-status") is not None:
        return ["nfd-status", "-j"]
    return None


def _flatten_general_status(obj) -> dict:
    """Pull the numeric forwarder counters out of an nfdc JSON report.

    Robust against schema drift: walk the JSON, and for any dict named
    "generalStatus" (or the top-level object as a fallback) emit its scalar
    numeric fields. Returns {} if nothing usable is found.
    """
    found: dict = {}

    def visit(node):
        if isinstance(node, dict):
            if "generalStatus" in node and isinstance(node["generalStatus"], dict):
                for key, value in node["generalStatus"].items():
                    if isinstance(value, (int, float)) and not isinstance(value, bool):
                        found[key] = value
            for value in node.values():
                visit(value)
        elif isinstance(node, list):
            for item in node:
                visit(item)

    visit(obj)
    if not found and isinstance(obj, dict):
        for key, value in obj.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                found[key] = value
    return found


def scrape_nfd_counters_once() -> bool:
    """Emit one `metric.nfd_counters` event; return True if counters emitted.

    Best-effort: any failure (nfdc absent, non-JSON output, parse error) is a
    silent no-op so a dev box or a transient forwarder hiccup never disturbs
    the role.
    """
    cmd = _nfd_status_command()
    if cmd is None:
        return False
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=5)
        if result.returncode != 0 or not result.stdout.strip():
            return False
        report = json.loads(result.stdout)
    except Exception:
        return False
    counters = _flatten_general_status(report)
    if not counters:
        return False
    print_json("metric.nfd_counters", **counters)
    return True


def start_nfd_counter_scrape(interval_s: float = 30.0, *, enabled: bool = True):
    """Start a daemon thread scraping NFD counters into the metrics journal.

    No-op (returns None) when disabled or when neither nfdc nor nfd-status is
    on PATH, so it is safe to leave on by default. Returns a threading.Event
    that, when set, stops the loop.
    """
    if not enabled:
        return None
    if _nfd_status_command() is None:
        print_json("metric.nfd_counters.skipped", reason="nfdc/nfd-status not on PATH")
        return None
    stop = threading.Event()

    def loop() -> None:
        while not stop.is_set():
            scrape_nfd_counters_once()
            if stop.wait(interval_s):
                break

    threading.Thread(target=loop, name="nfd-counter-scrape", daemon=True).start()
    print_json("metric.nfd_counters.started", interval_s=interval_s)
    return stop


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
