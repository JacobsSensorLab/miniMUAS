"""NDN fabric cell state and the operator's stack switch (dashboard + agent).

Each node runs one fabric cell at a time — "nfd wifi" (C++ NFD) or "ndn-fwd
wifi" (Rust ndn-fwd) — chosen by `muas-fabric set <fwd> <link>`
(minidronesys-configurations nix/nixos/common/minimuas/fabric.nix). Its state
lives in one-line files under /var/lib/minimuas/fabric; `active` is written
by muas-fabric-apply only after the cell came up healthy, so its mtime is the
moment the node's last fabric disturbance ended. Field days compare the two
stacks, and data inside SETTLE_S of a disturbance does not count, so every
node's (cell, since) pair is published: the agent tags its telemetry with it
and the dashboard rebroadcasts the whole fleet's.

`muas-fabric set` restarts muas-fabric-apply, and every role unit (agent,
dashboard, ...) Requires= muas-fabric.target, so systemd stops them the moment
the switch begins and starts them again once the new cell is up (GCS journal,
2026-10-12 16:17:52: the dashboard stopped 0.3 s before apply restarted). A
role unit must therefore never run the switch as its own child — it would be
killed with it — and must say everything it has to say (the reply to the
dashboard, the dashboard's own events) before the switch starts.
`launch_switch` runs it in a transient unit, which outlives the caller, on a
short timer that leaves that time.
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

FABRIC_DIR = "/var/lib/minimuas/fabric"
# Absolute: the role units' PATH is coreutils/findutils/grep/sed/systemd only
# (`systemctl show muas-v2-dashboard -p Environment` on minidronesys-03,
# 2026-10-12); muas-fabric lives in the system profile.
FABRIC_CLI = "/run/current-system/sw/bin/muas-fabric"
# A transient unit inherits the manager's environment, which on the nodes is
# PATH=<systemd>/bin only (`systemctl show-environment`, minidronesys-03,
# 2026-10-12). muas-fabric calls `timeout` and `sleep` unqualified, so without
# the system profile on PATH its health gate would fail and every switch
# would auto-revert. This is the PATH an operator's `sudo muas-fabric` sees.
SWITCH_PATH = "/run/current-system/sw/bin:/run/current-system/sw/sbin"

# ndn-fleet's settle invariant: data within this long of any fabric
# disturbance is not comparable data.
SETTLE_S = 150

# The cells the field comparison switches between, name -> `set` arguments.
# The radio cells are deliberately absent: the operator never selects them.
CELLS = {
    "nfd wifi": ("nfd", "wifi"),
    "ndn-fwd wifi": ("ndn-fwd", "wifi"),
}

# Re-read the state file at most this often: the telemetry loop runs at 4 Hz
# and a cell change takes 30-75 s, so a few seconds of lag costs nothing.
ACTIVE_REFRESH_S = 3.0


def add_fabric_arguments(parser) -> None:
    parser.add_argument(
        "--fabric-dir", default=FABRIC_DIR,
        help="muas-fabric state directory; its `active` file names this "
        "node's running cell (mtime = when that cell came up).",
    )
    parser.add_argument(
        "--fabric-cli", default=FABRIC_CLI,
        help="muas-fabric executable the stack switch runs.",
    )


def read_active(fabric_dir: str) -> dict:
    """{"cell", "since_unix"} for this node; None values when unreadable
    (no fabric on a bench host, or the first bring-up has not finished)."""
    path = Path(fabric_dir) / "active"
    try:
        cell = path.read_text().strip()
        since = path.stat().st_mtime
    except OSError:
        return {"cell": None, "since_unix": None}
    return {"cell": cell or None, "since_unix": round(since, 3) if cell else None}


class ActiveCell:
    """read_active, cached for ACTIVE_REFRESH_S (called per telemetry sample)."""

    def __init__(self, fabric_dir: str) -> None:
        self._dir = fabric_dir
        self._at = float("-inf")
        self._value = {"cell": None, "since_unix": None}

    def get(self) -> dict:
        now = time.monotonic()
        if now - self._at >= ACTIVE_REFRESH_S:
            self._value = read_active(self._dir)
            self._at = now
        return dict(self._value)


# Delay between launching the switch and its start: covers one targeted
# reply (50-100 ms on the fleet) and a re-issue that raced it (the dashboard
# re-issues every 2 s), and the dashboard's event broadcast + recorder flush.
SWITCH_DELAY_S = 2.0


def launch_switch(cell: str, cli: str) -> tuple[str, str]:
    """Schedule `muas-fabric set` for `cell` in its own transient unit,
    starting SWITCH_DELAY_S from now.

    -> (unit, error); error is "" when systemd accepted the unit. The switch
    itself runs on (30-75 s, health-gated, auto-reverting): its outcome is
    the `active` file, and `journalctl -u <unit>` has the CLI's own report.
    """
    fwd, link = CELLS[cell]
    # ms resolution: the dashboard and an agent on one host (bench) can launch
    # within the same second, and a duplicate unit name is refused.
    unit = f"muas-fabric-switch-{time.time_ns() // 1_000_000}"
    argv = [
        "systemd-run", "--no-block", f"--unit={unit}",
        f"--on-active={SWITCH_DELAY_S:g}s",
        f"--setenv=PATH={SWITCH_PATH}",
        cli, "set", fwd, link,
    ]
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return unit, str(exc)
    if proc.returncode != 0:
        return unit, (proc.stderr or proc.stdout).strip() or f"rc={proc.returncode}"
    return unit, ""


def unit_state(unit: str) -> str:
    """`systemctl is-active` of a transient switch unit: activating/active
    while it runs, failed if it failed, inactive once it exited cleanly (a
    successful transient unit is unloaded, which also reads inactive)."""
    try:
        proc = subprocess.run(
            ["systemctl", "is-active", unit],
            capture_output=True, text=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"unknown ({exc})"
    return proc.stdout.strip() or "unknown"


def switch_blockers(
    vehicles: list[str],
    samples: dict[str, dict],
    ages_s: dict[str, float],
    stale_s: float,
) -> dict[str, str]:
    """Why each vehicle may not change stacks now ({} = every vehicle may).

    Fails closed: a vehicle we have no sample from, whose newest sample is
    older than `stale_s`, or whose sample does not state `armed` explicitly
    is a refusal — an aircraft we cannot see might be flying, and one we
    cannot hear would not receive the switch either (it would stay on the old
    stack, splitting the fleet). `ages_s` is seconds since the vehicle's last
    NEW sample on the dashboard's own clock, so it needs no clock sync.
    """
    blockers: dict[str, str] = {}
    for vid in vehicles:
        sample = samples.get(vid)
        age = ages_s.get(vid)
        if sample is None or age is None:
            blockers[vid] = "no telemetry"
        elif age > stale_s:
            blockers[vid] = f"telemetry stale ({age:.1f} s > {stale_s:g} s)"
        elif sample.get("armed") is not False:
            blockers[vid] = (
                "ARMED" if sample.get("armed") is True else "armed state unknown"
            )
        elif sample.get("busy"):
            blockers[vid] = f"busy:{sample['busy']}"
    return blockers


class VehicleRound:
    """Collects every vehicle's answer to one `system/fabric` request.

    The GCS switches only after EVERY vehicle accepted: the GCS is the hub all
    vehicle traffic crosses, so switching it first would cut off a vehicle
    whose request had not been delivered yet. One refusal or timeout means
    the GCS stays put (the vehicles that accepted switch regardless; the
    operator sees exactly which).
    """

    def __init__(self, vehicles: list[str]) -> None:
        self.pending = set(vehicles)
        self.results: dict[str, dict] = {}

    def record(self, vid: str, status: str, message: str = "") -> str | None:
        """Note one vehicle's answer -> None while answers are outstanding,
        then once: "switch_gcs" (all accepted) or "failed"."""
        if vid not in self.pending:
            return None  # unknown vehicle or a duplicate answer
        self.pending.discard(vid)
        self.results[vid] = {"status": status, "message": message}
        if self.pending:
            return None
        if all(r["status"] == "accepted" for r in self.results.values()):
            return "switch_gcs"
        return "failed"

    def refusals(self) -> dict[str, str]:
        return {
            vid: r["status"] + (f": {r['message']}" if r["message"] else "")
            for vid, r in self.results.items() if r["status"] != "accepted"
        }


def converged(cell: str, nodes: dict[str, dict], fresh: set[str]) -> bool:
    """Every node reports `cell`; a vehicle's report counts only while its
    telemetry is fresh (a stale tag is the cell it was on, not is on)."""
    for node, state in nodes.items():
        if state.get("cell") != cell:
            return False
        if node != "gcs" and node not in fresh:
            return False
    return True
