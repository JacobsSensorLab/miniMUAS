"""Fleet clock offset, as chrony measures it.

Every drone disciplines its clock to the GCS over NTP (`server <gcs> minpoll 2
maxpoll 4` in the fleet's nixos minimuas/v2.nix) and the GCS disciplines to the
public pool when it can reach it. chrony's filtered exchange measures each node's
offset from its reference to ~0.1 ms (sourcestats std dev 68-104 us, offsets
3-29 us, fleet 2026-09-23). The dashboard used to show its own clock minus a
telemetry sample's publish stamp as "clock delta": 300-700 ms, almost all of it
the sample's age (publish period + fetch + poll staleness), not clock error.
chrony's reading is the clock error; the age is reported separately.
"""

from __future__ import annotations

import ipaddress
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass

# The services' PATH does not include the system profile on NixOS.
CHRONYC = shutil.which("chronyc") or "/run/current-system/sw/bin/chronyc"
# chrony polls the GCS every 4-16 s, so a reading older than 5 s shows nothing new.
REFRESH_S = 5.0


@dataclass(frozen=True)
class ClockReading:
    """`ref` is chrony's selected reference ("" = unknown or unsynchronised);
    `offset_ms` is local clock minus that reference; `rms_ms` is chrony's
    long-term RMS offset (-1 = unknown)."""

    ref: str
    offset_ms: float
    rms_ms: float

    @property
    def known(self) -> bool:
        return self.ref != ""


UNKNOWN = ClockReading("", 0.0, -1.0)


def parse_tracking_csv(text: str) -> ClockReading:
    """Parse `chronyc -c tracking`.

    Fields: 0 ref id, 1 ref name/IP, 2 stratum, 3 ref time, 4 system time (s,
    POSITIVE = local clock SLOW of the reference: `+0.000162608` printed as
    "0.000162834 seconds slow of NTP time" on the fleet), 5 last offset,
    6 RMS offset, ..., 13 leap status.
    """
    fields = text.strip().split(",")
    if len(fields) < 14 or fields[13] != "Normal":
        return UNKNOWN  # "Not synchronised": there is no reference to be offset from
    try:
        return ClockReading(
            ref=fields[1],
            offset_ms=-float(fields[4]) * 1e3,
            rms_ms=float(fields[6]) * 1e3,
        )
    except ValueError:
        return UNKNOWN


def read_chrony(timeout_s: float = 2.0) -> ClockReading:
    # -n: the reference must come back as the IP chrony uses (offset_vs_local_ms
    # compares it with this host's addresses), and a reverse-DNS lookup in an
    # isolated field has no resolver to answer it inside `timeout_s`.
    try:
        out = subprocess.run(
            [CHRONYC, "-n", "-c", "tracking"],
            capture_output=True, text=True, timeout=timeout_s,
        )
    except (OSError, subprocess.TimeoutExpired):
        return UNKNOWN
    if out.returncode != 0:
        return UNKNOWN
    return parse_tracking_csv(out.stdout)


_LOCAL_ADDRESSES: set[str] = set()


def is_local_address(ref: str) -> bool:
    """True when `ref` is an IP address assigned to this host.

    Binding a socket to an address succeeds only for a local one
    (EADDRNOTAVAIL otherwise), which covers every interface without parsing
    `ip addr`. Only hits are cached: mesh0 may get its address after start.
    """
    if ref in _LOCAL_ADDRESSES:
        return True
    try:
        family = socket.AF_INET6 if ipaddress.ip_address(ref).version == 6 else socket.AF_INET
    except ValueError:
        return False
    try:
        with socket.socket(family, socket.SOCK_DGRAM) as s:
            s.bind((ref, 0))
    except OSError:
        return False
    _LOCAL_ADDRESSES.add(ref)
    return True


def offset_vs_local_ms(ref: str, offset_ms: float, ours: ClockReading) -> float | None:
    """A remote node's clock minus THIS host's clock, ms; None if unrelated.

    `ref`/`offset_ms` are the node's own chrony reading (node minus `ref`).
      - The node disciplines to us (every drone -> the GCS): its offset already
        IS node minus us. Our own reference is irrelevant, so this holds in an
        isolated field where the GCS serves a hand-set or free-running clock
        (chrony `local stratum 10`: reference 7F7F0101 with no name, which
        parse_tracking_csv reads as unknown). Field 2026-10-13: all three
        drones reported clock_ref 192.168.1.13 at 0.0-0.05 ms while the GCS
        was on that local reference, and the dashboard showed no offset all day.
      - Both discipline to the same upstream (bench drones on the pool):
        (node - R) - (us - R) = node - us.
    """
    if not ref:
        return None
    if is_local_address(ref):
        return offset_ms
    if ours.known and ref == ours.ref:
        return offset_ms - ours.offset_ms
    return None


class ClockMonitor:
    """The latest chrony reading, refreshed on its own thread so a slow
    `chronyc` never stalls the telemetry or poll loop that reads it."""

    def __init__(self) -> None:
        self._reading = UNKNOWN
        threading.Thread(target=self._run, name="clock-monitor", daemon=True).start()

    def reading(self) -> ClockReading:
        return self._reading

    def _run(self) -> None:
        while True:
            self._reading = read_chrony()
            time.sleep(REFRESH_S)
