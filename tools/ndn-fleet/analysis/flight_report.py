#!/usr/bin/env python3
"""Compare the NDN stacks over one collected flight (`ndn-fleet flight collect`).

    flight_report.py <results>/flights/<id> [--fleet fleet.toml] [--settle-s 150] [--no-wire]

Input: the flight directory, `<node>/recorder/` (cells.log, link/, wire/, chrony.log; GCS gps.log
and ping/ping.jsonl), `<node>/log/*.jsonl` (role journals) and `<gcs>/replays/dash-*.jsonl`
(every dashboard broadcast). Any of it may be missing; each section says what it lacked.

Output, in the flight directory: report.md (nfd vs ndn-fwd side by side), report.json
(everything), series/<node>.csv (one row per second).

Stack periods: the fleet's cell over time, from every node's recorder/cells.log (a node with no
cells.log falls back to the cell the dashboard saw: telemetry `sample.fabric`, the `fabric`
broadcast). A period is a stretch where every node with a known cell is on the same one;
any cells.log line (switch, recorder or node restart) starts a new one, and stretches where
nodes disagree are "mixed" and compared with nothing. The first `settle_s` of each period are
reported separately as settling (PROTOCOL.md I3: churn after a switch is not the stack).

Sorties are not identical, so the steady numbers are also broken down by the vehicle's
distance from the GCS, its altitude (armed samples only) and the video load (streams on),
with the sample count of every cell: compare like with like.

Clocks: every node is chrony-synced to the GCS (recorder/chrony.log). Dashboard records are
stamped by the GCS on receipt; per-sample telemetry age is the dashboard's own `age_ms`
(publish -> delivery with the chrony offset applied).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime
import glob
import gzip
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict, deque
from statistics import median
from typing import Dict, Iterator, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import link  # noqa: E402
import losses  # noqa: E402
import wire  # noqa: E402
from capture_report import load_nodes  # noqa: E402

SETTLE_S = 150
# The dashboard emits `video_stats` at the first frame >= 2 s after the previous one
# (run_dashboard.py on_frame), so a gap G between two of them holds a frame-free stretch of at
# least G - 2 s: freezes are LOWER bounds, and freezes shorter than the window are invisible.
VIDEO_STATS_WINDOW_S = 2.0
FREEZE_S = 0.5
# `telemetry_stale` repeats every second while a vehicle stays dark: one event per run.
STALE_MERGE_S = 1.5
# A telemetry sample places its vehicle for this long either side.
POSITION_MAX_AGE_S = 2.0
# Ring-file switches leave no gap; a recorder restart or the disk cap's deletions do.
CAPTURE_GAP_US = 5_000_000
# Wire matching works on chunks of this length, with this much slack either side for datagrams
# in flight across a chunk edge.
WIRE_CHUNK_S = 60
WIRE_EDGE_US = 2_000_000
# Cores on every node (ODROID-C4): a thread's share of one core = d(ticks) / (d(all) / NCPU).
NCPU = 4
FORWARDERS = ("nfd", "ndn-fwd")

DIST_BINS = [(0, 50), (50, 100), (100, 200), (200, 400), (400, math.inf)]
ALT_BINS = [(-math.inf, 2), (2, 10), (10, 30), (30, 60), (60, math.inf)]
STACK_ORDER = ["nfd wifi", "ndn-fwd wifi", "nfd radio", "ndn-fwd radio"]


# ---------------------------------------------------------------------------------------------
# small helpers

def pct(xs: List[float], q: float) -> Optional[float]:
    return None if not xs else losses.pct(xs, q)


def dist_summary(xs: List[float], qs=(0.5, 0.95, 0.99)) -> dict:
    out = {"n": len(xs)}
    for q in qs:
        v = pct(xs, q)
        out[f"p{round(q * 100)}"] = None if v is None else round(v, 2)
    out["max"] = round(max(xs), 2) if xs else None
    out["mean"] = round(sum(xs) / len(xs), 2) if xs else None
    return out


def haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 6_371_000 * math.asin(math.sqrt(h))


def bin_label(bins, v: Optional[float], unit: str) -> Optional[str]:
    if v is None:
        return None
    for lo, hi in bins:
        if lo <= v < hi:
            if hi == math.inf:
                return f">={lo:g} {unit}"
            if lo == -math.inf:
                return f"<{hi:g} {unit}"
            return f"{lo:g}-{hi:g} {unit}"
    return None


def iso(t: Optional[float]) -> str:
    if t is None:
        return "-"
    return datetime.datetime.fromtimestamp(t, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def open_text(path: str):
    return gzip.open(path, "rt", errors="replace") if path.endswith(".gz") else open(path, errors="replace")


def jsonl(path: str) -> Iterator[dict]:
    with open_text(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if isinstance(d, dict):
                yield d


# ---------------------------------------------------------------------------------------------
# stack periods

def read_cells(path: str) -> List[Tuple[float, str]]:
    out = []
    try:
        with open(path) as f:
            for line in f:
                ns, _, cell = line.strip().partition(" ")
                if ns.isdigit() and cell:
                    out.append((int(ns) / 1e9, cell.strip()))
    except OSError:
        pass
    return sorted(out)


class Periods:
    """The fleet's cell over time. `segments`: start, end, cell ("mixed" when nodes disagree),
    steady_from (start + settle), and each node's cell."""

    def __init__(self, changes: Dict[str, List[Tuple[float, str]]], t_end: float, settle_s: float):
        events = sorted((t, n, c) for n, ch in changes.items() for t, c in ch)
        state: Dict[str, str] = {}
        self.segments: List[dict] = []
        for i, (t, node, cell) in enumerate(events):
            state[node] = cell
            # Simultaneous lines (same ns) collapse into one boundary.
            if i + 1 < len(events) and events[i + 1][0] == t:
                continue
            cells = set(state.values())
            label = cells.pop() if len(cells) == 1 else "mixed"
            if self.segments:
                self.segments[-1]["end"] = t
            self.segments.append({"start": t, "end": t_end, "cell": label,
                                  "steady_from": t + settle_s if label != "mixed" else None,
                                  "nodes": dict(state), "trigger": node})
        self.segments = [s for s in self.segments if s["end"] > s["start"]]
        for i, s in enumerate(self.segments):
            s["i"] = i
        self._starts = [s["start"] for s in self.segments]

    def at(self, t: float) -> Optional[dict]:
        i = bisect.bisect_right(self._starts, t) - 1
        if i < 0 or t >= self.segments[i]["end"]:
            return None
        return self.segments[i]

    def phase(self, t: float) -> Optional[Tuple[str, str, int]]:
        """(cell, steady|settling|mixed, segment index) at `t`, None outside every period."""
        s = self.at(t)
        if s is None:
            return None
        if s["cell"] == "mixed":
            return ("mixed", "mixed", s["i"])
        return (s["cell"], "steady" if t >= s["steady_from"] else "settling", s["i"])


# ---------------------------------------------------------------------------------------------
# the dashboard recording

class Dash:
    """Everything the stack comparison reads from the dashboard's dash-*.jsonl."""

    def __init__(self):
        self.files: List[str] = []
        self.records = 0
        self.tel: Dict[str, List[tuple]] = defaultdict(list)  # (ts, age_ms, derived, sample)
        self.stale: Dict[str, List[Tuple[float, Optional[float]]]] = defaultdict(list)
        self.vstats: Dict[str, List[tuple]] = defaultdict(list)  # (ts, fps, kbps, lag_ms)
        self.vsettings: Dict[str, List[Tuple[float, str]]] = defaultdict(list)
        self.vcontrol: Dict[str, List[Tuple[float, bool]]] = defaultdict(list)
        self.resubscribes: Dict[str, List[float]] = defaultdict(list)
        self.commands: List[dict] = []
        self.fabric_events: List[dict] = []
        self.fabric_bcast: List[Tuple[float, dict]] = []

    def load(self, paths: List[str]) -> None:
        for p in sorted(paths):
            self.files.append(p)
            for rec in jsonl(p):
                ts, m = rec.get("ts"), rec.get("m")
                if not isinstance(m, dict) or not isinstance(ts, (int, float)):
                    continue
                self.records += 1
                self._one(float(ts), m)
        for d in (self.tel, self.stale, self.vstats, self.vsettings, self.vcontrol, self.resubscribes):
            for v in d.values():
                v.sort(key=lambda x: x[0] if isinstance(x, tuple) else x)
        self.commands.sort(key=lambda e: e["ts"])

    def _one(self, ts: float, m: dict) -> None:
        typ = m.get("type")
        vid = m.get("vehicle")
        if typ == "telemetry" and vid:
            s = m.get("sample") or {}
            age, derived = m.get("age_ms"), False
            if age is None and m.get("clock_ms") is not None and s.get("gps_time_ns"):
                # Recordings before `age_ms` existed: the same formula on the record's receipt
                # stamp (the dashboard computes it a few ms earlier, at delivery).
                age, derived = ts * 1000 - s["gps_time_ns"] / 1e6 + m["clock_ms"], True
            self.tel[vid].append((ts, age, derived, s))
        elif typ == "telemetry_stale" and vid:
            self.stale[vid].append((ts, m.get("silent_s")))
        elif typ == "video_stats" and vid:
            self.vstats[vid].append((ts, m.get("fps"), m.get("kbps"), m.get("lag_ms")))
        elif typ == "video_settings" and vid:
            fps = m.get("fps")
            preset = f"{m.get('width')}x{m.get('height')}@{fps:g}fps q{m.get('quality')} " \
                     f"{m.get('transport')}" if isinstance(fps, (int, float)) else "?"
            self.vsettings[vid].append((ts, preset))
        elif typ == "fabric":
            self.fabric_bcast.append((ts, m.get("nodes") or {}))
        elif typ == "event":
            kind = m.get("kind", "")
            if kind == "video.control" and vid and "enable" in m:
                self.vcontrol[vid].append((ts, bool(m["enable"])))
            elif kind == "video.stream_resubscribe" and vid:
                self.resubscribes[vid].append(ts)
            elif kind.startswith("command."):
                self.commands.append({**m, "ts": ts})
            elif kind.startswith("fabric."):
                self.fabric_events.append({**m, "ts": ts})

    def span(self) -> Tuple[Optional[float], Optional[float]]:
        ts = [v[0][0] for v in self.tel.values() if v] + [v[-1][0] for v in self.tel.values() if v]
        return (min(ts), max(ts)) if ts else (None, None)

    def cell_changes(self, node_vehicle: Dict[str, str]) -> Dict[str, List[Tuple[float, str]]]:
        """Per node, the cell the dashboard saw, as change points (fallback for cells.log)."""
        seen: Dict[str, List[Tuple[float, str]]] = defaultdict(list)

        def note(node: str, t: float, cell: Optional[str]) -> None:
            if cell and (not seen[node] or seen[node][-1][1] != cell):
                seen[node].append((t, cell))

        by_vehicle = {v: n for n, v in node_vehicle.items()}
        for vid, samples in self.tel.items():
            node = by_vehicle.get(vid)
            for ts, _a, _d, s in samples:
                if node:
                    note(node, ts, (s.get("fabric") or {}).get("cell"))
        gcs = by_vehicle.get("gcs")
        for ts, nodes in self.fabric_bcast:
            if gcs:
                note(gcs, ts, (nodes.get("gcs") or {}).get("cell"))
        return seen


# ---------------------------------------------------------------------------------------------
# where the vehicles were, and what was streaming

class Context:
    def __init__(self, dash: Dash, gcs_pos: Optional[Tuple[float, float]]):
        self.gcs_pos = gcs_pos
        self.pos: Dict[str, Tuple[List[float], List[dict]]] = {}
        for vid, samples in dash.tel.items():
            self.pos[vid] = ([s[0] for s in samples], [s[3] for s in samples])
        times = lambda d: {v: [x[0] for x in xs] for v, xs in d.items()}
        self.dash = dash
        self.streams = sorted(dash.vstats.keys() | dash.vcontrol.keys() | dash.vsettings.keys())
        self._ctl_t, self._stats_t, self._set_t = times(dash.vcontrol), times(dash.vstats), times(dash.vsettings)
        self._load_cache: Dict[int, str] = {}
        # When the dashboard recorded anything: outside it the video load is unknown, not "none".
        self._dash_t = sorted(x[0] for xs in dash.tel.values() for x in xs)

    def sample(self, vid: str, t: float) -> Optional[dict]:
        ts, ss = self.pos.get(vid, ([], []))
        i = bisect.bisect_left(ts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(ts) and abs(ts[j] - t) <= POSITION_MAX_AGE_S:
                if best is None or abs(ts[j] - t) < abs(ts[best] - t):
                    best = j
        return None if best is None else ss[best]

    def distance_m(self, vid: str, t: float) -> Optional[float]:
        s = self.sample(vid, t)
        if s is None or self.gcs_pos is None:
            return None
        lat, lon = s.get("lat_deg"), s.get("lon_deg")
        if not lat and not lon:  # 0/0: no fix
            return None
        return haversine_m(self.gcs_pos, (lat, lon))

    def bins(self, vid: Optional[str], t: float) -> Dict[str, str]:
        """{"distance": .., "altitude": .., "video load": ..} for a sample at `t`."""
        out = {"video load": self.load(t)}
        if vid is None or vid == "gcs":
            return out
        d = bin_label(DIST_BINS, self.distance_m(vid, t), "m")
        if d:
            out["distance"] = d
        s = self.sample(vid, t)
        if s is not None:
            out["altitude"] = bin_label(ALT_BINS, s.get("alt_m"), "m") if s.get("armed") \
                else "disarmed"
        return out

    def video_on(self, vid: str, t: float) -> bool:
        i = bisect.bisect_right(self._ctl_t.get(vid, []), t) - 1
        if i >= 0:
            return self.dash.vcontrol[vid][i][1]
        # No control event (yet): streaming if a stats window closed close by.
        st = self._stats_t.get(vid, [])
        i = bisect.bisect_left(st, t - VIDEO_STATS_WINDOW_S - 1)
        return i < len(st) and st[i] <= t + VIDEO_STATS_WINDOW_S + 1

    def preset(self, vid: str, t: float) -> str:
        i = bisect.bisect_right(self._set_t.get(vid, []), t) - 1
        return self.dash.vsettings[vid][i][1] if i >= 0 else "preset unknown"

    def load(self, t: float) -> str:
        """Which streams were on at `t`, with their presets ("none" when video was off)."""
        k = int(t)
        if k not in self._load_cache:
            i = bisect.bisect_left(self._dash_t, k - 10)
            if i == len(self._dash_t) or self._dash_t[i] > k + 10:
                self._load_cache[k] = "unknown (no dashboard recording)"
            else:
                on = [f"{v} {self.preset(v, k + 0.5)}" for v in self.streams if self.video_on(v, k + 0.5)]
                self._load_cache[k] = "; ".join(on) or "none"
        return self._load_cache[k]


def gcs_position(gps_log: str) -> Tuple[Optional[Tuple[float, float]], int]:
    lats, lons = [], []
    try:
        with open(gps_log, errors="replace") as f:
            for line in f:
                _, _, js = line.partition(" ")
                try:
                    d = json.loads(js)
                except ValueError:
                    continue
                if d.get("class") == "TPV" and (d.get("mode") or 0) >= 2 \
                        and d.get("lat") is not None and d.get("lon") is not None:
                    lats.append(d["lat"])
                    lons.append(d["lon"])
    except OSError:
        pass
    return ((median(lats), median(lons)) if lats else None), len(lats)


def chrony_offsets_ms(path: str) -> List[float]:
    """Every `System time` offset in recorder/chrony.log (`<unix_ns>` + `chronyc -n tracking`)."""
    out = []
    try:
        with open(path, errors="replace") as f:
            for line in f:
                m = re.match(r"System time\s*:\s*([\d.]+) seconds (fast|slow)", line)
                if m:
                    v = float(m.group(1)) * 1000
                    out.append(v if m.group(2) == "fast" else -v)
    except OSError:
        pass
    return out


def first_armed_positions(dash: Dash) -> Dict[str, Tuple[float, float]]:
    out = {}
    for vid, samples in dash.tel.items():
        for _ts, _a, _d, s in samples:
            if s.get("armed") and (s.get("lat_deg") or s.get("lon_deg")):
                out[vid] = (s["lat_deg"], s["lon_deg"])
                break
    return out


# ---------------------------------------------------------------------------------------------
# accumulation

class Acc:
    """Lists and counters keyed by tuples; `put` also files the value under each bin."""

    def __init__(self):
        self.v: Dict[tuple, List[float]] = defaultdict(list)
        self.c: Dict[tuple, float] = defaultdict(float)

    def put(self, key: tuple, x: float, bins: Optional[Dict[str, str]] = None) -> None:
        self.v[key].append(x)
        for dim, b in (bins or {}).items():
            self.v[("bin", dim, b) + key].append(x)

    def add(self, key: tuple, n: float = 1, bins: Optional[Dict[str, str]] = None) -> None:
        self.c[key] += n
        for dim, b in (bins or {}).items():
            self.c[("bin", dim, b) + key] += n


class Series:
    """Per node, per second: column -> value (last write wins, or summed with `add`)."""

    def __init__(self):
        self.rows: Dict[str, Dict[int, Dict[str, object]]] = defaultdict(lambda: defaultdict(dict))

    def set(self, node: str, t: float, col: str, v) -> None:
        self.rows[node][int(t)][col] = v

    def add(self, node: str, t: float, col: str, v: float = 1) -> None:
        row = self.rows[node][int(t)]
        row[col] = row.get(col, 0) + v


# ---------------------------------------------------------------------------------------------
# sections

def telemetry(dash: Dash, periods: Periods, ctx: Context, series: Series, veh_node) -> dict:
    acc = Acc()
    derived = Counter()
    for vid, samples in dash.tel.items():
        prev = None
        for ts, age, was_derived, s in samples:
            ph = periods.phase(ts)
            if ph is None:
                prev = None
                continue
            cell, phase, seg = ph
            key = (cell, phase, vid)
            bins = ctx.bins(vid, ts) if phase == "steady" else None
            acc.add(("n",) + key)
            if age is not None:
                acc.put(("age",) + key, age, bins)
                derived[was_derived] += 1
            if prev is not None and prev[1] == seg and prev[2] == phase:
                gap = ts - prev[0]
                acc.put(("gap",) + key, gap)
                acc.add(("covered",) + key, gap)
            prev = (ts, seg, phase)
            if s.get("armed"):
                acc.add(("armed_samples",) + key)
            node = veh_node.get(vid)
            if node:
                series.add(node, ts, "tel_samples")
                if age is not None:
                    series.set(node, ts, "tel_age_ms", round(age, 1))
                for col, k in (("armed", "armed"), ("alt_m", "alt_m"), ("groundspeed_m_s", "groundspeed_m_s")):
                    if s.get(k) is not None:
                        series.set(node, ts, col, s[k])
                d = ctx.distance_m(vid, ts)
                if d is not None:
                    series.set(node, ts, "dist_m", round(d, 1))
        # staleness: runs of telemetry_stale records
        run = None
        events = []
        for ts, silent in dash.stale.get(vid, []):
            if run and ts - run[1] <= STALE_MERGE_S:
                run = (run[0], ts, max(run[2], silent or 0))
            else:
                if run:
                    events.append(run)
                run = (ts, ts, silent or 0)
        if run:
            events.append(run)
        for start, _end, silent in events:
            ph = periods.phase(start)
            if ph:
                acc.add(("stale_events", ph[0], ph[1], vid))
                acc.put(("stale_s", ph[0], ph[1], vid), silent)
    out = {"age_source": "derived from receipt stamp (no age_ms in the recording)"
           if derived[True] and not derived[False] else
           ("age_ms" if not derived[True] else "age_ms, partly derived"),
           "per": {}}
    for (what, cell, phase, vid), n in acc.c.items():
        if what != "n":
            continue
        k = (cell, phase, vid)
        gaps = acc.v.get(("gap",) + k, [])
        covered = acc.c.get(("covered",) + k, 0.0)
        out["per"].setdefault(cell, {}).setdefault(phase, {})[vid] = {
            "samples": int(n),
            "covered_s": round(covered, 1),
            "rate_hz": round(n / covered, 2) if covered else None,
            "gap_s": dist_summary(gaps),
            "gaps_over_1s": sum(1 for g in gaps if g > 1),
            "gaps_over_2s": sum(1 for g in gaps if g > 2),
            "age_ms": dist_summary(acc.v.get(("age",) + k, [])),
            "stale_events": int(acc.c.get(("stale_events",) + k, 0)),
            "stale_max_s": max(acc.v.get(("stale_s",) + k, []), default=None),
        }
    out["bins"] = _bins(acc, "age", lambda xs: dist_summary(xs))
    return out


def _bins(acc: Acc, what: str, summarize) -> dict:
    """{dimension: {bin: {cell: summary}}} over steady samples, pooled across vehicles."""
    pooled: Dict[tuple, List[float]] = defaultdict(list)
    for key, xs in acc.v.items():
        if key[0] == "bin" and key[3] == what and key[5] == "steady":
            pooled[(key[1], key[2], key[4])].extend(xs)
    out: dict = {}
    for (dim, b, cell), xs in pooled.items():
        out.setdefault(dim, {}).setdefault(b, {})[cell] = summarize(xs)
    return out


def video(dash: Dash, periods: Periods, ctx: Context, series: Series, veh_node,
          t0: float, t1: float) -> dict:
    acc = Acc()
    for vid, stats in dash.vstats.items():
        prev = None
        node = veh_node.get(vid)
        for ts, fps, kbps, lag in stats:
            ph = periods.phase(ts)
            if ph is None:
                prev = None
                continue
            cell, phase, seg = ph
            key = (cell, phase, vid)
            bins = ctx.bins(vid, ts) if phase == "steady" else None
            acc.add(("windows",) + key)
            if fps is not None:
                acc.put(("fps",) + key, fps)
            if kbps is not None:
                acc.put(("kbps",) + key, kbps)
            if lag is not None:
                acc.put(("lag",) + key, lag, bins)
            if prev is not None and prev[1] == seg and ctx.video_on(vid, (prev[0] + ts) / 2):
                frozen = ts - prev[0] - VIDEO_STATS_WINDOW_S
                if frozen >= FREEZE_S:
                    acc.put(("freeze",) + key, frozen, bins)
            prev = (ts, seg)
            if node:
                series.set(node, ts, "video_fps", fps)
                series.set(node, ts, "video_kbps", kbps)
                series.set(node, ts, "video_lag_ms", lag)
        for ts in dash.resubscribes.get(vid, []):
            ph = periods.phase(ts)
            if ph:
                acc.add(("resubscribes", ph[0], ph[1], vid))
    # Preset on-time and the video load's share of each stack's time, second by second.
    vids = sorted(dash.vstats.keys() | dash.vcontrol.keys() | dash.vsettings.keys())
    preset_s: Dict[tuple, int] = Counter()
    load_s: Dict[tuple, int] = Counter()
    if vids and t0 is not None:
        for sec in range(int(t0), int(t1) + 1):
            ph = periods.phase(sec + 0.5)
            if ph is None:
                continue
            load_s[(ph[0], ph[1], ctx.load(sec))] += 1
            for vid in vids:
                if ctx.video_on(vid, sec + 0.5):
                    preset_s[(ph[0], ph[1], vid, ctx.preset(vid, sec + 0.5))] += 1
    out: dict = {"per": {}, "presets": {}, "load_s": {}}
    for (what, cell, phase, vid), n in acc.c.items():
        if what != "windows":
            continue
        k = (cell, phase, vid)
        fr = acc.v.get(("freeze",) + k, [])
        out["per"].setdefault(cell, {}).setdefault(phase, {})[vid] = {
            "stats_windows": int(n),
            "fps": dist_summary(acc.v.get(("fps",) + k, []), qs=(0.05, 0.5)),
            "kbps": dist_summary(acc.v.get(("kbps",) + k, []), qs=(0.5,)),
            "lag_ms": dist_summary(acc.v.get(("lag",) + k, [])),
            "freezes": len(fr),
            "freeze_total_s": round(sum(fr), 1),
            "freeze_max_s": round(max(fr), 2) if fr else None,
            "resubscribes": int(acc.c.get(("resubscribes",) + k, 0)),
        }
    for (cell, phase, vid, preset), s in preset_s.items():
        out["presets"].setdefault(cell, {}).setdefault(phase, {}).setdefault(vid, {})[preset] = s
    for (cell, phase, load), s in load_s.items():
        out["load_s"].setdefault(cell, {}).setdefault(phase, {})[load] = s
    out["bins"] = {"lag": _bins(acc, "lag", dist_summary),
                   "freeze": _bins(acc, "freeze", lambda xs: {"n": len(xs), "total_s": round(sum(xs), 1)})}
    return out


def commands(dash: Dash, journals: Dict[str, List[str]], periods: Periods, ctx: Context) -> dict:
    acc = Acc()
    events = list(dash.commands)
    source = "dashboard recording"
    if not events:
        # No recording: the dashboard journal carries the same events as dash.command.*.
        source = "gcs-dashboard journal"
        for path in (p for ps in journals.values() for p in ps if "dashboard" in os.path.basename(p)):
            for e in jsonl(path):
                ev = e.get("event", "")
                if ev.startswith("dash.command."):
                    events.append({**e, "kind": ev[len("dash."):]})
    for e in events:
        ph = periods.phase(e["ts"])
        if ph is None:
            continue
        cell, phase, _ = ph
        label = e.get("label") or e.get("service") or "?"
        key = (cell, phase, label)
        bins = ctx.bins(e.get("vehicle"), e["ts"]) if phase == "steady" else None
        kind = e.get("kind")
        if kind == "command.ok":
            acc.add(("ok",) + key)
            acc.put(("latency",) + key, e.get("latency_ms") or 0, bins)
            acc.put(("attempts",) + key, e.get("attempts") or 1)
            if e.get("status") is False:
                acc.add(("refused",) + key)
        elif kind == "command.failed":
            acc.add(("failed",) + key)
            acc.put(("attempts",) + key, e.get("attempts") or 1)
        elif kind == "command.retry":
            acc.add(("retries",) + key)
    out: dict = {"source": source, "per": {}}
    keys = {k[1:] for k in acc.c if k[0] in ("ok", "failed")}
    for cell, phase, label in keys:
        k = (cell, phase, label)
        att = acc.v.get(("attempts",) + k, [])
        out["per"].setdefault(cell, {}).setdefault(phase, {})[label] = {
            "ok": int(acc.c.get(("ok",) + k, 0)),
            "failed": int(acc.c.get(("failed",) + k, 0)),
            "refused": int(acc.c.get(("refused",) + k, 0)),
            "retries": int(acc.c.get(("retries",) + k, 0)),
            "attempts_mean": round(sum(att) / len(att), 2) if att else None,
            "attempts_max": max(att) if att else None,
            "latency_ms": dist_summary(acc.v.get(("latency",) + k, []), qs=(0.5, 0.95)),
        }
    out["bins"] = _bins(acc, "latency", lambda xs: dist_summary(xs, qs=(0.5, 0.95)))
    # metric.latency from every role journal (services, fetches, crypto)
    ml = Acc()
    for node, paths in journals.items():
        for path in paths:
            for e in jsonl(path):
                if e.get("event") != "metric.latency" or not isinstance(e.get("ts"), (int, float)):
                    continue
                ph = periods.phase(e["ts"])
                if ph is None:
                    continue
                key = (ph[0], ph[1], f"{e.get('stage')}/{e.get('name')}")
                ml.put(("total",) + key, e.get("total_ms") or 0)
                if e.get("status") is False:
                    ml.add(("bad",) + key)
    out["metric_latency"] = {}
    for (what, cell, phase, name), xs in ml.v.items():
        out["metric_latency"].setdefault(cell, {}).setdefault(phase, {})[name] = {
            **dist_summary(xs, qs=(0.5, 0.95)), "status_false": int(ml.c.get(("bad", cell, phase, name), 0))}
    return out


def ping(path: str, periods: Periods, ctx: Context, series: Series, veh_node) -> Optional[dict]:
    if not os.path.exists(path):
        return None
    acc = Acc()
    cell_mismatch = 0
    for e in jsonl(path):
        t = (e.get("t_ns") or 0) / 1e9
        ph = periods.phase(t)
        vid = e.get("vehicle")
        if ph is None or not vid:
            continue
        cell, phase, _ = ph
        if e.get("cell") and cell != "mixed" and e["cell"] != cell:
            cell_mismatch += 1
        key = (cell, phase, vid)
        bins = ctx.bins(vid, t) if phase == "steady" else None
        res = e.get("result")
        acc.add(("probes",) + key, 1, bins)
        if res == "ok" and e.get("rtt_ms") is not None:
            acc.put(("rtt",) + key, e["rtt_ms"], bins)
        elif res == "timeout":
            acc.add(("timeout",) + key, 1, bins)
        elif res == "nack":
            acc.add(("nack",) + key, 1, bins)
        node = veh_node.get(vid)
        if node:
            series.set(node, t, "ping_rtt_ms", e.get("rtt_ms"))
            series.set(node, t, "ping_result", res)
    out: dict = {"cell_mismatch": cell_mismatch, "per": {}, "bins": {}}
    for key, n in acc.c.items():
        if key[0] == "probes":
            _, cell, phase, vid = key
            k = (cell, phase, vid)
            lost = acc.c.get(("timeout",) + k, 0)
            out["per"].setdefault(cell, {}).setdefault(phase, {})[vid] = {
                "probes": int(n), "timeouts": int(lost), "nacks": int(acc.c.get(("nack",) + k, 0)),
                "loss_pct": round(100 * lost / n, 2) if n else None,
                "rtt_ms": dist_summary(acc.v.get(("rtt",) + k, [])),
            }
        elif key[0] == "bin" and key[3] == "probes" and key[5] == "steady":
            _, dim, b, _w, cell, _p, vid = key
            slot = out["bins"].setdefault(dim, {}).setdefault(b, {}).setdefault(
                cell, {"probes": 0, "timeouts": 0, "rtt": []})
            slot["probes"] += n
            slot["timeouts"] += acc.c.get(("bin", dim, b, "timeout", cell, "steady", vid), 0)
            slot["rtt"] += acc.v.get(("bin", dim, b, "rtt", cell, "steady", vid), [])
    for dim in out["bins"].values():
        for cells in dim.values():
            for cell, slot in cells.items():
                rtt = slot.pop("rtt")
                slot["loss_pct"] = round(100 * slot["timeouts"] / slot["probes"], 2) if slot["probes"] else None
                slot["rtt_ms"] = dist_summary(rtt, qs=(0.5, 0.95))
    return out


def link_files(rec_dir: str) -> List[str]:
    return sorted(glob.glob(os.path.join(rec_dir, "link", "link-*.txt*")))


def radio_and_host(node: dict, files: List[str], periods: Periods, ctx: Context, series: Series,
                   mac_vehicle: Dict[str, str]) -> dict:
    """Per peer radio metrics and the forwarder's host metrics from the link sampler."""
    acc = Acc()
    name = node["name"]
    prev = None
    ticks_n = 0
    for path in files:
        for t_ns, sec in link.ticks(path):
            t = t_ns / 1e9
            ph = periods.phase(t)
            snap = {
                "stations": link.stations(sec.get("station", "")),
                "rtl": link.rtl_link(sec),
                "snmp": link.snmp(sec.get("snmp", "")),
                "sock_drops": link.udp_socket_drops(sec.get("udp", "")),
                "sockets": link.sockets(sec.get("sockets", "")),
                "rss": link.process_rss_kb(sec.get("mem", "")),
                "names": link.process_names(sec.get("mem", "")),
                "ticks": link.process_ticks(sec.get("tasks", "")),
                "cpu": link.cpu_total(sec.get("cpu", "")),
            }
            dt = (t - prev[0]) if prev else None
            usable = prev is not None and 0 < dt <= 5
            ticks_n += 1
            if ph is not None:
                cell, phase, _ = ph
                fwd = next(((pid, n) for pid, n in snap["names"].items() if n in FORWARDERS), None)
                if fwd:
                    rss = snap["rss"].get(fwd[1])
                    if rss:
                        acc.put(("rss_mb", cell, phase, "host"), rss / 1024)
                        series.set(name, t, "fwd_rss_mb", round(rss / 1024, 1))
                    if usable and prev[1]["cpu"] and snap["cpu"] > prev[1]["cpu"] \
                            and fwd[0] in prev[1]["ticks"] and fwd[0] in snap["ticks"]:
                        per_cpu = (snap["cpu"] - prev[1]["cpu"]) / NCPU
                        c = 100 * (snap["ticks"][fwd[0]] - prev[1]["ticks"][fwd[0]]) / per_cpu
                        acc.put(("cpu_pct", cell, phase, "host"), c)
                        series.set(name, t, "fwd_cpu_pct", round(c, 1))
                if usable:
                    for k in ("Udp.RcvbufErrors", "Udp.SndbufErrors", "Udp.InErrors"):
                        d = snap["snmp"].get(k, 0) - prev[1]["snmp"].get(k, 0)
                        if d >= 0:
                            acc.add((k, cell, phase, "host"), d)
                            series.set(name, t, k.replace(".", "_"), d)
                    d = snap["sock_drops"] - prev[1]["sock_drops"]
                    if d >= 0:
                        acc.add(("sock_drops", cell, phase, "host"), d)
                    acc.add(("host_s", cell, phase, "host"), dt)
                acc.put(("txq", cell, phase, "host"), snap["sockets"]["tx_queue_max"])
                acc.put(("rxq", cell, phase, "host"), snap["sockets"]["rx_queue_max"])
                series.set(name, t, "udp_txq_max_b", snap["sockets"]["tx_queue_max"])
                # radio: the GCS AP per station; a vehicle's rtl driver toward the AP
                for mac, st in snap["stations"].items():
                    peer = mac_vehicle.get(mac, mac)
                    bins = ctx.bins(peer if peer in ctx.pos else None, t) if phase == "steady" else None
                    for k, col in (("signal", "signal_dbm"), ("tx_bitrate", "tx_mbps"),
                                   ("rx_bitrate", "rx_mbps")):
                        if k in st:
                            acc.put((col, cell, phase, peer), st[k], bins)
                            series.set(name, t, f"{peer}_{col}", st[k])
                    p = prev[1]["stations"].get(mac) if usable else None
                    if p:
                        dtx = st.get("tx_packets", 0) - p.get("tx_packets", 0)
                        dre = st.get("tx_retries", 0) - p.get("tx_retries", 0)
                        dfa = st.get("tx_failed", 0) - p.get("tx_failed", 0)
                        if dtx >= 0 and dre >= 0 and dfa >= 0:
                            acc.add(("tx_packets", cell, phase, peer), dtx)
                            acc.add(("tx_retries", cell, phase, peer), dre, bins)
                            acc.add(("tx_failed", cell, phase, peer), dfa, bins)
                            acc.add(("tx_packets_b", cell, phase, peer), dtx, bins)
                            series.set(name, t, f"{peer}_tx_retries", dre)
                            series.set(name, t, f"{peer}_tx_failed", dfa)
                r = snap["rtl"]
                if r:
                    peer = "gcs"
                    bins = ctx.bins(node["vehicle"], t) if phase == "steady" else None
                    for k, col in (("signal_dbm", "signal_dbm"), ("retry_ratio", "retry_ratio"),
                                   ("tp_tx_kbps", "tp_tx_kbps"), ("tp_rx_kbps", "tp_rx_kbps")):
                        if k in r:
                            acc.put((col, cell, phase, peer), r[k], bins)
                            series.set(name, t, f"ap_{col}", r[k])
                    for k in ("tx_rate", "rx_rate"):
                        if k in r:
                            acc.add((k + "=" + str(r[k]), cell, phase, peer))
            prev = (t, snap)
    out: dict = {"ticks": ticks_n, "host": {}, "peers": {}}
    cells = {(k[1], k[2]) for k in list(acc.v) + list(acc.c) if k[0] != "bin"}
    for cell, phase in cells:
        hk = (cell, phase, "host")
        host_s = acc.c.get(("host_s",) + hk, 0)

        def per_s(k):
            return round(acc.c.get((k,) + hk, 0) / host_s, 3) if host_s else None

        h = {
            "fwd_cpu_pct": dist_summary(acc.v.get(("cpu_pct",) + hk, [])),
            "fwd_rss_mb_max": round(max(acc.v.get(("rss_mb",) + hk, [0])), 1) or None,
            "udp_rcvbuf_errors": int(acc.c.get(("Udp.RcvbufErrors",) + hk, 0)),
            "udp_rcvbuf_errors_per_s": per_s("Udp.RcvbufErrors"),
            "udp_sndbuf_errors": int(acc.c.get(("Udp.SndbufErrors",) + hk, 0)),
            "udp_in_errors": int(acc.c.get(("Udp.InErrors",) + hk, 0)),
            "socket_drops": int(acc.c.get(("sock_drops",) + hk, 0)),
            "udp_txq_bytes": dist_summary(acc.v.get(("txq",) + hk, []), qs=(0.95,)),
            "udp_rxq_bytes": dist_summary(acc.v.get(("rxq",) + hk, []), qs=(0.95,)),
            "seconds": round(host_s),
        }
        out["host"].setdefault(cell, {})[phase] = h
        peers = {k[3] for k in list(acc.v) + list(acc.c) if k[0] != "bin" and k[1:3] == (cell, phase)
                 and k[3] != "host"}
        for peer in peers:
            pk = (cell, phase, peer)
            txp = acc.c.get(("tx_packets",) + pk, 0)
            rates: Dict[str, List[Tuple[float, str]]] = defaultdict(list)
            for k, n in acc.c.items():
                if "=" in k[0] and k[1:] == pk:
                    which, rate = k[0].split("=", 1)
                    rates[which].append((n, rate))
            out["peers"].setdefault(cell, {}).setdefault(phase, {})[peer] = {
                "signal_dbm": dist_summary(acc.v.get(("signal_dbm",) + pk, []), qs=(0.05, 0.5)),
                "tx_mbps": dist_summary(acc.v.get(("tx_mbps",) + pk, []), qs=(0.05, 0.5)),
                "rx_mbps": dist_summary(acc.v.get(("rx_mbps",) + pk, []), qs=(0.05, 0.5)),
                "tx_retries": int(acc.c.get(("tx_retries",) + pk, 0)),
                "tx_failed": int(acc.c.get(("tx_failed",) + pk, 0)),
                "retries_per_100_tx": round(100 * acc.c.get(("tx_retries",) + pk, 0) / txp, 2) if txp else None,
                "retry_ratio": dist_summary(acc.v.get(("retry_ratio",) + pk, []), qs=(0.5, 0.95)),
                "tp_kbps_tx": dist_summary(acc.v.get(("tp_tx_kbps",) + pk, []), qs=(0.5,)),
                "rates": {k: max(v)[1] for k, v in rates.items()},  # most common PHY rate
            }
    bins: dict = {}
    for key, xs in acc.v.items():
        if key[0] == "bin" and key[3] == "signal_dbm" and key[5] == "steady":
            bins.setdefault(key[1], {}).setdefault(key[2], {}).setdefault(key[4], []).extend(xs)
    out["signal_bins"] = {d: {b: {c: dist_summary(xs, qs=(0.05, 0.5)) for c, xs in cs.items()}
                              for b, cs in bs.items()} for d, bs in bins.items()}
    retry_bins: dict = {}
    for key, n in acc.c.items():
        if key[0] == "bin" and key[3] in ("tx_retries", "tx_packets_b") and key[5] == "steady":
            slot = retry_bins.setdefault(key[1], {}).setdefault(key[2], {}).setdefault(key[4], [0, 0])
            slot[0 if key[3] == "tx_retries" else 1] += n
    out["retry_bins"] = {d: {b: {c: (round(100 * r / p, 2) if p else None) for c, (r, p) in cs.items()}
                             for b, cs in bs.items()} for d, bs in retry_bins.items()}
    return out


class WireStream:
    """One node's capture ring as a time-ordered datagram stream, with what it covered."""

    def __init__(self, files: List[str], keep):
        self.files = files
        self.keep = keep
        self.buf: deque = deque()
        self.cover: List[List[int]] = []  # [first, last] us, merged across contiguous files
        self.stats: List[dict] = []
        self._gen = self._all()
        self.done = False

    def _all(self):
        for path in self.files:
            yield from wire.datagrams(path, keep=self._note, stats=self.stats)

    def _note(self, t_us: int) -> bool:
        if self.cover and t_us - self.cover[-1][1] <= CAPTURE_GAP_US:
            self.cover[-1][1] = max(self.cover[-1][1], t_us)
        else:
            self.cover.append([t_us, t_us])
        return self.keep(t_us)

    def fill_to(self, t_us: int) -> None:
        while not self.done and (not self.buf or self.buf[-1].t_us < t_us):
            try:
                self.buf.append(next(self._gen))
            except StopIteration:
                self.done = True

    def drop_before(self, t_us: int) -> None:
        while self.buf and self.buf[0].t_us < t_us:
            self.buf.popleft()

    def covers(self, lo: int, hi: int) -> bool:
        for a, b in self.cover:
            if a <= lo and hi <= b:
                return True
        return False


def wire_loss(nodes: List[dict], flight: str, periods: Periods, ctx: Context, series: Series) -> Optional[dict]:
    files = {n["name"]: sorted(p for p in glob.glob(os.path.join(flight, n["name"], "recorder", "wire", "*"))
                               if os.path.isfile(p))
             for n in nodes}
    files = {k: v for k, v in files.items() if v}
    if not files:
        return None
    chunks = []
    for s in periods.segments:
        if s["cell"] == "mixed":
            continue
        a = s["start"]
        while a < s["end"]:
            b = min(a + WIRE_CHUNK_S, s["end"])
            chunks.append((int(a * 1e6), int(b * 1e6)))
            a = b
    starts = [c[0] - WIRE_EDGE_US for c in chunks]

    def keep(t_us: int) -> bool:
        i = bisect.bisect_right(starts, t_us) - 1
        return i >= 0 and t_us < chunks[i][1] + 2 * WIRE_EDGE_US

    streams = {n: WireStream(f, keep) for n, f in files.items()}
    names = {n["name"]: n for n in nodes}
    acc = Acc()
    retx: Dict[tuple, dict] = {}
    for a, b in chunks:
        for st in streams.values():
            st.fill_to(b + 2 * WIRE_EDGE_US)
        for src, s_st in streams.items():
            sa = names[src]["addr"]
            for dst, d_st in streams.items():
                if dst == src:
                    continue
                da = names[dst]["addr"]
                tx = [d for d in s_st.buf if d.src == sa and d.dst == da
                      and a - WIRE_EDGE_US <= d.t_us < b + WIRE_EDGE_US]
                if not tx:
                    continue
                rx = [d for d in d_st.buf if d.src == sa and d.dst == da
                      and a - WIRE_EDGE_US <= d.t_us < b + 2 * WIRE_EDGE_US]
                got = losses.arrivals(tx, rx)
                link_name = f"{names[src]['vehicle']}->{names[dst]['vehicle']}"
                in_chunk = [d for d in tx if a <= d.t_us < b]
                # Distance/altitude bins apply to a vehicle<->GCS link: the vehicle's.
                ends = (names[src], names[dst])
                vehicle = next((e["vehicle"] for e in ends if e["role"] != "gcs"), None) \
                    if any(e["role"] == "gcs" for e in ends) else None
                for d, t_rx in zip(tx, got):
                    if not a <= d.t_us < b:
                        continue
                    if not d_st.covers(d.t_us - 1_000_000, d.t_us + 1_000_000):
                        acc.add(("uncovered", link_name))
                        continue
                    ph = periods.phase(d.t_us / 1e6)
                    if ph is None:
                        continue
                    cell, phase, _ = ph
                    key = (cell, phase, link_name)
                    bins = ctx.bins(vehicle, d.t_us / 1e6) if phase == "steady" and vehicle else None
                    acc.add(("sent",) + key, 1, bins)
                    kind = d.kind or "undecoded"
                    acc.add(("sent_kind", cell, phase, link_name, kind))
                    if t_rx is None:
                        acc.add(("lost",) + key, 1, bins)
                        acc.add(("lost_kind", cell, phase, link_name, kind))
                        series.add(src, d.t_us / 1e6, f"wire_lost_to_{names[dst]['vehicle']}")
                    else:
                        acc.put(("delay",) + key, (t_rx - d.t_us) / 1000.0)
                    series.add(src, d.t_us / 1e6, f"wire_sent_to_{names[dst]['vehicle']}")
                ph = periods.phase(a / 1e6)
                if ph and in_chunk:
                    r = losses.retransmissions(in_chunk)
                    slot = retx.setdefault((ph[0], ph[1], link_name),
                                           {"lp_packets": 0, "retransmissions": 0, "identical_resends": 0,
                                            "sends_histogram": Counter()})
                    slot["lp_packets"] += r["lp_packets"]
                    slot["retransmissions"] += r["retransmissions"]
                    slot["identical_resends"] += losses.identical_resends(in_chunk)
                    slot["sends_histogram"].update(r["sends_histogram"])
        for st in streams.values():
            st.drop_before(b - WIRE_EDGE_US)
    out: dict = {"files": {n: len(f) for n, f in files.items()},
                 "capture_drops": {n: sum(s.get("dropped") or 0 for s in st.stats) for n, st in streams.items()},
                 "closed_files_with_stats": {n: len(st.stats) for n, st in streams.items()},
                 "uncovered": {k[1]: int(v) for k, v in acc.c.items() if k[0] == "uncovered"},
                 "per": {}, "bins": {}}
    for key, n in acc.c.items():
        if key[0] != "sent":
            continue
        _, cell, phase, ln = key
        k = (cell, phase, ln)
        lost = acc.c.get(("lost",) + k, 0)
        r = retx.get(k, {})
        hist = r.get("sends_histogram", Counter())
        frames = r.get("lp_packets", 0)
        out["per"].setdefault(cell, {}).setdefault(phase, {})[ln] = {
            "sent": int(n), "lost": int(lost), "loss_pct": round(100 * lost / n, 3) if n else None,
            "by_kind": {kk[4]: {"sent": int(v), "lost": int(acc.c.get(("lost_kind",) + kk[1:], 0))}
                        for kk, v in acc.c.items() if kk[0] == "sent_kind" and kk[1:4] == k},
            "delay_ms": dist_summary(acc.v.get(("delay",) + k, [])),
            "lp_packets": frames,
            "lp_retransmissions": r.get("retransmissions", 0),
            "identical_resends": r.get("identical_resends", 0),
            "lp_given_up_pct": round(100 * hist.get(4, 0) / frames, 3) if frames else None,
        }
    for key, n in acc.c.items():
        if key[0] == "bin" and key[3] == "sent" and key[5] == "steady":
            _, dim, b, _w, cell, _p, ln = key
            slot = out["bins"].setdefault(dim, {}).setdefault(b, {}).setdefault(cell, {"sent": 0, "lost": 0})
            slot["sent"] += int(n)
            slot["lost"] += int(acc.c.get(("bin", dim, b, "lost", cell, "steady", ln), 0))
    for dim in out["bins"].values():
        for cells in dim.values():
            for slot in cells.values():
                slot["loss_pct"] = round(100 * slot["lost"] / slot["sent"], 3) if slot["sent"] else None
    return out


def context_summary(dash: Dash, periods: Periods, ctx: Context) -> dict:
    """Per stack and vehicle: armed time, altitude, speed and distance while armed."""
    acc = Acc()
    for vid, samples in dash.tel.items():
        prev = None
        for ts, _a, _d, s in samples:
            ph = periods.phase(ts)
            if ph is None or ph[1] != "steady":
                prev = None
                continue
            if s.get("armed"):
                if prev is not None and ts - prev <= POSITION_MAX_AGE_S:
                    acc.add(("armed_s", ph[0], vid), ts - prev)
                for k in ("alt_m", "groundspeed_m_s"):
                    if s.get(k) is not None:
                        acc.put((k, ph[0], vid), s[k])
                d = ctx.distance_m(vid, ts)
                if d is not None:
                    acc.put(("dist", ph[0], vid), d)
            prev = ts if s.get("armed") else None
    out: dict = {}
    for cell in {k[1] for k in list(acc.c) + list(acc.v)}:
        for vid in {k[2] for k in list(acc.c) + list(acc.v) if k[1] == cell}:
            out.setdefault(cell, {})[vid] = {
                "armed_s": round(acc.c.get(("armed_s", cell, vid), 0)),
                "alt_m": dist_summary(acc.v.get(("alt_m", cell, vid), []), qs=(0.5, 0.95)),
                "groundspeed_m_s": dist_summary(acc.v.get(("groundspeed_m_s", cell, vid), []), qs=(0.5, 0.95)),
                "distance_m": dist_summary(acc.v.get(("dist", cell, vid), []), qs=(0.5, 0.95)),
            }
    return out


# ---------------------------------------------------------------------------------------------
# rendering

def stacks_in(*sections) -> List[str]:
    found = set()
    for s in sections:
        if isinstance(s, dict):
            found |= {k for k in s if k != "mixed"}
    return sorted(found, key=lambda c: (STACK_ORDER.index(c) if c in STACK_ORDER else 99, c))


def f(x, unit: str = "", nd: int = 1) -> str:
    if x is None:
        return "–"
    if isinstance(x, float):
        x = round(x, nd)
        if x == int(x) and abs(x) >= 100:
            x = int(x)
    return f"{x}{unit}"


def table(header: List[str], rows: List[List[str]]) -> str:
    if not rows:
        return "_no data_\n"
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out) + "\n"


def side_by_side(per: dict, stacks: List[str], fields: List[Tuple[str, callable]], label: str) -> str:
    """Rows = entity × metric, one column per stack (steady phase)."""
    entities = sorted({e for c in stacks for e in (per.get(c, {}).get("steady") or {})})
    rows = []
    for e in entities:
        for name, get in fields:
            cells = []
            for c in stacks:
                v = (per.get(c, {}).get("steady") or {}).get(e)
                cells.append(get(v) if v is not None else "–")
            if any(x != "–" for x in cells):
                rows.append([e, name] + cells)
    return table([label, "metric"] + stacks, rows)


def bins_table(bins: dict, stacks: List[str], fmt) -> str:
    out = []
    for dim in ("distance", "altitude", "video load"):
        b = bins.get(dim)
        if not b:
            out.append(f"_by {dim}: no data_\n")
            continue
        rows = [[k] + [fmt(b[k].get(c)) for c in stacks] for k in sorted(b, key=_bin_order)]
        out.append(f"By {dim}:\n\n" + table([dim] + stacks, rows))
    return "\n".join(out)


def _bin_order(label: str):
    m = re.match(r"[<>=]*(-?[\d.]+)", label)
    return (0, float(m.group(1))) if m else (1, label)


def render(R: dict) -> str:
    stacks = R["stacks"]
    L = [f"# Flight report — {R['flight']}\n",
         f"Stacks compared: {', '.join(stacks) or 'none found'}. Steady state only (each period's "
         f"first {R['settle_s']} s excluded, see Settling); `n` = samples behind each number. "
         "Distance/altitude/video-load tables pool all vehicles.\n"]
    L.append("## Data inventory\n")
    L.append(table(["node", "vehicle", "cells.log", "link ticks", "wire files", "journals", "other"],
                   [[n, i["vehicle"], i["cells"], str(i["link_ticks"]), str(i["wire_files"]),
                     str(i["journals"]), i["other"]] for n, i in R["inventory"].items()]))
    for note in R["notes"]:
        L.append(f"- {note}")
    L.append("")
    L.append("## Stack periods\n")
    L.append(table(["#", "cell", "start (UTC)", "end", "length s", "steady s", "started by"],
                   [[str(s["i"]), s["cell"], iso(s["start"]), iso(s["end"]), f(s["end"] - s["start"], nd=0),
                     f(max(0.0, s["end"] - s["steady_from"]) if s["steady_from"] else 0.0, nd=0),
                     s["trigger"]] for s in R["periods"]]))
    if R["fabric_events"]:
        L.append("Stack switches from the dashboard:\n")
        L.append(table(["time", "event", "detail"],
                       [[iso(e["ts"]), e.get("kind", ""), json.dumps({k: v for k, v in e.items()
                                                                      if k not in ("ts", "t", "kind", "type")})[:120]]
                        for e in R["fabric_events"]]))
    L.append("## Flight context (confounds)\n")
    L.append("What each stack was asked to carry. Unequal rows mean the stacks did not fly the same "
             "sortie; read the binned tables below before the totals.\n")
    ctxs = R["context"]
    rows = []
    for vid in sorted({v for c in stacks for v in ctxs.get(c, {})}):
        for name, get in (("armed s", lambda v: f(v["armed_s"])),
                          ("alt m p50/p95", lambda v: f"{f(v['alt_m']['p50'])}/{f(v['alt_m']['p95'])}"),
                          ("speed m/s p50/p95", lambda v: f"{f(v['groundspeed_m_s']['p50'])}/{f(v['groundspeed_m_s']['p95'])}"),
                          ("distance m p50/p95 (n)", lambda v: f"{f(v['distance_m']['p50'])}/{f(v['distance_m']['p95'])} ({v['distance_m']['n']})")):
            rows.append([vid, name] + [get(ctxs[c][vid]) if vid in ctxs.get(c, {}) else "–" for c in stacks])
    L.append(table(["vehicle", "while armed"] + stacks, rows) if rows else
             "_no vehicle was armed in a steady period_\n")
    load = R["video"].get("load_s", {})
    loads = sorted({k for c in stacks for k in (load.get(c, {}).get("steady") or {})})
    L.append("Video load (seconds of steady time):\n")
    L.append(table(["load"] + stacks, [[k] + [f((load.get(c, {}).get("steady") or {}).get(k)) for c in stacks]
                                        for k in loads]))
    pres = R["video"].get("presets", {})
    prow = []
    for vid in sorted({v for c in stacks for v in (pres.get(c, {}).get("steady") or {})}):
        for p in sorted({p for c in stacks for p in ((pres.get(c, {}).get("steady") or {}).get(vid) or {})}):
            prow.append([vid, p] + [f(((pres.get(c, {}).get("steady") or {}).get(vid) or {}).get(p)) for c in stacks])
    L.append("Preset on-time (s):\n")
    L.append(table(["vehicle", "preset"] + stacks, prow))

    T = R["telemetry"]
    L.append(f"## Telemetry (dashboard receipt; age source: {T['age_source']})\n")
    L.append(side_by_side(T["per"], stacks, [
        ("samples / rate Hz", lambda v: f"{v['samples']} / {f(v['rate_hz'], nd=2)}"),
        ("gap s p50/p95/max", lambda v: f"{f(v['gap_s']['p50'], nd=2)}/{f(v['gap_s']['p95'], nd=2)}/{f(v['gap_s']['max'], nd=2)}"),
        ("gaps >1 s / >2 s", lambda v: f"{v['gaps_over_1s']} / {v['gaps_over_2s']}"),
        ("age ms p50/p95/p99 (n)", lambda v: f"{f(v['age_ms']['p50'])}/{f(v['age_ms']['p95'])}/{f(v['age_ms']['p99'])} ({v['age_ms']['n']})"),
        ("stale events (max s)", lambda v: f"{v['stale_events']} ({f(v['stale_max_s'])})"),
    ], "vehicle"))
    L.append("Age ms p50/p95 (n):\n")
    L.append(bins_table(T["bins"], stacks, lambda v: "–" if not v else f"{f(v['p50'])}/{f(v['p95'])} ({v['n']})"))

    V = R["video"]
    L.append("## Video (dashboard `video_stats`, ~2 s windows)\n")
    L.append(f"Freezes are lower bounds: a gap G between stats windows proves >= G - "
             f"{VIDEO_STATS_WINDOW_S:g} s without a frame; counted when >= {FREEZE_S * 1000:g} ms.\n")
    L.append(side_by_side(V["per"], stacks, [
        ("windows", lambda v: str(v["stats_windows"])),
        ("fps mean / p5", lambda v: f"{f(v['fps']['mean'])} / {f(v['fps']['p5'])}"),
        ("kbps mean", lambda v: f(v["kbps"]["mean"], nd=0)),
        ("lag ms p50/p95/max", lambda v: f"{f(v['lag_ms']['p50'])}/{f(v['lag_ms']['p95'])}/{f(v['lag_ms']['max'])}"),
        ("freezes n / total s / max s", lambda v: f"{v['freezes']} / {f(v['freeze_total_s'])} / {f(v['freeze_max_s'])}"),
        ("resubscribes", lambda v: str(v["resubscribes"])),
    ], "stream"))
    L.append("Lag ms p50/p95 (n):\n")
    L.append(bins_table(V["bins"]["lag"], stacks, lambda v: "–" if not v else f"{f(v['p50'])}/{f(v['p95'])} ({v['n']})"))

    C = R["commands"]
    L.append(f"## Commands (source: {C['source']})\n")
    L.append(side_by_side(C["per"], stacks, [
        ("ok / failed / refused", lambda v: f"{v['ok']} / {v['failed']} / {v['refused']}"),
        ("retries; attempts mean/max", lambda v: f"{v['retries']}; {f(v['attempts_mean'], nd=2)}/{f(v['attempts_max'])}"),
        ("latency ms p50/p95/max", lambda v: f"{f(v['latency_ms']['p50'], nd=0)}/{f(v['latency_ms']['p95'], nd=0)}/{f(v['latency_ms']['max'], nd=0)}"),
    ], "command"))
    L.append("Latency ms p50/p95 (n):\n")
    L.append(bins_table(C["bins"], stacks, lambda v: "–" if not v else f"{f(v['p50'], nd=0)}/{f(v['p95'], nd=0)} ({v['n']})"))
    L.append("`metric.latency` (role journals; stage/name):\n")
    L.append(side_by_side(C["metric_latency"], stacks, [
        ("ms p50/p95/max (n)", lambda v: f"{f(v['p50'])}/{f(v['p95'])}/{f(v['max'])} ({v['n']})"),
        ("status false", lambda v: str(v["status_false"])),
    ], "stage/name"))

    P = R["ping"]
    L.append("## NDN ping GCS -> vehicle (recorder ping.jsonl, 1 Hz)\n")
    if P is None:
        L.append("_no ping.jsonl in the GCS recorder_\n")
    else:
        if P["cell_mismatch"]:
            L.append(f"{P['cell_mismatch']} probes were labelled with a cell other than the period's "
                     "(the GCS's own view lagging a switch).\n")
        L.append(side_by_side(P["per"], stacks, [
            ("probes / loss %", lambda v: f"{v['probes']} / {f(v['loss_pct'], nd=2)}"),
            ("nacks", lambda v: str(v["nacks"])),
            ("rtt ms p50/p95/p99", lambda v: f"{f(v['rtt_ms']['p50'])}/{f(v['rtt_ms']['p95'])}/{f(v['rtt_ms']['p99'])}"),
        ], "vehicle"))
        L.append("Loss % ; rtt p50/p95 ms (probes):\n")
        L.append(bins_table(P["bins"], stacks, lambda v: "–" if not v else
                            f"{f(v['loss_pct'], nd=2)} ; {f(v['rtt_ms']['p50'])}/{f(v['rtt_ms']['p95'])} ({int(v['probes'])})"))

    L.append("## Radio link per peer (link sampler, 1 Hz)\n")
    L.append("GCS rows are the AP's view of each station (`iw station dump`); vehicle rows are the "
             "rtl88x2eu driver's view of the AP (signal, retry ratio, PHY rate).\n")
    rows = []
    for node, rh in R["radio"].items():
        for c in stacks:
            for peer, v in sorted(((rh["peers"].get(c) or {}).get("steady") or {}).items()):
                rows.append([node, peer, c,
                             f"{f(v['signal_dbm']['p50'])}/{f(v['signal_dbm']['p5'])} ({v['signal_dbm']['n']})",
                             f"{f(v['tx_mbps']['p50'])}/{f(v['rx_mbps']['p50'])}" if v["tx_mbps"]["n"] else
                             f"{v['rates'].get('tx_rate', '–')}/{v['rates'].get('rx_rate', '–')}",
                             f"{v['tx_retries']} ({f(v['retries_per_100_tx'], nd=2)}/100 tx)" if v["tx_mbps"]["n"]
                             else f"ratio p50/p95 {f(v['retry_ratio']['p50'])}/{f(v['retry_ratio']['p95'])}",
                             str(v["tx_failed"]) if v["tx_mbps"]["n"] else "–"])
    L.append(table(["node", "peer", "stack", "signal dBm p50/p5 (n)", "tx/rx rate", "retries", "tx failed"], rows))
    for node, rh in R["radio"].items():
        if rh.get("signal_bins"):
            L.append(f"{node} signal dBm p50/p5 (n):\n")
            L.append(bins_table(rh["signal_bins"], stacks, lambda v: "–" if not v else f"{f(v['p50'])}/{f(v['p5'])} ({v['n']})"))
        if rh.get("retry_bins"):
            L.append(f"{node} 802.11 retries per 100 tx:\n")
            L.append(bins_table(rh["retry_bins"], stacks, lambda v: f(v, nd=2)))
    L.append("## Forwarder host (link sampler)\n")
    rows = []
    for node, rh in R["radio"].items():
        for c in stacks:
            h = (rh["host"].get(c) or {}).get("steady")
            if not h:
                continue
            rows.append([node, c, f"{f(h['fwd_cpu_pct']['p50'])}/{f(h['fwd_cpu_pct']['p95'])}/{f(h['fwd_cpu_pct']['max'])}",
                         f(h["fwd_rss_mb_max"]), f"{h['udp_rcvbuf_errors']} ({f(h['udp_rcvbuf_errors_per_s'], nd=3)}/s)",
                         str(h["udp_sndbuf_errors"]), str(h["socket_drops"]),
                         f"{f(h['udp_txq_bytes']['p95'], nd=0)}/{f(h['udp_txq_bytes']['max'], nd=0)}",
                         f"{f(h['udp_rxq_bytes']['p95'], nd=0)}/{f(h['udp_rxq_bytes']['max'], nd=0)}",
                         str(h["seconds"])])
    L.append(table(["node", "stack", "fwd CPU % p50/p95/max", "RSS MB max", "UDP RcvbufErrors",
                    "SndbufErrors", "socket drops", "tx queue B p95/max", "rx queue B p95/max", "s"], rows))

    W = R["wire"]
    L.append("## Wire loss per directed link (dumpcap ring on both ends)\n")
    if W is None:
        L.append("_no wire captures in any recorder_\n")
    else:
        L.append("A datagram counts when the receiver's capture was running around it. Each arrival "
                 "matches one transmission (ndn-fwd resends LP frames byte-identically, see "
                 "losses.py), so a lost original is lost even when its resend arrived. Loss here is "
                 "after the sender's qdisc and before the receiver's socket. Packets dumpcap itself "
                 "dropped (closed ring files only; they would read as loss): "
                 + ", ".join(f"{n} {d} in {W['closed_files_with_stats'][n]} files"
                             for n, d in sorted(W["capture_drops"].items())) + ".\n")
        L.append(side_by_side(W["per"], stacks, [
            ("sent / lost / loss %", lambda v: f"{v['sent']} / {v['lost']} / {f(v['loss_pct'], nd=3)}"),
            ("delay ms p50/p95/p99", lambda v: f"{f(v['delay_ms']['p50'], nd=2)}/{f(v['delay_ms']['p95'], nd=2)}/{f(v['delay_ms']['p99'], nd=2)}"),
            ("LP frames / retx / identical resends", lambda v: f"{v['lp_packets']} / {v['lp_retransmissions']} / {v['identical_resends']}"),
            ("LP given up %", lambda v: f(v["lp_given_up_pct"], nd=3)),
        ], "link"))
        if W["uncovered"]:
            L.append("Not counted (receiver capture not running around them): "
                     + ", ".join(f"{k} {v}" for k, v in sorted(W["uncovered"].items())) + "\n")
        L.append("Loss % (sent), vehicle<->GCS links:\n")
        L.append(bins_table(W["bins"], stacks, lambda v: "–" if not v else f"{f(v['loss_pct'], nd=3)} ({v['sent']})"))

    L.append("## Settling (first seconds of each period, excluded above)\n")
    rows = []
    for c in stacks:
        tel = (T["per"].get(c) or {}).get("settling") or {}
        ages = [v["age_ms"]["p95"] for v in tel.values() if v["age_ms"]["p95"] is not None]
        pg = ((P or {}).get("per", {}).get(c) or {}).get("settling") or {}
        probes = sum(v["probes"] for v in pg.values())
        lost = sum(v["timeouts"] for v in pg.values())
        rows.append([c, str(sum(v["samples"] for v in tel.values())), f(max(ages) if ages else None),
                     str(sum(v["gaps_over_2s"] for v in tel.values())),
                     f"{probes} / {f(100 * lost / probes if probes else None, nd=2)}"])
    L.append(table(["stack", "telemetry samples", "worst vehicle age p95 ms", "gaps >2 s", "ping probes / loss %"], rows))
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("flight")
    ap.add_argument("--fleet", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fleet.toml"))
    ap.add_argument("--settle-s", type=float, default=SETTLE_S)
    ap.add_argument("--no-wire", action="store_true", help="skip the packet captures (the slowest part)")
    args = ap.parse_args()
    flight = os.path.abspath(args.flight)
    nodes = load_nodes(args.fleet)
    gcs = next((n for n in nodes if n["role"] == "gcs"), None)
    veh_node = {n["vehicle"]: n["name"] for n in nodes}
    notes: List[str] = []

    def log(msg: str) -> None:
        print(f"flight_report: {msg}", file=sys.stderr)

    dash = Dash()
    if gcs:
        dash.load(glob.glob(os.path.join(flight, gcs["name"], "replays", "dash-*.jsonl")))
    log(f"dashboard: {dash.records} records from {len(dash.files)} files")
    journals = {n["name"]: sorted(glob.glob(os.path.join(flight, n["name"], "log", "*.jsonl"))) for n in nodes}

    changes, inventory = {}, {}
    fallback = dash.cell_changes(veh_node)
    for n in nodes:
        rec = os.path.join(flight, n["name"], "recorder")
        ch = read_cells(os.path.join(rec, "cells.log"))
        src = f"{len(ch)} lines"
        if not ch and fallback.get(n["name"]):
            ch = fallback[n["name"]]
            src = f"missing; {len(ch)} changes from the dashboard"
        elif not ch:
            src = "missing"
        if ch:
            changes[n["name"]] = ch
        other = [x for x in ("gps.log",) if os.path.exists(os.path.join(rec, x))]
        offs = chrony_offsets_ms(os.path.join(rec, "chrony.log"))
        if offs:
            # Wire delays and telemetry ages are cross-node differences: this bounds their error.
            other.append(f"chrony max abs offset {max(abs(o) for o in offs):.2f} ms ({len(offs)} readings)")
        if os.path.exists(os.path.join(rec, "ping", "ping.jsonl")):
            other.append("ping")
        if n is gcs:
            other.append(f"{len(dash.files)} replays")
        inventory[n["name"]] = {"vehicle": n["vehicle"], "cells": src, "link_ticks": 0,
                                "wire_files": len(glob.glob(os.path.join(rec, "wire", "*"))),
                                "journals": len(journals[n["name"]]), "other": ", ".join(other) or "–"}
    if not changes:
        notes.append("No cell history on any node (no recorder/cells.log, no fabric tags in the "
                     "dashboard recording): no stack periods, nothing is compared.")
    # The flight ends at the last thing anyone recorded.
    ends = [dash.span()[1] or 0]
    for n in nodes:
        for p in link_files(os.path.join(flight, n["name"], "recorder"))[-1:]:
            ends.append(os.path.getmtime(p))
        ping_p = os.path.join(flight, n["name"], "recorder", "ping", "ping.jsonl")
        if os.path.exists(ping_p):
            ends.append(os.path.getmtime(ping_p))
        for p in glob.glob(os.path.join(flight, n["name"], "recorder", "wire", "*")):
            ends.append(os.path.getmtime(p))
    t_end = max(ends + [t for ch in changes.values() for t, _ in ch[-1:]]) + 1
    periods = Periods(changes, t_end, args.settle_s)

    gcs_pos, fixes = (None, 0)
    if gcs:
        gcs_pos, fixes = gcs_position(os.path.join(flight, gcs["name"], "recorder", "gps.log"))
    if gcs_pos:
        notes.append(f"GCS position: median of {fixes} gpsd TPV fixes ({gcs_pos[0]:.6f}, {gcs_pos[1]:.6f}).")
    else:
        firsts = first_armed_positions(dash)
        if firsts:
            gcs_pos = (sum(p[0] for p in firsts.values()) / len(firsts),
                       sum(p[1] for p in firsts.values()) / len(firsts))
            notes.append("GCS position: NO GPS fix in recorder/gps.log; distances are from the centroid of "
                         f"the vehicles' first armed positions ({', '.join(sorted(firsts))}), i.e. the "
                         "take-off spot, not the GCS antenna.")
        else:
            notes.append("GCS position: no GPS fix and no armed vehicle position: no distance bins.")
    ctx = Context(dash, gcs_pos)
    series = Series()
    t0, t1 = dash.span()

    log("telemetry, video, commands")
    tel = telemetry(dash, periods, ctx, series, veh_node)
    vid = video(dash, periods, ctx, series, veh_node, t0, t1)
    cmd = commands(dash, journals, periods, ctx)
    log("ping")
    png = ping(os.path.join(flight, gcs["name"], "recorder", "ping", "ping.jsonl"), periods, ctx, series,
               veh_node) if gcs else None

    mac_vehicle: Dict[str, str] = {}
    by_addr = {n["addr"]: n["vehicle"] for n in nodes}
    if gcs:
        for p in sorted(glob.glob(os.path.join(flight, gcs["name"], "recorder", "wire", "*")))[:3]:
            for ip, mac in wire.mac_of_addr(p).items():
                if ip in by_addr:
                    mac_vehicle[mac] = by_addr[ip]
    if not mac_vehicle:
        notes.append("Station MACs could not be tied to vehicles (no GCS capture): GCS radio rows are by MAC.")
    radio = {}
    for n in nodes:
        files = link_files(os.path.join(flight, n["name"], "recorder"))
        if not files:
            continue
        log(f"link sampler {n['name']} ({len(files)} files)")
        radio[n["name"]] = radio_and_host(n, files, periods, ctx, series, mac_vehicle)
        inventory[n["name"]]["link_ticks"] = radio[n["name"]]["ticks"]
    wl = None
    if args.no_wire:
        notes.append("Wire captures skipped (--no-wire).")
    else:
        log("wire captures")
        wl = wire_loss(nodes, flight, periods, ctx, series)

    stacks = stacks_in(tel["per"], vid["per"], cmd["per"], (png or {}).get("per"),
                       *[r["host"] for r in radio.values()], (wl or {}).get("per"))
    if any(s["cell"] == "mixed" for s in periods.segments):
        notes.append("Mixed stretches (nodes on different cells) are excluded from every comparison.")
    if tel["age_source"] != "age_ms":
        notes.append(f"Telemetry age: {tel['age_source']}.")
    R = {
        "flight": os.path.basename(flight),
        "settle_s": args.settle_s,
        "stacks": stacks,
        "inventory": inventory,
        "notes": notes,
        "periods": periods.segments,
        "fabric_events": dash.fabric_events,
        "context": context_summary(dash, periods, ctx),
        "telemetry": tel,
        "video": vid,
        "commands": cmd,
        "ping": png,
        "radio": radio,
        "wire": wl,
    }
    with open(os.path.join(flight, "report.json"), "w") as fh:
        json.dump(R, fh, indent=1, default=str)
    with open(os.path.join(flight, "report.md"), "w") as fh:
        fh.write(render(R))
    sdir = os.path.join(flight, "series")
    os.makedirs(sdir, exist_ok=True)
    for node, rows in series.rows.items():
        cols = sorted({c for r in rows.values() for c in r})
        with open(os.path.join(sdir, f"{node}.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_unix", "cell", "phase"] + cols)
            for sec in sorted(rows):
                ph = periods.phase(sec + 0.5)
                w.writerow([sec, ph[0] if ph else "", ph[1] if ph else ""] + [rows[sec].get(c, "") for c in cols])
    log(f"wrote {flight}/report.md, report.json, series/*.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
