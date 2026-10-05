"""Parse the capture's link/PHY/host sampler (`link.txt.gz`): one tick per `@@ <unix ns>` line,
then `@ <section>` blocks with the raw output of each probe."""

from __future__ import annotations

import gzip
import re
from typing import Dict, Iterator, List, Tuple

Tick = Tuple[int, Dict[str, str]]  # (unix ns, section -> raw text)


def ticks(path: str) -> Iterator[Tick]:
    opener = gzip.open if path.endswith(".gz") else open
    t, sections, name, buf = None, {}, None, []
    with opener(path, "rt", errors="replace") as f:
        for line in f:
            if line.startswith("@@ "):
                if t is not None:
                    if name is not None:
                        sections[name] = "".join(buf)
                    yield t, sections
                t, sections, name, buf = int(line[3:].strip() or 0), {}, None, []
            elif line.startswith("@ "):
                if name is not None:
                    sections[name] = "".join(buf)
                name, buf = line[2:].strip(), []
            else:
                buf.append(line)
    if t is not None:
        if name is not None:
            sections[name] = "".join(buf)
        yield t, sections


def _int(s: str) -> int:
    try:
        return int(s)
    except ValueError:
        return 0


def stations(text: str) -> Dict[str, Dict[str, float]]:
    """`iw dev X station dump` -> {mac: counters}."""
    out: Dict[str, Dict[str, float]] = {}
    cur = None
    for line in text.splitlines():
        m = re.match(r"Station ([0-9a-f:]{17})", line)
        if m:
            cur = out.setdefault(m.group(1), {})
            continue
        if cur is None or ":" not in line:
            continue
        k, v = line.strip().split(":", 1)
        v = v.strip()
        num = re.match(r"-?\d+(\.\d+)?", v)
        if not num:
            continue
        key = k.strip().replace(" ", "_")
        cur[key] = float(num.group(0))
    return out


def survey_in_use(text: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    block = None
    for line in text.splitlines():
        if "frequency:" in line:
            block = "[in use]" in line
            continue
        if block:
            m = re.match(r"\s*channel (\w+(?: \w+)?) time:\s+(\d+) ms", line)
            if m:
                out[m.group(1).replace(" ", "_") + "_ms"] = float(m.group(2))
            m = re.match(r"\s*noise:\s+(-?\d+) dBm", line)
            if m:
                out["noise_dbm"] = float(m.group(1))
    return out


def netdev(text: str) -> Dict[str, float]:
    return {
        line.split(":")[0].rsplit("/", 1)[-1]: float(_int(line.rsplit(":", 1)[-1].strip()))
        for line in text.splitlines()
        if ":" in line
    }


def qdisc(text: str) -> Dict[str, float]:
    """Sum over every qdisc on the device of the counters `tc -s` prints."""
    out = {"sent_pkts": 0.0, "dropped": 0.0, "overlimits": 0.0, "requeues": 0.0}
    for m in re.finditer(
        r"Sent \d+ bytes (\d+) pkt \(dropped (\d+), overlimits (\d+) requeues (\d+)\)", text
    ):
        out["sent_pkts"] += int(m.group(1))
        out["dropped"] += int(m.group(2))
        out["overlimits"] += int(m.group(3))
        out["requeues"] += int(m.group(4))
    return out


def snmp(text: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    lines = text.splitlines()
    for i in range(0, len(lines) - 1, 2):
        head, vals = lines[i].split(), lines[i + 1].split()
        if head and vals and head[0] == vals[0]:
            proto = head[0].rstrip(":")
            for k, v in zip(head[1:], vals[1:]):
                out[f"{proto}.{k}"] = float(_int(v))
    return out


def udp_socket_drops(text: str) -> float:
    """Sum of the `drops` column of the port-6363 sockets in /proc/net/udp{,6}."""
    total = 0
    for line in text.splitlines():
        cols = line.split()
        if len(cols) >= 13 and cols[-1].isdigit():
            total += int(cols[-1])
    return float(total)


def softnet(text: str) -> Dict[str, float]:
    out = {"processed": 0.0, "dropped": 0.0, "time_squeeze": 0.0}
    for line in text.splitlines():
        c = line.split()
        if len(c) >= 3:
            out["processed"] += int(c[0], 16)
            out["dropped"] += int(c[1], 16)
            out["time_squeeze"] += int(c[2], 16)
    return out


def cpu_total(text: str) -> float:
    for line in text.splitlines():
        if line.startswith("cpu "):
            return float(sum(_int(x) for x in line.split()[1:]))
    return 0.0


def tasks(text: str) -> Dict[int, Tuple[str, float]]:
    """/proc/<pid>/task/<tid>/stat lines -> {tid: (comm, utime+stime ticks)}."""
    out: Dict[int, Tuple[str, float]] = {}
    for line in text.splitlines():
        m = re.match(r"(\d+) \((.*)\) (.*)", line)
        if not m:
            continue
        rest = m.group(3).split()
        if len(rest) > 12:
            out[int(m.group(1))] = (m.group(2), float(_int(rest[11]) + _int(rest[12])))
    return out


RTL_KEYS = {
    "RX: Count of Packets dropped by Driver": "rx_dropped_by_driver",
    "Rx: Packet Loss Counts": "rx_packet_loss",
    "Rx: Reorder Time-out Trigger Counts": "rx_reorder_timeouts",
    "Rx: Counts of Packets Whose Seq_Num Less Than Reorder Control Seq_Num": "rx_seq_behind_window",
    "Rx: AMPDU BA window shift Count": "rx_ba_window_shift",
    "Rx: Duplicate Management Frame Drop Count": "rx_dup_mgmt_drop",
}


def rtl(sections: Dict[str, str]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for line in sections.get("rtl.trx_info", "").splitlines():
        for k, name in RTL_KEYS.items():
            if line.startswith(k + ":"):
                out[name] = float(_int(line.rsplit(":", 1)[-1].strip()))
        m = re.match(r"free_xmitbuf_cnt=(\d+), free_xmitframe_cnt=(\d+)", line)
        if m:
            out["free_xmitbuf"], out["free_xmitframe"] = float(m.group(1)), float(m.group(2))
    for line in sections.get("rtl.trx_info_debug", "").splitlines():
        m = re.match(r"(curr_retry_ratio|rssi)\s*:\s*(\d+)", line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def aqm(text: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for line in text.splitlines():
        c = line.split()
        if len(c) == 3 and c[0] in ("R", "RW") and c[2].isdigit():
            out[c[1]] = float(c[2])
    return out


def sta_aqm(text: str) -> Dict[str, float]:
    """Per-station TXQ table: sum the drops/overlimit/collision/backlog columns over TIDs."""
    lines = [l.split() for l in text.splitlines() if l.strip()]
    header = next((l for l in lines if "drops" in l), None)
    if header is None:
        return {}
    idx = {k: header.index(k) for k in ("backlog-packets", "drops", "overlimit", "tx-packets") if k in header}
    out = {k: 0.0 for k in idx}
    for row in lines[lines.index(header) + 1 :]:
        if len(row) != len(header):
            continue
        for k, i in idx.items():
            out[k] += float(_int(row[i]))
    return out


def nfd_faces(text: str) -> Dict[str, Dict[str, float]]:
    """`nfdc face list` -> {remote: in/out interests, data, nacks, bytes}."""
    out: Dict[str, Dict[str, float]] = {}
    for line in text.splitlines():
        rm = re.search(r"remote=(\S+)", line)
        cm = re.search(r"counters=\{in=\{(\d+)i (\d+)d (\d+)n (\d+)B\} out=\{(\d+)i (\d+)d (\d+)n (\d+)B\}\}", line)
        if rm and cm:
            v = [float(x) for x in cm.groups()]
            out[rm.group(1)] = dict(zip(
                ["in_interests", "in_data", "in_nacks", "in_bytes",
                 "out_interests", "out_data", "out_nacks", "out_bytes"], v))
    return out


def nfd_face_remotes(text: str) -> Dict[int, str]:
    """`nfdc face list` -> {faceid: remote URI}."""
    out: Dict[int, str] = {}
    for line in text.splitlines():
        m = re.search(r"faceid=(\d+) remote=(\S+)", line)
        if m:
            out[int(m.group(1))] = m.group(2)
    return out


def first_section(path: str, name: str) -> str:
    """The raw text of `name` in the first tick that has it."""
    for _t, s in ticks(path):
        if s.get(name):
            return s[name]
    return ""


def nfd_status(text: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for line in text.splitlines():
        m = re.match(r"\s*(n[A-Z]\w+)=(\d+)", line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def summarize_node(path: str) -> dict:
    """Window deltas of every counter, plus per-second series of the loss counters."""
    first: dict = {}
    last: dict = {}
    series: List[dict] = []
    task_prev: Dict[int, Tuple[str, float]] = {}
    cpu_prev = None
    busiest: Dict[int, float] = {}
    t_first = t_last = None
    # The collector is stopped by a signal, so its last tick can be cut mid-write: only ticks
    # with (nearly) every section count, or a missing section would read as a counter reset.
    all_ticks = list(ticks(path))
    full = max((len(s) for _, s in all_ticks), default=0)
    for t, s in all_ticks:
        if len(s) < full - 1:
            continue
        snap = {
            "station": stations(s.get("station", "")),
            "survey": survey_in_use(s.get("survey", "")),
            "netdev": netdev(s.get("netdev", "")),
            "qdisc": qdisc(s.get("qdisc", "")),
            "snmp": snmp(s.get("snmp", "")),
            "udp_socket_drops": udp_socket_drops(s.get("udp", "")),
            "softnet": softnet(s.get("softnet", "")),
            "rtl": rtl(s),
            "aqm": aqm(s.get("mac80211.aqm", "")),
            "sta_aqm": {k.split(".")[1]: sta_aqm(v) for k, v in s.items()
                        if k.startswith("sta.") and k.endswith(".aqm")},
            "nfd_faces": nfd_faces(s.get("nfd.faces", "")),
            "nfd_status": nfd_status(s.get("nfd.status", "")),
        }
        cpu = cpu_total(s.get("cpu", ""))
        tk = tasks(s.get("tasks", ""))
        if cpu_prev is not None and cpu > cpu_prev:
            # utime/stime and /proc/stat are both in clock ticks; a thread's share of ONE cpu
            # = d(thread) / (d(all cpus) / ncpu). ncpu = 4 on every node.
            per_cpu = (cpu - cpu_prev) / 4.0
            for tid, (comm, v) in tk.items():
                if tid in task_prev:
                    pct = 100.0 * (v - task_prev[tid][1]) / per_cpu
                    busiest[tid] = max(busiest.get(tid, 0.0), pct)
        cpu_prev, task_prev = cpu, tk
        if t_first is None:
            t_first, first = t, snap
        t_last, last = t, snap
        series.append({"t_ns": t, **_flat(snap)})
    if t_first is None:
        return {"ticks": 0}
    delta = _delta(_flat(first), _flat(last))
    names = {tid: comm for tid, (comm, _) in task_prev.items()}
    hot = sorted(busiest.items(), key=lambda kv: -kv[1])[:8]
    return {
        "ticks": len(series),
        "window_s": (t_last - t_first) / 1e9,
        "delta": delta,
        "last": _flat(last),
        "busiest_threads": [{"tid": tid, "comm": names.get(tid, "?"), "max_cpu_pct_1s": round(p, 1)}
                            for tid, p in hot],
        "series": series,
    }


def _flat(d: dict, prefix: str = "") -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(_flat(v, key + "."))
        elif isinstance(v, (int, float)):
            out[key] = float(v)
    return out


def _delta(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    return {k: b[k] - a[k] for k in b if k in a}
