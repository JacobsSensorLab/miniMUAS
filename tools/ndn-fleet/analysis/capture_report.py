#!/usr/bin/env python3
"""Loss accounting and freeze attribution for one ndn-fleet capture run (PROTOCOL.md I10).

    capture_report.py <results>/runs/<run_id> [--fleet fleet.toml] [--freeze-ms 500]

Writes <run>/capture/report.json (everything) and <run>/capture/report.md (the summary).

Every node's clock is chrony-synced to the GCS (offsets in chrony-{start,stop}.txt, reported
below); cross-node times are compared as recorded. Times come from each record's own stamp:
NFD's and NDNSF's log timestamps, the frame records' wall clock, the capture's packet time --
never journald's receive time, which lagged by up to 3 s on the GCS.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import journal  # noqa: E402
import link  # noqa: E402
import losses  # noqa: E402
import wire  # noqa: E402

# /muas/v2/<vid>/NDNSF/STREAM-MAP/<stream>/v/<epoch>/seq=<cursor>: a predictive stream item.
ITEM = re.compile(r"^/muas/v2/([^/]+)/NDNSF/STREAM-MAP/([^/]+)/v/[^/]+/seq=(\d+)$")
FACE_ID = re.compile(r"(?:in|out)=(\d+)")


def load_nodes(fleet_toml: str) -> list:
    """[{name, addr, vehicle, role}] from fleet.toml's [[node]] blocks (no toml module on 3.10)."""
    nodes, cur = [], None
    for line in open(fleet_toml):
        line = line.strip()
        if line == "[[node]]":
            cur = {}
            nodes.append(cur)
        elif line.startswith("[") and cur is not None:
            cur = None
        elif cur is not None and "=" in line:
            k, v = (x.strip() for x in line.split("=", 1))
            cur[k] = v.split("#")[0].strip().strip('"')
    return nodes


def chrony_offset_ms(path: str):
    try:
        for line in open(path):
            m = re.match(r"System time\s*:\s*([\d.]+) seconds (fast|slow)", line)
            if m:
                v = float(m.group(1)) * 1000
                return round(v if m.group(2) == "fast" else -v, 3)
    except OSError:
        pass
    return None


def item_key(name):
    m = ITEM.match(name or "")
    return (m.group(2), int(m.group(3))) if m else None


# ---------------------------------------------------------------------------
# per node parsing
# ---------------------------------------------------------------------------


class NodeLogs:
    """Everything one node's journal slice says, indexed for the joins below."""

    def __init__(self, paths: list, peer_faces: set):
        self.fwd = Counter()  # Forwarder event -> count
        self.lp = Counter()  # LpReliability/Reassembler/GLS message class -> count
        self.lp_by_face = defaultdict(Counter)
        self.unsolicited_by_face = Counter()
        self.app_in_interest = {}  # item -> first t (Interest from a local app face)
        self.app_out_data = {}  # item -> first t (Data handed to a local app face)
        self.peer_in_data = {}  # item -> first t (Data from a peer face)
        self.peer_in_interest = {}  # item -> first t (Interest from a peer face)
        self.out_interest_faces = defaultdict(set)  # (item, nonce) -> faces it went out on
        self.finalize = defaultdict(Counter)  # stream -> satisfied/unsatisfied
        self.admitted = {}  # item -> t (NDNSF consumer admitted)
        self.fast_retx = defaultdict(list)  # stream -> [t]
        self.future_interest = {}  # item -> first t
        self.provider = defaultdict(dict)  # event -> {item: t}
        self.dash_frames = defaultdict(list)  # vehicle -> [frame record]
        self.agent_frames = []  # [frame record]
        self.stream_status = defaultdict(list)  # vehicle -> [status]
        self.resubscribes = defaultdict(list)  # vehicle -> [(t, reason)]
        self.peer_faces = set(peer_faces)
        self.lines = 0
        for jt, ident, msg in (x for p, ident in paths for x in journal.lines(p, ident)):
            self.lines += 1
            ev = journal.json_event(msg)
            if ev is not None:
                self._json(jt, ev)
                continue
            rec = journal.ndn_log(msg)
            if rec is None:
                continue
            t, _lvl, module, text = rec
            if module == "nfd.Forwarder":
                self._forwarder(t, text)
            elif module in ("nfd.LpReliability", "nfd.LpReassembler", "nfd.GenericLinkService",
                            "nfd.LpFragmenter", "nfd.Transport", "nfd.UnicastUdpTransport"):
                self._lp(module, text)
            elif module == "ndn_service_framework.StreamFacade":
                self._facade(t, text)
            elif module == "ndn_service_framework.TimelineTrace":
                tl = journal.timeline(text)
                if tl and tl.get("role") == "provider" and tl.get("cursor") is not None:
                    ts = int(tl.get("timestamp_us", 0)) / 1e6
                    self.provider[tl["event"]].setdefault((tl["stream"], tl["cursor"]), ts)

    def _json(self, jt, ev):
        e = ev.get("event", "")
        if e == "dash.video.frame":
            self.dash_frames[ev.get("vehicle")].append(ev)
        elif e == "agent.video.frame":
            self.agent_frames.append(ev)
        elif e == "dash.video.stream_status":
            ev["_t"] = ev["emit_unix_us"] / 1e6 if "emit_unix_us" in ev else jt
            self.stream_status[ev.get("vehicle")].append(ev)
        elif e == "dash.video.stream_resubscribe":
            self.resubscribes[ev.get("vehicle")].append((jt, ev.get("reason")))

    def _forwarder(self, t, text):
        op = text.split(" ", 1)[0]
        self.fwd[op] += 1
        m = re.search(r"(?:interest|data)=(\S+)", text)
        key = item_key(m.group(1)) if m else None
        fm = FACE_ID.search(text)
        face = int(fm.group(1)) if fm else None
        if op == "onDataUnsolicited" and face is not None:
            self.unsolicited_by_face[face] += 1
        if key is None:
            return
        local = face is not None and face not in self.peer_faces
        if op == "onIncomingInterest":
            (self.app_in_interest if local else self.peer_in_interest).setdefault(key, t)
        elif op == "onOutgoingInterest":
            n = re.search(r"nonce=(\w+)", text)
            self.out_interest_faces[(key, n.group(1) if n else "")].add(face)
        elif op == "onIncomingData" and not local:
            self.peer_in_data.setdefault(key, t)
        elif op == "onOutgoingData" and local:
            self.app_out_data.setdefault(key, t)
        elif op == "onInterestFinalize":
            self.finalize[key[0]]["satisfied" if text.endswith("satisfied") and
                                  not text.endswith("unsatisfied") else "unsatisfied"] += 1

    def _lp(self, module, text):
        fm = re.match(r"\[id=(\d+),", text)
        body = re.sub(r"^\[[^\]]*\] ", "", text)
        cls = re.sub(r"\d+", "N", body)[:70]
        self.lp[f"{module.split('.')[1]}: {cls}"] += 1
        if fm:
            self.lp_by_face[int(fm.group(1))][cls] += 1

    def _facade(self, t, text):
        m = re.match(r"(STREAM_[A-Z_]+) stream=(\S+)(?: sequence=(\d+))?", text)
        if not m:
            return
        kind, stream, seq = m.group(1), m.group(2), m.group(3)
        if seq is None:
            return
        key = (stream, int(seq))
        if kind == "STREAM_ITEM_ADMITTED":
            self.admitted.setdefault(key, t)
        elif kind == "STREAM_FAST_RETRANSMIT":
            self.fast_retx[stream].append(t)
        elif kind == "STREAM_FUTURE_INTEREST":
            self.future_interest.setdefault(key, t)


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------


def frame_gaps(frames, freeze_ms):
    frames = sorted(frames, key=lambda f: f.get("complete_unix_us", 0))
    done = [f for f in frames if not f.get("abandoned")]
    ts = [f["complete_unix_us"] / 1e6 for f in done]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    freezes = [(done[i], done[i + 1], gaps[i]) for i in range(len(gaps)) if gaps[i] * 1000 >= freeze_ms]
    return done, gaps, freezes


def pctl(xs, q):
    return losses.pct(xs, q) if xs else None


def attribute(freeze, gcs: NodeLogs, prod: NodeLogs, wire_idx, vid, stream):
    """Follow the cursors of the first frame that arrived late through every hop.

    Returns the hop timeline and the first stage that took >= half the freeze, or was never
    reached, as the attribution.
    """
    prev, nxt, gap = freeze
    t0 = prev["complete_unix_us"] / 1e6
    cursors = range(int(nxt["cursor_first"]), int(nxt["cursor_last"]) + 1)
    # The cursor that completed last is what held the frame back (delivery is in order).
    worst, worst_t = None, -1.0
    for c in cursors:
        t = gcs.admitted.get((stream, c))
        if t is not None and t > worst_t:
            worst, worst_t = c, t
    c = worst if worst is not None else int(nxt["cursor_first"])
    key = (stream, c)
    pub = next((f for f in prod.agent_frames
                if int(f["cursor_first"]) <= c <= int(f["cursor_last"])), None)
    w = wire_idx.get(key, {})
    hops = [
        ("producer published frame", pub and pub["published_unix_us"] / 1e6),
        ("producer NDNSF signed", prod.provider.get("signed-and-materialized", {}).get(key)),
        ("GCS NDNSF expressed Interest (NFD app-face in)", gcs.app_in_interest.get(key)),
        ("GCS wire: Interest out", w.get("gcs_tx_interest")),
        ("producer wire: Interest in", w.get("prod_rx_interest")),
        ("producer NFD: Interest from peer", prod.peer_in_interest.get(key)),
        ("producer NDNSF: Interest arrived", prod.provider.get("payload-interest-arrived", {}).get(key)),
        ("producer NDNSF: Data put", prod.provider.get("data-put", {}).get(key)),
        ("producer wire: Data out (first frag)", w.get("prod_tx_data")),
        ("GCS wire: Data in (last frag of first copy)", w.get("gcs_rx_data")),
        ("GCS NFD: Data from peer", gcs.peer_in_data.get(key)),
        ("GCS NFD: Data to NDNSF (app face)", gcs.app_out_data.get(key)),
        ("GCS NDNSF admitted item", gcs.admitted.get(key)),
        ("dashboard drain took first chunk", nxt["first_chunk_unix_us"] / 1e6),
        ("dashboard frame complete", nxt["complete_unix_us"] / 1e6),
    ]
    timeline = [{"hop": h, "t_rel_ms": None if t is None else round((t - t0) * 1000, 1)} for h, t in hops]
    present = [(h, t) for h, t in hops if t is not None]
    missing = [h for h, t in hops if t is None]
    verdict = "undetermined"
    # The largest step between consecutive recorded hops is where the time went.
    steps = [(b[1] - a[1], a[0], b[0]) for a, b in zip(present, present[1:])]
    if steps:
        big = max(steps)
        if big[0] >= 0.5 * gap:
            verdict = f"{big[0] * 1000:.0f} ms between '{big[1]}' and '{big[2]}'"
    status = [
        {k: s.get(k) for k in ("_t", "next_cursor", "oldest_ready", "ready_q", "in_flight",
                               "pending", "timeouts", "nacks", "lag_ms", "dropped", "trace_dropped")}
        for s in gcs.stream_status.get(vid, []) if t0 - 1.0 <= s["_t"] <= t0 + gap + 1.0
    ]
    return {
        "vehicle": vid,
        "gap_ms": round(gap * 1000),
        "ndnsf_status_around": status,
        "from_unix_s": round(t0, 3),
        "frames": [prev.get("frame"), nxt.get("frame")],
        "blocking_cursor": c,
        "fast_retx_during": sum(1 for t in gcs.fast_retx.get(stream, []) if t0 <= t <= t0 + gap),
        "timeline": timeline,
        "missing_hops": missing,
        "largest_step": verdict,
        "stage": classify(present, gap),
        # A resubscribe (lag or failure) starts a new subscription at the live edge while the
        # old one delivers its last frames: the gap is the resubscribe, not a hop.
        "resubscribe_during": [r for r in gcs.resubscribes.get(vid, []) if t0 - 1.0 <= r[0] <= t0 + gap],
    }


STAGES = [
    ("source", {"producer published frame", "producer NDNSF signed"}),
    ("consumer-ndnsf-request", {"GCS NDNSF expressed Interest (NFD app-face in)"}),
    ("gcs-nfd-out", {"GCS wire: Interest out"}),
    ("air-uplink-interest", {"producer wire: Interest in"}),
    ("producer-host", {"producer NFD: Interest from peer", "producer NDNSF: Interest arrived",
                       "producer NDNSF: Data put", "producer wire: Data out (first frag)"}),
    ("air-downlink-data", {"GCS wire: Data in (last frag of first copy)"}),
    ("gcs-nfd-in", {"GCS NFD: Data from peer", "GCS NFD: Data to NDNSF (app face)"}),
    ("consumer-ndnsf-admission", {"GCS NDNSF admitted item"}),
    # Admitted, but the app's drain thread took it late: NDNSF's in-order release (check the
    # status: next_cursor stuck below the frame with ready_q > 0) or the app process itself.
    ("consumer-release-or-app", {"dashboard drain took first chunk"}),
    ("app-reassembly", {"dashboard frame complete"}),
]


def classify(present, gap):
    """The stage whose hop received the largest step (>= half the gap)."""
    best = None
    for (ha, ta), (hb, tb) in zip(present, present[1:]):
        if best is None or tb - ta > best[0]:
            best = (tb - ta, hb)
    if best is None or best[0] < 0.5 * gap:
        return "spread (no single hop took half the gap)"
    for name, hops in STAGES:
        if best[1] in hops:
            return name
    return "unknown"


def wire_index(gcs_dgrams, prod_dgrams, gcs_addr, prod_addr):
    """item -> first times on the wire at both ends (Interest GCS->producer, Data back)."""
    idx = defaultdict(dict)
    # Data fragments carry the name only in fragment 0; later fragments share the Sequence
    # range [seq0, seq0 + count).
    def data_times(dgrams, src, dst, key_name):
        frag0 = {}
        for d in dgrams:
            if d.src == src and d.dst == dst and d.kind == "data" and d.frag_index == 0 and d.seq is not None:
                k = item_key(d.name)
                if k:
                    frag0.setdefault(d.seq, (k, d.frag_count))
        last = {}
        for d in dgrams:
            if d.src != src or d.dst != dst or d.seq is None:
                continue
            for s0 in (d.seq - d.frag_index,):
                if s0 in frag0:
                    k, n = frag0[s0]
                    rec = last.setdefault((k, s0), [0, 0.0, n])
                    rec[0] += 1
                    rec[1] = max(rec[1], d.t_us / 1e6)
        for (k, _s0), (got, t, n) in last.items():
            if got >= n:
                prev = idx[k].get(key_name)
                if prev is None or t < prev:
                    idx[k][key_name] = t
        for s0, (k, _n) in frag0.items():
            pass

    for d in gcs_dgrams:
        if d.src == gcs_addr and d.dst == prod_addr and d.kind == "interest":
            k = item_key(d.name)
            if k:
                idx[k].setdefault("gcs_tx_interest", d.t_us / 1e6)
    for d in prod_dgrams:
        if d.src == gcs_addr and d.dst == prod_addr and d.kind == "interest":
            k = item_key(d.name)
            if k:
                idx[k].setdefault("prod_rx_interest", d.t_us / 1e6)
        if d.src == prod_addr and d.dst == gcs_addr and d.kind == "data" and d.frag_index == 0:
            k = item_key(d.name)
            if k:
                idx[k].setdefault("prod_tx_data", d.t_us / 1e6)
    data_times(gcs_dgrams, prod_addr, gcs_addr, "gcs_rx_data")
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--fleet", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fleet.toml"))
    ap.add_argument("--freeze-ms", type=float, default=500.0)
    args = ap.parse_args()
    cap = os.path.join(args.run, "capture")
    nodes = [n for n in load_nodes(args.fleet) if os.path.isdir(os.path.join(cap, n["name"]))]
    gcs = next(n for n in nodes if n.get("role") == "gcs" or n.get("vehicle") in ("", "gcs"))
    report = {"run": os.path.basename(args.run.rstrip("/")), "freeze_ms": args.freeze_ms, "nodes": {}}

    dgrams, logs, linkstats = {}, {}, {}
    windows = []
    for n in nodes:
        d = os.path.join(cap, n["name"])
        err = open(os.path.join(d, "dumpcap.err"), errors="replace").read() if os.path.exists(os.path.join(d, "dumpcap.err")) else ""
        pcap = os.path.join(d, "wire.pcapng.gz")
        dgrams[n["name"]] = list(wire.datagrams(pcap)) if os.path.exists(pcap) else []
        # Peer faces from the sampler's `nfdc face list`: NFD's own "added face" lines predate
        # the window.
        remotes = link.nfd_face_remotes(link.first_section(os.path.join(d, "link.txt.gz"), "nfd.faces"))
        peers = {f for f, r in remotes.items() if r.startswith("udp")}
        sources = [(os.path.join(d, "journal.txt.gz"), None)]
        sources += [(os.path.join(d, f), f[4:-7]) for f in sorted(os.listdir(d))
                    if f.startswith("log-") and f.endswith(".log.gz")]
        logs[n["name"]] = NodeLogs(sources, peers)
        linkstats[n["name"]] = link.summarize_node(os.path.join(d, "link.txt.gz"))
        st = int(open(os.path.join(d, "started_ns")).read()) // 1000
        sp = int(open(os.path.join(d, "stopped_ns")).read()) // 1000
        windows.append((st, sp))
        report["nodes"][n["name"]] = {
            "vehicle": n.get("vehicle"), "addr": n.get("addr"),
            "capture_drops": wire.dumpcap_drops(err),
            "datagrams": len(dgrams[n["name"]]),
            "journal_lines": logs[n["name"]].lines,
            "clock_offset_ms": {"start": chrony_offset_ms(os.path.join(d, "chrony-start.txt")),
                                "stop": chrony_offset_ms(os.path.join(d, "chrony-stop.txt"))},
        }
    window = (max(w[0] for w in windows), min(w[1] for w in windows))

    # --- wire: per directed link -------------------------------------------------------
    sent, recv = defaultdict(list), defaultdict(list)
    for n in nodes:
        tx, rx = losses.split_by_link(dgrams[n["name"]], n["addr"])
        for k, v in tx.items():
            sent[k].extend(v)
        for k, v in rx.items():
            recv[k].extend(v)
    wire_links = losses.match(dict(sent), dict(recv), window)
    retx = {f"{k[0]}->{k[1]}": losses.retransmissions(v) for k, v in sent.items()}
    addr_name = {n["addr"]: n["name"] for n in nodes}
    report["wire"] = {}
    for k, v in wire_links.items():
        s, d = k.split("->")
        v["retransmissions"] = retx.get(k)
        v["link"] = f"{addr_name.get(s, s)} -> {addr_name.get(d, d)}"
        v["lost_sample"] = v.pop("lost_datagrams")[:200]
        report["wire"][k] = v

    # --- multicast amplification (GCS forwarder) -----------------------------------------
    g = logs[gcs["name"]]
    fanout = Counter(len(f) for f in g.out_interest_faces.values())
    report["gcs_interest_fanout"] = {
        "item_interests": sum(fanout.values()),
        "faces_per_interest": dict(sorted(fanout.items())),
        "interest_loops": g.fwd.get("onInterestLoop", 0),
        "unsolicited_data": g.fwd.get("onDataUnsolicited", 0),
        "unsolicited_by_face": dict(g.unsolicited_by_face),
    }

    # --- NFD per node ---------------------------------------------------------------------
    report["nfd"] = {n["name"]: {"forwarder": dict(logs[n["name"]].fwd),
                                 "lp": dict(logs[n["name"]].lp.most_common(12))} for n in nodes}

    # --- link/PHY/host -----------------------------------------------------------------
    report["link"] = {}
    for n in nodes:
        s = linkstats[n["name"]]
        if not s.get("ticks"):
            continue
        dl = s["delta"]
        sta = {}
        for k, v in dl.items():
            m = re.match(r"station\.([0-9a-f:]+)\.(tx_packets|tx_retries|tx_failed|rx_packets|rx_drop_misc|beacon_loss)$", k)
            if m:
                sta.setdefault(m.group(1), {})[m.group(2)] = v
        for mac, c in sta.items():
            tp = c.get("tx_packets") or 0
            c["retry_pct"] = round(100 * c.get("tx_retries", 0) / tp, 2) if tp else None
            c["mac_drop_pct"] = round(100 * c.get("tx_failed", 0) / tp, 3) if tp else None
            c["signal_dbm"] = s["last"].get(f"station.{mac}.signal_avg")
            c["tx_bitrate_mbps"] = s["last"].get(f"station.{mac}.tx_bitrate")
        report["link"][n["name"]] = {
            "window_s": round(s["window_s"], 1),
            "stations": sta,
            "qdisc": {k[6:]: v for k, v in dl.items() if k.startswith("qdisc.")},
            "netdev": {k[7:]: v for k, v in dl.items() if k.startswith("netdev.")},
            "udp_socket_drops": dl.get("udp_socket_drops"),
            # Datagrams the kernel delivered to the host but dropped at a full UDP socket (NFD
            # did not read fast enough): lost after this node's capture saw them arrive.
            "udp_rcvbuf_drop_pct": _share(dl.get("snmp.Udp.RcvbufErrors"), dl.get("snmp.Udp.InDatagrams")),
            "udp_snmp": {k[5:]: v for k, v in dl.items() if k.startswith("snmp.Udp.") and v},
            "softnet": {k[8:]: v for k, v in dl.items() if k.startswith("softnet.")},
            "driver": {k[4:]: v for k, v in dl.items() if k.startswith("rtl.") and k[4:] in link.RTL_KEYS.values() or k[4:] in ("data_rx_cnt", "duplicate_cnt")},
            "mac80211_aqm": {k[4:]: v for k, v in dl.items() if k.startswith("aqm.") and v},
            "sta_aqm": {k[8:]: v for k, v in dl.items() if k.startswith("sta_aqm.") and v},
            "survey": {k[7:]: v for k, v in dl.items() if k.startswith("survey.")},
            "busiest_threads": s["busiest_threads"],
        }

    # --- app / NDNSF per stream, freezes ----------------------------------------------
    report["streams"] = {}
    producers = {n["vehicle"]: n for n in nodes if n.get("vehicle") and n is not gcs}
    attributions = []
    for vid, frames in sorted(g.dash_frames.items()):
        done, gaps, freezes = frame_gaps(frames, args.freeze_ms)
        prod = producers.get(vid)
        pl = logs[prod["name"]] if prod else None
        published = len(pl.agent_frames) if pl else None
        pub_ts = sorted(f["published_unix_us"] / 1e6 for f in pl.agent_frames) if pl else []
        pub_gaps = [b - a for a, b in zip(pub_ts, pub_ts[1:])]
        stream = vid
        admitted = [k for k in g.admitted if k[0] == stream]
        provided = [k for k in (pl.provider.get("signed-and-materialized", {}) if pl else {}) if k[0] == stream]
        widx = wire_index(dgrams[gcs["name"]], dgrams[prod["name"]], gcs["addr"], prod["addr"]) if prod else {}
        span = (done[-1]["complete_unix_us"] - done[0]["complete_unix_us"]) / 1e6 if len(done) > 1 else 0
        report["streams"][vid] = {
            "frames_published": published,
            "frames_delivered": len(done),
            "frames_abandoned": sum(1 for f in frames if f.get("abandoned")),
            "delivered_fps": round((len(done) - 1) / span, 2) if span else None,
            "published_fps": round((len(pub_ts) - 1) / ((pub_ts[-1] - pub_ts[0]) or 1), 2) if len(pub_ts) > 1 else None,
            "frame_gap_ms": {"p50": _ms(pctl(gaps, .5)), "p95": _ms(pctl(gaps, .95)), "max": _ms(max(gaps) if gaps else None)},
            "publish_gap_ms": {"p50": _ms(pctl(pub_gaps, .5)), "p95": _ms(pctl(pub_gaps, .95)), "max": _ms(max(pub_gaps) if pub_gaps else None)},
            "freezes": len(freezes),
            "items_signed_by_producer": len(provided),
            "items_admitted_by_consumer": len(admitted),
            "fast_retransmits": len(g.fast_retx.get(stream, [])),
            "interest_finalize": dict(g.finalize.get(stream, {})),
            "resubscribes": g.resubscribes.get(vid, []),
        }
        for fz in freezes:
            if pl:
                attributions.append(attribute(fz, g, pl, widx, vid, stream))
    report["freezes"] = attributions
    for a in attributions:
        if a["resubscribe_during"] or a["frames"][1] < a["frames"][0]:
            a["stage"] = "resubscribe"
    report["freeze_stages"] = dict(Counter(a["stage"] for a in attributions))

    out_json = os.path.join(cap, "report.json")
    with open(out_json, "w") as f:
        json.dump(report, f, indent=1, default=str)
    with open(os.path.join(cap, "report.md"), "w") as f:
        f.write(render_md(report))
    print(render_md(report))


def _share(drops, delivered):
    if drops is None or delivered is None or drops + delivered <= 0:
        return None
    return round(100.0 * drops / (drops + delivered), 2)


def _ms(x):
    return None if x is None else round(x * 1000)


def render_md(r: dict) -> str:
    o = [f"# Capture report {r['run']}", ""]
    o.append("## Capture validity")
    o.append("| node | vehicle | datagrams | capture drops | journal lines | chrony offset ms (start/stop) |")
    o.append("|---|---|---|---|---|---|")
    for n, v in r["nodes"].items():
        c = v["clock_offset_ms"]
        o.append(f"| {n} | {v['vehicle']} | {v['datagrams']} | {v['capture_drops']} | {v['journal_lines']} | {c['start']} / {c['stop']} |")
    o += ["", "## Wire: datagram loss between the two ends' captures (after 802.11 retries)",
          "| link | sent | lost | loss % | data loss % | interest loss % | NDNLP packets retransmitted % / given up % | one-way ms p50/p99/max | longest loss run |",
          "|---|---|---|---|---|---|---|---|---|"]
    for k, v in sorted(r["wire"].items(), key=lambda kv: -kv[1]["sent"]):
        bk = v["by_kind"]
        rt = v.get("retransmissions") or {}
        dly = v["delay_ms"]
        o.append(f"| {v['link']} | {v['sent']} | {v['lost']} | {v['loss_pct']} | "
                 f"{bk.get('data', {}).get('loss_pct')} / frag {bk.get('fragment', {}).get('loss_pct')} | "
                 f"{bk.get('interest', {}).get('loss_pct')} | {rt.get('retransmitted_pct')} / {rt.get('sent_4x_pct')} | "
                 f"{dly['p50']} / {dly['p99']} / {dly['max']} | {v['max_loss_burst']['datagrams']} in {v['max_loss_burst']['ms']} ms |")
    fo = r["gcs_interest_fanout"]
    o += ["", "## GCS forwarder: Interest fan-out and duplicates",
          f"- stream-item Interests sent: {fo['item_interests']}; peer faces per Interest: {fo['faces_per_interest']}",
          f"- Interest loops (duplicate nonce back): {fo['interest_loops']}; unsolicited Data dropped: {fo['unsolicited_data']} (by face {fo['unsolicited_by_face']})"]
    o += ["", "## Link / PHY / host deltas over the window"]
    for n, v in r["link"].items():
        o.append(f"### {n} ({v['window_s']} s)")
        for mac, c in v["stations"].items():
            o.append(f"- station {mac}: tx {c.get('tx_packets')} retries {c.get('tx_retries')} ({c.get('retry_pct')} %), "
                     f"MAC give-ups {c.get('tx_failed')} ({c.get('mac_drop_pct')} %), rx {c.get('rx_packets')}, "
                     f"rx drop misc {c.get('rx_drop_misc')}, signal {c.get('signal_dbm')} dBm, tx rate {c.get('tx_bitrate_mbps')} Mb/s")
        o.append(f"- **UDP receive-buffer drops: {v['udp_rcvbuf_drop_pct']} % of datagrams delivered to the host** (snmp {v['udp_snmp']})")
        o.append(f"- qdisc {v['qdisc']}; netdev {v['netdev']}; softnet {v['softnet']}")
        if v["driver"]:
            o.append(f"- driver (rtl88x2eu) {v['driver']}")
        if v["mac80211_aqm"] or v["sta_aqm"]:
            o.append(f"- mac80211 AQM {v['mac80211_aqm']}; per-station TXQ {v['sta_aqm']}")
        o.append(f"- survey {v['survey']}")
        o.append("- busiest threads (max 1 s CPU %): " + ", ".join(f"{t['comm']}[{t['tid']}] {t['max_cpu_pct_1s']}" for t in v["busiest_threads"][:5]))
    o += ["", "## Streams (operator video)",
          "| vehicle | published fps | delivered fps | frames pub/deliv/abandoned | frame gap ms p50/p95/max | publish gap max ms | freezes | items signed/admitted | fast retx | NFD finalize |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for vid, s in r["streams"].items():
        fg, pg = s["frame_gap_ms"], s["publish_gap_ms"]
        o.append(f"| {vid} | {s['published_fps']} | {s['delivered_fps']} | {s['frames_published']}/{s['frames_delivered']}/{s['frames_abandoned']} | "
                 f"{fg['p50']}/{fg['p95']}/{fg['max']} | {pg['max']} | {s['freezes']} | {s['items_signed_by_producer']}/{s['items_admitted_by_consumer']} | "
                 f"{s['fast_retransmits']} | {s['interest_finalize']} |")
    o += ["", f"## Freezes (frame gap >= {r['freeze_ms']:.0f} ms) by originating stage", f"{r['freeze_stages']}", ""]
    for a in r["freezes"][:20]:
        o.append(f"### {a['vehicle']} {a['gap_ms']} ms at {a['from_unix_s']} (frames {a['frames']}, blocking cursor {a['blocking_cursor']}) -> {a['stage']}")
        o.append(f"largest step: {a['largest_step']}; fast retransmits during: {a['fast_retx_during']}; never recorded: {a['missing_hops']}")
        o.append("| hop | ms after last good frame |")
        o.append("|---|---|")
        for h in a["timeline"]:
            o.append(f"| {h['hop']} | {h['t_rel_ms']} |")
        o.append("")
    return "\n".join(o) + "\n"


if __name__ == "__main__":
    main()
