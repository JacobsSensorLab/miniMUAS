"""End-to-end datagram loss and one-way delay per directed link, from the two ends' captures.

A datagram A->B is "sent" when A's capture saw it leave and "received" when B's capture saw it
arrive, matched on the digest of the captured UDP payload. NFD's NDNLP reliability gives every
transmission its own TxSequence, so a payload digest is unique per transmission, retransmissions
included. What the sender's capture sees is what the kernel handed the driver, after the qdisc;
what the receiver's capture sees is what its driver delivered. A difference is therefore lost
in the driver, the radio, the air or the AP's relay, after any 802.11 retries -- not in NFD and
not in a socket buffer (those are counted separately from the host counters).

Clocks are chrony-synced (offsets recorded per node); delays are reported as measured.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from statistics import median
from typing import Dict, List, Tuple

from wire import Datagram

LinkKey = Tuple[str, str]  # (src addr, dst addr)


def pct(xs: List[float], q: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * q))]


def match(sent: Dict[LinkKey, List[Datagram]], received: Dict[LinkKey, List[Datagram]],
          window_us: Tuple[int, int]) -> dict:
    """Per link: sent/received/lost, loss by kind and per second, delays.

    Only datagrams sent inside both captures' common window count (`window_us`, trimmed by 1 s
    at each end so in-flight packets at start/stop are not called lost).
    """
    lo, hi = window_us[0] + 1_000_000, window_us[1] - 1_000_000
    out = {}
    for key, tx in sorted(sent.items()):
        rx_by_digest: Dict[bytes, int] = {}
        for d in received.get(key, []):
            rx_by_digest.setdefault(d.digest, d.t_us)
        n = lost = 0
        by_kind = defaultdict(lambda: [0, 0])
        per_s = defaultdict(lambda: [0, 0])
        delays: List[float] = []
        lost_list: List[dict] = []
        for d in tx:
            if not lo <= d.t_us <= hi:
                continue
            n += 1
            k = d.kind or "undecoded"
            by_kind[k][0] += 1
            sec = d.t_us // 1_000_000
            per_s[sec][0] += 1
            t_rx = rx_by_digest.get(d.digest)
            if t_rx is None:
                lost += 1
                by_kind[k][1] += 1
                per_s[sec][1] += 1
                lost_list.append({"t_us": d.t_us, "kind": k, "name": d.name, "seq": d.seq,
                                  "frag": f"{d.frag_index}/{d.frag_count}", "tx_seq": d.tx_seq,
                                  "len": d.length})
            else:
                delays.append((t_rx - d.t_us) / 1000.0)
        worst = sorted(((l / c, s, c, l) for s, (c, l) in per_s.items() if c >= 20), reverse=True)[:5]
        out[f"{key[0]}->{key[1]}"] = {
            "sent": n,
            "lost": lost,
            "loss_pct": round(100.0 * lost / n, 3) if n else None,
            "by_kind": {k: {"sent": c, "lost": l, "loss_pct": round(100.0 * l / c, 3) if c else None}
                        for k, (c, l) in sorted(by_kind.items())},
            "delay_ms": {"p50": round(pct(delays, .5), 2), "p95": round(pct(delays, .95), 2),
                         "p99": round(pct(delays, .99), 2), "max": round(max(delays), 2) if delays else None},
            "worst_seconds": [{"unix_s": s, "sent": c, "lost": l, "loss_pct": round(100 * r, 1)}
                              for r, s, c, l in worst],
            "max_loss_burst": _longest_burst(tx, rx_by_digest, lo, hi),
            "lost_datagrams": lost_list,
        }
    return out


def _longest_burst(tx: List[Datagram], rx: Dict[bytes, int], lo: int, hi: int) -> dict:
    """Longest run of consecutive lost datagrams on the link, and how long it lasted."""
    best = (0, 0, 0)
    run, start = 0, 0
    for d in tx:
        if not lo <= d.t_us <= hi:
            continue
        if d.digest not in rx:
            if run == 0:
                start = d.t_us
            run += 1
            if run > best[0]:
                best = (run, start, d.t_us)
        else:
            run = 0
    return {"datagrams": best[0], "from_unix_us": best[1], "ms": round((best[2] - best[1]) / 1000, 1)}


def split_by_link(dgrams: List[Datagram], local: str) -> Tuple[Dict[LinkKey, List[Datagram]],
                                                              Dict[LinkKey, List[Datagram]]]:
    """A node's capture as (sent by it, received by it), keyed by (src, dst)."""
    tx, rx = defaultdict(list), defaultdict(list)
    for d in dgrams:
        if d.src == local:
            tx[(d.src, d.dst)].append(d)
        elif d.dst == local:
            rx[(d.src, d.dst)].append(d)
    return tx, rx


def retransmissions(tx: List[Datagram]) -> dict:
    """NDNLP retransmissions seen on the wire: the same (Sequence, FragIndex) sent again."""
    seen: Dict[Tuple[int, int], int] = {}
    retx = 0
    for d in tx:
        if d.seq is None or d.kind == "ack-only":
            continue
        k = (d.seq, d.frag_index)
        if k in seen:
            retx += 1
        seen[k] = seen.get(k, 0) + 1
    frames = len(seen)
    sends = Counter(seen.values())
    return {"lp_packets": frames, "retransmissions": retx,
            "retx_per_packet_pct": round(100.0 * retx / frames, 3) if frames else None,
            "retransmitted_pct": round(100.0 * (frames - sends.get(1, 0)) / frames, 2) if frames else None,
            # NFD retries 3 times: a packet sent 4 times was given up on.
            "sent_4x_pct": round(100.0 * sends.get(4, 0) / frames, 2) if frames else None,
            "sends_histogram": dict(sorted(sends.items()))}


def median_or_none(xs: List[float]):
    return round(median(xs), 2) if xs else None
