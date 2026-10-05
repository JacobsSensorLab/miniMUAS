"""Read a fabric capture: pcapng -> UDP/6363 datagrams -> NDNLP fields -> NDN name.

Only the bytes dumpcap kept (snaplen) are available, so decoding stops cleanly wherever the
capture was cut: a datagram always yields its addresses, time and payload digest; the LP
fields and the name come out when they fit.
"""

from __future__ import annotations

import gzip
import hashlib
import struct
from dataclasses import dataclass
from typing import Iterator, Optional

NDN_PORT = 6363

# NDN / NDNLPv2 TLV types.
T_LP_PACKET = 0x64
T_FRAGMENT = 0x50
T_SEQUENCE = 0x51
T_FRAG_INDEX = 0x52
T_FRAG_COUNT = 0x53
T_TX_SEQUENCE = 0x348
T_ACK = 0x344
T_NACK = 0x320
T_INTEREST = 0x05
T_DATA = 0x06
T_NAME = 0x07


@dataclass
class Datagram:
    t_us: int  # capture time, unix microseconds
    src: str
    dst: str
    sport: int
    dport: int
    length: int  # UDP payload length on the wire
    digest: bytes  # of the captured UDP payload: identifies one transmission
    seq: Optional[int] = None
    frag_index: int = 0
    frag_count: int = 1
    tx_seq: Optional[int] = None
    acks: int = 0
    nack: bool = False
    kind: str = ""  # "interest" | "data" | "ack-only" | "other" | "" (not decoded)
    name: Optional[str] = None


def _read_var(buf: bytes, i: int):
    if i >= len(buf):
        raise IndexError
    b = buf[i]
    if b < 253:
        return b, i + 1
    if b == 253:
        return struct.unpack_from(">H", buf, i + 1)[0], i + 3
    if b == 254:
        return struct.unpack_from(">I", buf, i + 1)[0], i + 5
    return struct.unpack_from(">Q", buf, i + 1)[0], i + 9


def _tlv(buf: bytes, i: int):
    """-> (type, value_start, value_end, next) ; value_end may exceed len(buf) when cut."""
    t, i = _read_var(buf, i)
    n, i = _read_var(buf, i)
    return t, i, i + n, i + n


def _nonneg(buf: bytes) -> int:
    return int.from_bytes(buf, "big") if buf else 0


def _component_uri(t: int, v: bytes) -> str:
    if t == 0x08:  # GenericNameComponent
        out = []
        for c in v:
            ch = chr(c)
            if ch.isalnum() or ch in "-._~":
                out.append(ch)
            else:
                out.append("%%%02X" % c)
        s = "".join(out)
        return s if s.strip(".") else s + "..."
    if t == 0x32:  # SegmentNameComponent
        return "seg=%d" % _nonneg(v)
    if t == 0x36:  # VersionNameComponent
        return "v=%d" % _nonneg(v)
    if t == 0x3A:  # SequenceNumNameComponent
        return "seq=%d" % _nonneg(v)
    if t == 0x34:  # ByteOffset
        return "off=%d" % _nonneg(v)
    return "%d=%s" % (t, v.hex())


def _name(buf: bytes, start: int, end: int) -> Optional[str]:
    """URI of a Name TLV value; None if the capture cut it."""
    if end > len(buf):
        return None
    parts, i = [], start
    while i < end:
        t, vs, ve, i = _tlv(buf, i)
        if ve > end:
            return None
        parts.append(_component_uri(t, buf[vs:ve]))
    return "/" + "/".join(parts)


def decode_lp(d: Datagram, payload: bytes) -> None:
    """Fill the NDNLP/NDN fields of `d` from the captured payload (best effort)."""
    try:
        t, vs, ve, _ = _tlv(payload, 0)
        if t != T_LP_PACKET:
            # Bare NDN packet (no LP framing).
            _decode_net(d, payload, 0)
            return
        i = vs
        end = min(ve, len(payload))
        while i < end:
            ft, fvs, fve, i = _tlv(payload, i)
            if ft == T_FRAGMENT:
                if d.frag_index == 0:
                    _decode_net(d, payload, fvs)
                else:
                    d.kind = "fragment"
                return
            v = payload[fvs:min(fve, len(payload))]
            if ft == T_SEQUENCE:
                d.seq = _nonneg(v)
            elif ft == T_FRAG_INDEX:
                d.frag_index = _nonneg(v)
            elif ft == T_FRAG_COUNT:
                d.frag_count = _nonneg(v)
            elif ft == T_TX_SEQUENCE:
                d.tx_seq = _nonneg(v)
            elif ft == T_ACK:
                d.acks += 1
            elif ft == T_NACK:
                d.nack = True
        d.kind = "ack-only"
    except (IndexError, struct.error):
        pass


def _decode_net(d: Datagram, buf: bytes, i: int) -> None:
    t, vs, ve, _ = _tlv(buf, i)
    if t == T_INTEREST:
        d.kind = "interest"
    elif t == T_DATA:
        d.kind = "data"
    else:
        d.kind = "other"
        return
    nt, nvs, nve, _ = _tlv(buf, vs)
    if nt == T_NAME:
        d.name = _name(buf, nvs, nve)


def _ipv4_udp(frame: bytes, linktype: int):
    """-> (src, dst, sport, dport, udp_len, payload) for an IPv4 UDP frame, else None."""
    if linktype == 1:  # Ethernet
        if len(frame) < 14:
            return None
        et = struct.unpack_from(">H", frame, 12)[0]
        off = 14
        if et == 0x8100:  # VLAN
            et = struct.unpack_from(">H", frame, 16)[0]
            off = 18
        if et != 0x0800:
            return None
    elif linktype == 113:  # Linux cooked (SLL)
        if len(frame) < 16 or struct.unpack_from(">H", frame, 14)[0] != 0x0800:
            return None
        off = 16
    elif linktype == 276:  # SLL2
        if len(frame) < 20 or struct.unpack_from(">H", frame, 0)[0] != 0x0800:
            return None
        off = 20
    elif linktype in (101, 228):  # raw IP / IPv4
        off = 0
    else:
        return None
    if len(frame) < off + 20 or frame[off] >> 4 != 4 or frame[off + 9] != 17:
        return None
    ihl = (frame[off] & 0x0F) * 4
    frag = struct.unpack_from(">H", frame, off + 6)[0]
    if frag & 0x1FFF:  # non-first IP fragment: no UDP header
        return None
    src = ".".join(str(b) for b in frame[off + 12 : off + 16])
    dst = ".".join(str(b) for b in frame[off + 16 : off + 20])
    u = off + ihl
    if len(frame) < u + 8:
        return None
    sport, dport, ulen = struct.unpack_from(">HHH", frame, u)
    return src, dst, sport, dport, ulen - 8, frame[u + 8 :]


def datagrams(path: str) -> Iterator[Datagram]:
    """Every UDP/6363 IPv4 datagram in a (gzipped) pcapng file, in file order."""
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rb") as f:
        data = f.read()
    i, endian = 0, "<"
    ifaces: list = []  # (linktype, ticks per second)
    while i + 12 <= len(data):
        btype = struct.unpack_from(endian + "I", data, i)[0]
        if btype == 0x0A0D0D0A:  # Section Header: byte order may change per section
            magic = data[i + 8 : i + 12]
            endian = "<" if magic == b"\x4d\x3c\x2b\x1a" else ">"
            ifaces = []
        blen = struct.unpack_from(endian + "I", data, i + 4)[0]
        if blen < 12 or i + blen > len(data):
            break
        body = data[i + 8 : i + blen - 4]
        if btype == 1:  # Interface Description
            linktype = struct.unpack_from(endian + "H", body, 0)[0]
            tps = 10**6
            j = 8
            while j + 4 <= len(body):
                code, olen = struct.unpack_from(endian + "HH", body, j)
                if code == 0:
                    break
                if code == 9 and olen >= 1:  # if_tsresol
                    r = body[j + 4]
                    tps = 2 ** (r & 0x7F) if r & 0x80 else 10**r
                j += 4 + ((olen + 3) & ~3)
            ifaces.append((linktype, tps))
        elif btype == 6:  # Enhanced Packet
            iface, th, tl, caplen, _orig = struct.unpack_from(endian + "IIIII", body, 0)
            frame = body[20 : 20 + caplen]
            linktype, tps = ifaces[iface] if iface < len(ifaces) else (1, 10**6)
            ip = _ipv4_udp(frame, linktype)
            if ip is not None:
                src, dst, sport, dport, ulen, payload = ip
                if NDN_PORT in (sport, dport):
                    ts = (th << 32) | tl
                    d = Datagram(
                        t_us=ts * 10**6 // tps, src=src, dst=dst, sport=sport, dport=dport,
                        length=ulen, digest=hashlib.blake2b(payload, digest_size=12).digest(),
                    )
                    decode_lp(d, payload)
                    yield d
        i += blen


def dumpcap_drops(err_text: str) -> Optional[int]:
    """Packets dumpcap itself dropped, from its closing statistics line."""
    import re

    m = re.search(r"received/dropped on interface '[^']+': (\d+)/(\d+)", err_text)
    return int(m.group(2)) if m else None
