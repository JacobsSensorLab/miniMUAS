"""Parse a capture's journal slice (`journalctl -o short-unix`): NDNSF's stream timeline, the
miniMUAS JSON events, and NFD's own log lines."""

from __future__ import annotations

import gzip
import json
import re
from typing import Iterator, Optional, Tuple

# "<unix.usec> <host> <ident>[<pid>]: <message>"
_JOURNAL = re.compile(r"^(\d+\.\d+) \S+ ([^\[:]+)(?:\[\d+\])?: (.*)$")
# ndn-cxx log prefix inside the message: "<unix.usec> <LEVEL>: [<module>] <text>"
_NDN_LOG = re.compile(r"^(\d+\.\d+)\s+([A-Z]+): \[([^\]]+)\] (.*)$")
_KV = re.compile(r"(\w+)=(\S+)")


def lines(path: str, plain_ident: Optional[str] = None) -> Iterator[Tuple[float, str, str]]:
    """-> (unix s, unit ident, message).

    A journald slice yields journald's receive time. A capture's RAM log (a traced unit's
    stderr, `plain_ident` naming the unit) has no journald prefix: its ndn-cxx lines yield
    their own time; anything else in it (a Python traceback) is skipped.
    """
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")
            m = _JOURNAL.match(line)
            if m and plain_ident is None:
                yield float(m.group(1)), m.group(2).strip(), m.group(3)
            elif plain_ident is not None:
                n = _NDN_LOG.match(line)
                if n:
                    yield float(n.group(1)), plain_ident, line


def ndn_log(msg: str) -> Optional[Tuple[float, str, str, str]]:
    """ndn-cxx/NFD log line -> (own unix s, level, module, text)."""
    m = _NDN_LOG.match(msg)
    if not m:
        return None
    return float(m.group(1)), m.group(2), m.group(3), m.group(4)


def timeline(text: str) -> Optional[dict]:
    """`NDNSF_TIMELINE role=.. event=.. steady_us=.. timestamp_us=.. requestId=.. k=v ...`.

    For stream events the requestId is /NDNSF/STREAM/TIMELINE/<stream>/<epoch>/<cursor>, the
    last two as NonNegativeInteger components ("%01%A3" style), decoded here to ints.
    """
    i = text.find("NDNSF_TIMELINE ")
    if i < 0:
        return None
    rec = dict(_KV.findall(text[i:]))
    rid = rec.get("requestId", "")
    if rid.startswith("/NDNSF/STREAM/TIMELINE/"):
        parts = rid.split("/")
        if len(parts) >= 7:
            rec["stream"] = parts[4]
            rec["epoch"] = _nni(parts[5])
            rec["cursor"] = _nni(parts[6])
    return rec


def _nni(component: str) -> Optional[int]:
    """A NonNegativeInteger name component as printed in a URI (percent-encoded bytes)."""
    raw = bytearray()
    i = 0
    while i < len(component):
        if component[i] == "%" and i + 2 < len(component) + 1:
            raw.append(int(component[i + 1 : i + 3], 16))
            i += 3
        else:
            raw.append(ord(component[i]))
            i += 1
    if not raw or len(raw) > 8:
        return None
    return int.from_bytes(bytes(raw), "big")


def json_event(msg: str) -> Optional[dict]:
    if not msg.startswith("{"):
        return None
    try:
        return json.loads(msg)
    except json.JSONDecodeError:
        return None
