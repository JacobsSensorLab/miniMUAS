"""At-least-once service commands.

NDNSF delivers a service request at most once: it is one SVS publication and
nothing retransmits it (ServiceUser has no retransmit path). Measured on the
fleet 2026-09-25 by restarting iuas-01's agent while sending a video command
every 5 s: the 6 commands sent while the agent was starting and 2 of the 3
that arrived while it waited for its decryption key were silently lost (15 s
timeouts); every later command was handled in 50-100 ms. Operator commands
must not be lost, so:

- the sender re-issues the same command until it is answered or its deadline
  passes (`CommandSender` in run_dashboard, via `with_command_id`), and
- the vehicle executes each command once: every command carries a
  `command_id`, and `CommandDeduplicator` returns the stored response for an id
  it has already executed, so a re-issued takeoff or capture never runs twice.

A re-issue that arrives while the first delivery is still executing (a takeoff
or an investigation runs for many seconds) is answered at once with an
IN_PROGRESS error rather than waiting: the provider has only a few handler
workers, and parking duplicates on them could starve an RTL. The sender ignores
IN_PROGRESS and keeps re-issuing; once the command completes, the next
re-issue gets the stored result, which also recovers a lost final response.
"""

from __future__ import annotations

import functools
import json
import threading
import time
from collections import OrderedDict
from typing import Any, Callable, Optional

COMMAND_ID_KEY = "command_id"
IN_PROGRESS = "command-in-progress"

# How long a re-issue waits for the first delivery of the same command to
# finish before answering IN_PROGRESS: long enough for a fast command that
# merely raced its re-issue, short enough never to hold a handler worker.
DUPLICATE_WAIT_S = 0.5


def is_in_progress(response) -> bool:
    """True for the IN_PROGRESS answer to a re-issued, still-executing command."""
    return (not getattr(response, "status", True)
            and str(getattr(response, "error", "")).startswith(IN_PROGRESS))


def with_command_id(payload: bytes, command_id: str) -> bytes:
    """`payload` (a JSON object) with `command_id` added."""
    value = json.loads(payload.decode() or "{}")
    if not isinstance(value, dict):
        raise ValueError("command payload must be a JSON object")
    value[COMMAND_ID_KEY] = command_id
    return json.dumps(value).encode()


def command_id_of(payload: bytes) -> Optional[str]:
    try:
        value = json.loads(payload.decode())
    except (UnicodeDecodeError, ValueError):
        return None
    cid = value.get(COMMAND_ID_KEY) if isinstance(value, dict) else None
    return cid if isinstance(cid, str) and cid else None


class _Entry:
    __slots__ = ("done", "failed", "result", "at")

    def __init__(self) -> None:
        self.done = threading.Event()
        self.failed = False
        self.result: Any = None
        self.at = time.monotonic()


class CommandDeduplicator:
    """Execute each `command_id` once; answer re-deliveries from the first result.

    A handler that raises is forgotten, so the next delivery of that command
    runs it again (the sender only re-issues until it gets an answer).
    """

    def __init__(self, log: Callable[..., None], *, capacity: int = 1024,
                 ttl_s: float = 600.0) -> None:
        self._log = log
        self._capacity = capacity
        self._ttl_s = ttl_s
        self._entries: "OrderedDict[str, _Entry]" = OrderedDict()
        self._lock = threading.Lock()

    def wrap(self, handler: Callable[[bytes], Any]) -> Callable[[bytes], Any]:
        @functools.wraps(handler)
        def once(payload: bytes) -> Any:
            cid = command_id_of(payload)
            if cid is None:
                return handler(payload)
            with self._lock:
                self._evict_locked()
                entry = self._entries.get(cid)
                first = entry is None
                if first:
                    entry = self._entries[cid] = _Entry()
            if not first:
                entry.done.wait(DUPLICATE_WAIT_S)
                answered = entry.done.is_set() and not entry.failed
                self._log("command.duplicate", handler=handler.__name__,
                          command_id=cid, answered=answered)
                if answered:
                    return entry.result
                if entry.failed:
                    raise RuntimeError(f"command {cid} failed on its first delivery")
                from ndnsf import ServiceResponse

                return ServiceResponse(status=False, error=f"{IN_PROGRESS}:{cid}")
            try:
                result = handler(payload)
            except BaseException:
                with self._lock:
                    self._entries.pop(cid, None)
                entry.failed = True
                entry.done.set()
                raise
            entry.result = result
            entry.done.set()
            return result

        return once

    def _evict_locked(self) -> None:
        now = time.monotonic()
        while self._entries:
            cid, entry = next(iter(self._entries.items()))
            if len(self._entries) <= self._capacity and now - entry.at < self._ttl_s:
                break
            if not entry.done.is_set():
                break  # never drop a command that is still executing
            self._entries.popitem(last=False)
