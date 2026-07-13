"""Python-boundary latency metrics for the miniMUAS v2 NDNSF app.

This module records per-stage NDNSF pipeline latency and emits it as
`metric.latency` events through the existing `print_json` journal (the
fsync-per-line JSONL in `ndnsf_runtime`, which survives a pulled battery on
the companion computers). An offline aggregator (`tools/aggregate_latency.py`)
replays those events into the v1 C++ `metrics.hpp` stats/CSV shape so results
are directly comparable for the NDNSF maintainer.

Import-safe on any host: nothing NDN is imported at module top, and the
default emit path lazily imports `ndnsf_runtime.print_json` only when an event
is actually recorded. Every helper also accepts an injected `emit` callable so
the module can be exercised without NDN (see the self-tests in
`tools/aggregate_latency.py --selftest`).

Timing contract (shared with the NDNSF pythonWrapper, added concurrently):

  * The harness stamps `request_sent_ns = time.time_ns()` (CLOCK_REALTIME)
    before a service call and `response_received_ns` after it returns.
  * `ServiceResponse` MAY gain a `timing` dict, wall-clock-ns keys set by the
    provider: `request_received_ns`, `response_sent_ns`. It is EMPTY `{}` (or
    the attribute is absent) when the wrapper has not been updated yet, so we
    read it defensively via `getattr(resp, "timing", {}) or {}` and degrade to
    total-only.
  * Four-point breakdown when provider timing is present:
        out_ms  = request_received - request_sent   (request in flight)
        proc_ms = response_sent - request_received   (provider processing)
        back_ms = response_received - response_sent  (response in flight)
        total_ms = response_received - request_sent  (round trip)
    The one-way legs (out_ms/back_ms) are only meaningful when the two nodes'
    clocks are GPS/NTP-disciplined (the deployment GCS is a GPS time source);
    on an undisciplined fleet they carry the inter-node clock skew and should
    be read as such. total_ms is same-node differencing and is ALWAYS valid.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Mapping, Optional

EmitFn = Callable[..., None]


def _now_ns() -> int:
    return time.time_ns()


def stamp() -> int:
    """CLOCK_REALTIME nanosecond stamp taken just before a call is submitted.

    Kept as a named helper so the async request sites read the same way the
    synchronous wrappers do.
    """
    return _now_ns()


def _default_emit(event: str, **fields: Any) -> None:
    # Lazy import keeps this module import-safe on non-NDN hosts and testable
    # with an injected emit callable.
    from ndnsf_runtime import print_json

    print_json(event, **fields)


def _resolve_emit(emit: Optional[EmitFn]) -> EmitFn:
    return emit if emit is not None else _default_emit


def record_latency(
    stage: str,
    name: str,
    total_ms: float,
    *,
    status: Optional[bool] = None,
    out_ms: Optional[float] = None,
    proc_ms: Optional[float] = None,
    back_ms: Optional[float] = None,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> None:
    """Emit one `metric.latency` event.

    `stage` is the pipeline stage (service/fetch/crypto/nfd); `name` a stable
    label the aggregator groups on. Optional leg breakdowns are omitted from
    the event when None so the schema stays compact for total-only records.
    Extra keyword context (service, vehicle, target, ...) is passed through.
    """
    fields: dict[str, Any] = {"stage": stage, "name": name, "total_ms": round(total_ms, 3)}
    if status is not None:
        fields["status"] = bool(status)
    if out_ms is not None:
        fields["out_ms"] = round(out_ms, 3)
    if proc_ms is not None:
        fields["proc_ms"] = round(proc_ms, 3)
    if back_ms is not None:
        fields["back_ms"] = round(back_ms, 3)
    for key, value in ctx.items():
        if value is not None:
            fields[key] = value
    _resolve_emit(emit)("metric.latency", **fields)


def _breakdown(
    request_sent_ns: int,
    response_received_ns: int,
    timing: Mapping[str, Any],
) -> tuple[float, Optional[float], Optional[float], Optional[float]]:
    """Compute (total, out, proc, back) in ms; legs None when timing absent."""
    total_ms = (response_received_ns - request_sent_ns) / 1e6
    out_ms = proc_ms = back_ms = None
    req_recv = timing.get("request_received_ns") if timing else None
    resp_sent = timing.get("response_sent_ns") if timing else None
    if req_recv is not None and resp_sent is not None:
        out_ms = (req_recv - request_sent_ns) / 1e6
        proc_ms = (resp_sent - req_recv) / 1e6
        back_ms = (response_received_ns - resp_sent) / 1e6
    return total_ms, out_ms, proc_ms, back_ms


def record_service_result(
    name: str,
    request_sent_ns: int,
    resp: Any,
    *,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> Any:
    """Record a completed service call from its response object.

    Reads `resp.timing` defensively (empty/absent -> total + status only) and
    `resp.status` for the success flag. Returns `resp` unchanged so it can wrap
    a call inline.
    """
    received = _now_ns()
    timing = getattr(resp, "timing", {}) or {}
    total_ms, out_ms, proc_ms, back_ms = _breakdown(request_sent_ns, received, timing)
    status = getattr(resp, "status", None)
    record_latency(
        "service",
        name,
        total_ms,
        status=status,
        out_ms=out_ms,
        proc_ms=proc_ms,
        back_ms=back_ms,
        emit=emit,
        **ctx,
    )
    return resp


def record_service_timeout(
    name: str,
    request_sent_ns: int,
    *,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> None:
    """Record a service call that timed out (no response, so status=False)."""
    total_ms = (_now_ns() - request_sent_ns) / 1e6
    record_latency("service", name, total_ms, status=False, emit=emit, timeout=True, **ctx)


def time_service_call(
    call: Callable[[], Any],
    *,
    service: str,
    name: Optional[str] = None,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> Any:
    """Wrap a synchronous service call, emitting `metric.latency` (stage
    service) with the four-point breakdown when the response carries timing.
    Returns the response object.
    """
    sent = stamp()
    resp = call()
    return record_service_result(
        name or service, sent, resp, service=service, emit=emit, **ctx
    )


def time_fetch(
    fetch: Callable[[], Any],
    *,
    name: str,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> Any:
    """Wrap a data-plane fetch, emitting `metric.latency` (stage fetch).

    No provider timing is available for a raw segmented fetch, so only total +
    status are recorded. A raised exception is recorded as status=False and
    then re-raised (graceful: never swallow the fetch failure).
    """
    sent = stamp()
    ok = False
    try:
        result = fetch()
        ok = True
        return result
    finally:
        total_ms = (_now_ns() - sent) / 1e6
        record_latency("fetch", name, total_ms, status=ok, emit=emit, **ctx)


def time_crypto(
    op: Callable[[], Any],
    *,
    name: str,
    emit: Optional[EmitFn] = None,
    **ctx: Any,
) -> Any:
    """Wrap an encrypt/decrypt boundary, emitting `metric.latency` (stage
    crypto). Present for parity with the aggregator's stages; the v2 app does
    not currently call the NDNSF encrypted-large-data wrappers, so this is
    wired only if/when it does.
    """
    sent = stamp()
    ok = False
    try:
        result = op()
        ok = True
        return result
    finally:
        total_ms = (_now_ns() - sent) / 1e6
        record_latency("crypto", name, total_ms, status=ok, emit=emit, **ctx)
