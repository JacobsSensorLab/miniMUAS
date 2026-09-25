"""Vehicle telemetry over one NDNSF predictive stream per vehicle.

Replaces the latest-wins ``/muas/v2/<vid>/telemetry/live`` name, which the
agent republished through a fresh ``publish_segmented`` producer on every
sample (a new producer + registration each time) and every consumer polled.
Polling fetched through ``runtimeFetchSegmented`` with no signature check, and
the dashboard and every PeerGuard each paid one blocking fetch per poll. NDNSF
names live telemetry as a stream workload (README "Choosing the transfer
API"): one long-lived session, items signed by the vehicle identity (the Core
refuses a push whose signer is outside the stream provider's authority), and
subscribers keep future Interests outstanding instead of polling.

The descriptor a subscriber needs is published on the small-payload plane at
``vehicle_telemetry_descriptor_name``; a subscriber refetches it whenever the
feed goes silent, so an agent restart (new session epoch) is picked up without
coordination.

Subscribing works from a ``ServiceUser`` (dashboard) or a ``ServiceProvider``
(a vehicle's PeerGuard): both expose ``subscribe_stream(descriptor, options)``.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Callable, Optional

from contracts import (
    gps_time_ns,
    vehicle_telemetry_descriptor_name,
    vehicle_telemetry_stream_name,
)

TELEMETRY_STREAM_ID = "telemetry"

# One sample is one Data: a TelemetrySample is ~0.6 KB of JSON, far under the
# 8800-byte signed-wire cap, so there is no chunking and no FEC. XOR repair over
# a one-item group is a second copy of every sample, and a lost sample is
# acceptable -- the next one supersedes it 250 ms later.
#
# Mapping blocks sized like video's (MAPPING_BLOCK_CAPACITY, video_stream.py): a
# block must fit one link fragment or one lost fragment loses the whole block.
# 2 items x ~0.7 KB stays inside the fleet's 1452-byte datagram MTU.
MAPPING_BLOCK_CAPACITY = 2
# Names reserved ahead = 2 x 4 = 8 samples = 2 s at 4 Hz: enough for the
# subscriber's future horizon below, without parking Interests past their life.
MAPPING_AHEAD_BLOCKS = 4
# 30 s of history at 4 Hz: a subscriber that stalls can still resume in order
# instead of hitting the retention edge (video_stream.RETENTION_SECONDS).
RETAINED_ITEMS = 120

# Prefetch window in items. NDNSF keeps at most half of it as future Interests
# (horizon = min(lookahead, limit) / 2), so 8 keeps 4 samples = 1 s ahead at
# 4 Hz. The binding rule is  horizon / rate <= Interest lifetime  (see
# video_stream.LIVE_INTEREST_LIFETIME_MS); telemetry is far below the path BDP.
INTEREST_LIMIT = 8

# Sample freshness for the stream Data. A cached sample older than this is
# useless to a live consumer.
FRESHNESS_MS = 1000

# A feed with no new sample for this long refetches the descriptor and, if the
# vehicle started a new session, subscribes to it; otherwise it resubscribes at
# the live edge. 12 missed samples at 4 Hz: well past fast retransmit (3 later
# items) and one Interest lifetime, so only a genuinely stuck subscription.
SILENT_RESUBSCRIBE_S = 3.0
# Resubscribe at the live edge once delivered samples fall this far behind
# live. Lag is transit time (our clock minus the sample stamp) above its running
# minimum, so a vehicle whose clock is off by a constant cannot trigger it
# (same method as VideoStreamConsumer.lag_ms). The ordered drain can hold later
# samples behind a lost one; PeerGuard must not act on positions that old.
LAG_RESUBSCRIBE_S = 1.5
# At most one resubscribe per feed per this many seconds, so a vehicle that is
# down degrades to periodic attempts instead of a fetch loop.
RESUBSCRIBE_MIN_S = 3.0
# Descriptor fetch timeout. The fetch holds the GIL for its duration (ndn-cxx
# Faces are not thread-safe), so keep it short; it runs only on (re)subscribe.
DESCRIPTOR_TIMEOUT_MS = 800
WATCH_PERIOD_S = 0.5
DRAIN_BATCH_ITEMS = 32
DRAIN_WAIT_MS = 200


def interest_lifetime_ms(sample_period_ms: float) -> int:
    """Twice the longest wait of a future Interest: horizon x sample period."""
    horizon_ms = (INTEREST_LIMIT / 2) * max(1.0, float(sample_period_ms))
    return int(min(4000, max(500, 2 * horizon_ms)))


def telemetry_stream_config(vehicle_id: str, hz: float):
    from ndnsf import (
        LiveStreamFecOptions,
        SampleClassProfile,
        StreamAdvancedOptions,
        StreamConfig,
    )

    return StreamConfig(
        stream_id=TELEMETRY_STREAM_ID,
        data_prefix=vehicle_telemetry_stream_name(vehicle_id),
        sample_period_ms=1000.0 / max(0.2, float(hz)),
        sample_classes=(SampleClassProfile("telemetry", 1, 1),),
        fec=LiveStreamFecOptions.none(),
        advanced=StreamAdvancedOptions(
            mapping_block_capacity=MAPPING_BLOCK_CAPACITY,
            mapping_ahead_blocks=MAPPING_AHEAD_BLOCKS,
            retained_items=RETAINED_ITEMS,
        ),
    )


class TelemetryStreamProducer:
    """The vehicle side: one stream session, one signed Data per sample."""

    def __init__(self, provider, vehicle_id: str, *, hz: float,
                 signing_identity: str) -> None:
        self._stream = provider.create_stream(
            telemetry_stream_config(vehicle_id, hz)
        )
        self._descriptor = self._stream.start()
        self._signing_identity = signing_identity
        self._seq = 0
        self._lock = threading.Lock()

    @property
    def descriptor_json(self) -> bytes:
        return json.dumps(self._descriptor.to_dict()).encode()

    def publish(self, payload: bytes) -> None:
        with self._lock:
            self._stream.push_signed_batch(
                self._seq, [payload], self._signing_identity, FRESHNESS_MS, True,
            )
            self._seq += 1

    def stop(self) -> None:
        try:
            self._stream.stop()
        except Exception:
            pass


def _session_of(descriptor: dict) -> tuple:
    d = descriptor.get("definition", {})
    return (d.get("provider"), d.get("sessionEpoch"))


class TelemetryFeed:
    """One vehicle's telemetry stream on the consumer side; keeps itself live.

    ``on_sample(payload, sample)`` runs on this feed's drain thread for every
    delivered sample (``sample`` is the decoded JSON dict); keep it light.
    ``latest(max_age_s)`` serves pull-style consumers (PeerGuard).

    ``fetch`` is the process's segmented fetch (``dataplane.fetch_segmented``),
    used only for the descriptor. ``log(event, **fields)`` receives every
    subscription change with its reason and the NDNSF status at that moment.
    """

    def __init__(
        self,
        host,
        vehicle_id: str,
        *,
        fetch: Callable[..., bytes],
        log: Callable[..., None],
        on_sample: Optional[Callable[[bytes, dict], None]] = None,
    ) -> None:
        self.vehicle_id = vehicle_id
        self._host = host
        self._fetch = fetch
        self._log = log
        self._on_sample = on_sample
        self._provider = f"/muas/v2/{vehicle_id}"
        self._data_prefix = vehicle_telemetry_stream_name(vehicle_id)
        self._lock = threading.Lock()
        self._latest: Optional[dict] = None
        self._latest_mono = 0.0
        self._latest_ns = 0
        self._transit_floor_ns: Optional[int] = None
        self._lag_ns = 0
        self._sub = None  # {"subscriber", "queue", "drainer", "session", "descriptor", "at"}
        self._resub_at = 0.0
        self.delivered = 0
        self.rejected = 0
        self.resubscribes = 0
        self._stop = threading.Event()
        self._watcher = threading.Thread(
            target=self._watch, name=f"telemetry-{vehicle_id}", daemon=True
        )

    def start(self) -> "TelemetryFeed":
        """Begin subscribing (descriptor fetch on the next watch tick)."""
        self._watcher.start()
        return self

    # -- consumer API ------------------------------------------------------

    def latest(self, max_age_s: float) -> Optional[dict]:
        """The newest sample if it arrived within ``max_age_s``, else None."""
        with self._lock:
            if self._latest is None:
                return None
            if time.monotonic() - self._latest_mono > max_age_s:
                return None
            return self._latest

    def silent_s(self) -> Optional[float]:
        """Seconds since the last sample; None if none ever arrived."""
        with self._lock:
            if self._latest is None:
                return None
            return time.monotonic() - self._latest_mono

    def stop(self) -> None:
        self._stop.set()
        self._close_sub(self._sub)
        self._sub = None

    # -- delivery ----------------------------------------------------------

    def _deliver(self, content: bytes) -> None:
        try:
            sample = json.loads(bytes(content).decode())
        except Exception:
            self.rejected += 1
            return
        # The stream name already binds the item to this vehicle's provider
        # identity; the payload's own id must agree or the item is not ours.
        if sample.get("vehicle_id") != self.vehicle_id:
            self.rejected += 1
            return
        stamp = int(sample.get("gps_time_ns", 0))
        transit = gps_time_ns() - stamp
        with self._lock:
            if self._transit_floor_ns is None or transit < self._transit_floor_ns:
                self._transit_floor_ns = transit
            self._lag_ns = transit - self._transit_floor_ns
            # Ordered delivery can release samples in a burst after a stall;
            # only a newer sample replaces the latest.
            if stamp >= self._latest_ns:
                self._latest = sample
                self._latest_ns = stamp
                self._latest_mono = time.monotonic()
        self.delivered += 1
        if self._on_sample is not None:
            try:
                self._on_sample(content, sample)
            except Exception as exc:
                self._log("telemetry.on_sample_error", vehicle=self.vehicle_id,
                          error=str(exc))

    def _drain(self, queue, closed: threading.Event) -> None:
        # Checked per batch, not only when the queue runs dry: a replaced
        # subscription must stop delivering at once, busy or not.
        while not closed.is_set():
            for _cursor, content, _recovered in queue.take(
                DRAIN_BATCH_ITEMS, DRAIN_WAIT_MS
            ):
                self._deliver(content)

    # -- subscription lifecycle -------------------------------------------

    def _fetch_descriptor(self) -> Optional[dict]:
        name = vehicle_telemetry_descriptor_name(self.vehicle_id)
        try:
            descriptor = json.loads(
                self._fetch(name, timeout_ms=DESCRIPTOR_TIMEOUT_MS).decode()
            )
        except Exception as exc:
            self._log("telemetry.descriptor_unavailable", vehicle=self.vehicle_id,
                      error=str(exc))
            return None
        d = descriptor.get("definition", {})
        prefix = str(d.get("semanticDataPrefix", ""))
        # The descriptor comes over the unvalidated segmented plane; refuse one
        # that points anywhere but this vehicle's own telemetry stream. NDNSF
        # appends a session version to the configured data prefix
        # (".../telemetry/stream/v=<n>"), so the prefix is matched by name.
        if (d.get("provider") != self._provider
                or not (prefix == self._data_prefix
                        or prefix.startswith(self._data_prefix + "/"))
                or d.get("streamId") != TELEMETRY_STREAM_ID):
            self._log("telemetry.descriptor_rejected", vehicle=self.vehicle_id,
                      provider=d.get("provider"),
                      prefix=d.get("semanticDataPrefix"),
                      stream_id=d.get("streamId"))
            return None
        return descriptor

    def _subscribe(self, descriptor: dict, reason: str) -> None:
        from ndnsf import (
            PredictiveStreamDescriptor,
            StreamItemQueue,
            StreamSubscriptionOptions,
        )

        old = self._sub
        self._sub = None
        old_status = self._close_sub(old)
        queue = StreamItemQueue(capacity=RETAINED_ITEMS)
        subscriber = self._host.subscribe_stream(
            PredictiveStreamDescriptor.from_dict(descriptor),
            StreamSubscriptionOptions(
                item_queue=queue,
                start="latest",
                aggregate_interest_limit=INTEREST_LIMIT,
                enable_fec_recovery=False,
                require_full_delivery=False,
                interest_lifetime_ms=interest_lifetime_ms(
                    descriptor["definition"].get("samplePeriodMs", 250.0)
                ),
            ),
        )
        with self._lock:
            # Lag is judged on this subscription's own deliveries; the floor
            # (the clock offset) carries over.
            self._lag_ns = 0
        closed = threading.Event()
        drainer = threading.Thread(
            target=self._drain, args=(queue, closed),
            name=f"telemetry-drain-{self.vehicle_id}", daemon=True,
        )
        drainer.start()
        now = time.monotonic()
        self._sub = {
            "subscriber": subscriber, "queue": queue, "drainer": drainer,
            "closed": closed, "session": _session_of(descriptor),
            "descriptor": descriptor, "at": now,
        }
        self._resub_at = now
        if old is not None:
            self.resubscribes += 1
        self._log(
            "telemetry.subscribed", vehicle=self.vehicle_id, reason=reason,
            epoch=descriptor["definition"].get("sessionEpoch"),
            resubscribes=self.resubscribes, delivered=self.delivered,
            previous_status=old_status,
        )

    @staticmethod
    def _close_sub(sub) -> Optional[str]:
        if sub is None:
            return None
        status = None
        try:
            status = str(sub["subscriber"].status())
        except Exception:
            pass
        try:
            sub["subscriber"].stop()
        except Exception:
            pass
        sub["closed"].set()
        sub["queue"].close()
        sub["drainer"].join(timeout=2.0)
        return status

    def _lag_s(self) -> Optional[float]:
        """Lag of the most recently delivered sample behind live."""
        with self._lock:
            if self._latest is None:
                return None
            return self._lag_ns / 1e9

    def _watch(self) -> None:
        while not self._stop.wait(WATCH_PERIOD_S):
            try:
                self._tick()
            except Exception as exc:
                self._log("telemetry.watch_error", vehicle=self.vehicle_id,
                          error=str(exc))

    def _tick(self) -> None:
        now = time.monotonic()
        if self._resub_at and now - self._resub_at < RESUBSCRIBE_MIN_S:
            return
        sub = self._sub
        if sub is None:
            descriptor = self._fetch_descriptor()
            if descriptor is not None:
                self._subscribe(descriptor, "initial")
            else:
                self._resub_at = now  # throttle descriptor retries too
            return
        silent = self.silent_s()
        since_sub = now - sub["at"]
        # silent since the last sample, or since subscribing if none came yet
        quiet = min(silent, since_sub) if silent is not None else since_sub
        lag = self._lag_s()
        if quiet >= SILENT_RESUBSCRIBE_S:
            descriptor = self._fetch_descriptor()
            if descriptor is None:
                self._resub_at = now
                return
            new_session = _session_of(descriptor) != sub["session"]
            self._subscribe(
                descriptor,
                "new-session" if new_session else f"silent {quiet:.1f}s",
            )
        # Lag only counts while samples are arriving: right after a
        # resubscribe the last sample is old by construction.
        elif (lag is not None and silent is not None and silent < 1.0
              and lag >= LAG_RESUBSCRIBE_S):
            self._subscribe(sub["descriptor"], f"lag {lag:.2f}s")
