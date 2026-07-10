# Network visualization: fabric truth on the operator map

Status: PHASE 1 SHIPPED, 2026-07-10. Companion piece to
`ARCHITECTURE.md` (dashboard/agent split) and the virtual deployment
(`crates/muas-sim/src/bin/virtual_deployment.rs`). The renderer lives in
`crates/muas-dashboard/assets/dashboard.html` (`drawNet`, `NET`).

## Why

Operators fly the *network* as much as they fly the vehicles: a stale tile,
a slow capture, a missed detection are usually link problems, not vehicle
problems. The map already shows where every vehicle is; this layer shows
what the fabric between them is doing, on the same canvas, through the same
world-to-screen transform — so "iuas-02 is far away AND its link is
starving" is one glance, not a log dive.

## The layering rule

The design splits network truth into two layers and refuses to blur them:

- **Fabric layer** — NDN faces, interests/data, bytes, drops. In a virtual
  deployment these are *real measurements* of the ndn-sim fabric the
  traffic actually crosses. The deployment owns the fabric, so the
  deployment exports them.
- **Radio layer** — RSSI, MCS, spectrum, airtime. These belong to a radio
  stack. The simulator does **not** synthesize them; a virtual deployment
  showing invented RSSI would train operators on fiction. Radio truths
  appear only in phase 3, exported by nodes that have real radios.

Everything below the dashboard hub is transport-agnostic: the dashboard
renders any `type: "net"` message; it does not know or care whether a
simulator or a field exporter produced it.

## Phase 1 (shipped): fabric links + live flow

### Producer

`net_export_loop` in `virtual_deployment.rs`, 1 Hz:

1. read every node's per-face counters off the running fabric
   (`RunningSimulation::face_stats`), keeping only `FaceKind::Link` faces
   whose far end is another labeled node (vehicles by id, console as
   `"gcs"`; sink faces are dropped);
2. difference against the previous sample to get `rate_out_bps` /
   `rate_out_interests_hz`;
3. publish the snapshot to the deployment control endpoint (`/netstats`,
   for scripts and `--verify`) and broadcast it to every dashboard client
   via the hub.

### Wire schema

```json
{
  "type": "net",
  "t": 1783700000.0,
  "profile": { "name": "lossy-wifi", "delay_ms": 20, "jitter_ms": 5,
               "loss_rate": 0.02, "bandwidth_bps": 20000000 },
  "links": [
    { "from": "iuas-01", "to": "gcs",
      "out_interests": 412, "in_interests": 398,
      "out_data": 371, "in_data": 380,
      "out_bytes": 812345, "in_bytes": 90211, "out_drops": 3,
      "rate_out_bps": 61432.0, "rate_out_interests_hz": 9.8 }
  ]
}
```

Links are DIRECTED (one entry per face, i.e. per sender). `profile` is the
active link profile from the run config — the *configured* impairment, so
the operator can compare configured vs observed.

### Renderer semantics (`drawNet`)

- Toggled by the header `Network` button, default off; state resets on
  replay reset. Latest-wins: only the newest snapshot is kept.
- Directed entries are aggregated per node pair; both directions sum into
  one line.
- Line endpoints go through `toPx`, so links rotate/zoom with heading-up
  and fleet-framing modes. The GCS is screen-anchored bottom-left (it has
  no geographic pose in a virtual deployment).
- **Width** encodes traffic (log of summed bps). **Marching dashes** appear
  above ~200 bps — flow is visible as motion, idle links sit still.
- **Color** encodes link health, deliberately derived from *telemetry
  freshness of the endpoints* (green < 4 s, yellow < 10 s, red beyond,
  gray unknown) rather than from the counters: counters describe traffic
  volume, staleness describes whether the path is alive — a silent link
  with fresh telemetry is healthy-idle, not dead.
- Mid-link label: `kb/s · interests/s · drops` (drops only when nonzero).
- Legend chip: the active link profile (name, delay ± jitter, loss %,
  bandwidth), so observed behavior always renders next to configured
  impairment.

## Phase 2 (planned): span-derived path traces

The fabric already timestamps each interest/data hop (spans). Phase 2
aggregates spans per name prefix into *path traces*: for a selected stream
(e.g. `iuas-01/video/live`), highlight the actual hop sequence and per-hop
latency on the map, and expose per-prefix rates rather than per-face
totals. Rides the same `type: "net"` message with an additive `paths`
key — the phase 1 renderer ignores keys it does not know.

## Phase 3 (planned): native radio telemetry from real deployments

On real hardware the exporter moves into the node stack: each companion
publishes its own fabric counters plus radio-layer truths (RSSI, MCS,
airtime from the actual bearer — see `docs/v3/radio-comparison.md`) as a
sensor-like NDN stream; the GCS aggregates and broadcasts the same
`type: "net"` shape with an additive `radio` key per link. The dashboard
gains signal-quality rendering only when a real radio is reporting —
never synthesized.

## Deliberate non-goals

- No synthesized radio metrics in simulation (layering rule above).
- No historical charting on the map; the map shows *now*. History belongs
  to `/netstats` scrapes.
- No topology editing from the UI; the fabric is the deployment's.
