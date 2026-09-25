# ndn-fleet — the fleet deploy + measurement protocol

One MCP server (`ndn-fleet mcp`) is the only way an agent deploys to the miniMUAS fleet or measures
on it. It exists because every session improvised both, differently, and the improvisation was
the dominant source of wrong conclusions (see "Why" at the end). The server does not offer a
toolbox; it enforces a sequence. Anything the sequence requires happens inside a tool, in the same
order, every time, and is recorded.

## Fleet model

Defined in `fleet.toml` (committed, the single inventory):

| node | host | role | vehicle | addr |
|---|---|---|---|---|
| minidronesys-01 | minidronesys-01.uom.memphis.edu | iuas | iuas-01 | 192.168.1.11 |
| minidronesys-02 | minidronesys-02.uom.memphis.edu | wuas | wuas-01 | 192.168.1.12 |
| minidronesys-03 | minidronesys-03.uom.memphis.edu | gcs | gcs | 192.168.1.13 |
| minidronesys-04 | minidronesys-04.uom.memphis.edu | iuas | iuas-02 | 192.168.1.14 |

ODROID-C4, NixOS 25.05, aarch64. Deploy source: `minidronesys-configurations-copy`, branch
`mini-muas-v2`; ndn-fwd pins in `nix/ndn-packages/ndn-fwd/default.nix` (rev + SRI hash per repo);
builds on nixbuild.net. Forwarder selection is runtime state (`muas-fabric`, cells
`"<nfd|ndn-fwd> <wifi|radio>"`, state in `/var/lib/minimuas/fabric/{desired,good,active}`).
Workloads are driven through the GCS dashboard WebSocket by `miniMUAS/tools/flightcheck.py`
(the one implementation of the operator-path verdict) or `miniMUAS/tools/fabric-bench`.

## Invariants (enforced by the server, not by the caller)

- **I1 — one mutation at a time.** `fleet_deploy`, `fleet_set_cell`, `fleet_measure` and
  `fleet_restore` take the fleet lock (`<results>/state/lock`, stale if its pid is dead). A second
  call is refused with the holder's job id.
- **I2 — never under an armed vehicle.** Every mutation first reads each vehicle's `armed` flag from
  the dashboard (flightcheck `--probe`). Any armed ⇒ refused. If the dashboard cannot answer, the
  mutation is refused unless the caller passes `assume_disarmed: true`, which is recorded.
- **I3 — the settle clock.** Every disturbance (deploy, cell change, service restart, restore) stamps
  `<results>/state/state.json.last_disturbance`. A measurement run may only start when
  `now − last_disturbance ≥ spec.settle_s` (default 150 s). `fleet_measure` waits for the remainder
  (reporting it) rather than measuring early; it never measures inside churn.
- **I4 — deploy only pushed, clean, pinned code.** Revs come from local repos whose HEAD is clean and
  equal to its upstream tip. Rev and SRI hash are always written together (hash from
  `nix flake prefetch`). The pin commit is pushed before building. After rollout every node's
  `/run/current-system` equals the closure built for it, and every node runs the same ndn-fwd
  package path.
- **I5 — canary first, GCS last, fixed restart order, one restart per role.** Rollout: canary
  airframe → verify → other airframes → GCS. Then journals >2 MB are truncated and services are
  brought up in order: forwarder (fabric apply started, never restarted + health) → agents →
  controller/gcs → dashboard. A role unit is restarted only if it has not already started since
  the node's activation and its forwarder's last start (switch-to-configuration, a PartOf=
  cascade, or an earlier group may have done it).
- **I6 — streams are stopped around every sample.** Before a run: all video disabled and the
  dashboard quiet (no frames for 3 s). After: disabled again. A sample whose first frame precedes
  its own enable (backlog) is rejected, not reported.
- **I7 — counters are deltas.** Face/CS counters are snapshotted on every node before and after the
  workload; results report the delta, never lifetime totals.
- **I8 — everything is recorded.** Every tool call appends to `<results>/ledger.jsonl`. Every
  deploy writes `deploys/<id>.json`; every measurement writes `runs/<id>/` (manifest, raw outputs,
  counters before/after, deltas, workload JSON, summary). A run manifest carries the full fleet
  identity: per-node system path, ndn-fwd path, cell, and the pins of the deploy in force.
- **I9 — the fleet is left as found.** An A/B measurement that switches cells restores the cell it
  started from and re-verifies health; `fleet_restore` returns to the known-good cell and runs the
  flight check.

## Tools

Read-only (no lock, no armed check):

| tool | purpose |
|---|---|
| `fleet_status` | Preflight: per node reachable, system path, ndn-fwd path, cell (desired/good/active), role units active, `/muas` strategy = multicast with 3 nexthops, oversize journals, uptime, clock offset; vehicles' armed flags; seconds since last disturbance; verdict `READY` / `DEGRADED` / `UNREACHABLE` with reasons. |
| `fleet_counters` | Current face/CS counters on every node (parsed), optionally as a delta against a previous snapshot id. |
| `fleet_logs` | `journalctl -u <unit>` on a node since a time, bounded lines. |
| `fleet_specs` | The measurement protocols in `specs/*.toml`: name, description, settle, duration, repeats, arms, workload. |
| `fleet_results` | List runs/deploys, show one, or compare two runs metric by metric. |
| `fleet_job` | Status, result and log tail of a background job; long-polls up to `wait_s`. |

Mutating (lock + armed check + ledger + disturbance stamp):

| tool | purpose |
|---|---|
| `fleet_deploy` | Two-phase. Without `plan_id`: resolves revs (default: current HEAD of every pinned repo), verifies clean + pushed, computes per-repo commit lists and hashes, and returns a plan (id, old→new revs, commits, nodes). With `plan_id`: executes it as a job — edit pins, commit, push, build all closures, canary, rollout, verify, restart order, health gate. |
| `fleet_set_cell` | `muas-fabric set <cell>` on every node (airframes, then GCS), health gate, disturbance stamp. |
| `fleet_measure` | Runs a spec (optionally overridden) as a job: per repeat × arm — set cell if the arm needs it, settle, stop streams, counters before, workload, counters after, stop streams, write the run. Arms are interleaved per repeat so each comparison shares a window. Restores the starting cell. |
| `fleet_restore` | Known-good cell on every node, health gate, flight check verdict. |

Long operations return `{job_id}` immediately; poll with `fleet_job`.

## Measurement specs (`specs/<name>.toml`)

```toml
name = "video-3stream"
description = "…"
settle_s = 150          # I3 minimum quiet time before each sample
duration_s = 90         # workload window
repeats = 2
arms = [ { label = "ndn-fwd", cell = "ndn-fwd wifi" }, { label = "nfd", cell = "nfd wifi" } ]  # optional
[workload]
kind = "flightcheck"    # flightcheck | fabric-bench | idle
video = ["iuas-01", "iuas-02", "wuas-01"]
width = 640
fps = 15
quality = 50
transport = "stream"
audio = ""              # vehicle to task an audio capture on, or empty
expect = ["iuas-01", "iuas-02", "wuas-01"]
```

A spec is the unit of comparability: two runs of the same spec at different builds are comparable;
ad-hoc parameters are passed as `overrides` and recorded in the manifest.

## Results layout (`<results>` = `tools/ndn-fleet/results/` in miniMUAS, not committed)

```
ledger.jsonl                     one line per tool call and state change
state/state.json, state/lock
deploys/<id>.json                plan, pins old/new, commits, built paths, per-node outcome, timings
runs/<id>/manifest.json          spec (+overrides), arm, repeat, node t0, fleet identity, deploy id
runs/<id>/counters-{before,after}.json, deltas.json
runs/<id>/workload.json          flightcheck --json / fabric-bench summary
runs/<id>/raw/                   stdout/stderr of every command
runs/<id>/summary.json           the metrics a comparison reads
jobs/<id>.log
```

## Why each invariant exists (from the fleet record)

- I3: three wrong attributions in one round came from measuring inside "four deploys and many
  service restarts"; a restart-adjacent sample mis-called results twice
  (ndn-rs/docs/nfd-divergence-findings.md, rounds 3–7).
- I4: bumping a rev without its hash silently rebuilt the old source; a dirty tree cannot be built
  by the fleet.
- I5: wrong restart order produced "Targeted ProviderToken is unknown or expired" and wiped prefix
  registrations; oversized journals blocked agent startup. Unconditional restarts bounced every
  role 2–3 times per deploy (a fabric-apply restart cascades to all role units via Requires=),
  and commands sent while a provider starts are lost.
- I6: a 40 s quiet gap does not stop streams; one sample reported 40 fps against a 15 fps request
  while draining backlog.
- I7/I8: counter snapshots, captures and write-ups were ad hoc and irreproducible; "logging
  everything is paramount".
- I2/I9: the fabric must never be yanked under a flying vehicle, and a session must not leave the
  fleet on an experimental cell.
