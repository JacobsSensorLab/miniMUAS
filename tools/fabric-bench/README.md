# fabric-bench

Measures NDN fabric throughput, latency, loss and inter-flow fairness over the
wireless link, with **no dependency on the miniMUAS application stack**.

## Why

Every forwarder comparison run through the miniMUAS video path was confounded
by the application rather than the fabric:

| looked like | actually was |
|---|---|
| a forwarder throughput ceiling | `make_signed_data` building an `ndn::KeyChain` per Data packet (14.4 ms → 0.39 ms once cached) |
| a forwarder framerate ceiling | a hard `min(request.fps, 15.0)` clamp in the agent |
| ndn-fwd "starving" one drone | an FEC group sized per frame instead of per packet, so every multi-chunk push was rejected |
| a slower local-face handoff | NDNSF re-verifying the producer's own signature on every push — ~23 ms/frame, identical on both stacks |

None were the forwarder. This harness removes the application: `ndn-iperf` with
`--sign-mode none` touches no keychain, no ABE, no NDNSF and no agent, so what
it reports is the fabric.

For scale: the fabric sustains ~50 Mbps aggregate where the video path tops out
near 5 Mbps. The wireless link was never the constraint.

## Method guards

Each one exists because its absence produced a wrong conclusion:

- **Settle after a cell switch.** A sample taken inside that window is
  worthless, including as one arm of an A/B.
- **Counters as BEFORE→AFTER deltas.** They are cumulative since forwarder
  start; absolute values silently span every previous run.
- **Percentiles, never means.** Unloaded RTT here is ~6 ms while p95 under
  three flows is ~880 ms. A mean hides exactly the tail that stutters video.
- **Concurrent flows.** Single-flow throughput cannot show whether a forwarder
  shares capacity. A wide per-flow spread is the signal.
- **Verify, don't assume.** `ndn-ctl route add` takes `--face <FACE> <PREFIX>`;
  the positional form prints usage and exits non-zero. The harness checks the
  route is in the RIB before measuring, because an unverified "routed" message
  turns every later number into a NoRoute Nack.
- **Log everything.** Raw tool output is kept for every flow, so a run can be
  re-analysed without re-running it. Both parsing bugs found while building
  this were invisible in the summary and obvious in the raw log.

## Usage

    export BENCH_SERVER=minidronesys@minidronesys-01.uom.memphis.edu   # produces
    export BENCH_CLIENT=minidronesys@minidronesys-03.uom.memphis.edu   # consumes

    ./fabric-bench.sh --check                      # validate tooling on both nodes
    ./fabric-bench.sh --no-switch --duration 15    # measure the current cell
    ./fabric-bench.sh --stack both --flows 3       # switch cells, settle, compare

Options: `--stack nfd|ndn-fwd|both`, `--server`, `--client`, `--duration`,
`--flows`, `--size`, `--window`, `--settle`, `--ping-count`, `--no-switch`,
`--check`.

`--no-switch` measures whatever cell is active and will not iterate stacks.
Switching requires `muas-fabric` on the nodes; everything else needs only
`ndn-iperf`, `ndnping`/`ndnpingserver` and `ndn-ctl`.

## Output

One directory per run under `results/<UTC timestamp>/` (override with
`BENCH_OUT`):

| file | contents |
|---|---|
| `manifest.txt` | every invocation parameter |
| `runlog.txt` | every remote command with verbatim output and exit code |
| `nodes.txt` | per-node hostname, uptime, active system store path, fabric cell, face table |
| `<stack>-ping.txt` | raw `ndnping` output |
| `<stack>-flowN-client.txt` | raw `ndn-iperf` client output (throughput, RTT percentiles, retransmits) |
| `<stack>-flowN-server.txt` | raw server log |
| `counters.txt` | forwarder Data counters before and after |
| `summary.jsonl` | one machine-readable record per stack |

## Reading the results

- **Aggregate vs per-flow.** The aggregate is the sum; the *spread* between
  flows is the fairness signal.
- **Unloaded vs loaded RTT.** `ndnping` runs before the load, so comparing it
  with the per-flow RTT percentiles separates path latency from queueing.
- **Retransmits** alongside loss distinguishes a lossy link from a congested one.
- **`in_data` delta** cross-checks the reported throughput against what the
  forwarder actually counted — they should agree.
