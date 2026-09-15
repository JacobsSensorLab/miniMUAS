# miniMUAS v2 — flight readiness, 2026-09-15

Supersedes `SESSION-HANDOFF-2026-09-10.md` for anything about telemetry, video
or the radio. Every number here was measured on the live fleet (GCS
`minidronesys-03` + drone `minidronesys-01` / iuas-01), not inferred.

## Verdict

**Fly on the Wi-Fi fabric cell (`nfd wifi`) with video on the `stream`
transport.** That is the deployed default and it is reboot-safe. The
named-data radio is not flight-worthy and is not on the flight path.

| | before this session | after |
|---|---|---|
| Telemetry rate | 0.38 /s | **3.2 /s** |
| Telemetry worst gap | 21.4 s | **1.6 s** (zero gaps > 2 s over 3 min) |
| Video | 1.4 fps, 17 stutters / 75 s | **9.8 fps, ZERO stutters > 1 s over 400 s** (gap p50 0.10 s, max 0.59 s) |
| Drone agent CPU (idle) | 83 % of a core | **26 %** |
| Drone agent CPU (video on) | 92 % | **31 %** |

## What was actually wrong

The fleet had been flying on the experimental named-data radio, and the drone
was running application code from 2026-08-06 that predated every video fix.

1. **Telemetry drop-out — flying on the research link.** The radio loses
   50-75 % of Interests. Switching both nodes to the validated `nfd wifi` cell
   took telemetry from 0.38/s to 3.2/s immediately. `desired`/`good` are
   persisted, so a reboot comes back on Wi-Fi.

2. **The camera loop was GIL-saturating the drone.** `CameraHub._reader` ran
   `cap.read()` — grab **plus** a full-res 1280x800 MJPEG->BGR decode — at
   camera rate, whether or not anyone wanted video. 83 % of a core at idle.
   Python's GIL meant the telemetry publisher, NDNSF service handlers and
   flight commands all queued behind it; that is why service calls took 8-12 s
   and telemetry gapped for 15-21 s. Now grabs always (cheap, releases the
   GIL, frames stay fresh) and decodes only at the rate a consumer needs.

3. **The predictive video stream could never have worked.** The agent built
   its `VideoStreamProducer` without `signing_identity`, so the Core rejected
   every push (`signer != definition.provider`). Fixed in the bench earlier,
   never in the agent.

4. **Prefetch horizon vs Interest lifetime.** A live subscriber prefetches
   *future* cursors — Interests for frames not yet taken, parked at the
   producer — so `prefetch_depth / fps <= interest_lifetime` must hold. The
   generic defaults (depth 64, lifetime 500 ms) reach 6.4 s ahead at 10 fps
   while each Interest dies after 0.5 s. Result: timeouts, retry and recovery
   exhausted, `terminal-gap:timeout`, delivery dead after ~16 frames. Now
   lifetime 4 s / depth 16 — that alone took delivery from 16 frames to 251.

5. **The journal republisher was killing the video.** Video hit a terminal
   gap on a startlingly regular period — 42.7/42.4/43.3 s, identical across
   frame sizes. `journal.publish.ready` fires at exactly 43 s (30 s sleep +
   ~13 s of work): it re-segments and re-signs the whole journal, one RSA
   signature per ~6 KB segment, starving the video producer past the
   consumer's retry budget.

   Moving the interval to 300 s made this *rarer, not gone* — a 180 s check
   still caught a 2.11 s stall, and the agent journal bracketed it exactly
   (`journal.publish.truncated` at 1789486160.99, `.ready` at 1789486163.08,
   zero NDNSF timeline events in between). Interval tuning was the wrong axis.
   The snapshot is served **by the node itself**, so a fresher over-NDN copy
   buys nothing while the node is up — and if the node is lost, the copy is
   lost with it. Mid-flight freshness has no value; the burst has a measured
   cost. The republisher now **defers entirely while video is live**, skips a
   snapshot that has not grown, and caps the tail at 512 KB so that even a
   refresh that does fire is ~85 segments / ~0.5 s rather than ~333 / ~2 s.
   Verified: over 400 s the deferral fired at both 300 s boundaries
   (`journal.publish.deferred`) with zero video stalls.

6. **Unbounded journal took iuas-02 out of service.** Found with a **54 MB**
   journal being republished in full every 30 s (~9,000 RSA signatures per
   cycle). It starved its own startup until NDNSF certificate bootstrap timed
   out, and the agent crash-looped. The published payload is now capped at
   512 KB of tail (see 5).

7. **Radio slot MAC was enabled against upstream's own measurement.** The
   committed radio cell set `NDN_SCHED_SLOT=16:10000`. Upstream records, in
   `ndn-radio docs/mac-synthesis.md:95`: *"Slot gate improves delivery at N=2 |
   **on air (negative)** | #111 (actuates but costs 6-9x throughput at N=2 on
   a clean channel)"*. Measured here: 51 % Interest satisfaction with it on vs
   45 % off — no benefit, full cost. Removed.

## Things that were *not* the problem (don't re-chase)

- **RSA signing.** The fleet does sign every packet with 2048-bit RSA, and on
  this board that is **5.59 ms vs 0.13 ms for ECDSA — 42x**. But the drone
  only produces ~13 Data/s, so it is ~7 % of a core, not the bottleneck. (It
  *is* why the journal republisher was so expensive: that path signs
  hundreds of segments at once.)
- **The dashboard.** Zero exceptions in its journal. The "dashboard errors"
  were downstream symptoms of the lossy radio link.
- **Mapping-block exhaustion** as the stream stall (`map_int` was 0
  throughout), and **regulatory/channel** as the video cause.

## Final verified state (iuas-01, 400 s continuous)

```
===== FLIGHT CHECK [STALL HUNT] 400s =====
 OK  TELEM iuas-01: rate=3.01/s p50=0.3s p95=0.33s max=1.6s gaps>2s=0
 OK  VIDEO iuas-01: n=3912 fps=9.8 kbps=380 first_frame=1.0s
                    gap p50=0.10s p95=0.21s max=0.59s stutters>1s=0
===== FLIGHT-READY =====
```

Long enough to cross two 300 s journal boundaries; both deferred cleanly, and
the agent logged no errors for the whole run.

**Residual, accepted:** roughly one run in two shows a single sub-1.5 s video
hiccup at a non-periodic point. It is not frame loss — across it the consumer
stays `ACTIVE`, `timeouts` does not increment, and `delivered` keeps climbing
~99-100 per 10 s window, with `in_flight` briefly rising 2 -> 8. The producer
pauses and catches up. Telemetry is unaffected. Separately, joining a live
stream costs a one-time settling gap in the first few seconds (the producer
keeps producing through it — NDNSF timeline events are continuous), which
`flightcheck` reports as `warm-up` and excludes from the verdict.

Stream supervision stays in place as a safety net (`on_status` watches for a
terminal reason and rejoins at the live edge, so a future stall costs a
sub-second hiccup rather than a dead feed) — but with the 43 s journal
trigger removed it did not fire once in the final run.

## Residual, known

### iuas-02 (node 04) — root-caused; fix needs a fleet-wide decision

**It is not an ABE problem, and iuas-02 is not the broken node.** The earlier
handoff said this was "the same long-standing iuas-02 ABE universe/identity
problem". It is not. `muas-v2-ensure-identities` imported a fleet identity
only when its *name* was absent, so every node kept whatever KEY it first
received, forever, across every deploy:

```sh
if ! ndnsec list | grep -q "$identity\$"; then ndnsec import ...; fi
```

Measured on the live fleet: the controller, **both** GCS roles and iuas-01 all
run `/muas/v2/controller/KEY/%F5%9E%0A%1B%16%FF%9E%D5` (v=1781235610495,
June). The keyset package — and iuas-02 — ship
`%5C%3C%0E%8E%3Em%0F%0F` (v=1783970812252, July). iuas-02 encrypts its
bootstrap request to a controller key nobody running can decrypt, which
surfaces as `encrypted bootstrap request decrypt failed` with the identity
present and correctly named throughout.

So **iuas-02 is the only node matching the packaged keyset**; everything that
works is frozen on the superseded June generation. With the key-id check added
(below), iuas-01 logs 5 `KEYSET MISMATCH` lines and iuas-02 logs none.

Why an earlier session ruled this out: iuas-02's *own* `/muas/v2/iuas-02` key
**does** match the GCS copy — it was provisioned late, so the name guard let
it through. Checking that one key shows a match and looks like proof.

`ensure-identities` now compares the key id (resolving the packaged key in a
throwaway keychain, so the check is read-only) and logs a loud
`KEYSET MISMATCH` naming both keys. Repair is opt-in:
`services.muasV2.syncKeysetOnMismatch`, **default false**, because it deletes
a live identity and a node whose controller key disagrees with the running
controller cannot bootstrap at all.

**To actually fix it** the fleet must converge on ONE generation in a SINGLE
deploy — controller, both GCS roles, iuas-01 and iuas-02 together. Converging
one node at a time drops the others instead. Re-keying the controller identity
is also the ABE re-mint hazard, so this is a deliberate maintenance window,
not a routine deploy. Node 04's PIB is backed up at
`~/.ndn-muas/agent.bak-1789491776`, and its agent is stopped.

### Other airframes
wuas-01 (node 02) and node 05 are **powered off** — no ARP entry, no mesh
station. Nothing to fix remotely; they need power.

### Radio
The radio cell remains available (`muas-fabric set ndn-fwd radio`) for
experiments. It loses 50-75 % of Interests; the causes are RF/PHY and N=2
economics, not wiring — the MAC hardware gate *is* correctly wired now that
both ends are `rtl8822e`.

### Residual video hiccup — narrowed, not closed
Roughly one run in two shows a 1-4 s video hiccup; a 240 s run came back
completely clean. What it is **not**, all measured:
- **not the video producer** — the loop now times each phase (idle / capture /
  push) and logs any cycle > 400 ms. Across every run since, `slow_frame`
  count is **0**. The producer never pauses.
- **not RF** — sampling the mesh station 1 Hz through a stall: tx retries flat
  (23057 across the window), `tx failed` frozen at 721 for the whole run,
  signal -21 dBm, bitrate 173.3 Mbps steady.
- **not GCS load** — load average 0.46, dashboard 8.5 % of a core.
- **not frame loss** — across a stall the consumer stays `ACTIVE`, `timeouts`
  does not increment and `delivered` keeps climbing ~99-100 per 10 s window
  while `in_flight` rises 2 -> 8. Frames arrive late in a burst.

What it **is**: telemetry and video — two independent paths — stall at the
same instant (e.g. telemetry gap t+43.6 s, video stall t+44.7 s; again at
t+284.4/t+284.9), and the agent's own NDNSF timeline shows a matching hole
(`data-put` 1789490225.921, then nothing until 1789490227.354 = 1.43 s). That
puts it in the layer both share below the agent's video loop — the agent's
single NDN Face or NFD itself. A journald hypothesis (the trace writes one
DEBUG line per packet to a 4 GB journal on SD with `SyncIntervalSec=10s`) was
tested by disabling the trace and **refuted**: stalls got worse, not better
(8 vs 1), though that A/B was confounded by restarts.

### Agent restart makes the drone briefly uncommandable
Reproduced twice: for ~1-2 minutes after an agent restart the dashboard's
targeted service path fails — first `Targeted ProviderToken is unknown or
expired`, then `video.control_timeout` — while telemetry keeps flowing at
3/s. It self-heals, but the first command after any restart is silently lost.
Not yet fixed.

## How to check flight readiness yourself

`tools/flightcheck.py` drives the real operator path — it speaks the
dashboard's WebSocket, enables video, and reports telemetry cadence, video fps
and gap distribution, with an explicit FLIGHT-READY / NOT verdict:

```
python3 tools/flightcheck.py --seconds 180 --video iuas-01           # stream (default)
python3 tools/flightcheck.py --seconds 180 --video iuas-01 --transport segmented
python3 tools/flightcheck.py --seconds 120 --video iuas-01 --passive # observe, don't command
python3 tools/flightcheck.py --seconds 180 --expect iuas-01,iuas-02  # require both airframes
```

Vehicles outside `--expect` that report nothing are shown as OFFLINE rather
than failed, so a powered-down airframe doesn't read as a system fault.
Video occasionally shows a ~2 s hiccup (roughly one per two minutes); the
supervisor rejoins at the live edge. Telemetry has been gap-free (> 2 s)
across every run since the fixes.

**Measurement warning:** the fleet runs an island clock ~158 s off this
workstation. Anything that filters recorder timestamps by *local* time
silently returns nothing — take `t0` from the node. That class of error
produced contradictory results repeatedly before it was caught.
