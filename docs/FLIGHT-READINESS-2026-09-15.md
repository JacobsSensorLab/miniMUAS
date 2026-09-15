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
| Video | 1.4 fps, 17 stutters / 75 s | **9.8 fps, ZERO stutters > 1 s over 3 min** (gap p50 0.10 s, max 0.42 s) |
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

5. **The journal republisher was killing the video every 43 s.** Video hit a
   terminal gap on a startlingly regular period — 42.7/42.4/43.3 s, identical
   across frame sizes. `journal.publish.ready` fires at exactly 43 s (30 s
   sleep + ~13 s of work): it re-segments and re-signs the whole ~700 KB
   journal, one RSA signature per ~6 KB segment, starving the video producer
   past the consumer's retry budget. Now every 300 s.

6. **Unbounded journal took iuas-02 out of service.** Found with a **54 MB**
   journal being republished in full every 30 s (~9,000 RSA signatures per
   cycle). It starved its own startup until NDNSF certificate bootstrap timed
   out, and the agent crash-looped. The published payload is now capped at
   2 MB of tail.

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

## Final verified state (iuas-01, 190 s continuous)

```
OK  TELEM iuas-01: rate=2.90/s p50=0.30s p95=0.37s max=1.60s gaps>2s=0
OK  VIDEO iuas-01: n=1813 fps=9.8 kbps=469 first_frame=1.4s
                   gap p50=0.10s p95=0.21s max=0.42s stutters>1s=0
```

Stream supervision stays in place as a safety net (`on_status` watches for a
terminal reason and rejoins at the live edge, so a future stall costs a
sub-second hiccup rather than a dead feed) — but with the 43 s journal
trigger removed it did not fire once in the final run.

## Residual, known
- **iuas-02 (node 04) is down**: `certificate bootstrap timed out`,
  crash-looping, and still on the pre-fabric June generation. Its journal has
  been rotated; it needs the current generation deployed. wuas-01 (node 02)
  and node 05 are powered off.
- The radio cell remains available (`muas-fabric set ndn-fwd radio`) for
  experiments. It loses 50-75 % of Interests; the causes are RF/PHY and N=2
  economics, not wiring — the MAC hardware gate *is* correctly wired now that
  both ends are `rtl8822e`.

## How to check flight readiness yourself

`scratchpad/flightcheck.py` drives the real operator path — it speaks the
dashboard's WebSocket, enables video, and reports telemetry cadence, video fps
and gap distribution, with an explicit FLIGHT-READY / NOT verdict:

```
python3 flightcheck.py --seconds 180 --video iuas-01            # stream (default)
python3 flightcheck.py --seconds 180 --video iuas-01 --transport segmented
python3 flightcheck.py --seconds 120 --video iuas-01 --passive  # observe, don't command
```

**Measurement warning:** the fleet runs an island clock ~158 s off this
workstation. Anything that filters recorder timestamps by *local* time
silently returns nothing — take `t0` from the node. That class of error
produced contradictory results repeatedly before it was caught.
