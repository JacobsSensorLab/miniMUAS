# NDNSF bug report — segmented responses, targeted requests, and the "provider stalls after ~N" root cause

Environment: NDN_Service_Framework (Quarmire fork @ d2313d6) + ndn-svs (matianxing1992 @ 0521665) + NAC-ABE + ndn-cxx 0.9.0, Python wrapper (`ndnsf`), aarch64/NixOS, 1-hop Wi-Fi. All claims reproduced on the live fleet.

## Summary

The headline "provider stalls after ~N requests, only a fresh provider recovers" is **not** a request-count limit or GIL contention — it's that **any service response larger than ndn-svs `MAX_DATA_SIZE` gets segmented, silently fails to deliver, and poisons the producer.** Along the way we also fixed a provider crash and made targeted requests work.

| # | Issue | Status |
|---|---|---|
| 1 | Segmented (>~6 KB) responses never deliver **and poison the producer** — the real "stall after ~N" cause | **Fix PROPOSED (upstream ndn-svs); operational workaround deployed** |
| 2 | Oversized response → provider SIGABRT (8800-octet limit) | **Workaround DEPLOYED** (stops crash, exposes #1) |
| 3 | Targeted requests non-functional via the Python wrapper | **Fix DEPLOYED** (patch, verified) |
| 4 | Targeted request ignores `timeout_ms` against a wedged provider | **Fix PROPOSED** |
| 5 | Crash-on-exit in NAC-ABE/OpenABE teardown (benign) | **Fix PROPOSED** |

---

## 1. Segmented responses fail silently and poison the producer — *fix proposed, workaround deployed*

**Behaviour (confirmed, fresh provider per trial):**
- ≤ ~5 KB (single Data): 100% reliable.
- > `MAX_DATA_SIZE` (tested 6.5–16 KB): **0% delivery, every trial** (0/60, 0/40). No crash, no error to the requester — pure timeout.
- **Poisoning:** a burst of segmented publishes wedges the producer for *all* payloads. Baseline 64 B = 10/10 → after 60 segmented publishes 64 B = **2/15**; after an 80-request segmented cell, even a non-segmented 4 KB = **0/12**.
- **Only a *provider* restart recovers it** (fresh SVS node-id) — the consumer is a new process each trial and still fails. So the stuck state is **producer-side**, in the SVS node's per-node seq/store.
- **Independent of `NDNSF_SVS_ASYNC_PUBLISH`** (on → 2/15, off → 0/15) ⇒ the defect is in the ndn-svs core segment path (`SVSPubSub::publish` → `insertDataAtSeq` → double-encapsulated outer sync Data, `svsync-base.cpp:110-126`), not the NDNSF async wrapper.

**Workaround (deployed, operational):** keep all NDNSF response payloads under the single-Data budget (~5 KB) so responses never segment. Large media rides the separate segmented data-plane, which is unaffected.

**Fix (proposed — priority):** (a) make segmented `SVSPubSub::publish` actually deliver + reassemble at the consumer; (b) ensure a failed/oversized segment cannot wedge the producer's node state; (c) meanwhile, return an error to the producer when a publication would segment beyond a deliverable size, instead of silently accepting it.

---

## 2. Oversized response → provider SIGABRT — *workaround deployed*

`Data ... encodes into 8877 octets, exceeding the implementation limit of 8800 octets` → SIGABRT. A response over ~6–8 KB is segmented and **double-encapsulated** by ndn-svs (inner segment Data becomes the content of an outer sync Data with its own long name + second signature); the outer `wireEncode()` exceeds 8800 and throws from a `boost::asio::post` handler *outside* the request-handler try/catch, so it escapes the run loop.

**Workaround (deployed):** lower ndn-svs `MAX_DATA_SIZE` 8000→6000, and wrap `publishSvs` (`ServiceProvider.cpp:~218`) in try/catch that logs+drops instead of aborting. **Caveat:** this only stops the *crash* — it converts it into the silent, producer-poisoning delivery failure of #1. The real fix is #1.

---

## 3. Targeted requests non-functional in the Python wrapper — *fix deployed (patch, verified)*

`request_service_targeted_async(...)` always timed out. Two binding defects:
- **Provider token mode hardcoded on:** `_ndnsf.cpp` sets `setUseTokens(true)` and exposes no `set_use_tokens` (only `ServiceUser` does). A tokens-off user's request is silently dropped (`if (m_useTokens && userToken.empty()) return;`). This also broke *two-phase* whenever tokens were off.
- **`addService` registers a Normal-mode handler only** (3-arg overload) → targeted returns "Targeted service has no handler".

**Fix (deployed):** expose/bind `ServiceProvider.set_use_tokens`, run provider tokens-off to match the user, and register via `ServiceInvocationMode::NormalAndTargeted`. Verified: targeted p50 ~85 ms vs two-phase ~175 ms.

---

## 4. Targeted request ignores `timeout_ms` against a wedged provider — *fix proposed*
Once a provider is in the #1 poisoned state, `request_service_targeted_async` **hangs indefinitely** instead of firing `on_timeout` (two-phase to the same provider times out cleanly). Targeted should honour `timeout_ms` unconditionally.

## 5. Crash-on-exit in NAC-ABE/OpenABE teardown — *fix proposed (benign)*
Every process SIGSEGVs at exit inside `ABESupport::~ABESupport → oabe::ShutdownOpenABE → rand_clean` (RELIC), from `__run_exit_handlers`. Harmless to the workload (runs after all work) but cores every run and can drop block-buffered stdout. Likely a static-destruction-order / double-shutdown issue; guard `ShutdownOpenABE()`.

---

## What we shipped vs. what's for the maintainer

- **Deployed on our fork (patches):** #3 targeted-request fix, and #2 crash→logged-drop workaround (`targeted-tokens-oversized-fix.patch` for NDNSF + a one-line `MAX_DATA_SIZE` reduction for ndn-svs).
- **Operational workaround:** #1 — cap response payloads < ~5 KB (avoids segmentation entirely).
- **Needs the maintainer (upstream):** #1 real fix (priority), #4, #5.

**Performance note (not a bug):** with the built-in timeline trace we measured the framework's own contribution — on an idle provider it's ~0.1% of latency (dominated by the two SVS sync round-trips), but on a CPU-loaded provider the single-threaded ndn-cxx face + Python GIL serialize everything: a real field mission running YOLO in the request handler saw framework crypto phases balloon from ~7 ms to **hundreds of ms–seconds** purely from queuing. This is the case for the async-pipeline / non-GIL direction. Full data in the accompanying performance report; raw per-request JSONL available on request.
