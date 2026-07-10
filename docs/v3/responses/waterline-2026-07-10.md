# waterline → miniMUAS — response to consumer feedback #1 (2026-07-10)

Same-day dispositions. Everything below is committed and green
(`cargo test --workspace`: 80 checks; `bench vectors`: 23/23).

| Your item | Disposition |
|---|---|
| Ask 2 · fact routing (THE blocker) | **Fixed — WL-11.** Facts route by `describes` vs the panel's declared subject prefix (C5), most-specific-wins (presence's rule). Your diagnosis was exact; the verb arm was indeed the pattern. Two latent siblings also fixed: chain heads advanced on every chain panel, and never advanced for typed facts. Pinned by `waterline-console-core/tests/multi_instrument.rs` — three drones, nested prefixes, journal/log separation. Declaration subjects are now uniformly the stream's name prefix. **The live multi-instrument offer is accepted — dock when ready.** |
| Ask 1 · closed five-decl docking | **Done.** `PanelState::Generic{manifest}`: anything the matcher binds docks as a manifest card (label · describes · entries · verdict chips); the typed five stay fast-pathed. Your video-tile decl now docks. |
| Ask 3 · compiled-in onboarding | **Done.** `sextant-tty --follow root,writer,writer-key-hex` (repeatable; the trust pin rides along), `--principal/--device/--key-seed`, `--floor <file>` (`verb|danger|safing|ceremony` rows, refused loudly on bad words). Demo defaults remain as defaults only. |
| Ask 4 · vocabulary injection | **Done.** `Adoption::with(dag, contracts, frontier)`, `add_document(bytes) -> Hash`, `add_contract(hash)`, `admit(hash)`. Bring your lifecycle-records stratum; C10 now reaches it. |
| §c gate-flip silent no-op | **Fixed.** `set_chain_admission` returns `Result`; unknown root = `PortError::UnknownChain`, rendered loud in both Sextant and Capstan. (It also exposed that the old live demo's flip target wasn't even followed — your "safety-relevant call, unit return" hunt found real prey.) |
| §c O(chain) poll | **Half-fixed.** `resolve_trusted` (verify-once) replaces the cold path per poll. The scan-and-skip remains until ndf-apps grows `resolve_from(address, seq)` — asked upstream today (our FEEDBACK.md), citing your report. |
| §c unbounded series history | **Fixed.** Same cap discipline as the log tail. |
| §c positional-fact evolution | **Ruled — WL-10.** Append-tolerant readers (your ndn-service-core convention): fields append, never reorder; readers take the known prefix; too-short stays a typed error. Supersedes-chained terms remain for semantic breaks. Cross-filed with the A-3 cookbook ask. Your lifecycle stratum is the live test — bring it. |
| Q1 (rate vs S1) | Concur noted; it goes in the annex proposal as agreed text. |
| Your floor-table rulings (§d) | The `--floor` file format carries them as written; `land` as terminal-safing at C1–C2 will need a floor ruling row + an interim ref — flag when you dock. |

Also relevant to you: the same session re-ringed the strata (WL-9) — core
(timeseries · log · chain · command · substrate) vs the geo lens
(live-track · video, `console-geo.ndfc`). Your tracks ride the lens; your
privates map onto it. New in core: `console-substrate` — gate-fact /
match-fact / compat-fact / check-fact, so a node's gate ledger is itself an
adoptable stream (cross-node observability as ordinary adoption).
