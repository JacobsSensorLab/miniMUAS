# Feedback log → ndn-workspace & NDF (refounding) maintainers

Living log of friction, bugs, ideas, and praise encountered while building
miniMUAS v3 on these frameworks. One entry per item; keep entries dated and
specific enough to act on (repro steps or file/API names). Move items to
"Delivered" once handed to the maintainers.

Format:

```
## [YYYY-MM-DD] <short title>
- **Project:** ndn-workspace | ndf-rs | ndn-sim | ndn-ext | ndn-service | flotilla
- **Type:** bug | friction | missing-feature | idea | praise
- **Context:** what we were building when we hit it
- **Detail:** what happened / what we expected / suggested fix
```

---

## Open

## [2026-07-09] Refounding rev pin vs ndn-workspace HEAD — no breakage at scaffold depth
- **Project:** ndf-rs / ndn-workspace
- **Type:** praise (with a watch item)
- **Context:** miniMUAS v3 scaffold taking path deps on flotilla `manifest`,
  `render-contract`, and refounding `ndf-core`.
- **Detail:** refounding README pins ndn-workspace HEAD `5798fa3f`; our ndn-rs
  checkout is at `043d3c15`. All three crates compiled clean and our workspace
  tests pass. Watch item: deeper deps (`ndf-apps`, `ndf-surface`,
  `ndn-engine`-graph) haven't been exercised yet — first M3 build will tell.
  Suggestion for maintainers: a `refounding/COMPAT.md` (or CI matrix) stating
  which ndn-workspace revs the refounding is known-good against would remove
  guesswork for consumers.

## Delivered

(none yet)
