//! # muas-artifacts — no data silos
//!
//! The very same data miniMUAS collects — app-level events (journals,
//! coordination, services) through network/link stats (telemetry
//! inter-arrival, RTTs, spark loss) — powers an operator report, a
//! stakeholder deck, a live demo page, and an inter-run comparison, showing
//! as much or as little as each audience needs, **without copying data
//! between app and artifact silos**. Artifacts are *lenses over the same
//! named, content-addressed data*; provenance is the Block hash, not a
//! file copy.
//!
//! The human-facing layer leads with **input→output association** — "what
//! settings produced this result" (run configuration ↔ outcomes, setting
//! deltas ↔ outcome deltas across runs). The hashes assure the *system*:
//! they live behind progressive disclosure in the artifacts and are the
//! whole point of the `--audit` surface.
//!
//! Module map:
//! - [`dataset`] — the ONE `MissionDataset`: run config + every datum with
//!   its `(block hash, chain, seq)` provenance.
//! - [`chains`] — the resolver: live over NDN (`AppRuntime` follow/resolve;
//!   zero file reads for mission data) and the journal fallback that
//!   republishes JSONL through the identical publish path (the transport
//!   differs, the data identity doesn't). Plus the standalone chain-mirror
//!   for dashboard recordings.
//! - [`contracts`] — the `mission-dataset` / `run-set` manifests and the
//!   four render contracts (`artifact.report|deck|demo|compare`), matched
//!   through uas-console's Binder (match → authorize → instantiate).
//! - [`metrics`] — outcome metrics computed on demand from the dataset
//!   (no per-artifact transformation caches).
//! - [`render`] — the four `Via::Native` renderers, all capturing the same
//!   `Arc`-shared run set.
//! - [`audit`] — artifact → citations → re-fetch + re-hash verification.

#![deny(missing_docs)]

pub mod audit;
pub mod chains;
pub mod contracts;
pub mod dataset;
pub mod metrics;
pub mod render;
