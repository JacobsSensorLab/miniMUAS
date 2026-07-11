//! Strategy-record integration seam (ROUND-3 §2) for the miniMUAS agent and
//! dashboard.
//!
//! `uas-fleet-strategy` ships the typed strategy records plus the
//! deterministic evaluator ([`provider_decision`] / [`rank_candidates`] /
//! [`reask_schedule`]). This module re-exports the pieces the agent's ack
//! path (`muas-agent/src/queue.rs`) and the dashboard's dispatcher
//! (`muas-dashboard/src/mission.rs`) interpret, and adds a small
//! **strategy-load seam** so both sides resolve the active strategy the same
//! way.
//!
//! It lives in `muas-contracts` on purpose: the dashboard reaches the
//! evaluator through here (its own `Cargo.toml` is owned by the RC surface
//! and left untouched), and the agent uses the same loader so "no strategy"
//! behaves identically on both planes.
//!
//! Sources ([`StrategySource`]):
//! - `None` → crate defaults. Every `uas-fleet-strategy` `Default` reproduces
//!   today's hardcoded miniMUAS behavior, so absence is behavior-neutral.
//! - [`StrategySource::Reference`] (`--strategy reference`) → the owner's
//!   reference scenario shipped **as data** — the
//!   `uas-fleet-strategy/reference/*.json` envelopes, embedded here and folded
//!   through the crate's own [`StrategyChainHistory::active`].
//! - [`StrategySource::ChainDir`] (`--strategy-chain <dir>`) → fold every
//!   strategy-envelope JSON file in a directory. A filesystem stand-in for a
//!   followed fleet/mission chain; the live NDF-chain fold is
//!   `uas_fleet_strategy::StrategyReader::active`, wired by the deployment.
//!
//! Load happens once at startup; a caller may re-invoke [`load_active`] to
//! swap (the agent exposes `AgentShared::reload_strategy`). Refresh-on-change
//! (following a live chain) is a deployment follow.

use std::path::{Path, PathBuf};

pub use uas_fleet_strategy::{
    deny_code, deny_when, provider_decision, rank_candidates, rank_term, reask_schedule,
    AcceptMode, Active, ActiveStrategies, CandidateSnapshot, DenyCondition, DispatchStrategy,
    FanOut, ObjectiveRecord, ProviderDecision, ProviderStrategy, QueueSnapshot, RankTerm,
    ReaskPolicy, RequesterStrategy, StrategyChainHistory, StrategyEntry, StrategyEnvelope,
    StrategyError, TieBreak,
};

/// Where the active strategy comes from. `None` (the absent case, not a
/// variant) everywhere = crate defaults = today's behavior.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StrategySource {
    /// The shipped reference scenario (embedded `reference/*.json`).
    Reference,
    /// Fold every `*.json` strategy envelope in this directory.
    ChainDir(PathBuf),
}

impl StrategySource {
    /// Parse a `--strategy <value>` flag: `reference` selects the shipped
    /// scenario; anything else is treated as a chain directory path
    /// (equivalent to `--strategy-chain <value>`).
    pub fn parse(value: &str) -> Self {
        if value == "reference" {
            StrategySource::Reference
        } else {
            StrategySource::ChainDir(PathBuf::from(value))
        }
    }
}

/// The reference scenario records, shipped as data (kept byte-identical to
/// `uas-fleet-strategy/reference/*.json`, which that crate's
/// `tests/reference.rs` pins against the typed records — so this embedding
/// never drifts from the sibling's canonical scenario).
const REFERENCE_ENVELOPES: [&str; 4] = [
    include_str!("../../../../uas-fleet/crates/uas-fleet-strategy/reference/provider.json"),
    include_str!("../../../../uas-fleet/crates/uas-fleet-strategy/reference/requester.json"),
    include_str!("../../../../uas-fleet/crates/uas-fleet-strategy/reference/dispatch.json"),
    include_str!("../../../../uas-fleet/crates/uas-fleet-strategy/reference/objective.json"),
];

/// Why a strategy source failed to load.
#[derive(Debug, thiserror::Error)]
pub enum StrategyLoadError {
    /// A directory read or a file read failed.
    #[error("strategy chain path {path}: {source}")]
    Io {
        /// The path that failed.
        path: String,
        /// The underlying IO error.
        source: std::io::Error,
    },
    /// A file's bytes were not a decodable strategy envelope.
    #[error("strategy envelope {file}: {source}")]
    Envelope {
        /// The file (or embedded slot) that failed.
        file: String,
        /// The underlying decode error.
        source: StrategyError,
    },
}

/// Resolve the active strategy for a source (or crate defaults when `None`).
///
/// The fold is `uas-fleet-strategy`'s own ([`StrategyChainHistory::active`]):
/// latest valid record per kind wins, `supersedes` honored, unknown
/// kinds/versions reported not fatal. The result's
/// `provider()`/`dispatch()`/`requester()` accessors fall back to today's
/// defaults for any kind the source didn't carry.
pub fn load_active(source: Option<&StrategySource>) -> Result<ActiveStrategies, StrategyLoadError> {
    match source {
        None => Ok(ActiveStrategies::default()),
        Some(StrategySource::Reference) => reference_active(),
        Some(StrategySource::ChainDir(dir)) => load_chain_dir(dir),
    }
}

/// The reference scenario folded into its active strategies (the embedded
/// `reference/*.json`).
pub fn reference_active() -> Result<ActiveStrategies, StrategyLoadError> {
    fold_envelopes(
        REFERENCE_ENVELOPES
            .iter()
            .enumerate()
            .map(|(i, text)| (format!("ref-{i}"), text.as_bytes().to_vec())),
    )
}

fn load_chain_dir(dir: &Path) -> Result<ActiveStrategies, StrategyLoadError> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|source| StrategyLoadError::Io { path: dir.display().to_string(), source })?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().and_then(|ext| ext.to_str()) == Some("json"))
        .collect();
    files.sort(); // deterministic chain order from filenames
    let mut named = Vec::with_capacity(files.len());
    for path in files {
        let bytes = std::fs::read(&path)
            .map_err(|source| StrategyLoadError::Io { path: path.display().to_string(), source })?;
        let hash = path.file_stem().and_then(|s| s.to_str()).unwrap_or_default().to_string();
        named.push((hash, bytes));
    }
    fold_envelopes(named)
}

/// Parse `(hash, bytes)` pairs as strategy envelopes, in order, and fold them.
fn fold_envelopes(
    named: impl IntoIterator<Item = (String, Vec<u8>)>,
) -> Result<ActiveStrategies, StrategyLoadError> {
    let mut entries = Vec::new();
    for (seq, (hash, bytes)) in named.into_iter().enumerate() {
        let envelope = StrategyEnvelope::from_bytes(&bytes)
            .map_err(|source| StrategyLoadError::Envelope { file: hash.clone(), source })?;
        entries.push(StrategyEntry { hash, seq: seq as u64, envelope });
    }
    Ok(StrategyChainHistory { entries, skipped: Vec::new() }.active())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference scenario dir, reachable in the dev path-dep checkout.
    const REFERENCE_DIR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../../uas-fleet/crates/uas-fleet-strategy/reference");

    #[test]
    fn absent_source_is_todays_defaults() {
        let active = load_active(None).unwrap();
        // Behavior-neutral: every accessor is the crate default = today's
        // hardcoded miniMUAS behavior.
        assert_eq!(active.provider(), ProviderStrategy::default());
        assert_eq!(active.dispatch(), DispatchStrategy::default());
        assert_eq!(active.requester(), RequesterStrategy::default());
        assert_eq!(active.provider().queue_depth, 4);
    }

    #[test]
    fn reference_records_load_from_embedded_data() {
        let active = reference_active().unwrap();
        let provider = active.provider();
        assert_eq!(provider.queue_depth, 1, "reference queues one extra interrogation");
        assert!(
            provider.deny_when.iter().any(|c| c.when == deny_when::FLIGHT_TIME_BELOW_S),
            "reference protects a flight-time reserve"
        );
        let dispatch = active.dispatch();
        assert_eq!(dispatch.ranking.len(), 4, "flight-floor, idle, least-queued, nearest");
        let requester = active.requester();
        assert_eq!(requester.reask.max_attempts, 5);
        assert_eq!(active.objective().objective, uas_fleet_strategy::Objective::InvestigateAll);
    }

    /// `--strategy-chain <dir>` folds the same shipped files to the same
    /// active strategy the embedded `--strategy reference` produces.
    #[test]
    fn chain_dir_load_matches_embedded_reference() {
        let from_dir = load_active(Some(&StrategySource::ChainDir(REFERENCE_DIR.into()))).unwrap();
        let embedded = reference_active().unwrap();
        assert_eq!(from_dir.provider(), embedded.provider());
        assert_eq!(from_dir.dispatch(), embedded.dispatch());
        assert_eq!(from_dir.requester(), embedded.requester());
    }

    /// The reference records drive the scenario's documented decisions end to
    /// end through the same three evaluator seams the agent + dispatcher call.
    #[test]
    fn reference_records_drive_the_documented_scenario() {
        let active = reference_active().unwrap();
        let provider = active.provider();
        let dispatch = active.dispatch();
        let requester = active.requester();

        // Provider (agent ack path): one camera IUAS busy on target 1; target
        // 2 confirms → queued slot 1; a third → queue-full; low flight-time →
        // deny; RTL → deny.
        let busy = QueueSnapshot {
            active_kind: Some("investigate".into()),
            battery_pct: Some(71.0),
            depth: 0,
            flight_time_est_s: Some(840.0),
        };
        assert_eq!(provider_decision(&provider, &busy), ProviderDecision::Queue { position: 1 });
        assert_eq!(
            provider_decision(&provider, &QueueSnapshot { depth: 1, ..busy.clone() }),
            ProviderDecision::Deny { code: deny_code::QUEUE_FULL.into() }
        );
        assert_eq!(
            provider_decision(
                &provider,
                &QueueSnapshot { flight_time_est_s: Some(120.0), ..busy }
            ),
            ProviderDecision::Deny { code: deny_when::FLIGHT_TIME_BELOW_S.into() }
        );

        // Dispatch (dashboard): the idle IUAS takes the job iff it clears the
        // 300 s flight-time floor.
        let flying = CandidateSnapshot {
            distance_m: Some(40.0),
            flight_time_est_s: Some(900.0),
            idle: false,
            queued: 1,
        };
        let idle = |ft: f64| CandidateSnapshot {
            distance_m: Some(120.0),
            flight_time_est_s: Some(ft),
            idle: true,
            queued: 0,
        };
        assert_eq!(rank_candidates(&dispatch, &[flying.clone(), idle(600.0)]), vec![1, 0]);
        assert_eq!(rank_candidates(&dispatch, &[flying, idle(120.0)]), vec![0, 1]);

        // Requester: 5 s doubling, 5 attempts, 120 s horizon.
        let schedule: Vec<Option<f64>> = (1..=5).map(|a| reask_schedule(&requester, a)).collect();
        assert_eq!(schedule, vec![Some(5.0), Some(10.0), Some(20.0), Some(40.0), None]);
    }
}
