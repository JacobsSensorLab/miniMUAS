//! The provenance proof (`--audit`): artifact → the `(block hash, chain,
//! seq)` set it consumed → each hash verified by re-fetch + re-hash. This
//! is the system-assurance surface — the artifacts themselves lead with the
//! human-valuable association layer and keep hashes behind disclosure; the
//! audit is where the hashes ARE the point.

use std::collections::BTreeMap;

use serde_json::{json, Value};

use crate::chains::Mission;
use crate::render::RenderedArtifact;

/// Build the audit manifest. For every citation of every artifact:
/// - re-fetch the Block from its chain (cold `resolve`, which re-verifies
///   the signed envelope against the pinned writer key), and
/// - re-hash the fetched packet and compare with the cited hash.
///
/// Also reports which hashes are shared across artifacts — the "two lenses,
/// one datum, one hash" property.
pub fn audit(
    missions: &[&Mission],
    artifacts: &BTreeMap<String, RenderedArtifact>,
) -> Result<Value, String> {
    // Keyed by content hash — the identity. Chain names can repeat across
    // runs of the same fleet; the hash never collides across distinct data.
    let mut fetched: BTreeMap<String, (String, u64, String)> = BTreeMap::new();
    let mut blocks_fetched = 0usize;
    for mission in missions {
        for b in mission.refetch_all()? {
            blocks_fetched += 1;
            fetched.insert(b.stored.clone(), (b.chain, b.seq, b.rehashed));
        }
    }

    let mut shared: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut all_verified = true;
    let mut artifact_reports = serde_json::Map::new();
    for (name, art) in artifacts {
        let mut cites = Vec::new();
        for c in &art.citations {
            let (verified, refetched) = match fetched.get(&c.hash) {
                Some((chain, seq, rehashed)) => (
                    chain == &c.chain && *seq == c.seq && rehashed == &c.hash,
                    Some(rehashed.clone()),
                ),
                None => (false, None),
            };
            all_verified &= verified;
            shared.entry(c.hash.clone()).or_default().push(name.clone());
            cites.push(json!({
                "hash": c.hash,
                "chain": c.chain,
                "seq": c.seq,
                "verified": verified,
                "rehashed": refetched,
            }));
        }
        artifact_reports.insert(
            name.clone(),
            json!({ "citations": cites, "count": art.citations.len() }),
        );
    }

    let shared_hashes: Vec<Value> = shared
        .iter()
        .filter(|(_, artifacts)| artifacts.len() > 1)
        .map(|(hash, artifacts)| json!({ "hash": hash, "artifacts": artifacts }))
        .collect();

    Ok(json!({
        "artifacts": Value::Object(artifact_reports),
        "shared": shared_hashes,
        "blocks_fetched": blocks_fetched,
        "verified": all_verified,
        "note": "verified = cited hash == stored chain-node hash == sha256 of the \
                 re-fetched signed packet; two artifacts citing one datum cite one hash",
    }))
}

/// Convenience: assert every citation verified (used by tests and the CLI
/// exit code).
pub fn all_verified(report: &Value) -> bool {
    report["verified"].as_bool() == Some(true)
}

/// Convenience: the hashes cited by more than one artifact.
pub fn shared_hashes(report: &Value) -> Vec<String> {
    report["shared"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|s| s["hash"].as_str().map(str::to_string))
                .collect()
        })
        .unwrap_or_default()
}
