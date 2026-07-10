//! End-to-end gates for the no-data-silos artifact generator:
//! dataset building with real Block provenance (offline fallback AND live
//! over NDN), the four bindings Express through the console Binder,
//! shared-hash provenance across artifacts, deterministic artifact bytes,
//! and the audit re-fetch/re-hash proof.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use muas_artifacts::audit::{all_verified, audit, shared_hashes};
use muas_artifacts::chains::{from_journal_dir, mirror_lines_into_chain, resolve_live, Bootstrap};
use muas_artifacts::contracts::intent;
use muas_artifacts::dataset::{hex, DatumKind};
use muas_artifacts::render::{produce, RunSet};

fn fixture_run(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fixtures/runs").join(name)
}

#[tokio::test(flavor = "multi_thread")]
async fn fallback_dataset_carries_block_provenance() {
    let mission = from_journal_dir(&fixture_run("radio-ndr-good")).await.expect("mission");
    let ds = &mission.dataset;

    // Run-scoped: config + outcomes under one run id.
    assert_eq!(ds.run.run_id.as_deref(), Some("radio-ndr-good"));
    assert!(!ds.run.synthesized, "run.config record was present");
    assert_eq!(ds.run.display("link_profile.loss_pct").as_deref(), Some("1.0"));
    assert_eq!(ds.run.display("coord.grace_s").as_deref(), Some("1.5"));

    // Three chains: two vehicle journals + one recording.
    let chains: std::collections::BTreeSet<_> =
        ds.blocks.iter().map(|b| b.chain.clone()).collect();
    assert!(chains.contains("/muas/v3/iuas-01/journal/companion"));
    assert!(chains.contains("/muas/v3/wuas-01/journal/companion"));
    assert!(chains.iter().any(|c| c.contains("/gcs/recording/")));

    // Every datum cites a real block; hashes are 64 hex chars.
    assert!(!ds.data.is_empty());
    for d in &ds.data {
        let c = ds.citation_of(d);
        assert_eq!(c.hash.len(), 64);
        assert!(c.hash.chars().all(|ch| ch.is_ascii_hexdigit()));
    }
    // All the datum families arrived.
    for kind in [DatumKind::Telemetry, DatumKind::Coord, DatumKind::Service, DatumKind::Rtl, DatumKind::Link]
    {
        assert!(ds.of_kind(kind).count() > 0, "missing {kind:?} data");
    }
    mission.shutdown().await;
}

#[tokio::test(flavor = "multi_thread")]
async fn fallback_hashes_are_deterministic_across_rebuilds() {
    let a = from_journal_dir(&fixture_run("radio-apsta")).await.expect("first build");
    let b = from_journal_dir(&fixture_run("radio-apsta")).await.expect("second build");
    let ha: Vec<String> = a.dataset.blocks.iter().map(|blk| hex(&blk.hash)).collect();
    let hb: Vec<String> = b.dataset.blocks.iter().map(|blk| hex(&blk.hash)).collect();
    assert_eq!(ha, hb, "same journals => same Block identities (deterministic republish)");
    a.shutdown().await;
    b.shutdown().await;
}

#[tokio::test(flavor = "multi_thread")]
async fn three_lenses_bind_express_and_share_hashes() {
    let mission = from_journal_dir(&fixture_run("radio-ndr-good")).await.expect("mission");
    let set = Arc::new(RunSet { runs: vec![Arc::clone(&mission.dataset)] });

    // The flotilla dogfood: match → authorize → instantiate must actually run.
    let artifacts = produce(&set, &[intent::REPORT, intent::DECK, intent::DEMO]).expect("bound + rendered");
    assert_eq!(artifacts.len(), 3);
    for name in ["report.html", "deck.html", "demo.html"] {
        let art = artifacts.get(name).unwrap_or_else(|| panic!("{name} produced"));
        assert!(art.html.starts_with("<!doctype html>"), "{name} is a full page");
        assert!(!art.citations.is_empty(), "{name} carries provenance");
    }

    // Shared-hash assertion: a datum consumed by two artifacts cites ONE hash.
    let ds = &mission.dataset;
    let coop = ds
        .data
        .iter()
        .find(|d| d.label == "coord.coop")
        .expect("a cooperative episode in the fixture");
    let coop_cite = ds.citation_of(coop);
    for name in ["report.html", "deck.html", "demo.html"] {
        assert!(
            artifacts[name].citations.contains(&coop_cite),
            "{name} cites the coop datum's block by the same hash"
        );
    }

    // The association layer leads: the report presents the run config, not hashes.
    let report = &artifacts["report.html"].html;
    assert!(report.contains("Run configuration"));
    assert!(report.contains("associated settings"));
    // Hashes are behind progressive disclosure (drawer), present but not headline.
    assert!(report.contains("provenance —"));
    assert!(report.contains(&coop_cite.hash), "full hash available in tooltips/drawers");

    mission.shutdown().await;
}

#[tokio::test(flavor = "multi_thread")]
async fn artifact_bytes_are_deterministic() {
    let a = from_journal_dir(&fixture_run("radio-apsta")).await.expect("first");
    let b = from_journal_dir(&fixture_run("radio-apsta")).await.expect("second");
    let set_a = Arc::new(RunSet { runs: vec![Arc::clone(&a.dataset)] });
    let set_b = Arc::new(RunSet { runs: vec![Arc::clone(&b.dataset)] });
    let intents = [intent::REPORT, intent::DECK, intent::DEMO];
    let out_a = produce(&set_a, &intents).expect("render a");
    let out_b = produce(&set_b, &intents).expect("render b");
    for name in ["report.html", "deck.html", "demo.html"] {
        assert_eq!(out_a[name].html, out_b[name].html, "{name}: same dataset => same bytes");
    }
    a.shutdown().await;
    b.shutdown().await;
}

#[tokio::test(flavor = "multi_thread")]
async fn compare_lens_binds_and_leads_with_setting_deltas() {
    let mut missions = Vec::new();
    for run in ["radio-apsta", "radio-ndr-good", "radio-ndr-contested"] {
        missions.push(from_journal_dir(&fixture_run(run)).await.expect(run));
    }
    let set = Arc::new(RunSet {
        runs: missions.iter().map(|m| Arc::clone(&m.dataset)).collect(),
    });
    let artifacts = produce(&set, &[intent::COMPARE]).expect("compare bound + rendered");
    let cmp = &artifacts["compare.html"];

    // The association: link-profile settings are the deltas; grace is shared.
    assert!(cmp.html.contains("link_profile.loss_pct"), "delta setting surfaces as a column");
    assert!(cmp.html.contains("shared settings"));
    assert!(cmp.html.contains("coord.grace_s"), "unchanged setting collapses into shared header");
    assert!(cmp.html.contains("coop rate %"), "outcome column present");
    assert!(cmp.html.contains("download CSV"));
    assert!(cmp.html.contains("associated"), "association framing, not causality");
    // All three runs cited.
    for m in &missions {
        let one = m.dataset.blocks.first().expect("blocks");
        assert!(cmp.citations.iter().any(|c| c.hash == hex(&one.hash)), "run's blocks cited");
    }

    // Cross-artifact shared hash: the compare and the single-run report cite
    // the same block hash for the same datum.
    let report = produce(
        &Arc::new(RunSet { runs: vec![Arc::clone(&missions[0].dataset)] }),
        &[intent::REPORT],
    )
    .expect("report");
    let ds = &missions[0].dataset;
    let coop_cite = ds.citation_of(ds.data.iter().find(|d| d.label == "coord.coop").expect("coop"));
    assert!(report["report.html"].citations.contains(&coop_cite));
    assert!(cmp.citations.contains(&coop_cite), "one datum, one hash, two artifacts");

    for m in missions {
        m.shutdown().await;
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn audit_verifies_every_citation_by_refetch_and_rehash() {
    let mission = from_journal_dir(&fixture_run("radio-ndr-good")).await.expect("mission");
    let set = Arc::new(RunSet { runs: vec![Arc::clone(&mission.dataset)] });
    let artifacts = produce(&set, &[intent::REPORT, intent::DECK, intent::DEMO]).expect("artifacts");
    let map: BTreeMap<_, _> = artifacts.into_iter().collect();

    let report = audit(&[&mission], &map).expect("audit");
    assert!(all_verified(&report), "every citation re-fetches and re-hashes: {report:#}");
    // Two artifacts consuming the same datum cite the same hash.
    let shared = shared_hashes(&report);
    assert!(!shared.is_empty(), "hashes are shared across artifacts");
    let expected = mission.dataset.all_citations();
    for c in &expected {
        assert!(shared.contains(&c.hash), "block {} cited by all three artifacts", c.hash);
    }
    mission.shutdown().await;
}

/// The live path: a "vehicle" process publishes journal batches on its chain
/// over a real two-engine UDP link; the resolver fetches them BY NAME over
/// NDN (no file reads) and produces the identical dataset + hashes.
#[tokio::test(flavor = "multi_thread")]
async fn live_resolver_fetches_blocks_over_ndn() {
    use ed25519_dalek::SigningKey;
    use muas_artifacts::dataset::from_hex;
    use ndf_apps::{make_reachable, AppRuntime, Identity};
    use ndn_engine::builder::{EngineBuilder, EngineConfig};
    use ndn_face::UdpFace;
    use tokio_util::sync::CancellationToken;

    let cancel = CancellationToken::new();

    // ── the "vehicle": engine A + journal chain ─────────────────────────
    let (engine_a, shutdown_a) = EngineBuilder::new(EngineConfig::default())
        .build()
        .await
        .expect("engine a");
    let writer_seed = [7u8; 32];
    let identity = Identity::new(
        "/muas/v3/iuas-01",
        "companion",
        SigningKey::from_bytes(&writer_seed),
    );
    let writer_key = identity.public_key();
    let mut rt_a = AppRuntime::attach(engine_a.clone(), identity, cancel.child_token());
    let address = rt_a.identity().chain("journal");

    let t0 = 1_780_000_000_000_000_000u64;
    let lines: Vec<(u64, String)> = vec![
        (
            t0,
            serde_json::json!({"kind":"run.config","ts_ns":t0,"vehicle_id":"iuas-01",
                "run_id":"live-1","config":{"coord":{"grace_s":2.5},"carrier":"rpc"}})
            .to_string(),
        ),
        (
            t0 + 1_000_000_000,
            serde_json::json!({"kind":"coord.coop","ts_ns":t0 + 1_000_000_000,
                "vehicle_id":"iuas-01","peer":"wuas-01","bias_m":2.0,"run_id":"live-1"})
            .to_string(),
        ),
        (
            t0 + 5_000_000_000,
            serde_json::json!({"kind":"rtl.done","ts_ns":t0 + 5_000_000_000,
                "vehicle_id":"iuas-01","outcome":"landed","run_id":"live-1"})
            .to_string(),
        ),
    ];
    let receipts = mirror_lines_into_chain(&mut rt_a, &address, &lines).await.expect("published");
    assert_eq!(receipts.len(), 2, "two 2-second windows => two blocks");

    // ── UDP link between the two engines ────────────────────────────────
    let addr_a: std::net::SocketAddr = "127.0.0.1:47921".parse().expect("addr");
    let addr_b: std::net::SocketAddr = "127.0.0.1:47922".parse().expect("addr");
    let face_a = engine_a.faces().alloc_id();
    engine_a.add_face(
        UdpFace::bind(addr_a, addr_b, face_a).await.expect("bind a"),
        cancel.child_token(),
    );
    make_reachable(&engine_a, &address, face_a).expect("route a");

    // ── the resolver: bootstrap only (endpoint + chain + identity key) ──
    let bootstrap = Bootstrap::from_json(&format!(
        r#"{{
            "identity": {{
                "principal": "/muas/v3/gcs", "device": "artifacts",
                "key_seed_hex": "{reader}"
            }},
            "links": [ {{ "local": "{local}", "remote": "{remote}" }} ],
            "chains": [ {{
                "role": "vehicle-journal",
                "root": "{root}",
                "writer": "{writer}",
                "writer_key_hex": "{key}"
            }} ],
            "max_rounds": 120, "quiet_rounds": 2
        }}"#,
        reader = hex(&[9u8; 32]),
        local = addr_b,
        remote = addr_a,
        root = address.root,
        writer = address.writer,
        key = hex(&writer_key),
    ))
    .expect("bootstrap parses");
    // Sanity: the hex round-trips.
    assert_eq!(from_hex::<32>(&hex(&writer_key)).expect("hex"), writer_key);

    let mission = resolve_live(&bootstrap).await.expect("live resolve");
    let ds = &mission.dataset;
    assert_eq!(ds.run.run_id.as_deref(), Some("live-1"));
    assert_eq!(ds.blocks.len(), 2);
    // The hashes on the wire are the hashes in the receipts — data identity
    // survives the transport.
    let fetched: Vec<String> = ds.blocks.iter().map(|b| hex(&b.hash)).collect();
    let published: Vec<String> = receipts.iter().map(|r| hex(&r.block)).collect();
    assert_eq!(fetched, published);
    assert_eq!(ds.of_kind(DatumKind::Coord).count(), 1);

    // And the audit's re-fetch/re-hash holds on the live store too.
    let set = Arc::new(RunSet { runs: vec![Arc::clone(&mission.dataset)] });
    let artifacts = produce(&set, &[intent::REPORT]).expect("report over live data");
    let map: BTreeMap<_, _> = artifacts.into_iter().collect();
    let audit_report = audit(&[&mission], &map).expect("audit");
    assert!(all_verified(&audit_report), "{audit_report:#}");

    mission.shutdown().await;
    cancel.cancel();
    shutdown_a.shutdown().await;
}
