//! Catalog drift tripwire, muas-sim side: this crate broadcasts WS
//! messages through the dashboard's hub (`net`, `sim_anomalies` from the
//! deployment exporter), so the dashboard's surface catalog must list
//! them. The dashboard's own source-scan test cannot see this crate —
//! this closes that gap from the emitter's side.

use muas_dashboard::catalog;

#[test]
fn sim_emitted_ws_types_are_catalogued() {
    let kinds = catalog::kind_names();
    for emitted in ["net", "sim_anomalies"] {
        assert!(
            kinds.contains(&emitted),
            "muas-sim broadcasts type \"{emitted}\" — it must appear in \
             muas_dashboard::catalog::UNDERSTOOD_KINDS"
        );
    }
}

#[test]
fn prefix_grouping_matches_what_the_lens_documents() {
    // The catalog's `net` kind promises per-prefix rates grouped at the
    // semantic namespace; nettap::group_prefix is that grouping.
    assert_eq!(
        muas_sim::nettap::group_prefix("/muas/v3/iuas-02/coord/status"),
        "/muas/v3/iuas-02/coord"
    );
    let net_kind = catalog::UNDERSTOOD_KINDS
        .iter()
        .find(|k| k.kind == "net")
        .expect("net kind catalogued");
    assert!(
        net_kind.description.contains("per-prefix"),
        "the catalog documents the namespace-lens feed"
    );
}
