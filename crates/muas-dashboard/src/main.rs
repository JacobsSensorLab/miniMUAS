//! miniMUAS v3 GCS dashboard. Lands at milestone M4: lossless v2 parity
//! (audited against docs/v3/surveys/minimuas-v2.md), views bound through
//! flotilla manifests + the render-contract matcher, replay from the
//! mission Block chain.

use tracing::info;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();
    info!(
        prefix = muas_contracts::names::APP_PREFIX,
        "muas-dashboard scaffold — parity build lands at M4"
    );
}
