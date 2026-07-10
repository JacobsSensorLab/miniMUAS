//! miniMUAS v3 drone agent. Lands at milestone M3: flight services over
//! ndn-service (pluggable carrier), telemetry/coord as Sparks, journals as
//! Block chains, PeerGuard fleet coordination.

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
        "muas-agent scaffold — services land at M3"
    );
}
