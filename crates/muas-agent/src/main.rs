//! miniMUAS v3 drone agent binary — a thin CLI shell over the `muas_agent`
//! library (muas-sim embeds the same [`muas_agent::Agent`] directly).

use std::process::ExitCode;

use muas_agent::{Agent, ParseOutcome, HELP};
use tracing::{error, info};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let config = match muas_agent::config::parse_args(&args) {
        Ok(ParseOutcome::Help) => {
            print!("{HELP}");
            return ExitCode::SUCCESS;
        }
        Ok(ParseOutcome::Run(config)) => *config,
        Err(err) => {
            eprintln!("muas-agent: {err}");
            return ExitCode::from(2);
        }
    };

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let runtime = match tokio::runtime::Builder::new_multi_thread().enable_all().build() {
        Ok(runtime) => runtime,
        Err(err) => {
            error!(%err, "tokio runtime build failed");
            return ExitCode::FAILURE;
        }
    };

    let outcome = runtime.block_on(async move {
        let handle = Agent::start(config).await?;
        tokio::select! {
            _ = tokio::signal::ctrl_c() => info!("ctrl-c: shutting down"),
            _ = handle.cancelled() => info!("agent cancelled (shutdown service or embedder)"),
        }
        handle.shutdown().await;
        Ok::<(), String>(())
    });

    match outcome {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(%err, "agent failed to start");
            ExitCode::FAILURE
        }
    }
}
