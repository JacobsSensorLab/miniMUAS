//! miniMUAS v3 GCS dashboard binary — thin CLI shell over
//! [`muas_dashboard::start`] (v2 `run_dashboard.py`'s `main`).

use std::sync::Arc;

use muas_dashboard::providers::StubDetector;
use muas_dashboard::{parse_outcome, HELP};

fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let config = match parse_outcome(&args) {
        Ok(Some(config)) => config,
        Ok(None) => {
            print!("{HELP}");
            return std::process::ExitCode::SUCCESS;
        }
        Err(err) => {
            eprintln!("error: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info".into()),
        )
        .init();

    let runtime = match tokio::runtime::Runtime::new() {
        Ok(rt) => rt,
        Err(err) => {
            eprintln!("error: tokio runtime: {err}");
            return std::process::ExitCode::FAILURE;
        }
    };
    runtime.block_on(async move {
        let running = match muas_dashboard::start(config, Arc::new(StubDetector)).await {
            Ok(running) => running,
            Err(err) => {
                eprintln!("error: {err}");
                return std::process::ExitCode::FAILURE;
            }
        };
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                tracing::info!("ctrl-c: shutting down");
            }
            () = running.cancelled() => {}
        }
        running.shutdown().await;
        std::process::ExitCode::SUCCESS
    })
}
