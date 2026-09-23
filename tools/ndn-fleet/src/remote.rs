//! Running commands: on a node over ssh, or locally. Every call is bounded by a timeout, and the
//! full command, exit status, stdout and stderr come back so callers can record them verbatim
//! (PROTOCOL.md I8).

use std::path::Path;
use std::process::Stdio;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Serialize;
use tokio::process::Command;

use crate::config::{Config, Node};

#[derive(Debug, Clone, Serialize)]
pub struct Output {
    /// What ran, for the record (`ssh <host> '<cmd>'` or the local argv).
    pub command: String,
    /// `None` when the process was killed by the timeout.
    pub status: Option<i32>,
    pub stdout: String,
    pub stderr: String,
    pub elapsed_ms: u64,
}

impl Output {
    pub fn ok(&self) -> bool {
        self.status == Some(0)
    }

    /// `Ok(stdout)` on success, otherwise an error carrying the command and stderr.
    pub fn stdout_ok(&self) -> Result<&str> {
        if self.ok() {
            Ok(&self.stdout)
        } else {
            anyhow::bail!(
                "`{}` failed ({}): {}",
                self.command,
                self.status
                    .map_or_else(|| "timed out".to_string(), |s| format!("exit {s}")),
                self.stderr.trim()
            )
        }
    }
}

/// ssh's own failure code: connection-level, worth one retry through the jump host.
const SSH_CONNECT_FAILURE: i32 = 255;

fn ssh_base(cfg: &Config, jump: bool) -> Vec<String> {
    let mut args = vec![
        "-o".into(),
        "BatchMode=yes".into(),
        "-o".into(),
        "ConnectTimeout=10".into(),
        "-o".into(),
        "ServerAliveInterval=15".into(),
    ];
    if jump {
        args.push("-J".into());
        args.push(format!("{}@{}", cfg.fleet.ssh_user, cfg.fleet.jump_host));
    }
    args
}

/// Run `cmd` on `node` via ssh. A connection failure is retried once through the jump host,
/// addressing the node by its fleet IP.
pub async fn ssh(cfg: &Config, node: &Node, cmd: &str, timeout: Duration) -> Result<Output> {
    let direct = ssh_once(cfg, &node.host, cmd, timeout, false).await?;
    if direct.status != Some(SSH_CONNECT_FAILURE) {
        return Ok(direct);
    }
    ssh_once(cfg, &node.addr, cmd, timeout, true).await
}

async fn ssh_once(
    cfg: &Config,
    target: &str,
    cmd: &str,
    timeout: Duration,
    jump: bool,
) -> Result<Output> {
    let mut args = ssh_base(cfg, jump);
    args.push(format!("{}@{}", cfg.fleet.ssh_user, target));
    args.push(cmd.to_string());
    let via = if jump { "-J <jump> " } else { "" };
    let shown = format!("ssh {via}{}@{target} {}", cfg.fleet.ssh_user, sh_quote(cmd));
    run(Command::new("ssh").args(&args), shown, timeout).await
}

/// Run a local program. `envs` are added to the inherited environment.
pub async fn local(
    program: &str,
    args: &[&str],
    cwd: Option<&Path>,
    envs: &[(&str, &str)],
    timeout: Duration,
) -> Result<Output> {
    let mut command = Command::new(program);
    command.args(args);
    if let Some(dir) = cwd {
        command.current_dir(dir);
    }
    for (k, v) in envs {
        command.env(k, v);
    }
    let shown = std::iter::once(program)
        .chain(args.iter().copied())
        .collect::<Vec<_>>()
        .join(" ");
    run(&mut command, shown, timeout).await
}

async fn run(command: &mut Command, shown: String, timeout: Duration) -> Result<Output> {
    let started = Instant::now();
    let child = command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .with_context(|| format!("spawning `{shown}`"))?;
    let elapsed = |s: Instant| s.elapsed().as_millis() as u64;
    match tokio::time::timeout(timeout, child.wait_with_output()).await {
        Ok(out) => {
            let out = out.with_context(|| format!("waiting for `{shown}`"))?;
            Ok(Output {
                command: shown,
                status: out.status.code(),
                stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
                stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
                elapsed_ms: elapsed(started),
            })
        }
        // Dropping the future kills the child (kill_on_drop).
        Err(_) => Ok(Output {
            command: shown,
            status: None,
            stdout: String::new(),
            stderr: format!("timed out after {}s", timeout.as_secs()),
            elapsed_ms: elapsed(started),
        }),
    }
}

/// Quote `s` for a POSIX shell (single quotes, embedded quotes escaped).
pub fn sh_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', r"'\''"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn a_timeout_kills_the_command_and_reports_it() {
        let out = local("sleep", &["5"], None, &[], Duration::from_millis(200))
            .await
            .unwrap();
        assert_eq!(out.status, None);
        assert!(out.stdout_ok().is_err());
        assert!(out.elapsed_ms < 2000);
    }

    #[test]
    fn quoting_survives_embedded_single_quotes() {
        assert_eq!(sh_quote("a'b"), r"'a'\''b'");
    }
}
