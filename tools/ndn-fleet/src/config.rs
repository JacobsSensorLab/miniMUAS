//! `fleet.toml`: the single inventory. Relative paths resolve against the file's directory, so the
//! server behaves the same from any working directory.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub fleet: FleetCfg,
    pub dashboard: DashboardCfg,
    pub deploy: DeployCfg,
    pub workloads: WorkloadsCfg,
    pub restart: RestartCfg,
    #[serde(rename = "node")]
    pub nodes: Vec<Node>,
    /// Directory `fleet.toml` lives in; relative paths resolve against it.
    #[serde(skip)]
    pub root: PathBuf,
}

#[derive(Debug, Clone, Deserialize)]
pub struct FleetCfg {
    pub ssh_user: String,
    pub jump_host: String,
    pub results_dir: PathBuf,
    pub known_good_cell: String,
    pub canary: String,
    pub default_settle_s: u64,
    pub journal_truncate_bytes: u64,
    pub journal_glob: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DashboardCfg {
    pub host: String,
    pub port: u16,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DeployCfg {
    pub config_repo: PathBuf,
    pub config_branch: String,
    pub pins_file: PathBuf,
    /// `{node}` is replaced by the node name.
    pub flake_attr: String,
    pub build_retries: u32,
    /// Local miniMUAS checkout: the repo behind the config flake's `minimuas-src` input.
    #[serde(default)]
    pub minimuas_local: Option<PathBuf>,
    /// Path prefixes in the miniMUAS checkout that no fleet derivation builds. Uncommitted
    /// changes there cannot be mistaken for shipping, so they do not block a miniMUAS bump.
    #[serde(default)]
    pub minimuas_not_built: Vec<String>,
    #[serde(rename = "repo")]
    pub repos: Vec<PinnedRepo>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PinnedRepo {
    /// The `srcs.<name>` block in the pins file, and the GitHub repo name.
    pub name: String,
    pub owner: String,
    pub local: PathBuf,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WorkloadsCfg {
    pub flightcheck: PathBuf,
    pub fabric_bench: PathBuf,
    pub python: String,
}

/// Role services in restart order (PROTOCOL.md I5). `forwarder` units are started, never
/// restarted; the rest restart only when not already fresh (`cells::refresh_script`).
#[derive(Debug, Clone, Deserialize)]
pub struct RestartCfg {
    pub forwarder: Vec<String>,
    pub agents: Vec<String>,
    pub control: Vec<String>,
    pub dashboard: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
pub struct Node {
    pub name: String,
    pub host: String,
    pub addr: String,
    pub role: String,
    pub vehicle: String,
}

impl Node {
    pub fn is_gcs(&self) -> bool {
        self.role == "gcs"
    }
}

impl Config {
    /// Load `path`, resolving relative paths against its directory.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading fleet inventory {}", path.display()))?;
        let mut cfg: Config =
            toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
        let root = path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| PathBuf::from("."));
        let abs = |p: &Path| {
            if p.is_absolute() {
                p.to_path_buf()
            } else {
                root.join(p)
            }
        };
        cfg.fleet.results_dir = abs(&cfg.fleet.results_dir);
        cfg.deploy.config_repo = abs(&cfg.deploy.config_repo);
        for r in &mut cfg.deploy.repos {
            r.local = abs(&r.local);
        }
        cfg.deploy.minimuas_local = cfg.deploy.minimuas_local.as_deref().map(abs);
        cfg.workloads.flightcheck = abs(&cfg.workloads.flightcheck);
        cfg.workloads.fabric_bench = abs(&cfg.workloads.fabric_bench);
        cfg.root = root;
        cfg.validate()?;
        Ok(cfg)
    }

    fn validate(&self) -> Result<()> {
        if self.nodes.iter().filter(|n| n.is_gcs()).count() != 1 {
            bail!("fleet.toml must declare exactly one gcs node");
        }
        if self.node(&self.fleet.canary).is_none_or(Node::is_gcs) {
            bail!("canary '{}' must be a declared airframe", self.fleet.canary);
        }
        Ok(())
    }

    pub fn node(&self, name: &str) -> Option<&Node> {
        self.nodes.iter().find(|n| n.name == name)
    }

    pub fn gcs(&self) -> &Node {
        self.nodes.iter().find(|n| n.is_gcs()).expect("validated")
    }

    /// Rollout order (PROTOCOL.md I5): canary, other airframes, GCS last.
    pub fn rollout_order(&self) -> Vec<&Node> {
        let canary = self.node(&self.fleet.canary).expect("validated");
        let mut order = vec![canary];
        order.extend(self.nodes.iter().filter(|n| !n.is_gcs() && *n != canary));
        order.push(self.gcs());
        order
    }

    /// The default inventory: `tools/ndn-fleet/fleet.toml` next to the binary's crate, or
    /// `NDN_FLEET_CONFIG`.
    pub fn default_path() -> PathBuf {
        std::env::var_os("NDN_FLEET_CONFIG")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("fleet.toml"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_committed_inventory_loads_and_orders_the_rollout() {
        let cfg = Config::load(&Config::default_path()).expect("fleet.toml loads");
        let order: Vec<&str> = cfg
            .rollout_order()
            .iter()
            .map(|n| n.name.as_str())
            .collect();
        assert_eq!(
            order.first(),
            Some(&cfg.fleet.canary.as_str()),
            "canary first"
        );
        assert_eq!(order.last(), Some(&cfg.gcs().name.as_str()), "GCS last");
        assert_eq!(order.len(), cfg.nodes.len(), "every node exactly once");
        assert!(cfg.deploy.config_repo.is_absolute() && cfg.fleet.results_dir.is_absolute());
    }
}
