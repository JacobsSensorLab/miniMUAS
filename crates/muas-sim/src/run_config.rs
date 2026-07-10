//! The deployment `run-config` record: the INPUT side of a run as
//! first-class data — "what set of settings produced this output?".
//!
//! One [`RunConfig`] is built at deployment start and journaled as a single
//! typed `run-config` event into EVERY vehicle's power-loss-safe journal
//! AND the deployment's own log file. Every subsequent journal line carries
//! the same `run_id` (stamped by `muas_agent::journal`), so downstream
//! tooling can join all outputs — journals, replays, verdicts — back to the
//! exact configuration that produced them.
//!
//! Schema stability: this is serde-typed, additive-only JSON. Consumers
//! must tolerate unknown fields.

use std::collections::BTreeMap;
use std::time::Duration;

use serde::{Deserialize, Serialize};

/// Everything that configured one virtual-deployment run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunConfig {
    /// Timestamp-based run identifier (`run-<unix-seconds>`); stamped onto
    /// every journal line of the run.
    pub run_id: String,
    /// Wall clock at run start, nanoseconds since the Unix epoch.
    pub created_ns: u64,
    /// Per-vehicle agent configuration actually in force.
    pub vehicles: Vec<VehicleRunConfig>,
    /// The ndn-sim link profile every fabric SimLink carries.
    pub link_profile: LinkProfileConfig,
    /// The SITL processes backing the vehicles.
    pub sitl: SitlRunConfig,
    /// Source revisions of the stack (`git rev-parse HEAD` per repo at
    /// startup; `"unknown"` when a repo could not be read).
    pub stack_revs: BTreeMap<String, String>,
}

/// The [`muas_agent::AgentConfig`] facts that shape flight + coordination
/// behavior (a stable, serializable projection — AgentConfig itself carries
/// non-config runtime wiring like socket addrs).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleRunConfig {
    pub vehicle_id: String,
    pub fleet_ids: Vec<String>,
    /// `"sim"` or the mavlink endpoint string (`"mavlink:tcp:..."`).
    pub endpoint: String,
    pub telemetry_hz: f64,
    /// Service carrier: `"rpc"` or `"ndnsf"`.
    pub carrier: String,
    /// Capability extras advertised on `telemetry/state`.
    pub extras: Vec<String>,
    // -- field-safety rails --
    pub max_range_m: f64,
    pub min_agl_m: f64,
    pub max_agl_m: f64,
    // -- coordination knobs (PeerGuard) --
    pub hsep_m: f64,
    pub vsep_m: f64,
    pub horizon_s: f64,
    pub grace_s: f64,
    pub floor_agl_m: f64,
    // -- smart RTL --
    pub rtl_base_agl_m: f64,
    pub rtl_sep_m: f64,
}

impl VehicleRunConfig {
    /// Project the journal-relevant facts out of a full agent config.
    pub fn from_agent(config: &muas_agent::AgentConfig) -> Self {
        Self {
            vehicle_id: config.vehicle_id.clone(),
            fleet_ids: config.fleet_ids.clone(),
            endpoint: match &config.endpoint {
                muas_agent::Endpoint::Sim { lat_deg, lon_deg } => {
                    format!("sim:{lat_deg},{lon_deg}")
                }
                muas_agent::Endpoint::Mavlink(ep) => format!("mavlink:{ep}"),
            },
            telemetry_hz: config.telemetry_hz,
            carrier: match config.carrier {
                muas_agent::CarrierKind::Rpc => "rpc".to_string(),
                muas_agent::CarrierKind::Ndnsf => "ndnsf".to_string(),
            },
            extras: config.extras.clone(),
            max_range_m: config.max_range_m,
            min_agl_m: config.agl_bounds.min_agl_m,
            max_agl_m: config.agl_bounds.max_agl_m,
            hsep_m: config.hsep_m,
            vsep_m: config.vsep_m,
            horizon_s: config.horizon_s,
            grace_s: config.grace_s,
            floor_agl_m: config.floor_agl_m,
            rtl_base_agl_m: config.rtl_base_agl_m,
            rtl_sep_m: config.rtl_sep_m,
        }
    }
}

/// One ndn-sim [`ndn_sim::LinkConfig`] plus its human name.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkProfileConfig {
    /// Profile name (`"apsta"`, `"ndr-good"`, `"ndr-contested"`).
    pub name: String,
    pub delay_ms: f64,
    pub jitter_ms: f64,
    pub loss_rate: f64,
    pub bandwidth_bps: u64,
}

impl LinkProfileConfig {
    pub fn new(name: &str, link: &ndn_sim::LinkConfig) -> Self {
        Self {
            name: name.to_string(),
            delay_ms: link.delay.as_secs_f64() * 1000.0,
            jitter_ms: link.jitter.as_secs_f64() * 1000.0,
            loss_rate: link.loss_rate,
            bandwidth_bps: link.bandwidth_bps,
        }
    }

    pub fn link(&self) -> ndn_sim::LinkConfig {
        ndn_sim::LinkConfig {
            delay: Duration::from_secs_f64(self.delay_ms / 1000.0),
            jitter: Duration::from_secs_f64(self.jitter_ms / 1000.0),
            loss_rate: self.loss_rate,
            bandwidth_bps: self.bandwidth_bps,
        }
    }
}

/// The ArduPilot SITL parameters of the run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SitlRunConfig {
    /// SITL binary path.
    pub binary: String,
    /// Frame model (`"+"`).
    pub model: String,
    /// Defaults parameter file.
    pub defaults: String,
    /// Simulation speedup (1 = real time).
    pub speedup: f64,
    /// Per-vehicle home `lat,lon,alt_m,yaw_deg` strings, vehicle order.
    pub homes: Vec<String>,
    /// `git rev-parse HEAD` of the ardupilot checkout, when available.
    pub ardupilot_rev: String,
}

/// `git rev-parse HEAD` of `repo_dir`, or `"unknown"`.
pub fn git_rev(repo_dir: &std::path::Path) -> String {
    std::process::Command::new("git")
        .arg("-C")
        .arg(repo_dir)
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|out| out.status.success())
        .and_then(|out| String::from_utf8(out.stdout).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_config_round_trips_and_projects_agent_knobs() {
        let mut agent = muas_agent::AgentConfig {
            vehicle_id: "wuas-01".into(),
            fleet_ids: vec!["wuas-01".into(), "iuas-01".into()],
            endpoint: muas_agent::Endpoint::Mavlink("tcp:127.0.0.1:5760".into()),
            ..muas_agent::AgentConfig::default()
        };
        agent.extras = vec!["raster".into(), "camera".into()];
        agent.grace_s = 3.0;

        let vehicle = VehicleRunConfig::from_agent(&agent);
        assert_eq!(vehicle.endpoint, "mavlink:tcp:127.0.0.1:5760");
        assert_eq!(vehicle.carrier, "rpc");
        assert_eq!(vehicle.grace_s, 3.0);
        assert_eq!(vehicle.floor_agl_m, agent.floor_agl_m);

        let link = ndn_sim::LinkConfig {
            delay: Duration::from_millis(2),
            jitter: Duration::from_micros(500),
            loss_rate: 0.001,
            bandwidth_bps: 20_000_000,
        };
        let config = RunConfig {
            run_id: "run-1752000000".into(),
            created_ns: 1,
            vehicles: vec![vehicle],
            link_profile: LinkProfileConfig::new("apsta", &link),
            sitl: SitlRunConfig {
                binary: "/x/arducopter".into(),
                model: "+".into(),
                defaults: "/x/copter.parm".into(),
                speedup: 1.0,
                homes: vec!["35.36,-149.16,584,0".into()],
                ardupilot_rev: "unknown".into(),
            },
            stack_revs: [("minimuas".to_string(), "abc".to_string())].into(),
        };
        let json = serde_json::to_string(&config).unwrap();
        let back: RunConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back, config);
        assert_eq!(back.link_profile.link().loss_rate, 0.001);
    }
}
