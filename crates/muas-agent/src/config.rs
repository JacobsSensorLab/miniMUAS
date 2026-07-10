//! Agent configuration + a dependency-free CLI parser mirroring the v2
//! `run_drone_agent.py` flags where they still make sense in v3.

use std::net::SocketAddr;
use std::path::PathBuf;

use muas_contracts::policy::{AglBounds, DEFAULT_MAX_RANGE_M};

/// Which flight backend flies the vehicle.
#[derive(Debug, Clone, PartialEq)]
pub enum Endpoint {
    /// Kinematic bench vehicle at `(lat_deg, lon_deg)`.
    Sim { lat_deg: f64, lon_deg: f64 },
    /// Live autopilot over a pymavlink-style endpoint string
    /// (e.g. `udpin:0.0.0.0:14550`, `tcp:127.0.0.1:5760`).
    Mavlink(String),
}

/// What the vehicle does once a task completes and nothing else claims it
/// (`--idle-policy`, ROUND-3 §1 "post-task idle"). The decision is journaled
/// (`idle.policy`) whichever branch runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IdlePolicy {
    /// Hover where the task ended (the pre-round-3 behavior; default).
    Hold,
    /// Hover, then RTL (smart when a fleet is configured) after this many
    /// seconds still idle. A new task or an operator command cancels it.
    RtlAfter(f64),
    /// Climb to this vehicle's smart-RTL altitude slot and hold there —
    /// layered hovers, so idle vehicles never share an altitude. Falls back
    /// to `Hold` when no fleet slot exists.
    SlotHold,
}

impl IdlePolicy {
    /// Parse the `--idle-policy` flag value.
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "hold" => Ok(Self::Hold),
            "slot-hold" => Ok(Self::SlotHold),
            other => match other.strip_prefix("rtl-after:") {
                Some(seconds) => match seconds.parse::<f64>() {
                    Ok(s) if s > 0.0 => Ok(Self::RtlAfter(s)),
                    _ => Err(format!("--idle-policy: bad rtl-after seconds '{seconds}'")),
                },
                None => Err(format!(
                    "--idle-policy: expected hold|rtl-after:<s>|slot-hold, got '{value}'"
                )),
            },
        }
    }

    /// The journaled policy name.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Hold => "hold",
            Self::RtlAfter(_) => "rtl-after",
            Self::SlotHold => "slot-hold",
        }
    }
}

/// Service carrier hosting the vehicle service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CarrierKind {
    /// `FaceRpcCarrier` over the engine (default).
    Rpc,
    /// `NdnsfCarrier` four-phase over an SVS group (comparison lane).
    Ndnsf,
}

/// One point-to-point UDP face: bind `local`, exchange with `remote`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdpLink {
    pub local: SocketAddr,
    pub remote: SocketAddr,
    /// FIB prefix routed out this face (`None` = inbound-only link, e.g. a
    /// GCS/client that fetches from us).
    pub route: Option<String>,
}

/// Everything `Agent::start` needs. Field defaults mirror the v2 agent flags.
#[derive(Debug, Clone)]
pub struct AgentConfig {
    pub vehicle_id: String,
    /// Full fleet id list INCLUDING this vehicle (v2 `--fleet-ids`); empty =
    /// no coordination, native RTL.
    pub fleet_ids: Vec<String>,
    pub endpoint: Endpoint,
    /// Telemetry publish rate, Hz (v2 published at 4 Hz).
    pub telemetry_hz: f64,
    /// Capability extras advertised on `telemetry/state` (v2
    /// CapabilityProfile extras: `"orbit"`, `"camera"`, `"audio"`, ...).
    pub extras: Vec<String>,
    /// Deployment run id stamped onto every journal line (associates all
    /// journal output with the deployment's `run-config` record).
    pub run_id: Option<String>,
    pub carrier: CarrierKind,
    /// Point-to-point UDP faces toward peers / forwarders / clients.
    pub links: Vec<UdpLink>,

    // -- field-safety rails (muas-contracts policy) --
    pub max_range_m: f64,
    pub agl_bounds: AglBounds,
    pub audio_range_m: f64,

    // -- coordination knobs (PeerGuard) --
    pub hsep_m: f64,
    pub vsep_m: f64,
    pub horizon_s: f64,
    pub grace_s: f64,
    /// Fleet flight floor — must be identical fleet-wide.
    pub floor_agl_m: f64,

    // -- smart RTL knobs --
    pub rtl_base_agl_m: f64,
    pub rtl_sep_m: f64,

    // -- post-task idle policy (`--idle-policy`) --
    pub idle_policy: IdlePolicy,

    // -- journals --
    pub log_dir: Option<PathBuf>,
    /// Mirror journal events into an ndf-apps Block chain.
    pub journal_chain: bool,

    // -- sensor feed (pluggable; see muas_agent::sensor) --
    /// Which sensor feed backs captures/video (`None` = no sensors, the
    /// pre-v3.1 behavior; `Synthetic` renders from the deployment's anomaly
    /// ground truth fetched over the network).
    pub sensor_feed: crate::sensor::SensorFeedConfig,

    // -- spark telemetry lane --
    /// Emit every telemetry sample as an ndf-spark Spark over UDP to this
    /// destination (the real_socket_twin binding).
    pub spark_udp: Option<SocketAddr>,

    // -- ndnsf carrier transport (only read when carrier == Ndnsf) --
    pub ndnsf_listen: Option<SocketAddr>,
    pub ndnsf_peers: Vec<SocketAddr>,
}

impl Default for AgentConfig {
    fn default() -> Self {
        Self {
            vehicle_id: String::new(),
            fleet_ids: Vec::new(),
            endpoint: Endpoint::Sim {
                lat_deg: 35.0,
                lon_deg: -90.0,
            },
            telemetry_hz: 4.0,
            extras: Vec::new(),
            run_id: None,
            carrier: CarrierKind::Rpc,
            links: Vec::new(),
            max_range_m: DEFAULT_MAX_RANGE_M,
            agl_bounds: AglBounds::default(),
            audio_range_m: 30.0,
            hsep_m: 8.0,
            vsep_m: 4.0,
            horizon_s: 20.0,
            grace_s: 2.5,
            floor_agl_m: 3.5,
            rtl_base_agl_m: 8.0,
            rtl_sep_m: 3.0,
            idle_policy: IdlePolicy::Hold,
            log_dir: None,
            journal_chain: false,
            sensor_feed: crate::sensor::SensorFeedConfig::None,
            spark_udp: None,
            ndnsf_listen: None,
            ndnsf_peers: Vec::new(),
        }
    }
}

impl AgentConfig {
    /// Fleet peers = fleet ids minus this vehicle.
    pub fn peer_ids(&self) -> Vec<String> {
        self.fleet_ids
            .iter()
            .filter(|id| *id != &self.vehicle_id)
            .cloned()
            .collect()
    }

    /// Smart RTL requires this vehicle to hold a slot in the fleet table.
    pub fn smart_rtl_available(&self) -> bool {
        self.fleet_ids.iter().any(|id| id == &self.vehicle_id)
    }
}

/// `--help` text (also the flag reference).
pub const HELP: &str = "\
muas-agent — miniMUAS v3 drone agent

USAGE:
    muas-agent --vehicle-id <ID> [FLAGS]

VEHICLE:
    --vehicle-id <ID>          this vehicle's id (required), e.g. iuas-01
    --fleet-ids <A,B,..>       full fleet id list incl. self; enables PeerGuard
                               coordination + slot-layered smart RTL
    --endpoint <E>             sim (default) | mavlink:<endpoint>
                               e.g. mavlink:udpin:0.0.0.0:14550
    --sim-origin <LAT,LON>     sim start position (default 35.0,-90.0)
    --telemetry-hz <HZ>        telemetry/live publish rate (default 4)
    --extras <A,B,..>          capability extras advertised on telemetry/state
                               (v2 CapabilityProfile: orbit, camera, audio)
    --run-id <ID>              deployment run id stamped onto every journal line

NETWORK (point-to-point UDP faces; repeatable):
    --peer <VID=LOCAL,REMOTE>  face to fleet peer VID; routes /muas/v3/<VID>
    --forwarder <LOCAL,REMOTE> face to an upstream forwarder; routes /muas/v3
    --link <LOCAL,REMOTE>      inbound-only face (GCS / test client)

SERVICE CARRIER:
    --carrier <rpc|ndnsf>      rpc = FaceRpcCarrier over the engine (default)
                               ndnsf = four-phase NdnsfCarrier over SVS
    --ndnsf-listen <ADDR>      ndnsf sync-lane UDP bind (carrier=ndnsf)
    --ndnsf-peer <ADDR>        ndnsf sync-lane peer (repeatable)

FIELD-SAFETY RAILS:
    --max-range-m <M>          range guard from home (default 300)
    --min-agl-m <M>            AGL floor (default 3.5; sim benches may use 0.5)
    --max-agl-m <M>            AGL ceiling (default 20)
    --audio-range-m <M>        audio capture range guard (default 30)

COORDINATION:
    --hsep-m <M>               horizontal separation envelope (default 8)
    --vsep-m <M>               vertical separation envelope (default 4)
    --horizon-s <S>            conflict prediction horizon (default 20)
    --grace-s <S>              cooperative confirmation grace (default 2.5)
    --floor-agl-m <M>          fleet flight floor (default 3.5; SAME fleet-wide)
    --rtl-base-agl-m <M>       lowest smart-RTL slot (default 8)
    --rtl-sep-m <M>            slot separation (default 3)
    --idle-policy <P>          post-task idle behavior (journaled decision):
                               hold (default) | rtl-after:<s> | slot-hold

SENSORS:
    --sensor-feed <S>          none (default) | synthetic — synthetic renders
                               nadir frames / audio from the deployment's
                               anomaly ground truth, fetched over the network
                               (embedders pass a full SensorFeedConfig instead)

JOURNALS / TELEMETRY LANES:
    --log-dir <DIR>            power-loss-safe JSONL journal directory
    --journal-chain            also mirror journal events into an ndf Block chain
    --spark-udp <ADDR>         emit telemetry samples as ndf-spark Sparks to ADDR

    --help                     print this help
";

/// Parse outcome: run, or print help.
#[derive(Debug)]
pub enum ParseOutcome {
    Run(Box<AgentConfig>),
    Help,
}

fn parse_addr(s: &str, flag: &str) -> Result<SocketAddr, String> {
    s.parse()
        .map_err(|e| format!("{flag}: bad socket address '{s}': {e}"))
}

fn parse_f64(s: &str, flag: &str) -> Result<f64, String> {
    s.parse()
        .map_err(|e| format!("{flag}: bad number '{s}': {e}"))
}

fn split_pair<'a>(s: &'a str, sep: char, flag: &str) -> Result<(&'a str, &'a str), String> {
    s.split_once(sep)
        .ok_or_else(|| format!("{flag}: expected '<..>{sep}<..>', got '{s}'"))
}

/// Parse CLI args (without the program name).
pub fn parse_args(args: &[String]) -> Result<ParseOutcome, String> {
    let mut config = AgentConfig::default();
    let mut it = args.iter();
    let next = |flag: &str, it: &mut std::slice::Iter<'_, String>| -> Result<String, String> {
        it.next()
            .cloned()
            .ok_or_else(|| format!("{flag}: missing value"))
    };
    let mut sim_origin: Option<(f64, f64)> = None;
    let mut min_agl: Option<f64> = None;
    let mut max_agl: Option<f64> = None;
    let mut floor: Option<f64> = None;

    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(ParseOutcome::Help),
            "--vehicle-id" => config.vehicle_id = next(arg, &mut it)?,
            "--fleet-ids" => {
                config.fleet_ids = next(arg, &mut it)?
                    .split(',')
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect();
            }
            "--endpoint" => {
                let value = next(arg, &mut it)?;
                config.endpoint = match value.as_str() {
                    "sim" => Endpoint::Sim {
                        lat_deg: 35.0,
                        lon_deg: -90.0,
                    },
                    other => match other.strip_prefix("mavlink:") {
                        Some(ep) if !ep.is_empty() => Endpoint::Mavlink(ep.to_string()),
                        _ => {
                            return Err(format!(
                                "--endpoint: expected 'sim' or 'mavlink:<endpoint>', got '{value}'"
                            ))
                        }
                    },
                };
            }
            "--sim-origin" => {
                let value = next(arg, &mut it)?;
                let (lat, lon) = split_pair(&value, ',', arg)?;
                sim_origin = Some((parse_f64(lat, arg)?, parse_f64(lon, arg)?));
            }
            "--telemetry-hz" => config.telemetry_hz = parse_f64(&next(arg, &mut it)?, arg)?,
            "--extras" => {
                config.extras = next(arg, &mut it)?
                    .split(',')
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect();
            }
            "--run-id" => config.run_id = Some(next(arg, &mut it)?),
            "--carrier" => {
                config.carrier = match next(arg, &mut it)?.as_str() {
                    "rpc" => CarrierKind::Rpc,
                    "ndnsf" => CarrierKind::Ndnsf,
                    other => return Err(format!("--carrier: expected rpc|ndnsf, got '{other}'")),
                };
            }
            "--peer" => {
                let value = next(arg, &mut it)?;
                let (vid, addrs) = split_pair(&value, '=', arg)?;
                let (local, remote) = split_pair(addrs, ',', arg)?;
                config.links.push(UdpLink {
                    local: parse_addr(local, arg)?,
                    remote: parse_addr(remote, arg)?,
                    route: Some(muas_contracts::names::vehicle_prefix(vid)),
                });
            }
            "--forwarder" => {
                let value = next(arg, &mut it)?;
                let (local, remote) = split_pair(&value, ',', arg)?;
                config.links.push(UdpLink {
                    local: parse_addr(local, arg)?,
                    remote: parse_addr(remote, arg)?,
                    route: Some(muas_contracts::names::APP_PREFIX.to_string()),
                });
            }
            "--link" => {
                let value = next(arg, &mut it)?;
                let (local, remote) = split_pair(&value, ',', arg)?;
                config.links.push(UdpLink {
                    local: parse_addr(local, arg)?,
                    remote: parse_addr(remote, arg)?,
                    route: None,
                });
            }
            "--ndnsf-listen" => config.ndnsf_listen = Some(parse_addr(&next(arg, &mut it)?, arg)?),
            "--ndnsf-peer" => config.ndnsf_peers.push(parse_addr(&next(arg, &mut it)?, arg)?),
            "--max-range-m" => config.max_range_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--min-agl-m" => min_agl = Some(parse_f64(&next(arg, &mut it)?, arg)?),
            "--max-agl-m" => max_agl = Some(parse_f64(&next(arg, &mut it)?, arg)?),
            "--audio-range-m" => config.audio_range_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--hsep-m" => config.hsep_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--vsep-m" => config.vsep_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--horizon-s" => config.horizon_s = parse_f64(&next(arg, &mut it)?, arg)?,
            "--grace-s" => config.grace_s = parse_f64(&next(arg, &mut it)?, arg)?,
            "--floor-agl-m" => floor = Some(parse_f64(&next(arg, &mut it)?, arg)?),
            "--rtl-base-agl-m" => config.rtl_base_agl_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--rtl-sep-m" => config.rtl_sep_m = parse_f64(&next(arg, &mut it)?, arg)?,
            "--idle-policy" => config.idle_policy = IdlePolicy::parse(&next(arg, &mut it)?)?,
            "--sensor-feed" => {
                config.sensor_feed = match next(arg, &mut it)?.as_str() {
                    "none" => crate::sensor::SensorFeedConfig::None,
                    "synthetic" => crate::sensor::SensorFeedConfig::synthetic(),
                    other => {
                        return Err(format!("--sensor-feed: expected none|synthetic, got '{other}'"))
                    }
                };
            }
            "--log-dir" => config.log_dir = Some(PathBuf::from(next(arg, &mut it)?)),
            "--journal-chain" => config.journal_chain = true,
            "--spark-udp" => config.spark_udp = Some(parse_addr(&next(arg, &mut it)?, arg)?),
            other => return Err(format!("unknown flag '{other}' (see --help)")),
        }
    }

    if config.vehicle_id.is_empty() {
        return Err("--vehicle-id is required (see --help)".to_string());
    }
    if let (Endpoint::Sim { lat_deg, lon_deg }, Some((lat, lon))) =
        (&mut config.endpoint, sim_origin)
    {
        *lat_deg = lat;
        *lon_deg = lon;
    }
    if let Some(min) = min_agl {
        config.agl_bounds.min_agl_m = min;
    }
    if let Some(max) = max_agl {
        config.agl_bounds.max_agl_m = max;
    }
    // Fleet floor defaults to the commandable AGL floor, exactly the v2
    // plumbing (floor_agl_m = min_agl); an explicit flag overrides.
    config.floor_agl_m = floor.unwrap_or(config.agl_bounds.min_agl_m.max(3.5));
    if config.carrier == CarrierKind::Ndnsf && config.ndnsf_listen.is_none() {
        return Err("--carrier ndnsf requires --ndnsf-listen (see --help)".to_string());
    }
    Ok(ParseOutcome::Run(Box::new(config)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn help_short_circuits() {
        assert!(matches!(
            parse_args(&args(&["--help"])).unwrap(),
            ParseOutcome::Help
        ));
    }

    #[test]
    fn vehicle_id_is_required() {
        assert!(parse_args(&args(&["--telemetry-hz", "2"])).is_err());
    }

    #[test]
    fn full_flag_set_parses() {
        let out = parse_args(&args(&[
            "--vehicle-id",
            "iuas-01",
            "--fleet-ids",
            "iuas-01,wuas-01",
            "--endpoint",
            "mavlink:udpin:0.0.0.0:14550",
            "--telemetry-hz",
            "2",
            "--carrier",
            "rpc",
            "--peer",
            "wuas-01=127.0.0.1:47001,127.0.0.1:47101",
            "--link",
            "127.0.0.1:47002,127.0.0.1:47201",
            "--max-range-m",
            "250",
            "--min-agl-m",
            "3.5",
            "--max-agl-m",
            "18",
            "--grace-s",
            "3.0",
            "--rtl-base-agl-m",
            "10",
            "--log-dir",
            "/tmp/muas-log",
            "--journal-chain",
            "--spark-udp",
            "127.0.0.1:48000",
        ]))
        .unwrap();
        let ParseOutcome::Run(config) = out else {
            panic!("expected Run");
        };
        assert_eq!(config.vehicle_id, "iuas-01");
        assert_eq!(config.peer_ids(), vec!["wuas-01".to_string()]);
        assert!(config.smart_rtl_available());
        assert_eq!(config.endpoint, Endpoint::Mavlink("udpin:0.0.0.0:14550".into()));
        assert_eq!(config.links.len(), 2);
        assert_eq!(
            config.links[0].route.as_deref(),
            Some("/muas/v3/wuas-01"),
            "--peer routes the peer's vehicle prefix"
        );
        assert_eq!(config.links[1].route, None, "--link is inbound-only");
        assert_eq!(config.max_range_m, 250.0);
        assert_eq!(config.agl_bounds.max_agl_m, 18.0);
        assert_eq!(config.grace_s, 3.0);
        assert_eq!(config.floor_agl_m, 3.5);
        assert!(config.journal_chain);
        assert!(config.spark_udp.is_some());
    }

    #[test]
    fn sim_origin_applies_to_sim_endpoint() {
        let ParseOutcome::Run(config) = parse_args(&args(&[
            "--vehicle-id",
            "wuas-01",
            "--sim-origin",
            "35.5,-90.25",
        ]))
        .unwrap() else {
            panic!("expected Run");
        };
        assert_eq!(
            config.endpoint,
            Endpoint::Sim {
                lat_deg: 35.5,
                lon_deg: -90.25
            }
        );
    }

    #[test]
    fn idle_policy_parses_all_three_shapes() {
        assert_eq!(IdlePolicy::parse("hold"), Ok(IdlePolicy::Hold));
        assert_eq!(IdlePolicy::parse("slot-hold"), Ok(IdlePolicy::SlotHold));
        assert_eq!(IdlePolicy::parse("rtl-after:45"), Ok(IdlePolicy::RtlAfter(45.0)));
        assert!(IdlePolicy::parse("rtl-after:0").is_err());
        assert!(IdlePolicy::parse("rtl-after:soon").is_err());
        assert!(IdlePolicy::parse("wander").is_err());

        let ParseOutcome::Run(config) = parse_args(&args(&[
            "--vehicle-id",
            "iuas-01",
            "--idle-policy",
            "rtl-after:30",
        ]))
        .unwrap() else {
            panic!("expected Run");
        };
        assert_eq!(config.idle_policy, IdlePolicy::RtlAfter(30.0));
        // Default stays the pre-round-3 behavior.
        assert_eq!(AgentConfig::default().idle_policy, IdlePolicy::Hold);
    }

    #[test]
    fn ndnsf_carrier_requires_listen_addr() {
        let err = parse_args(&args(&["--vehicle-id", "a", "--carrier", "ndnsf"])).unwrap_err();
        assert!(err.contains("--ndnsf-listen"), "{err}");
    }
}
