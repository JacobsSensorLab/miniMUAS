//! Dashboard configuration + dependency-free CLI parser mirroring the v2
//! `run_dashboard.py` flags (plus the muas-agent UDP-face flags for the
//! engine's point-to-point links).

use std::net::SocketAddr;
use std::path::PathBuf;

/// Which RC carriage the Pilot surface publishes over (transport correction
/// 2026-07-11). Default is [`Spark`](Self::Spark): ndf-spark carried over the
/// engine (SP-3 replay refusal, merkle windows, checkpoint Blocks). [`Data`]
/// (frame-as-latest-wins-Data) is the demoted comparison bearer behind
/// `--rc-data` — it still crosses the fabric, but as discrete request/response
/// Data without the Spark stream properties.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RcCarriage {
    /// ndf-spark over the engine (default): `/muas/v3/<vid>/rc/spark/<index>`.
    #[default]
    Spark,
    /// COMPARISON bearer: frame-as-Data over the engine
    /// (`/muas/v3/<vid>/rc/frame`, latest-wins).
    Data,
}

/// One point-to-point UDP face: bind `local`, exchange with `remote`
/// (same shape as muas-agent's link config).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdpLink {
    pub local: SocketAddr,
    pub remote: SocketAddr,
    /// FIB prefix routed out this face (`None` = inbound-only).
    pub route: Option<String>,
}

/// Everything the dashboard needs. Field defaults mirror the v2 flags.
#[derive(Debug, Clone)]
pub struct DashConfig {
    pub http_host: String,
    pub http_port: u16,
    pub wuas_id: String,
    /// IUAS ids in dispatch-preference order (v2 `--iuas-ids`, defaulting
    /// to the single `--iuas-id`).
    pub iuas_ids: Vec<String>,
    pub confirm_count: u32,
    pub detect_timeout_ms: u64,
    pub search_margin_s: f64,
    pub investigate_timeout_ms: u64,
    /// Local satellite tile cache served at `/tiles/{z}/{x}/{y}`.
    pub tiles_dir: PathBuf,
    /// Upstream tile URL template (`{z}/{x}/{y}` placeholders); empty
    /// disables proxying (pure offline).
    pub tile_upstream: String,
    /// Mission recorder directory; `None` disables recording.
    pub record_dir: Option<PathBuf>,
    /// Run label recordings group under (`<run>-<mission>-<t>.jsonl`);
    /// empty = auto (`run-<dashboard start time>`).
    pub run_name: String,
    /// Point-to-point UDP faces toward vehicles / forwarders.
    pub links: Vec<UdpLink>,
    /// Surveyed GCS antenna position `(lat_deg, lon_deg)` (`--gcs`); rides
    /// the hello message as `gcs:{lat,lon,source:"manual"}` so the map's
    /// network layer anchors the GCS node to ground truth instead of the
    /// first-telemetry-fix heuristic. Field path: manual survey entry; a
    /// pluggable positioning backend is a later increment.
    pub gcs: Option<(f64, f64)>,
    /// RC-CONTROL R2 pilot-surface targets (`--rc-vehicle`): the vehicle ids
    /// the Pilot surface can drive. Targeting is by NAME — each vehicle's
    /// frames are published as latest-wins Data on `/muas/v3/<vid>/rc` over
    /// the dashboard engine and fetched by the agent over the fabric (no side
    /// socket; see `docs/v3/RC-CONTROL.md` "Transport correction"). Empty =
    /// no vehicle is RC-reachable and the Pilot surface stays inert.
    pub rc_vehicles: Vec<String>,
    /// Which carriage the Pilot surface publishes RC over (`--rc-data`
    /// demotes to the frame-as-Data comparison bearer; default = engine-Spark).
    pub rc_carriage: RcCarriage,
    /// Active dispatch/requester strategy source (ROUND-3 §2). `None` =
    /// crate defaults = today's idle-first/config-order behavior; the
    /// dispatcher and requeue backoff read the folded records otherwise.
    pub strategy: Option<muas_contracts::strategy::StrategySource>,
}

impl Default for DashConfig {
    fn default() -> Self {
        Self {
            http_host: "0.0.0.0".into(),
            http_port: 8080,
            wuas_id: "wuas-01".into(),
            iuas_ids: vec!["iuas-01".into()],
            confirm_count: 2,
            detect_timeout_ms: 30_000,
            search_margin_s: 60.0,
            investigate_timeout_ms: 120_000,
            tiles_dir: PathBuf::from("/var/lib/minimuas/tiles"),
            tile_upstream: "https://server.arcgisonline.com/ArcGIS/rest/services/\
                            World_Imagery/MapServer/tile/{z}/{y}/{x}"
                .into(),
            record_dir: Some(PathBuf::from("/var/lib/minimuas/replays")),
            run_name: String::new(),
            links: Vec::new(),
            gcs: None,
            rc_vehicles: Vec::new(),
            rc_carriage: RcCarriage::default(),
            strategy: None,
        }
    }
}

impl DashConfig {
    /// WUAS then IUAS — the wire vehicle ordering (binary video frames are
    /// prefixed with this index).
    pub fn vehicles(&self) -> Vec<String> {
        let mut v = vec![self.wuas_id.clone()];
        v.extend(self.iuas_ids.iter().cloned());
        v
    }
}

/// `--help` text (also the flag reference).
pub const HELP: &str = "\
muas-dashboard — miniMUAS v3 GCS mission console

USAGE:
    muas-dashboard [FLAGS]

WEB:
    --http-host <H>            bind address (default 0.0.0.0)
    --http-port <P>            bind port (default 8080)

FLEET:
    --wuas-id <ID>             searcher vehicle id (default wuas-01)
    --iuas-id <ID>             single inspector id (default iuas-01)
    --iuas-ids <A,B,..>        inspector ids; targets dispatch per requested
                               sensor to whichever idle enabled IUAS
                               advertises it (overrides --iuas-id)

NETWORK (point-to-point UDP faces; repeatable):
    --vehicle <VID=LOCAL,REMOTE>  face to vehicle VID; routes /muas/v3/<VID>
    --forwarder <LOCAL,REMOTE>    face to an upstream forwarder; routes /muas/v3
    --link <LOCAL,REMOTE>         inbound-only face

RC PILOT SURFACE (RC-CONTROL R2; repeatable):
    --rc-vehicle <VID>            a vehicle the Pilot surface may drive. RC
                                  frames are published as named data on
                                  /muas/v3/<VID>/rc over the dashboard engine
                                  and fetched by that vehicle's agent over the
                                  fabric — no side socket. The agent must run
                                  with --rc (its named-data receiver).
    --rc-vehicles <A,B,..>        the same, several at once (comma-separated)
    --rc-data                     DEMOTE the Pilot surface to the frame-as-Data
                                  comparison carriage (/muas/v3/<VID>/rc/frame);
                                  default is ndf-spark over the engine
                                  (/muas/v3/<VID>/rc/spark/<index>)

MISSION:
    --confirm-count <N>        detections (within target_separation_m) needed
                               before a candidate becomes a dispatched target
                               (default 2 — the false-positive guard)
    --detect-timeout-ms <MS>   detection request timeout (default 30000)
    --search-margin-s <S>      raster deadline margin over max_duration_s
                               (default 60)
    --investigate-timeout-ms <MS>  investigation timeout (default 120000)

GCS POSITION:
    --gcs <LAT,LON>            surveyed GCS antenna position (decimal
                               degrees, manual survey entry). Advertised to
                               the UI (hello gcs:{lat,lon,source:\"manual\"})
                               so the map's network layer anchors the GCS
                               node to ground truth instead of inferring it
                               from the first telemetry fix. A pluggable
                               positioning-backend source is a later
                               increment.

MAP / RECORDER:
    --tiles-dir <DIR>          local tile cache served at /tiles/{z}/{x}/{y}
                               (default /var/lib/minimuas/tiles)
    --tile-upstream <URL>      upstream tile template with {z}/{x}/{y};
                               empty string disables proxying (pure offline)
    --record-dir <DIR>         mission recorder JSONL directory (default
                               /var/lib/minimuas/replays); empty disables
    --run-name <NAME>          run label recordings group under, producing
                               <run>-<mission>-<t>.jsonl session files
                               (default: run-<dashboard start time>)

    --help                     print this help
";

/// Parse outcome: run, or print help.
#[derive(Debug)]
pub enum ParseOutcome {
    Run(Box<DashConfig>),
    Help,
}

fn parse_addr(s: &str, flag: &str) -> Result<SocketAddr, String> {
    s.parse().map_err(|e| format!("{flag}: bad socket address '{s}': {e}"))
}

fn split_pair<'a>(s: &'a str, sep: char, flag: &str) -> Result<(&'a str, &'a str), String> {
    s.split_once(sep)
        .ok_or_else(|| format!("{flag}: expected '<..>{sep}<..>', got '{s}'"))
}

/// Parse one RC-reachable vehicle id (`--rc-vehicle <VID>`) into the roster.
fn parse_rc_vehicle(spec: &str, flag: &str, out: &mut Vec<String>) -> Result<(), String> {
    let vid = spec.trim();
    if vid.is_empty() {
        return Err(format!("{flag}: empty vehicle id"));
    }
    if !out.iter().any(|v| v == vid) {
        out.push(vid.to_string());
    }
    Ok(())
}

/// Parse CLI args (without the program name).
pub fn parse_args(args: &[String]) -> Result<ParseOutcome, String> {
    let mut config = DashConfig::default();
    let mut iuas_id: Option<String> = None;
    let mut iuas_ids: Option<Vec<String>> = None;
    let mut it = args.iter();
    let next = |flag: &str, it: &mut std::slice::Iter<'_, String>| -> Result<String, String> {
        it.next().cloned().ok_or_else(|| format!("{flag}: missing value"))
    };

    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(ParseOutcome::Help),
            "--http-host" => config.http_host = next(arg, &mut it)?,
            "--http-port" => {
                config.http_port = next(arg, &mut it)?
                    .parse()
                    .map_err(|e| format!("{arg}: bad port: {e}"))?;
            }
            "--wuas-id" => config.wuas_id = next(arg, &mut it)?,
            "--iuas-id" => iuas_id = Some(next(arg, &mut it)?),
            "--iuas-ids" => {
                iuas_ids = Some(
                    next(arg, &mut it)?
                        .split(',')
                        .map(str::trim)
                        .filter(|s| !s.is_empty())
                        .map(str::to_string)
                        .collect(),
                );
            }
            "--confirm-count" => {
                config.confirm_count = next(arg, &mut it)?
                    .parse()
                    .map_err(|e| format!("{arg}: bad count: {e}"))?;
            }
            "--detect-timeout-ms" => {
                config.detect_timeout_ms = next(arg, &mut it)?
                    .parse()
                    .map_err(|e| format!("{arg}: bad timeout: {e}"))?;
            }
            "--search-margin-s" => {
                config.search_margin_s = next(arg, &mut it)?
                    .parse()
                    .map_err(|e| format!("{arg}: bad margin: {e}"))?;
            }
            "--investigate-timeout-ms" => {
                config.investigate_timeout_ms = next(arg, &mut it)?
                    .parse()
                    .map_err(|e| format!("{arg}: bad timeout: {e}"))?;
            }
            "--tiles-dir" => config.tiles_dir = PathBuf::from(next(arg, &mut it)?),
            "--tile-upstream" => config.tile_upstream = next(arg, &mut it)?,
            "--record-dir" => {
                let dir = next(arg, &mut it)?;
                config.record_dir = if dir.is_empty() { None } else { Some(PathBuf::from(dir)) };
            }
            "--run-name" => config.run_name = next(arg, &mut it)?,
            "--gcs" => {
                let value = next(arg, &mut it)?;
                let (lat, lon) = split_pair(&value, ',', arg)?;
                let parse = |s: &str| -> Result<f64, String> {
                    s.trim()
                        .parse()
                        .map_err(|e| format!("{arg}: bad coordinate '{s}': {e}"))
                };
                config.gcs = Some((parse(lat)?, parse(lon)?));
            }
            "--vehicle" => {
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
            "--rc-vehicle" => {
                parse_rc_vehicle(&next(arg, &mut it)?, arg, &mut config.rc_vehicles)?;
            }
            "--rc-vehicles" => {
                for spec in next(arg, &mut it)?.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                    parse_rc_vehicle(spec, arg, &mut config.rc_vehicles)?;
                }
            }
            "--rc-data" => config.rc_carriage = RcCarriage::Data,
            "--link" => {
                let value = next(arg, &mut it)?;
                let (local, remote) = split_pair(&value, ',', arg)?;
                config.links.push(UdpLink {
                    local: parse_addr(local, arg)?,
                    remote: parse_addr(remote, arg)?,
                    route: None,
                });
            }
            other => return Err(format!("unknown flag '{other}' (see --help)")),
        }
    }
    config.iuas_ids = match (iuas_ids, iuas_id) {
        (Some(ids), _) if !ids.is_empty() => ids,
        (_, Some(id)) => vec![id],
        _ => config.iuas_ids,
    };
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
        assert!(matches!(parse_args(&args(&["--help"])).unwrap(), ParseOutcome::Help));
    }

    #[test]
    fn defaults_match_v2() {
        let ParseOutcome::Run(c) = parse_args(&[]).unwrap() else { panic!("run") };
        assert_eq!(c.http_port, 8080);
        assert_eq!(c.vehicles(), vec!["wuas-01".to_string(), "iuas-01".to_string()]);
        assert_eq!(c.confirm_count, 2);
        assert!(c.tile_upstream.contains("{z}"));
    }

    #[test]
    fn gcs_flag_parses_and_defaults_off() {
        let ParseOutcome::Run(c) = parse_args(&[]).unwrap() else { panic!("run") };
        assert_eq!(c.gcs, None, "no flag: first-fix/NET fallback chain applies");

        let ParseOutcome::Run(c) =
            parse_args(&args(&["--gcs", "-35.3635, 149.1652"])).unwrap()
        else {
            panic!("run")
        };
        assert_eq!(c.gcs, Some((-35.3635, 149.1652)));

        assert!(parse_args(&args(&["--gcs", "nope"])).is_err());
        assert!(parse_args(&args(&["--gcs", "12.0,east"])).is_err());
    }

    #[test]
    fn iuas_ids_override_and_links_parse() {
        let ParseOutcome::Run(c) = parse_args(&args(&[
            "--iuas-ids",
            "iuas-01, iuas-02",
            "--vehicle",
            "wuas-01=127.0.0.1:47001,127.0.0.1:47101",
            "--record-dir",
            "",
        ]))
        .unwrap() else {
            panic!("run")
        };
        assert_eq!(c.iuas_ids, vec!["iuas-01".to_string(), "iuas-02".to_string()]);
        assert_eq!(c.links[0].route.as_deref(), Some("/muas/v3/wuas-01"));
        assert!(c.record_dir.is_none(), "empty --record-dir disables recording");
    }

    #[test]
    fn rc_vehicles_parse_repeatable_and_deduped() {
        let ParseOutcome::Run(c) = parse_args(&args(&[
            "--rc-vehicle",
            "iuas-02",
            "--rc-vehicle",
            "iuas-03",
            "--rc-vehicles",
            "wuas-01, iuas-04, iuas-02",
        ]))
        .unwrap() else {
            panic!("run")
        };
        // Targeting is by NAME now; iuas-02 repeated is deduped.
        assert_eq!(c.rc_vehicles, vec!["iuas-02", "iuas-03", "wuas-01", "iuas-04"]);
    }

    #[test]
    fn rc_vehicles_default_empty_and_reject_empty_id() {
        let ParseOutcome::Run(c) = parse_args(&[]).unwrap() else { panic!("run") };
        assert!(c.rc_vehicles.is_empty(), "no flag: pilot surface inert");
        assert!(parse_args(&args(&["--rc-vehicle", "  "])).is_err(), "empty id refused");
    }

    #[test]
    fn rc_carriage_defaults_to_spark_and_rc_data_demotes() {
        let ParseOutcome::Run(c) = parse_args(&[]).unwrap() else { panic!("run") };
        assert_eq!(c.rc_carriage, RcCarriage::Spark, "engine-Spark is the default carriage");
        let ParseOutcome::Run(c) =
            parse_args(&args(&["--rc-vehicle", "iuas-02", "--rc-data"])).unwrap()
        else {
            panic!("run")
        };
        assert_eq!(c.rc_carriage, RcCarriage::Data, "--rc-data selects the comparison bearer");
    }
}
