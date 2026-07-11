//! Dashboard configuration + dependency-free CLI parser mirroring the v2
//! `run_dashboard.py` flags (plus the muas-agent UDP-face flags for the
//! engine's point-to-point links).

use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::path::PathBuf;

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
    /// RC-CONTROL R2 pilot-surface targets: `vid -> the agent's `--rc
    /// listen:<addr>` UDP address` the dashboard's [`uas_rc::UdpRcSender`]
    /// aims at (`--rc-target`). Empty = no vehicle is RC-reachable and the
    /// Pilot surface stays inert. These frames travel exactly the pilot-node
    /// path: raw 26-byte [`uas_rc::RcFrame`]s onto the agent's real RC
    /// receiver socket — no dashboard-internal shortcut.
    pub rc_targets: BTreeMap<String, SocketAddr>,
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
            rc_targets: BTreeMap::new(),
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
    --rc-target <VID=ADDR>        the agent's `--rc listen:<addr>` UDP address
                                  the pilot surface aims its UdpRcSender at
                                  (a bare host:port, or listen:host:port).
                                  The dashboard sends raw uas-rc frames onto
                                  that real RC receiver socket — no shortcut.
    --rc-targets <VID=ADDR,..>    the same, several at once (comma-separated)

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

/// Parse one `VID=ADDR` RC target into the map. `ADDR` may carry a
/// `listen:` prefix (symmetry with the agent's `--rc listen:<addr>`); a
/// `spark:` prefix is refused (R2 rides the plain-UDP listen path, R1's
/// default).
fn parse_rc_target(spec: &str, flag: &str, out: &mut BTreeMap<String, SocketAddr>)
    -> Result<(), String> {
    let (vid, addr) = split_pair(spec, '=', flag)?;
    let vid = vid.trim();
    if vid.is_empty() {
        return Err(format!("{flag}: empty vehicle id in '{spec}'"));
    }
    let addr = addr.trim();
    let addr = match addr.split_once(':') {
        Some(("listen", rest)) => rest,
        Some(("spark", _)) => {
            return Err(format!(
                "{flag}: spark carriage is not wired in R2; use the plain-UDP \
                 listen address (got '{addr}')"
            ));
        }
        _ => addr,
    };
    out.insert(vid.to_string(), parse_addr(addr, flag)?);
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
            "--rc-target" => {
                parse_rc_target(&next(arg, &mut it)?, arg, &mut config.rc_targets)?;
            }
            "--rc-targets" => {
                for spec in next(arg, &mut it)?.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                    parse_rc_target(spec, arg, &mut config.rc_targets)?;
                }
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
    fn rc_targets_parse_repeatable_and_listed() {
        let ParseOutcome::Run(c) = parse_args(&args(&[
            "--rc-target",
            "iuas-02=127.0.0.1:14650",
            "--rc-target",
            "iuas-03=listen:127.0.0.1:14651",
            "--rc-targets",
            "wuas-01=127.0.0.1:14640, iuas-04=127.0.0.1:14652",
        ]))
        .unwrap() else {
            panic!("run")
        };
        assert_eq!(c.rc_targets.len(), 4);
        assert_eq!(c.rc_targets["iuas-02"], "127.0.0.1:14650".parse().unwrap());
        assert_eq!(
            c.rc_targets["iuas-03"],
            "127.0.0.1:14651".parse().unwrap(),
            "listen: prefix is stripped"
        );
        assert_eq!(c.rc_targets["wuas-01"], "127.0.0.1:14640".parse().unwrap());
    }

    #[test]
    fn rc_targets_default_empty_and_reject_bad_specs() {
        let ParseOutcome::Run(c) = parse_args(&[]).unwrap() else { panic!("run") };
        assert!(c.rc_targets.is_empty(), "no flag: pilot surface inert");
        assert!(parse_args(&args(&["--rc-target", "no-equals"])).is_err());
        assert!(parse_args(&args(&["--rc-target", "iuas-02=nope"])).is_err());
        assert!(
            parse_args(&args(&["--rc-target", "iuas-02=spark:127.0.0.1:1"])).is_err(),
            "spark carriage refused in R2"
        );
        assert!(parse_args(&args(&["--rc-target", "=127.0.0.1:1"])).is_err());
    }
}
