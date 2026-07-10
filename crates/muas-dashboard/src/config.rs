//! Dashboard configuration + dependency-free CLI parser mirroring the v2
//! `run_dashboard.py` flags (plus the muas-agent UDP-face flags for the
//! engine's point-to-point links).

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
    /// Point-to-point UDP faces toward vehicles / forwarders.
    pub links: Vec<UdpLink>,
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
            links: Vec::new(),
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

MISSION:
    --confirm-count <N>        detections (within target_separation_m) needed
                               before a candidate becomes a dispatched target
                               (default 2 — the false-positive guard)
    --detect-timeout-ms <MS>   detection request timeout (default 30000)
    --search-margin-s <S>      raster deadline margin over max_duration_s
                               (default 60)
    --investigate-timeout-ms <MS>  investigation timeout (default 120000)

MAP / RECORDER:
    --tiles-dir <DIR>          local tile cache served at /tiles/{z}/{x}/{y}
                               (default /var/lib/minimuas/tiles)
    --tile-upstream <URL>      upstream tile template with {z}/{x}/{y};
                               empty string disables proxying (pure offline)
    --record-dir <DIR>         mission recorder JSONL directory (default
                               /var/lib/minimuas/replays); empty disables

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
}
