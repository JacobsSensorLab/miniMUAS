//! The miniMUAS v3 **virtual deployment**: everything on one machine, real
//! code end to end —
//!
//! ```text
//! 3 × ArduCopter SITL (child processes, TCP 5760+10i)
//!         │ mavlink
//! 3 × muas-agent (in-process, lib embed)  ← wuas-01 / iuas-01 / iuas-02
//!         │ UDP bridge (lossless loopback)
//! ndn-sim fabric (SimLinks: apsta / ndr-good / ndr-contested)
//!         │ UDP bridge (console node)
//! muas-dashboard (in-process, axum @ 0.0.0.0:8080)
//! ```
//!
//! Interactive by default (banner + drive script, Ctrl-C tears everything
//! down); `--verify` instead scripts a mission through the dashboard's own
//! WS API and exits with a JSON verdict per check.
//!
//! At startup one typed [`muas_sim::RunConfig`] record is journaled into
//! every vehicle's journal and into the deployment's own log
//! (`deployment-run/`), and its `run_id` is stamped onto every subsequent
//! journal line — the input side of the run is first-class data.
//!
//! SITL care (hard-won): the children DIE on stdin EOF, so their stdin
//! pipes are held open for the whole run; and their TCP serial ports are
//! never probed externally — readiness comes from the SITL's own
//! "Waiting for connection" log line.

use std::collections::HashMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use muas_agent::{AgentConfig, Endpoint};
use muas_sim::run_config::{git_rev, LinkProfileConfig, SitlRunConfig, VehicleRunConfig};
use muas_sim::{FleetSim, RunConfig, VehicleSpec};
use ndn_sim::LinkConfig;
use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;

// ───────────────────────────── fixed layout ─────────────────────────────────

const SITL_BIN: &str = "/Users/pmle/Documents/Dev/ardupilot/build/sitl/bin/arducopter";
const SITL_DEFAULTS: &str =
    "/Users/pmle/Documents/Dev/ardupilot/Tools/autotest/default_params/copter.parm";
const ARDUPILOT_DIR: &str = "/Users/pmle/Documents/Dev/ardupilot";
const SITL_MODEL: &str = "+";
const SITL_SPEEDUP: f64 = 1.0; // real time: smooth for a human on the map
/// CMAC-ish anchor the three homes spread around (~40 m apart).
const HOME: (f64, f64, f64) = (35.3632621, 149.1652374, 584.0);
const M_PER_DEG_LAT: f64 = 111_320.0;

const VEHICLES: [&str; 3] = ["wuas-01", "iuas-01", "iuas-02"];

fn home_of(index: usize) -> (f64, f64) {
    // wuas-01 center, iuas-01 40 m north, iuas-02 40 m south.
    let north_m = [0.0, 40.0, -40.0][index];
    (HOME.0 + north_m / M_PER_DEG_LAT, HOME.1)
}

fn tcp_port(index: usize) -> u16 {
    5760 + 10 * index as u16
}

/// The link profiles of the M5 radio comparison (`apsta` default: clean
/// AP/STA Wi-Fi; the ndr profiles model the named-data radio).
fn link_profile(name: &str) -> Option<LinkConfig> {
    match name {
        "apsta" => Some(LinkConfig {
            delay: Duration::from_millis(2),
            jitter: Duration::from_micros(500),
            loss_rate: 0.001,
            bandwidth_bps: 20_000_000,
        }),
        "ndr-good" => Some(LinkConfig {
            delay: Duration::from_millis(5),
            jitter: Duration::from_millis(2),
            loss_rate: 0.01,
            bandwidth_bps: 6_000_000,
        }),
        "ndr-contested" => Some(LinkConfig {
            delay: Duration::from_millis(15),
            jitter: Duration::from_millis(8),
            loss_rate: 0.08,
            bandwidth_bps: 2_000_000,
        }),
        _ => None,
    }
}

// ───────────────────────────── CLI ──────────────────────────────────────────

struct Args {
    verify: bool,
    profile: String,
    http_port: u16,
    run_dir: PathBuf,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        verify: false,
        profile: "apsta".into(),
        http_port: 8080,
        run_dir: PathBuf::from("./deployment-run"),
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--verify" => args.verify = true,
            "--profile" => {
                args.profile = it.next().ok_or("--profile: missing value")?;
                if link_profile(&args.profile).is_none() {
                    return Err(format!(
                        "--profile: unknown '{}' (apsta | ndr-good | ndr-contested)",
                        args.profile
                    ));
                }
            }
            "--http-port" => {
                args.http_port = it
                    .next()
                    .ok_or("--http-port: missing value")?
                    .parse()
                    .map_err(|e| format!("--http-port: {e}"))?;
            }
            "--run-dir" => args.run_dir = PathBuf::from(it.next().ok_or("--run-dir: missing value")?),
            "--help" | "-h" => {
                println!(
                    "virtual_deployment — 3×SITL + 3×muas-agent + ndn-sim fabric + dashboard\n\n\
                     USAGE: virtual_deployment [--verify] [--profile apsta|ndr-good|ndr-contested]\n\
                            [--http-port 8080] [--run-dir ./deployment-run]\n\n\
                     Default: interactive (dashboard at :8080, Ctrl-C tears down).\n\
                     --verify: headless scripted mission via the dashboard WS API;\n\
                               exits 0 with JSON verdicts when every check passes."
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown flag '{other}' (--help)")),
        }
    }
    Ok(args)
}

// ───────────────────────────── SITL children ────────────────────────────────

struct SitlFleet {
    children: Vec<tokio::process::Child>,
    log_paths: Vec<PathBuf>,
}

impl SitlFleet {
    /// Spawn one arducopter per vehicle: own working dir (fresh eeprom),
    /// stdout+stderr to `sitl.log`, stdin held open (they exit on EOF),
    /// killed on drop as the last-ditch cleanup.
    fn spawn(run_dir: &Path) -> Result<Self, String> {
        if !Path::new(SITL_BIN).exists() {
            return Err(format!("SITL binary not found at {SITL_BIN}"));
        }
        let mut children = Vec::new();
        let mut log_paths = Vec::new();
        for (i, vid) in VEHICLES.iter().enumerate() {
            let dir = run_dir.join(format!("sitl-{vid}"));
            std::fs::create_dir_all(&dir).map_err(|e| format!("sitl dir: {e}"))?;
            let log_path = dir.join("sitl.log");
            let log = std::fs::File::create(&log_path).map_err(|e| format!("sitl log: {e}"))?;
            let err = log.try_clone().map_err(|e| format!("sitl log clone: {e}"))?;
            let (lat, lon) = home_of(i);
            let home = format!("{lat:.7},{lon:.7},{alt},0", alt = HOME.2);
            let child = tokio::process::Command::new(SITL_BIN)
                .current_dir(&dir)
                .args(["--model", SITL_MODEL])
                .arg("-S") // synthetic clock
                .args(["--speedup", &format!("{SITL_SPEEDUP}")])
                .args(["-I", &i.to_string()]) // ports 5760+10i etc.
                .args(["--defaults", SITL_DEFAULTS])
                .arg("-w") // wipe eeprom: deterministic params per run
                .args(["--home", &home])
                .stdin(std::process::Stdio::piped()) // hold open: EOF kills SITL
                .stdout(std::process::Stdio::from(log))
                .stderr(std::process::Stdio::from(err))
                .kill_on_drop(true)
                .spawn()
                .map_err(|e| format!("spawn {vid} SITL: {e}"))?;
            println!("  sitl {vid}: pid {:?}, tcp:{}, home {home}", child.id(), tcp_port(i));
            children.push(child);
            log_paths.push(log_path);
        }
        Ok(Self { children, log_paths })
    }

    /// Wait until every instance logs its serial0 "Waiting for connection"
    /// (never probe the 576x ports — SITL dislikes connect-and-drop).
    async fn wait_ready(&self, timeout: Duration) -> Result<(), String> {
        let deadline = tokio::time::Instant::now() + timeout;
        for (i, path) in self.log_paths.iter().enumerate() {
            loop {
                let text = std::fs::read_to_string(path).unwrap_or_default();
                if text.contains("Waiting for connection") {
                    break;
                }
                if tokio::time::Instant::now() > deadline {
                    return Err(format!(
                        "SITL {} not ready within {timeout:?} (see {})",
                        VEHICLES[i],
                        path.display()
                    ));
                }
                tokio::time::sleep(Duration::from_millis(300)).await;
            }
        }
        Ok(())
    }

    async fn kill(&mut self) {
        for child in &mut self.children {
            let _ = child.start_kill();
        }
        for child in &mut self.children {
            let _ = child.wait().await;
        }
    }
}

// ───────────────────────────── deployment log ───────────────────────────────

/// Append-only JSONL log of the deployment process itself (run-config
/// record first, then lifecycle + verdict lines).
struct DeployLog {
    file: std::fs::File,
    run_id: String,
}

impl DeployLog {
    fn create(run_dir: &Path, run_id: &str) -> Result<Self, String> {
        let path = run_dir.join(format!("deployment-{run_id}.jsonl"));
        let file = std::fs::File::create(&path).map_err(|e| format!("deploy log: {e}"))?;
        println!("  deployment log: {}", path.display());
        Ok(Self { file, run_id: run_id.to_string() })
    }

    fn line(&mut self, kind: &str, mut fields: Value) {
        if let Some(map) = fields.as_object_mut() {
            map.insert("kind".into(), json!(kind));
            map.insert("run_id".into(), json!(self.run_id));
            map.insert("ts_ns".into(), json!(now_ns()));
        }
        let _ = writeln!(self.file, "{fields}");
        let _ = self.file.flush();
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

// ───────────────────────────── bring-up ─────────────────────────────────────

fn agent_config_for(index: usize, run_id: &str, run_dir: &Path, config: &mut AgentConfig) {
    config.endpoint = Endpoint::Mavlink(format!("tcp:127.0.0.1:{}", tcp_port(index)));
    config.run_id = Some(run_id.to_string());
    config.log_dir = Some(run_dir.join("journals"));
    // Capabilities per v2: the WUAS rasters; the IUAS pair investigates
    // with camera, and iuas-02 is the mic airframe (audio extra).
    config.extras = match index {
        0 => vec!["raster".into(), "camera".into()],
        1 => vec!["orbit".into(), "camera".into()],
        _ => vec!["orbit".into(), "camera".into(), "audio".into()],
    };
}

fn build_run_config(run_id: &str, profile: &str, link: &LinkConfig, configs: &[AgentConfig]) -> RunConfig {
    let dev = Path::new("/Users/pmle/Documents/Dev");
    RunConfig {
        run_id: run_id.to_string(),
        created_ns: now_ns(),
        vehicles: configs.iter().map(VehicleRunConfig::from_agent).collect(),
        link_profile: LinkProfileConfig::new(profile, link),
        sitl: SitlRunConfig {
            binary: SITL_BIN.into(),
            model: SITL_MODEL.into(),
            defaults: SITL_DEFAULTS.into(),
            speedup: SITL_SPEEDUP,
            homes: (0..VEHICLES.len())
                .map(|i| {
                    let (lat, lon) = home_of(i);
                    format!("{lat:.7},{lon:.7},{},0", HOME.2)
                })
                .collect(),
            ardupilot_rev: git_rev(Path::new(ARDUPILOT_DIR)),
        },
        stack_revs: [
            ("minimuas", dev.join("miniMUAS")),
            ("uas-flight", dev.join("uas-flight")),
            ("uas-fleet", dev.join("uas-fleet")),
            ("uas-console", dev.join("uas-console")),
        ]
        .into_iter()
        .map(|(name, path)| (name.to_string(), git_rev(&path)))
        .collect(),
    }
}

fn banner(addr: std::net::SocketAddr, profile: &str, run_id: &str) {
    println!(
        "\n══════════════════════════════════════════════════════════════════════\n\
         miniMUAS v3 VIRTUAL DEPLOYMENT is up                     (run {run_id})\n\
         ══════════════════════════════════════════════════════════════════════\n\
         Dashboard   http://{addr}   (also http://localhost:{port})\n\
         Fabric      ndn-sim '{profile}' links between all vehicles + GCS\n\
         Vehicles    wuas-01  searcher   SITL tcp:5760   (raster, camera)\n\
                     iuas-01  inspector  SITL tcp:5770   (orbit, camera)\n\
                     iuas-02  inspector  SITL tcp:5780   (orbit, camera, mic)\n\
         Artifacts   ./deployment-run/  (journals/, replays/, sitl-*/, run-config)\n\
         \n\
         Suggested drive script\n\
         ----------------------\n\
          1. Open the dashboard; wait for all three markers to go green\n\
             (SITL EKF settles ~30 s after boot — takeoffs before that are\n\
             refused with 'arm refused', just press again).\n\
          2. Takeoff each vehicle from its tile (8 m is a good AGL).\n\
          3. Search editor → center mode → pick a point on the map near\n\
             wuas-01 → ~150 × 100 m, spacing 25 m, speed 4 m/s →\n\
             'Preview legs' → Start mission. Watch wuas-01 fly the\n\
             serpentine and the leg/frames banner tick.\n\
          4. While it flies, watch coop biases: the ⚠ badge + avoid_bias_m\n\
             appear on any pair whose paths converge (fly an IUAS across\n\
             the search area to provoke it).\n\
          5. RTL ALL (mission bar) — slot-layered smart RTL: 8/11/14 m\n\
             slots, land, disarm.\n\
          6. Replay: pick the recording from the Replay menu and scrub.\n\
         \n\
         Ctrl-C tears everything down (SITLs killed, journals flushed).\n\
         ══════════════════════════════════════════════════════════════════════\n",
        port = addr.port(),
    );
}

// ───────────────────────────── verify mode ──────────────────────────────────

/// One verification check's outcome.
struct Check {
    name: &'static str,
    pass: bool,
    details: Value,
}

impl Check {
    fn emit(&self, log: &mut DeployLog) {
        let line = json!({
            "check": self.name,
            "pass": self.pass,
            "details": self.details,
        });
        println!("DEPLOY_VERDICT {line}");
        log.line("verify.check", line);
    }
}

/// WS probe: one client on the dashboard's own WebSocket API, caching the
/// broadcast state the checks assert on.
struct WsProbe {
    ws: tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    telemetry: HashMap<String, Value>,
    last_search: Option<Value>,
    coord: Vec<Value>,
    hello: Option<Value>,
}

impl WsProbe {
    async fn connect(addr: std::net::SocketAddr) -> Result<Self, String> {
        let (ws, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{}/ws", addr.port()))
            .await
            .map_err(|e| format!("ws connect: {e}"))?;
        Ok(Self {
            ws,
            telemetry: HashMap::new(),
            last_search: None,
            coord: Vec::new(),
            hello: None,
        })
    }

    async fn send(&mut self, message: Value) -> Result<(), String> {
        self.ws
            .send(Message::Text(message.to_string().into()))
            .await
            .map_err(|e| format!("ws send: {e}"))
    }

    /// Read one frame (with timeout) and fold it into the caches.
    async fn pump(&mut self, timeout: Duration) {
        let Ok(Some(Ok(frame))) = tokio::time::timeout(timeout, self.ws.next()).await else {
            return;
        };
        let Message::Text(text) = frame else { return };
        let Ok(message) = serde_json::from_str::<Value>(&text) else { return };
        match message.get("type").and_then(Value::as_str) {
            Some("hello") => self.hello = Some(message),
            Some("telemetry") => {
                if let Some(vid) = message.get("vehicle").and_then(Value::as_str) {
                    self.telemetry
                        .insert(vid.to_string(), message["sample"].clone());
                }
            }
            Some("search_status") => self.last_search = Some(message["status"].clone()),
            Some("coord") => {
                let empty = message["entries"]
                    .as_array()
                    .map(|a| a.is_empty())
                    .unwrap_or(true);
                if !empty {
                    self.coord.push(message);
                }
            }
            _ => {}
        }
    }

    /// Pump until `pred(self)` or the budget lapses; true on success.
    async fn wait_until(&mut self, budget: Duration, pred: impl Fn(&Self) -> bool) -> bool {
        let deadline = tokio::time::Instant::now() + budget;
        while tokio::time::Instant::now() < deadline {
            if pred(self) {
                return true;
            }
            self.pump(Duration::from_millis(500)).await;
        }
        pred(self)
    }

    fn armed(&self, vid: &str) -> bool {
        self.telemetry
            .get(vid)
            .and_then(|s| s.get("armed"))
            .and_then(Value::as_bool)
            .unwrap_or(false)
    }

    fn agl(&self, vid: &str) -> f64 {
        self.telemetry
            .get(vid)
            .and_then(|s| s.get("agl_m"))
            .and_then(Value::as_f64)
            .unwrap_or(0.0)
    }

    fn pos(&self, vid: &str) -> Option<(f64, f64)> {
        let s = self.telemetry.get(vid)?;
        Some((s.get("lat_deg")?.as_f64()?, s.get("lon_deg")?.as_f64()?))
    }
}

/// Takeoff with retries: SITL prearm (EKF settling) refuses early arms, so
/// keep pressing the same dashboard button until telemetry shows the climb.
async fn takeoff_until_airborne(
    probe: &mut WsProbe,
    vid: &str,
    agl_m: f64,
    budget: Duration,
) -> bool {
    let deadline = tokio::time::Instant::now() + budget;
    while tokio::time::Instant::now() < deadline {
        let _ = probe
            .send(json!({
                "cmd": "flight", "vehicle": vid, "command": "takeoff",
                "params": { "target_agl_m": agl_m },
            }))
            .await;
        let vid_owned = vid.to_string();
        let settled = probe
            .wait_until(Duration::from_secs(15), |p| {
                p.armed(&vid_owned) && p.agl(&vid_owned) >= agl_m - 1.5
            })
            .await;
        if settled {
            return true;
        }
    }
    false
}

/// The scripted mission (documented drive): hello → telemetry×3 →
/// wuas takeoff (WS) → start_mission (WS) → search running ≥2 legs →
/// iuas takeoffs (WS) + crossing gotos (DIRECT backend access — the
/// dashboard has no raw-goto control, documented in the verdict) → coord
/// entries on the WS feed → RTL ALL (WS) → all landed + disarmed.
async fn run_verify(
    addr: std::net::SocketAddr,
    fleet: &FleetSim,
    log: &mut DeployLog,
) -> Result<Vec<Check>, String> {
    let mut checks = Vec::new();
    let mut probe = WsProbe::connect(addr).await?;

    // 1 — hello with the full roster.
    let hello_ok = probe
        .wait_until(Duration::from_secs(10), |p| p.hello.is_some())
        .await
        && probe.hello.as_ref().map(|h| h["vehicles"] == json!(VEHICLES)).unwrap_or(false);
    checks.push(Check {
        name: "ws-hello-roster",
        pass: hello_ok,
        details: json!({ "vehicles": probe.hello.as_ref().map(|h| h["vehicles"].clone()) }),
    });

    // 2 — live MAVLink telemetry for all three on the map feed.
    let telemetry_ok = probe
        .wait_until(Duration::from_secs(90), |p| {
            VEHICLES.iter().all(|v| {
                p.telemetry
                    .get(*v)
                    .map(|s| s["source"] == json!("mavlink") && s["lat_deg"].as_f64().unwrap_or(0.0) != 0.0)
                    .unwrap_or(false)
            })
        })
        .await;
    checks.push(Check {
        name: "telemetry-all-3-mavlink",
        pass: telemetry_ok,
        details: json!({
            "vehicles_seen": probe.telemetry.keys().cloned().collect::<Vec<_>>(),
        }),
    });
    if !telemetry_ok {
        return Ok(checks); // nothing else can pass without telemetry
    }

    // 3 — wuas-01 takeoff to 8 m via the dashboard (prearm retries).
    let up = takeoff_until_airborne(&mut probe, "wuas-01", 8.0, Duration::from_secs(180)).await;
    checks.push(Check {
        name: "wuas-takeoff-8m",
        pass: up,
        details: json!({ "agl_m": probe.agl("wuas-01"), "armed": probe.armed("wuas-01") }),
    });
    log.line("verify.progress", json!({ "stage": "wuas airborne", "ok": up }));

    // 4 — start a small raster mission over ~150×100 m around the WUAS.
    let center = probe.pos("wuas-01").unwrap_or((HOME.0, HOME.1));
    probe
        .send(json!({
            "cmd": "start_mission",
            "params": {
                "area": { "mode": "center", "center_lat": center.0, "center_lon": center.1,
                          "width_m": 150.0, "height_m": 100.0 },
                "agl_m": 8.0,
                "leg_spacing_m": 25.0,
                "capture_every_m": 20.0,
                "speed_m_s": 5.0,
                "max_duration_s": 420.0,
                "object_query": "person",
            },
        }))
        .await?;
    let running = probe
        .wait_until(Duration::from_secs(240), |p| {
            p.last_search
                .as_ref()
                .map(|s| s["state"] == json!("searching") && s["leg"].as_u64().unwrap_or(0) >= 1)
                .unwrap_or(false)
        })
        .await;
    checks.push(Check {
        name: "raster-running-2-legs",
        pass: running,
        details: json!({ "search_status": probe.last_search }),
    });
    log.line("verify.progress", json!({ "stage": "raster ≥2 legs", "ok": running }));

    // 5 — cooperative coordination: both IUAS airborne with 2 m vertical
    // spacing, then CROSSING gotos issued directly on the agents' flight
    // backends (documented: the dashboard exposes no raw goto; this is the
    // same seam the smoke tests fly) — coord entries must reach the
    // dashboard's WS feed.
    let iuas_up_1 =
        takeoff_until_airborne(&mut probe, "iuas-01", 8.0, Duration::from_secs(150)).await;
    let iuas_up_2 =
        takeoff_until_airborne(&mut probe, "iuas-02", 6.0, Duration::from_secs(150)).await;
    let mut coord_ok = false;
    if iuas_up_1 && iuas_up_2 {
        let p1 = probe.pos("iuas-01").unwrap_or(home_of(1));
        let p2 = probe.pos("iuas-02").unwrap_or(home_of(2));
        // Send each THROUGH the other's position (overshoot by the full
        // separation) so they keep converging until the guard engages.
        let beyond = |from: (f64, f64), to: (f64, f64)| (2.0 * to.0 - from.0, 2.0 * to.1 - from.1);
        let t1 = beyond(p1, p2);
        let t2 = beyond(p2, p1);
        fleet.with_backend(1, |b| {
            b.set_cruise_speed(3.0);
            b.goto(t1.0, t1.1, 8.0, None);
        });
        fleet.with_backend(2, |b| {
            b.set_cruise_speed(3.0);
            b.goto(t2.0, t2.1, 6.0, None);
        });
        coord_ok = probe
            .wait_until(Duration::from_secs(90), |p| !p.coord.is_empty())
            .await;
        // Freeze the pair before the RTL check.
        for vid in ["iuas-01", "iuas-02"] {
            let _ = probe
                .send(json!({ "cmd": "flight", "vehicle": vid, "command": "hold" }))
                .await;
        }
    }
    checks.push(Check {
        name: "coop-coord-on-ws-feed",
        pass: coord_ok,
        details: json!({
            "iuas_airborne": [iuas_up_1, iuas_up_2],
            "first_coord": probe.coord.first(),
            "drove": "per-vehicle WS takeoffs + crossing gotos via direct agent backend access (dashboard has no raw goto)",
        }),
    });
    log.line("verify.progress", json!({ "stage": "coord entries", "ok": coord_ok }));

    // 6 — RTL ALL from the dashboard; everyone lands and disarms (the
    // WUAS raster aborts within one control cycle on the abort flag).
    probe.send(json!({ "cmd": "all", "command": "rtl" })).await?;
    let down = probe
        .wait_until(Duration::from_secs(300), |p| {
            VEHICLES.iter().all(|v| !p.armed(v) && p.agl(v) < 1.0)
        })
        .await;
    checks.push(Check {
        name: "rtl-all-land-disarm",
        pass: down,
        details: json!({
            "agl": VEHICLES.iter().map(|v| (v.to_string(), probe.agl(v))).collect::<HashMap<_, _>>(),
            "armed": VEHICLES.iter().map(|v| (v.to_string(), probe.armed(v))).collect::<HashMap<_, _>>(),
        }),
    });
    Ok(checks)
}

// ───────────────────────────── main ─────────────────────────────────────────

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(err) => {
            eprintln!("error: {err}");
            std::process::exit(2);
        }
    };
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "warn,virtual_deployment=info".into()),
        )
        .init();

    match run(args).await {
        Ok(code) => std::process::exit(code),
        Err(err) => {
            eprintln!("deployment failed: {err}");
            std::process::exit(1);
        }
    }
}

async fn run(args: Args) -> Result<i32, String> {
    let run_id = format!("run-{}", now_ns() / 1_000_000_000);
    let run_dir = args.run_dir.clone();
    std::fs::create_dir_all(run_dir.join("journals")).map_err(|e| format!("run dir: {e}"))?;
    std::fs::create_dir_all(run_dir.join("replays")).map_err(|e| format!("run dir: {e}"))?;
    std::fs::create_dir_all(run_dir.join("tiles")).map_err(|e| format!("run dir: {e}"))?;
    let mut deploy_log = DeployLog::create(&run_dir, &run_id)?;
    let link = link_profile(&args.profile).expect("validated at parse");

    // 1 — SITL children.
    println!("[1/4] spawning {} ArduCopter SITL instances …", VEHICLES.len());
    let mut sitl = SitlFleet::spawn(&run_dir)?;
    if let Err(err) = sitl.wait_ready(Duration::from_secs(45)).await {
        sitl.kill().await;
        return Err(err);
    }
    println!("      all SITLs waiting for connection");

    // 2 — agents on the ndn-sim fabric (console node for the dashboard).
    println!("[2/4] starting agents on the ndn-sim fabric ('{}' links) …", args.profile);
    let specs: Vec<VehicleSpec> = VEHICLES
        .iter()
        .enumerate()
        .map(|(i, vid)| VehicleSpec::new(vid, home_of(i)))
        .collect();
    let fleet = {
        let run_id = run_id.clone();
        let run_dir = run_dir.clone();
        FleetSim::start_with_console(&specs, link.clone(), 7, move |i, config| {
            agent_config_for(i, &run_id, &run_dir, config)
        })
        .await?
    };

    // The run-config record: rebuild the per-vehicle configs exactly as the
    // configure hook shaped them, journal one typed record everywhere.
    let agent_configs: Vec<AgentConfig> = VEHICLES
        .iter()
        .enumerate()
        .map(|(i, vid)| {
            let mut config = AgentConfig {
                vehicle_id: vid.to_string(),
                fleet_ids: VEHICLES.iter().map(|v| v.to_string()).collect(),
                ..AgentConfig::default()
            };
            agent_config_for(i, &run_id, &run_dir, &mut config);
            config
        })
        .collect();
    let run_config = build_run_config(&run_id, &args.profile, &link, &agent_configs);
    let record = serde_json::to_value(&run_config).map_err(|e| e.to_string())?;
    for agent in &fleet.agents {
        agent.shared.journal.event("run-config", record.clone());
    }
    deploy_log.line("run-config", record.clone());
    let rc_path = run_dir.join(format!("run-config-{run_id}.json"));
    std::fs::write(&rc_path, serde_json::to_string_pretty(&run_config).unwrap_or_default())
        .map_err(|e| format!("run-config write: {e}"))?;
    println!("      run-config journaled to every vehicle + {}", rc_path.display());

    // 3 — dashboard over the console bridge.
    println!("[3/4] starting the dashboard …");
    let (client_addr, bridge_addr) = fleet.bridge_client().await?;
    let dash_config = muas_dashboard::DashConfig {
        http_host: "0.0.0.0".into(),
        http_port: args.http_port,
        wuas_id: "wuas-01".into(),
        iuas_ids: vec!["iuas-01".into(), "iuas-02".into()],
        tiles_dir: run_dir.join("tiles"),
        record_dir: Some(run_dir.join("replays")),
        links: vec![muas_dashboard::UdpLink {
            local: client_addr,
            remote: bridge_addr,
            route: Some(muas_contracts::names::APP_PREFIX.to_string()),
        }],
        ..muas_dashboard::DashConfig::default()
    };
    let dashboard = muas_dashboard::start(
        dash_config,
        Arc::new(muas_dashboard::providers::StubDetector),
    )
    .await?;
    println!("[4/4] up.");
    deploy_log.line(
        "deployment.up",
        json!({ "dashboard": dashboard.addr.to_string(), "profile": args.profile }),
    );

    let exit_code = if args.verify {
        // Give the MAVLink links + EKF a moment before scripting.
        tokio::time::sleep(Duration::from_secs(5)).await;
        let addr = dashboard.addr;
        let checks = run_verify(addr, &fleet, &mut deploy_log).await?;
        let all = checks.iter().all(|c| c.pass);
        for check in &checks {
            check.emit(&mut deploy_log);
        }
        let summary = json!({
            "verdict": if all { "pass" } else { "fail" },
            "passed": checks.iter().filter(|c| c.pass).count(),
            "total": checks.len(),
            "run_id": run_id,
        });
        println!("DEPLOY_SUMMARY {summary}");
        deploy_log.line("verify.summary", summary);
        i32::from(!all)
    } else {
        banner(dashboard.addr, &args.profile, &run_id);
        let _ = tokio::signal::ctrl_c().await;
        println!("\nshutting down …");
        0
    };

    // Teardown: dashboard, agents (journals flushed), fabric, SITLs.
    deploy_log.line("deployment.down", json!({}));
    dashboard.shutdown().await;
    fleet.shutdown().await;
    sitl.kill().await;
    println!("done.");
    Ok(exit_code)
}
