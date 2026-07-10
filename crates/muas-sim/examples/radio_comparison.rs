//! Radio-mode comparison harness (M5, sim side): the same 2-vehicle
//! converging-conflict mission run over a matrix of link profiles —
//! `apsta`, `ndr-good`, `ndr-contested` — measuring per profile:
//!
//! - telemetry inter-arrival p50/p95 as observed AT the peer (an NDN
//!   consumer on the peer agent's engine polling `telemetry/live` with
//!   MustBeFresh across the lossy fabric),
//! - coordination grace-window success rate over 20 INDEPENDENT conflict
//!   episodes (fresh 2-vehicle fleet each; coop vs unco vs undecided),
//! - vehicle-service round-trip p50/p95 (`video_control` probe acks),
//! - spark-lane frame loss (raw-UDP spark lane through an impairment
//!   relay carrying the same link parameters — ndn-sim cannot pass a
//!   foreign UDP flow over a SimLink).
//!
//! Output: one JSON summary per profile (`results/radio-<profile>.json`)
//! plus a markdown table (`results/radio-comparison.md`). The three
//! profiles run CONCURRENTLY (separate fabrics + agents + ports) so the
//! full run stays under five minutes on the wall clock; per-profile
//! timing metrics are wall-clock and carry normal scheduler noise.
//!
//! Optional OTLP export (no collector required): set `MUAS_SIM_OTLP` to an
//! OTLP/HTTP addr (e.g. `127.0.0.1:4318`) to also push each fabric's
//! engine metrics via `ndn_sim::OtlpExporter`.
//!
//! Compressed parameters (documented): telemetry 5 Hz, cruise 6 m/s,
//! conflict episodes converge from 150 m, `grace_s` 1.5 s (sharpens the
//! grace window to ~3 coord polls so link loss is visible in the
//! coop/unco ratio). Steady-state lanes run on a separate long-lived
//! hovering pair so maneuvers never contaminate the timing metrics.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use muas_agent::BackendExt;
use muas_contracts::names;
use muas_contracts::services::VideoRequest;
use muas_sim::metrics::{spawn_impairment_relay, spawn_spark_receiver, SparkStats};
use muas_sim::{summarize, FleetSim, Summary, VehicleSpec};
use ndn_packet::encode::InterestBuilder;
use ndn_packet::Name;
use ndn_sim::{LinkConfig, OtlpExporter};
use tokio_util::sync::CancellationToken;
use uas_fleet_data::kinds::TelemetrySample;
use uas_fleet_node::coordination::EARTH_M_PER_DEG_LAT;

const ORIGIN: (f64, f64) = (35.0, -90.0);
const EPISODES: usize = 20;
const CRUISE_M_S: f64 = 6.0;

#[derive(Clone)]
struct Profile {
    name: &'static str,
    link: LinkConfig,
    seed: u64,
}

fn profiles() -> Vec<Profile> {
    vec![
        Profile {
            name: "apsta",
            link: LinkConfig {
                delay: Duration::from_millis(2),
                jitter: Duration::from_micros(500),
                loss_rate: 0.001,
                bandwidth_bps: 20_000_000,
            },
            seed: 101,
        },
        Profile {
            name: "ndr-good",
            link: LinkConfig {
                delay: Duration::from_millis(5),
                jitter: Duration::from_millis(2),
                loss_rate: 0.01,
                bandwidth_bps: 6_000_000,
            },
            seed: 102,
        },
        Profile {
            name: "ndr-contested",
            link: LinkConfig {
                delay: Duration::from_millis(15),
                jitter: Duration::from_millis(8),
                loss_rate: 0.08,
                bandwidth_bps: 2_000_000,
            },
            seed: 103,
        },
    ]
}

#[derive(Debug, serde::Serialize)]
struct LinkSummary {
    loss_pct: f64,
    delay_ms: f64,
    jitter_ms: f64,
    bandwidth_mbps: f64,
}

#[derive(Debug, serde::Serialize)]
struct CoordReport {
    episodes: usize,
    coop: usize,
    unco: usize,
    undecided: usize,
    grace_window_success_rate: f64,
}

#[derive(Debug, serde::Serialize)]
struct SparkReport {
    received: u64,
    seq_span: u64,
    frame_loss_pct: Option<f64>,
}

#[derive(Debug, serde::Serialize)]
struct ProfileReport {
    profile: String,
    link: LinkSummary,
    telemetry_interarrival_ms: Summary,
    coord: CoordReport,
    service_rtt_ms: Summary,
    service_timeouts: usize,
    spark: SparkReport,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Outcome {
    Coop,
    Unco,
    Undecided,
}

fn north_of(origin: (f64, f64), metres: f64) -> (f64, f64) {
    (origin.0 + metres / EARTH_M_PER_DEG_LAT, origin.1)
}

/// Scan the episode's agent journals (power-loss-safe JSONL, one per
/// vehicle) for coordination events. Returns (sides that reached a
/// cooperative state, any unco escalation).
///
/// Why journals and not the published coord/status: the guard's
/// pending->coop CONFIRMATION only flips the entry in its active table and
/// emits `coord.confirmed` — `apply()` (which republishes coord/status)
/// runs on engage/release only, so the wire keeps saying "coop-pending"
/// until the maneuver quietly releases. The journal is the only unmodified
/// observable of the confirm (an M5 observability finding).
fn scan_journals(dir: &std::path::Path) -> (usize, bool) {
    let mut coop_sides = 0usize;
    let mut unco = false;
    let Ok(entries) = std::fs::read_dir(dir) else {
        return (0, false);
    };
    for entry in entries.flatten() {
        let Ok(text) = std::fs::read_to_string(entry.path()) else {
            continue;
        };
        let mut side_coop = false;
        for line in text.lines() {
            let Ok(value) = serde_json::from_str::<serde_json::Value>(line) else {
                continue;
            };
            match value.get("kind").and_then(|k| k.as_str()) {
                Some("coord.confirmed") | Some("coord.coop") => side_coop = true,
                Some("coord.unco") => unco = true,
                _ => {}
            }
        }
        if side_coop {
            coop_sides += 1;
        }
    }
    (coop_sides, unco)
}

/// Distinct-sample inter-arrival measurement: poll the peer's
/// `telemetry/live` fast (30 ms cadence, MustBeFresh) and record the gap
/// between arrivals of NEW samples (distinct `gps_time_ns`).
async fn measure_interarrival(
    mut consumer: ndn_app::Consumer,
    name: Name,
    cancel: CancellationToken,
) -> Vec<f64> {
    let mut deltas = Vec::new();
    let mut last_stamp: Option<u64> = None;
    let mut last_arrival = tokio::time::Instant::now();
    while !cancel.is_cancelled() {
        let fetched = consumer
            .fetch_with(
                InterestBuilder::new(name.clone())
                    .must_be_fresh()
                    .lifetime(Duration::from_millis(400)),
            )
            .await;
        if let Ok(data) = fetched {
            if let Some(content) = data.content() {
                if let Ok(sample) = serde_json::from_slice::<TelemetrySample>(content) {
                    if last_stamp != Some(sample.gps_time_ns) {
                        let now = tokio::time::Instant::now();
                        if last_stamp.is_some() {
                            deltas.push((now - last_arrival).as_secs_f64() * 1000.0);
                        }
                        last_stamp = Some(sample.gps_time_ns);
                        last_arrival = now;
                    }
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    deltas
}

/// Service round-trip prober: a benign `video_control` ack every 400 ms.
async fn measure_service_rtt(
    client: muas_contracts::services::VehicleServiceClient<ndn_rpc::FaceRpcCarrier>,
    cancel: CancellationToken,
) -> (Vec<f64>, usize) {
    let mut rtts = Vec::new();
    let mut timeouts = 0usize;
    while !cancel.is_cancelled() {
        let t0 = tokio::time::Instant::now();
        match client
            .video_control(VideoRequest {
                enabled: false,
                ..VideoRequest::default()
            })
            .await
        {
            Ok(_) => rtts.push(t0.elapsed().as_secs_f64() * 1000.0),
            Err(_) => timeouts += 1,
        }
        tokio::time::sleep(Duration::from_millis(400)).await;
    }
    (rtts, timeouts)
}

/// One INDEPENDENT conflict episode: a fresh 2-vehicle fleet (fresh guards,
/// fresh coord caches) converging from 150 m, classified within a 14 s
/// window, then torn down.
///
/// Why a fresh fleet per episode: reusing one pair exposes a coordination
/// limit cycle — after a maneuver releases, `adopt_remote` re-engages from
/// the peer's (stale-cached) coord entry with no geometric gate, so a
/// released pair keeps ping-ponging coop maneuvers indefinitely
/// (reproduced at 0.1% loss; see the M5 findings). Episode independence
/// keeps the grace-window metric clean and sidesteps the cycle.
///
/// Why 150 m: a freshly booted guard is blind for ~6 s (cache-empty tick
/// schedules a 3 s recheck; the next tick still holds the peer's
/// pre-takeoff sample, not separable, +3 s more), so the pair must not
/// physically cross before ~2×3 s + confirm time. At 150 m / 12 m/s
/// closing the crossing is ~15 s out — engagement at ~6 s, confirmation
/// (or grace escalation) comfortably inside the window. Also an M5
/// finding: cold-boot fleets coordinate no earlier than ~6 s even on a
/// perfect link.
async fn run_episode(ids: (&str, &str), link: LinkConfig, seed: u64) -> Result<Outcome, String> {
    let origin_a = ORIGIN;
    let origin_b = north_of(ORIGIN, 150.0);
    let specs = [
        VehicleSpec::new(ids.0, origin_a),
        VehicleSpec::new(ids.1, origin_b),
    ];
    // Per-episode journal dir: the agents' JSONL journals are the outcome
    // observable (see `scan_journals`).
    let log_dir = std::env::temp_dir().join(format!(
        "muas-sim-radio-{}-{}-{seed}",
        std::process::id(),
        ids.0
    ));
    let fleet = {
        let log_dir = log_dir.clone();
        FleetSim::start(&specs, link, false, seed, move |_, c| {
            c.telemetry_hz = 5.0;
            c.grace_s = 1.5;
            c.log_dir = Some(log_dir.clone());
        })
        .await?
    };
    // Parallel takeoffs (ensure_airborne blocks on the climb).
    let (shared_a, shared_b) = (fleet.agents[0].shared.clone(), fleet.agents[1].shared.clone());
    let climb_a = tokio::task::spawn_blocking(move || {
        let mut guard = shared_a
            .backend
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let backend = guard.as_dyn();
        let ok = backend.ensure_airborne(6.0);
        backend.set_cruise_speed(CRUISE_M_S);
        ok
    });
    let climb_b = tokio::task::spawn_blocking(move || {
        let mut guard = shared_b
            .backend
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let backend = guard.as_dyn();
        let ok = backend.ensure_airborne(4.0);
        backend.set_cruise_speed(CRUISE_M_S);
        ok
    });
    let (up_a, up_b) = tokio::try_join!(climb_a, climb_b).map_err(|e| e.to_string())?;
    if !(up_a && up_b) {
        return Err("episode takeoff failed".into());
    }
    fleet.with_backend(0, |b| b.goto(origin_b.0, origin_b.1, 6.0, None));
    fleet.with_backend(1, |b| b.goto(origin_a.0, origin_a.1, 4.0, None));

    let deadline = tokio::time::Instant::now() + Duration::from_secs(14);
    let mut outcome = Outcome::Undecided;
    loop {
        let (coop_sides, unco) = scan_journals(&log_dir);
        if unco {
            outcome = Outcome::Unco;
            break;
        }
        if coop_sides == 2 {
            outcome = Outcome::Coop;
            break;
        }
        if tokio::time::Instant::now() >= deadline {
            break;
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
    }
    fleet.shutdown().await;
    let _ = std::fs::remove_dir_all(&log_dir);
    Ok(outcome)
}

async fn run_profile(profile: Profile, index: usize) -> Result<ProfileReport, String> {
    let origin_a = ORIGIN;
    let origin_b = north_of(ORIGIN, 150.0);
    let id_a = format!("iuas-9{index}");
    let id_b = format!("wuas-9{index}");

    // Spark lane: agent A -> impairment relay (profile-matched) -> receiver.
    let lane_cancel = CancellationToken::new();
    let receiver_socket = tokio::net::UdpSocket::bind("127.0.0.1:0")
        .await
        .map_err(|e| format!("spark receiver bind: {e}"))?;
    let receiver_addr = receiver_socket.local_addr().map_err(|e| e.to_string())?;
    let relay_socket = tokio::net::UdpSocket::bind("127.0.0.1:0")
        .await
        .map_err(|e| format!("spark relay bind: {e}"))?;
    let relay_addr = relay_socket.local_addr().map_err(|e| e.to_string())?;
    let spark_stats = Arc::new(Mutex::new(SparkStats::default()));
    let relay_task = spawn_impairment_relay(
        relay_socket,
        receiver_addr,
        profile.link.loss_rate,
        profile.link.delay,
        profile.link.jitter,
        lane_cancel.clone(),
    );
    let receiver_task =
        spawn_spark_receiver(receiver_socket, spark_stats.clone(), lane_cancel.clone());

    let specs = [
        VehicleSpec::new(&id_a, origin_a),
        VehicleSpec::new(&id_b, origin_b),
    ];
    let fleet = FleetSim::start(&specs, profile.link.clone(), false, profile.seed, |i, c| {
        c.telemetry_hz = 5.0;
        c.grace_s = 1.5;
        if i == 0 {
            c.spark_udp = Some(relay_addr);
        }
    })
    .await?;

    // The lane fleet HOVERS 150 m apart for the whole run: no conflicts,
    // no coordination maneuvers, so the steady-state lanes (telemetry,
    // service RTT, spark) measure the link alone.
    fleet.with_backend(0, |b| {
        assert!(b.ensure_airborne(10.0));
        b.set_cruise_speed(CRUISE_M_S);
    });
    fleet.with_backend(1, |b| {
        assert!(b.ensure_airborne(8.0));
        b.set_cruise_speed(CRUISE_M_S);
    });

    // Concurrent measurement lanes for the whole episode window.
    let measure_cancel = CancellationToken::new();
    let telemetry_name: Name = names::vehicle_stream(&id_a, "telemetry/live")
        .parse()
        .map_err(|e| format!("telemetry name: {e:?}"))?;
    let interarrival_task = tokio::spawn(measure_interarrival(
        fleet.consumer(1),
        telemetry_name,
        measure_cancel.clone(),
    ));
    let rtt_task = tokio::spawn(measure_service_rtt(
        fleet.service_client(1, 0, Duration::from_secs(2))?,
        measure_cancel.clone(),
    ));

    // Grace-window episodes: sequential fresh mini-fleets on their own
    // fabrics/ports (ids distinct from the lane fleet's).
    let ep_ids = (format!("iuas-8{index}"), format!("wuas-8{index}"));
    let mut coop = 0usize;
    let mut unco = 0usize;
    let mut undecided = 0usize;
    for ep in 0..EPISODES {
        let seed = profile.seed * 1000 + ep as u64;
        match run_episode((&ep_ids.0, &ep_ids.1), profile.link.clone(), seed).await? {
            Outcome::Coop => coop += 1,
            Outcome::Unco => unco += 1,
            Outcome::Undecided => undecided += 1,
        }
    }

    measure_cancel.cancel();
    let mut interarrival = interarrival_task.await.map_err(|e| e.to_string())?;
    let (mut rtts, timeouts) = rtt_task.await.map_err(|e| e.to_string())?;

    // Optional OTLP export (flagged; tests/CI never need a collector).
    if let Ok(addr) = std::env::var("MUAS_SIM_OTLP") {
        let exporter = OtlpExporter::new(addr).with_service_name("muas-sim");
        match exporter.export_metrics(&fleet.fabric.snapshot_metrics()).await {
            Ok(status) => println!("[{}] otlp export status {status}", profile.name),
            Err(err) => eprintln!("[{}] otlp export failed: {err}", profile.name),
        }
    }

    fleet.shutdown().await;
    lane_cancel.cancel();
    relay_task.abort();
    receiver_task.abort();

    let spark = {
        let stats = spark_stats
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        SparkReport {
            received: stats.received,
            seq_span: stats.span(),
            frame_loss_pct: stats.loss_rate().map(|l| l * 100.0),
        }
    };
    Ok(ProfileReport {
        profile: profile.name.to_string(),
        link: LinkSummary {
            loss_pct: profile.link.loss_rate * 100.0,
            delay_ms: profile.link.delay.as_secs_f64() * 1000.0,
            jitter_ms: profile.link.jitter.as_secs_f64() * 1000.0,
            bandwidth_mbps: profile.link.bandwidth_bps as f64 / 1e6,
        },
        telemetry_interarrival_ms: summarize(&mut interarrival),
        coord: CoordReport {
            episodes: EPISODES,
            coop,
            unco,
            undecided,
            grace_window_success_rate: coop as f64 / EPISODES as f64,
        },
        service_rtt_ms: summarize(&mut rtts),
        service_timeouts: timeouts,
        spark,
    })
}

fn fmt1(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.1}")
    } else {
        "n/a".to_string()
    }
}

fn markdown_table(reports: &[ProfileReport]) -> String {
    let mut out = String::new();
    out.push_str("# Radio-mode comparison (M5 sim harness)\n\n");
    out.push_str(
        "Generated by `cargo run -p muas-sim --example radio_comparison` — real unmodified \
         `muas-agent` instances bridged onto ndn-sim fabrics (UDP bridges at the fabric \
         edge, wall-clock kernel, compressed parameters: telemetry 5 Hz, cruise 6 m/s, \
         grace 1.5 s). Steady-state lanes (telemetry / service RTT / spark) run on a \
         long-lived hovering pair; the coop rate comes from 20 independent conflict \
         episodes, each a fresh 2-vehicle fleet converging from 150 m. The spark lane is \
         raw UDP through an impairment relay carrying the same link parameters.\n\n",
    );
    out.push_str(
        "| profile | link (loss / delay / jitter / bw) | telemetry inter-arrival p50/p95 ms (n) \
         | coord coop rate (coop/unco/none) | service RTT p50/p95 ms (n, timeouts) | spark frame loss |\n",
    );
    out.push_str("|---|---|---|---|---|---|\n");
    for r in reports {
        let spark = match r.spark.frame_loss_pct {
            Some(loss) => format!("{:.1}% ({}/{})", loss, r.spark.received, r.spark.seq_span),
            None => "n/a".to_string(),
        };
        out.push_str(&format!(
            "| {} | {}% / {} ms / {} ms / {} Mbps | {} / {} ({}) | {:.0}% ({}/{}/{}) | {} / {} ({}, {}) | {} |\n",
            r.profile,
            r.link.loss_pct,
            fmt1(r.link.delay_ms),
            fmt1(r.link.jitter_ms),
            r.link.bandwidth_mbps,
            fmt1(r.telemetry_interarrival_ms.p50),
            fmt1(r.telemetry_interarrival_ms.p95),
            r.telemetry_interarrival_ms.n,
            r.coord.grace_window_success_rate * 100.0,
            r.coord.coop,
            r.coord.unco,
            r.coord.undecided,
            fmt1(r.service_rtt_ms.p50),
            fmt1(r.service_rtt_ms.p95),
            r.service_rtt_ms.n,
            r.service_timeouts,
            spark,
        ));
    }
    out
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), String> {
    let started = std::time::Instant::now();
    let mut handles = Vec::new();
    for (index, profile) in profiles().into_iter().enumerate() {
        handles.push((
            profile.name,
            tokio::spawn(run_profile(profile, index)),
        ));
    }
    let mut reports = Vec::new();
    for (name, handle) in handles {
        let report = handle
            .await
            .map_err(|e| format!("{name}: task panicked: {e}"))??;
        println!(
            "[{name}] {}",
            serde_json::to_string(&report).map_err(|e| e.to_string())?
        );
        reports.push(report);
    }

    let results_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../results");
    std::fs::create_dir_all(&results_dir).map_err(|e| format!("results dir: {e}"))?;
    for report in &reports {
        let path = results_dir.join(format!("radio-{}.json", report.profile));
        let json = serde_json::to_string_pretty(report).map_err(|e| e.to_string())?;
        std::fs::write(&path, json).map_err(|e| format!("write {}: {e}", path.display()))?;
    }
    let table = markdown_table(&reports);
    let md_path = results_dir.join("radio-comparison.md");
    std::fs::write(&md_path, &table).map_err(|e| format!("write {}: {e}", md_path.display()))?;

    println!("\n{table}");
    println!(
        "wrote {} (+ per-profile JSON) in {:.0} s",
        md_path.display(),
        started.elapsed().as_secs_f64()
    );
    Ok(())
}
