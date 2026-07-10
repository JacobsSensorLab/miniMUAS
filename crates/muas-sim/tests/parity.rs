//! Coordination parity scenarios (M3 close-out): the v2 SITL validation set
//! adapted to real agents on an ndn-sim fabric with lossy links.
//!
//! Each test drives *unmodified* `muas-agent` instances bridged onto an
//! ndn-lab fabric (`FleetSim`, see `muas_sim::fleet`) whose SimLinks carry
//! the ndr-good radio profile (1% loss, 5 ms delay, 2 ms jitter, 6 Mbps),
//! and emits a machine-readable `PARITY_VERDICT {json}` line per assertion.
//!
//! Wall-clock kernel with compressed parameters (documented in
//! `muas_sim` crate docs): 5 m/s cruise, 120–150 m engagement ranges.

use std::time::Duration;

use muas_agent::AgentCommand;
use muas_contracts::policy::dist_m;
use muas_sim::{FleetSim, VehicleSpec, Verdict};
use ndn_sim::LinkConfig;
use uas_fleet_node::coordination::EARTH_M_PER_DEG_LAT;

const ORIGIN: (f64, f64) = (35.0, -90.0);
const SCENARIO: &str = "coordination-parity";

/// The ndr-good link profile: realistic lossy fleet radio.
fn lossy_link() -> LinkConfig {
    LinkConfig {
        delay: Duration::from_millis(5),
        jitter: Duration::from_millis(2),
        loss_rate: 0.01,
        bandwidth_bps: 6_000_000,
    }
}

fn north_of(origin: (f64, f64), metres: f64) -> (f64, f64) {
    (origin.0 + metres / EARTH_M_PER_DEG_LAT, origin.1)
}

fn east_of(origin: (f64, f64), metres: f64) -> (f64, f64) {
    let m_per_deg = EARTH_M_PER_DEG_LAT * origin.0.to_radians().cos();
    (origin.0, origin.1 + metres / m_per_deg)
}

/// Arm + climb vehicle `i` to `agl_m`, set cruise speed, return home fix.
fn airborne(fleet: &FleetSim, i: usize, agl_m: f64, speed_m_s: f64) {
    fleet.with_backend(i, |b| {
        assert!(b.ensure_airborne(agl_m), "vehicle {i} failed to get airborne");
        b.set_cruise_speed(speed_m_s);
    });
}

/// Command vehicle `i` toward `target` at `agl_m`.
fn goto(fleet: &FleetSim, i: usize, target: (f64, f64), agl_m: f64) {
    fleet.with_backend(i, |b| b.goto(target.0, target.1, agl_m, None));
}

/// Poll `predicate` every 200 ms until it holds or `budget` elapses.
async fn wait_for(budget: Duration, mut predicate: impl FnMut() -> bool) -> bool {
    let deadline = tokio::time::Instant::now() + budget;
    loop {
        if predicate() {
            return true;
        }
        if tokio::time::Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

/// v2 assertion 1 — symmetric cooperation: converging missions drive BOTH
/// sides to "coop" with the complementary deterministic pair-plan biases
/// (+3 climber / −2 descender), over a lossy 3-node fabric (third vehicle
/// present on the fabric, parked on the ground).
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn symmetric_coop_with_complementary_biases() {
    let origin_b = north_of(ORIGIN, 150.0);
    let specs = [
        VehicleSpec::new("iuas-01", ORIGIN),
        VehicleSpec::new("wuas-01", origin_b),
        VehicleSpec::new("wuas-02", east_of(ORIGIN, 500.0)), // parked
    ];
    let fleet = FleetSim::start(&specs, lossy_link(), false, 11, |_, _| {})
        .await
        .expect("fleet up");

    airborne(&fleet, 0, 10.0, 5.0);
    airborne(&fleet, 1, 8.0, 5.0);
    goto(&fleet, 0, origin_b, 10.0);
    goto(&fleet, 1, ORIGIN, 8.0);

    let converged = wait_for(Duration::from_secs(45), || {
        fleet.publishes_mode(0, 1, "coop") && fleet.publishes_mode(1, 0, "coop")
    })
    .await;
    let bias_a = fleet.telemetry_of(0).avoid_bias_m;
    let bias_b = fleet.telemetry_of(1).avoid_bias_m;
    let pass =
        converged && (bias_a - 3.0).abs() < 1e-6 && (bias_b + 2.0).abs() < 1e-6;
    Verdict::new(
        SCENARIO,
        "symmetric-coop",
        pass,
        serde_json::json!({
            "coop_both_sides": converged,
            "bias_climber_m": bias_a,
            "bias_descender_m": bias_b,
        }),
    )
    .emit();
    fleet.shutdown().await;
    assert!(pass, "symmetric coop failed: converged={converged} bias_a={bias_a} bias_b={bias_b}");
}

/// v2 assertion 2 — uncooperative escalation: with the peer's coord/status
/// name blackholed (100%-loss route to a sink node; telemetry still flows),
/// the watcher reaches "unco" with the full upward burden (+6 m), while the
/// blackholed peer — which CAN see our coord — adopts cooperatively.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn unco_escalation_when_coord_blackholed() {
    let origin_b = north_of(ORIGIN, 150.0);
    let specs = [
        VehicleSpec::new("iuas-02", ORIGIN),
        VehicleSpec::new("wuas-02", origin_b),
    ];
    let fleet = FleetSim::start(&specs, lossy_link(), true, 12, |_, _| {})
        .await
        .expect("fleet up");
    fleet.blackhole_coord(0, 1).expect("blackhole route");

    airborne(&fleet, 0, 10.0, 5.0);
    airborne(&fleet, 1, 8.0, 5.0);
    goto(&fleet, 0, origin_b, 10.0);
    goto(&fleet, 1, ORIGIN, 8.0);

    let escalated = wait_for(Duration::from_secs(40), || {
        fleet.publishes_mode(0, 1, "unco")
    })
    .await;
    let bias_a = fleet.telemetry_of(0).avoid_bias_m;
    // The peer saw our coord (its path is clear) and adopted the pair plan.
    let peer_cooperated =
        fleet.publishes_mode(1, 0, "coop") || fleet.publishes_mode(1, 0, "coop-pending");
    let pass = escalated && (bias_a - 6.0).abs() < 1e-6;
    Verdict::new(
        SCENARIO,
        "unco-escalation",
        pass,
        serde_json::json!({
            "escalated_to_unco": escalated,
            "unco_bias_m": bias_a,
            "blackholed_peer_cooperated": peer_cooperated,
        }),
    )
    .emit();
    fleet.shutdown().await;
    assert!(pass, "unco escalation failed: escalated={escalated} bias_a={bias_a}");
}

/// v2 assertion 3 — smart-RTL slot layering: three vehicles engage RTL
/// simultaneously and cruise home at three distinct slot AGLs (8/11/14 m:
/// base 8, separation 3, sorted fleet ids) with no vertical overlap while
/// co-cruising.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn smart_rtl_slots_layer_without_overlap() {
    let specs = [
        VehicleSpec::new("iuas-01", ORIGIN),
        VehicleSpec::new("wuas-01", east_of(ORIGIN, 400.0)),
        VehicleSpec::new("wuas-02", east_of(ORIGIN, 800.0)),
    ];
    let slots = [8.0_f64, 11.0, 14.0]; // sorted ids: iuas-01, wuas-01, wuas-02
    let fleet = FleetSim::start(&specs, lossy_link(), false, 13, |_, _| {})
        .await
        .expect("fleet up");

    // Everyone flies 60 m north of home at 6 m AGL.
    let targets: Vec<(f64, f64)> = specs.iter().map(|s| north_of(s.origin, 60.0)).collect();
    for (i, target) in targets.iter().enumerate() {
        airborne(&fleet, i, 6.0, 5.0);
        goto(&fleet, i, *target, 6.0);
    }
    let on_station = wait_for(Duration::from_secs(30), || {
        (0..3).all(|i| {
            let t = fleet.telemetry_of(i);
            dist_m((t.lat_deg, t.lon_deg), targets[i]) < 5.0
        })
    })
    .await;
    assert!(on_station, "outbound legs did not settle");

    // Simultaneous RTL on all three (the service's flight_rtl path sends
    // this same command).
    for agent in &fleet.agents {
        agent
            .shared
            .commands
            .send(AgentCommand::SmartRtl)
            .expect("command channel");
    }

    // Sample at 5 Hz until everyone lands (or the budget runs out).
    let mut cruise_agls: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    let mut min_co_cruise_gap = f64::INFINITY;
    let mut landed = [false; 3];
    let deadline = tokio::time::Instant::now() + Duration::from_secs(90);
    while tokio::time::Instant::now() < deadline && !landed.iter().all(|l| *l) {
        let mut cruising = [false; 3];
        let mut agls = [0.0_f64; 3];
        for i in 0..3 {
            let t = fleet.telemetry_of(i);
            agls[i] = t.agl_m;
            let speed = t.vn_m_s.hypot(t.ve_m_s);
            cruising[i] = speed > 2.0 && (t.agl_m - slots[i]).abs() <= 1.0;
            if cruising[i] {
                cruise_agls[i].push(t.agl_m);
            }
            if t.mode == "LAND" {
                landed[i] = true;
            }
        }
        for i in 0..3 {
            for j in (i + 1)..3 {
                if cruising[i] && cruising[j] {
                    min_co_cruise_gap = min_co_cruise_gap.min((agls[i] - agls[j]).abs());
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }

    let mean = |v: &Vec<f64>| -> f64 {
        if v.is_empty() {
            f64::NAN
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        }
    };
    let observed: Vec<f64> = cruise_agls.iter().map(mean).collect();
    let all_cruised = cruise_agls.iter().all(|v| v.len() >= 3);
    let co_cruised = min_co_cruise_gap.is_finite();
    let no_overlap = co_cruised && min_co_cruise_gap >= 2.0;
    let all_landed = landed.iter().all(|l| *l);
    let pass = all_cruised && no_overlap && all_landed;
    Verdict::new(
        SCENARIO,
        "smart-rtl-slot-layering",
        pass,
        serde_json::json!({
            "slot_table_agl_m": slots,
            "observed_cruise_agl_m": observed,
            "min_co_cruise_gap_m": if co_cruised { min_co_cruise_gap } else { -1.0 },
            "all_reached_landing": all_landed,
        }),
    )
    .emit();
    fleet.shutdown().await;
    assert!(
        pass,
        "smart RTL slots failed: cruised={all_cruised} gap={min_co_cruise_gap} landed={landed:?} observed={observed:?}"
    );
}

/// v2 assertion 4 — fleet flight floor: a descender at 4.0 m AGL against
/// the 3.5 m fleet floor only gives the 0.5 m it has; the climber absorbs
/// the shortfall, and the descender never sinks below the floor.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn fleet_floor_respected_by_descender() {
    let origin_b = north_of(ORIGIN, 120.0);
    let specs = [
        VehicleSpec::new("iuas-03", ORIGIN),
        VehicleSpec::new("wuas-03", origin_b),
    ];
    let fleet = FleetSim::start(&specs, lossy_link(), false, 14, |_, _| {})
        .await
        .expect("fleet up");

    airborne(&fleet, 0, 7.5, 5.0);
    airborne(&fleet, 1, 4.0, 5.0);
    goto(&fleet, 0, origin_b, 7.5);
    goto(&fleet, 1, ORIGIN, 4.0);

    let converged = wait_for(Duration::from_secs(45), || {
        fleet.publishes_mode(0, 1, "coop") && fleet.publishes_mode(1, 0, "coop")
    })
    .await;
    let bias_a = fleet.telemetry_of(0).avoid_bias_m;
    let bias_b = fleet.telemetry_of(1).avoid_bias_m;

    // Watch the descender through the maneuver: it must never sink below
    // the fleet floor (3.5 m) minus a small kinematic tolerance.
    let mut min_agl_b = f64::INFINITY;
    for _ in 0..20 {
        min_agl_b = min_agl_b.min(fleet.telemetry_of(1).agl_m);
        tokio::time::sleep(Duration::from_millis(200)).await;
    }

    // Plan math: spread 5.0; the descender gives min(2.0, agl - 3.5) ≈ 0.5
    // and the climber absorbs the rest. Because the two sides plan at
    // different instants over a real network, the climber may observe the
    // peer already AT the floor (descent applied first) and absorb the
    // FULL spread (+5.0) — legitimate floor-awareness, so the pinned
    // invariants are: descender floor-limited, pair opens >= the full
    // spread, and the descender never sinks below the floor.
    let floor_limited = (-0.85..=-0.15).contains(&bias_b);
    let pair_spread = bias_a - bias_b;
    let climber_absorbed = bias_a >= 4.15 && pair_spread >= 4.85;
    let above_floor = min_agl_b >= 3.5 - 0.15;
    let pass = converged && floor_limited && climber_absorbed && above_floor;
    Verdict::new(
        SCENARIO,
        "fleet-floor-respected",
        pass,
        serde_json::json!({
            "coop_both_sides": converged,
            "descender_bias_m": bias_b,
            "climber_bias_m": bias_a,
            "descender_min_agl_m": min_agl_b,
            "floor_agl_m": 3.5,
        }),
    )
    .emit();
    fleet.shutdown().await;
    assert!(
        pass,
        "fleet floor failed: converged={converged} bias_a={bias_a} bias_b={bias_b} min_agl_b={min_agl_b}"
    );
}
