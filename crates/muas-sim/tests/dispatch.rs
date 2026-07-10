//! Second-target dispatch regression (ROUND-3 §1 item 2), integration-level:
//! the dashboard's mission state machine (`muas_dashboard::mission`, used as
//! a library — the muas-dashboard tests directory is owned by the dashboard
//! wave) driving TWO REAL agent service implementations over sim backends,
//! with the async layer simulated exactly as `muas-dashboard/src/lib.rs`
//! executes `Action::Dispatch` (accept-ack ⇒ `JobResult`), PLUS the busy
//! wiring the fix requires: `Mission::set_vehicle_busy` fed from each
//! vehicle's busy label — the same fact the dashboard's telemetry poller
//! already fetches on every sample.
//!
//! Defect being pinned (2026-07-10 eval, `deployment-run/journals/
//! agent-iuas-0*-17837086*.jsonl`): the accept-ack was mapped to job
//! COMPLETION, so `pump_dispatch`'s busy set went empty the moment a job was
//! accepted; the second target's jobs were dispatched at vehicles still
//! flying target 1, busy-refused, and terminally marked failed — only the
//! first target was ever investigated.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use muas_agent::service_impl::VehicleServiceImpl;
use muas_agent::{AgentShared, SharedBackend, TickableBackend};
use muas_contracts::services::{InvestigateRequest, VehicleService};
use muas_dashboard::mission::{
    Action, DetectOutcome, Detection, JobResult, Mission, MissionConfig,
};
use serde_json::json;
use uas_fleet_node::flight_backend::{SimFlightBackend, SIM_TICK_S};

const ORIGIN: (f64, f64) = (35.0, -90.0);
const M_PER_DEG_LAT: f64 = 111_111.0;

fn north_of(metres: f64) -> (f64, f64) {
    (ORIGIN.0 + metres / M_PER_DEG_LAT, ORIGIN.1)
}

/// One bench agent: real service impl + real busy semantics + real orbit
/// flights over a caller-ticked sim backend.
struct BenchAgent {
    vid: &'static str,
    shared: Arc<AgentShared>,
    service: VehicleServiceImpl,
}

impl BenchAgent {
    fn new(vid: &'static str) -> Self {
        let (journal, _task) = muas_agent::journal::spawn(vid, None, None, None);
        let (cmd_tx, _cmd_rx) = tokio::sync::mpsc::unbounded_channel();
        let sim = SimFlightBackend::new(ORIGIN.0, ORIGIN.1);
        let backend: SharedBackend =
            Arc::new(Mutex::new(Box::new(sim) as Box<dyn TickableBackend>));
        let shared = Arc::new(AgentShared::bench(vid, backend.clone(), journal, cmd_tx));
        // The sim motion ticker the agent itself would spawn.
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
            loop {
                interval.tick().await;
                let mut backend = backend.lock().unwrap();
                backend.advance(SIM_TICK_S);
            }
        });
        let service = VehicleServiceImpl::new(shared.clone());
        Self { vid, shared, service }
    }

    fn busy(&self) -> bool {
        !self
            .shared
            .busy
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .is_empty()
    }
}

/// The dispatch log + refusal counter of the simulated async layer.
#[derive(Default)]
struct Wire {
    dispatched: Vec<(usize, String)>, // (target_index, vehicle)
    busy_refusals: usize,
}

/// Execute mission actions exactly like `muas-dashboard/src/lib.rs`
/// `apply_actions` does for `Action::Dispatch` (ack → JobResult), feeding
/// results back into the machine until it settles.
async fn apply_actions(
    mission: &mut Mission,
    agents: &[&BenchAgent],
    wire: &mut Wire,
    actions: Vec<Action>,
) {
    for action in actions {
        if let Action::Dispatch { target_index, sensor, vehicle, order } = action {
            let agent = agents
                .iter()
                .find(|a| a.vid == vehicle)
                .expect("dispatched to a known vehicle");
            let ack = agent
                .service
                .investigate(InvestigateRequest {
                    lat_deg: order.lat_deg,
                    lon_deg: order.lon_deg,
                    agl_m: order.agl_m,
                    radius_m: order.radius_m,
                    turns: order.turns,
                    sensors: order.sensors.clone(),
                    mission_id: order.mission_id.clone(),
                    pattern: String::new(),
                })
                .await;
            if !ack.accepted && ack.code == "busy" {
                wire.busy_refusals += 1;
            } else if ack.accepted {
                wire.dispatched.push((target_index, vehicle.clone()));
            }
            // The lib.rs mapping under scrutiny: the ack IS the job result.
            let followup = mission.on_job_result(JobResult {
                target_index,
                sensor,
                ok: ack.accepted,
                artifacts: Vec::new(),
                note: ack.detail,
                artifact_items: Vec::new(),
            });
            Box::pin(apply_actions(mission, agents, wire, followup)).await;
        }
    }
}

/// The busy wiring the fix prescribes: push each vehicle's live busy state
/// into the machine (the dashboard side calls this from its telemetry
/// poller — the `busy` field already rides every sample).
async fn sync_busy(mission: &mut Mission, agents: &[&BenchAgent], wire: &mut Wire) {
    for agent in agents {
        let actions = mission.set_vehicle_busy(agent.vid, agent.busy());
        Box::pin(apply_actions(mission, agents, wire, actions)).await;
    }
}

fn mission_machine(iuas: &[&str]) -> Mission {
    let mut cfg = MissionConfig::new("wuas-01", iuas.iter().map(|s| s.to_string()).collect());
    cfg.confirm_count = 1; // one clean hit promotes; dispatch is under test
    Mission::new(cfg)
}

fn start(mission: &mut Mission) -> Vec<Action> {
    mission.start_mission(json!({
        "investigate_sensors": ["camera"],
        "min_confidence": 0.0,
        "target_separation_m": 5.0,
        "orbit_agl_m": 8.0,
        "orbit_radius_m": 6.0,
        "orbit_count": 1.0,
    }))
}

fn hit(mission: &mut Mission, frame: &str, at: (f64, f64)) -> Vec<Action> {
    mission.on_detect_outcome(
        frame,
        DetectOutcome::Hit(Detection {
            object_id: "tennis racket".into(),
            confidence: 0.9,
            lat_deg: at.0,
            lon_deg: at.1,
            offset_m: 1.0,
        }),
    )
}

/// Wait (paused tokio time) until `predicate`, syncing busy hints like the
/// telemetry poller would.
async fn wait_for(
    mission: &mut Mission,
    agents: &[&BenchAgent],
    wire: &mut Wire,
    budget_s: f64,
    mut predicate: impl FnMut(&Mission, &Wire) -> bool,
) -> bool {
    let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(budget_s);
    loop {
        sync_busy(mission, agents, wire).await;
        if predicate(mission, wire) {
            return true;
        }
        if tokio::time::Instant::now() > deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

/// Owner's scenario: two confirmed targets, one busy IUAS + one idle
/// capable IUAS — the idle one must get job 2 IMMEDIATELY (same action
/// batch as the confirmation), with no busy refusal anywhere.
#[tokio::test(start_paused = true)]
async fn second_target_goes_to_the_idle_iuas_immediately() {
    let a = BenchAgent::new("iuas-01");
    let b = BenchAgent::new("iuas-02");
    let agents = [&a, &b];
    let mut mission = mission_machine(&["iuas-01", "iuas-02"]);
    let mut wire = Wire::default();

    let actions = start(&mut mission);
    apply_actions(&mut mission, &agents, &mut wire, actions).await;

    // Target 1 confirms: job 1 -> iuas-01 (the first idle capable vehicle).
    let actions = hit(&mut mission, "frame/1/1", north_of(40.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    assert_eq!(wire.dispatched, vec![(0, "iuas-01".to_string())]);
    assert!(a.busy(), "iuas-01 is actually flying job 1");

    // The poller reports iuas-01 busy; now target 2 confirms mid-flight.
    sync_busy(&mut mission, &agents, &mut wire).await;
    let actions = hit(&mut mission, "frame/1/2", north_of(80.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;

    // Job 2 went straight to the idle capable IUAS — no refusal round-trip.
    assert_eq!(
        wire.dispatched,
        vec![(0, "iuas-01".to_string()), (1, "iuas-02".to_string())],
        "second target must dispatch to the idle IUAS immediately"
    );
    assert!(b.busy(), "iuas-02 is flying job 2");
    assert_eq!(wire.busy_refusals, 0, "no job may be burned on a busy refusal");

    // Both flights complete; with the search over, the mission completes
    // and every job is done (the v2 'queue must drain' contract).
    let drained = wait_for(&mut mission, &agents, &mut wire, 300.0, |_, _| {
        !a.busy() && !b.busy()
    })
    .await;
    assert!(drained, "both investigations must finish");
    let actions = mission.on_search_response(true, "done", 4, "");
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    sync_busy(&mut mission, &agents, &mut wire).await;
    assert_eq!(mission.state, "done");
    let statuses: Vec<String> = mission
        .targets
        .iter()
        .flat_map(|t| t.jobs.iter().map(|j| j.status.clone()))
        .collect();
    assert_eq!(statuses, vec!["done".to_string(), "done".to_string()]);
}

/// Single-IUAS scenario: job 2 must wait, then dispatch the moment job 1's
/// vehicle goes idle — never busy-refused, never marked failed.
#[tokio::test(start_paused = true)]
async fn single_iuas_takes_job_two_when_job_one_completes() {
    let a = BenchAgent::new("iuas-01");
    let agents = [&a];
    let mut mission = mission_machine(&["iuas-01"]);
    let mut wire = Wire::default();

    let actions = start(&mut mission);
    apply_actions(&mut mission, &agents, &mut wire, actions).await;

    let actions = hit(&mut mission, "frame/1/1", north_of(40.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    sync_busy(&mut mission, &agents, &mut wire).await;
    assert_eq!(wire.dispatched, vec![(0, "iuas-01".to_string())]);
    assert!(a.busy());

    // Target 2 confirms while the only IUAS is flying: the job QUEUES.
    let actions = hit(&mut mission, "frame/1/2", north_of(80.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    assert_eq!(wire.dispatched.len(), 1, "job 2 must wait for the busy IUAS");
    assert_eq!(wire.busy_refusals, 0);
    assert_eq!(mission.targets[1].jobs[0].status, "queued");

    // Job 1's flight ends -> busy clears -> the poller's busy sync pumps
    // the queue -> job 2 dispatches to the finishing IUAS.
    let took_job_two = wait_for(&mut mission, &agents, &mut wire, 300.0, |_, wire| {
        wire.dispatched.len() == 2
    })
    .await;
    assert!(took_job_two, "job 2 must dispatch when job 1 completes");
    assert_eq!(wire.dispatched[1], (1, "iuas-01".to_string()));
    assert_eq!(wire.busy_refusals, 0);

    // Drain to completion.
    let drained =
        wait_for(&mut mission, &agents, &mut wire, 300.0, |_, _| !a.busy()).await;
    assert!(drained);
    let actions = mission.on_search_response(true, "done", 4, "");
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    sync_busy(&mut mission, &agents, &mut wire).await;
    assert_eq!(mission.state, "done");
    assert!(mission
        .targets
        .iter()
        .all(|t| t.jobs.iter().all(|j| j.status == "done")));
}

/// Race guard: even WITHOUT the busy hint (stale telemetry), an agent
/// busy-refusal must requeue the job — not terminally fail it — remember
/// the refusing vehicle as busy, and route the job to another idle capable
/// vehicle in the same pump.
#[tokio::test(start_paused = true)]
async fn busy_refusal_requeues_instead_of_failing() {
    let a = BenchAgent::new("iuas-01");
    let b = BenchAgent::new("iuas-02");
    let agents = [&a, &b];
    let mut mission = mission_machine(&["iuas-01", "iuas-02"]);
    let mut wire = Wire::default();

    let actions = start(&mut mission);
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    let actions = hit(&mut mission, "frame/1/1", north_of(40.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    assert!(a.busy());

    // NO sync_busy here: the machine still believes iuas-01 is idle (the
    // pre-fix world). Target 2 confirms; the machine dispatches at the
    // still-flying iuas-01, which refuses busy — the fix must requeue and
    // re-pump onto iuas-02 within the same action cascade.
    let actions = hit(&mut mission, "frame/1/2", north_of(80.0));
    apply_actions(&mut mission, &agents, &mut wire, actions).await;
    assert_eq!(wire.busy_refusals, 1, "the stale dispatch was refused");
    assert_eq!(
        wire.dispatched.last(),
        Some(&(1, "iuas-02".to_string())),
        "the requeued job must land on the idle capable IUAS"
    );
    assert!(
        !mission
            .targets
            .iter()
            .flat_map(|t| t.jobs.iter())
            .any(|j| j.status == "failed"),
        "a busy refusal must never terminally fail a job"
    );
    // The refusal taught the machine that iuas-01 is busy.
    assert_eq!(mission.vehicle_busy.get("iuas-01"), Some(&true));
}
