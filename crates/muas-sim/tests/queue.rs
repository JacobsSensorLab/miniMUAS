//! Per-vehicle task queue, integration-level: REAL agent service
//! implementations over sim backends (the dispatch.rs bench pattern),
//! exercising the accept-and-queue contract the dashboard's next-stage
//! queue panel builds on:
//!
//! 1. two investigates submitted at one busy vehicle → the second acks
//!    `code="queued"` and both drain IN ORDER, with the busy string staying
//!    the active task kind throughout (the dashboard's busy→idle completion
//!    fires when the whole queue drains);
//! 2. `queue_reorder` swaps the pending order (execution follows);
//! 3. a reorder displacing the ACTIVE raster splits it — the prioritized
//!    investigate flies first, the `origin=split` continuation then
//!    finishes the raster with every planned capture fired exactly once.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use muas_agent::service_impl::VehicleServiceImpl;
use muas_agent::{AgentShared, SharedBackend, TickableBackend};
use muas_contracts::services::{
    InvestigateRequest, QueueReorderRequest, RasterRequest, VehicleService,
};
use muas_contracts::tasks::{task_origin, task_state, TaskQueueStatus};
use uas_fleet_data::kinds::SearchStatus;
use uas_fleet_node::flight_backend::{SimFlightBackend, SIM_TICK_S};

const ORIGIN: (f64, f64) = (35.0, -90.0);
const M_PER_DEG_LAT: f64 = 111_111.0;

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// One bench agent: real service impl + real queue engine + real flights
/// over a caller-ticked sim backend (dispatch.rs pattern).
struct BenchAgent {
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
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs_f64(SIM_TICK_S));
            loop {
                interval.tick().await;
                lock(&backend).advance(SIM_TICK_S);
            }
        });
        let service = VehicleServiceImpl::new(shared.clone());
        Self { shared, service }
    }

    fn busy(&self) -> String {
        lock(&self.shared.busy).clone()
    }

    fn queue(&self) -> TaskQueueStatus {
        serde_json::from_slice(
            lock(&self.shared.latest_tasks)
                .as_ref()
                .expect("tasks/queue published"),
        )
        .expect("tasks/queue decodes")
    }

    fn task_state(&self, task_id: &str) -> Option<String> {
        self.queue()
            .tasks
            .iter()
            .find(|t| t.task_id == task_id)
            .map(|t| t.state.clone())
    }

    fn search_status(&self) -> Option<SearchStatus> {
        lock(&self.shared.latest_search)
            .as_ref()
            .and_then(|b| serde_json::from_slice(b).ok())
    }
}

fn investigate_req(north_m: f64, turns: f64) -> InvestigateRequest {
    InvestigateRequest {
        lat_deg: ORIGIN.0 + north_m / M_PER_DEG_LAT,
        lon_deg: ORIGIN.1,
        agl_m: 8.0,
        radius_m: 6.0,
        turns,
        sensors: vec!["camera".into()],
        mission_id: "m-queue".into(),
        pattern: String::new(),
    }
}

fn raster_req() -> RasterRequest {
    let dlat = 30.0 / M_PER_DEG_LAT;
    let dlon = 60.0 / (M_PER_DEG_LAT * ORIGIN.0.to_radians().cos());
    RasterRequest {
        agl_m: 8.0,
        spacing_m: 20.0,
        capture_every_m: 15.0,
        speed_m_s: 5.0,
        corners: vec![
            (ORIGIN.0 + dlat, ORIGIN.1 - dlon),
            (ORIGIN.0 + dlat, ORIGIN.1 + dlon),
            (ORIGIN.0 - dlat, ORIGIN.1 + dlon),
            (ORIGIN.0 - dlat, ORIGIN.1 - dlon),
        ],
        ..RasterRequest::default()
    }
}

/// Poll `predicate` on paused tokio time.
async fn wait_until(budget_s: f64, mut predicate: impl FnMut() -> bool) -> bool {
    let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(budget_s);
    loop {
        if predicate() {
            return true;
        }
        if tokio::time::Instant::now() > deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// Two investigates on one vehicle: the first flies, the second queues
/// (accepted, advisory `queued` code) and runs when the first completes —
/// the vehicle stays observably busy for the whole burst.
#[tokio::test(start_paused = true)]
async fn two_investigates_queue_on_one_vehicle_and_drain_in_order() {
    let agent = BenchAgent::new("iuas-01");

    let first = agent.service.investigate(investigate_req(40.0, 1.0)).await;
    assert!(first.accepted && first.code.is_empty(), "{first:?}");
    assert_eq!(agent.busy(), "investigate");

    let second = agent.service.investigate(investigate_req(-40.0, 1.0)).await;
    assert!(second.accepted, "busy must queue, not refuse: {}", second.detail);
    assert_eq!(second.code, "queued");
    assert!(second.detail.contains("tsk-2"), "detail: {}", second.detail);

    // While both are outstanding, telemetry-visible busy stays non-empty
    // (the dashboard's busy→idle completion fires on full drain).
    assert_eq!(agent.queue().tasks[1].state, task_state::PENDING);
    assert!(
        wait_until(600.0, || agent.busy().is_empty()
            && agent.task_state("tsk-1").as_deref() == Some(task_state::DONE)
            && agent.task_state("tsk-2").as_deref() == Some(task_state::DONE))
        .await,
        "queue never drained: {:?}",
        agent.queue()
    );
    // In order: tsk-2 only started after tsk-1 finished (started_ns
    // ordering on the published stream).
    let status = agent.queue();
    let started =
        |id: &str| status.tasks.iter().find(|t| t.task_id == id).and_then(|t| t.started_ns);
    assert!(started("tsk-2") >= started("tsk-1"), "drained in order: {status:?}");
}

/// `queue_reorder` swaps two pending investigates; execution follows the
/// new order (started_ns ordering).
#[tokio::test(start_paused = true)]
async fn reorder_swaps_pending_order() {
    let agent = BenchAgent::new("iuas-02");
    assert!(agent.service.investigate(investigate_req(40.0, 2.0)).await.accepted);
    assert_eq!(agent.service.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
    assert_eq!(agent.service.investigate(investigate_req(60.0, 1.0)).await.code, "queued");

    let ack = agent
        .service
        .queue_reorder(QueueReorderRequest {
            ordered_task_ids: vec!["tsk-1".into(), "tsk-3".into(), "tsk-2".into()],
        })
        .await;
    assert!(ack.accepted, "detail: {}", ack.detail);
    let order: Vec<String> =
        agent.queue().tasks.iter().map(|t| t.task_id.clone()).take(3).collect();
    assert_eq!(order, vec!["tsk-1", "tsk-3", "tsk-2"], "pending order swapped");

    assert!(
        wait_until(900.0, || agent.busy().is_empty()
            && agent.task_state("tsk-2").as_deref() == Some(task_state::DONE))
        .await,
        "queue never drained: {:?}",
        agent.queue()
    );
    let status = agent.queue();
    let started =
        |id: &str| status.tasks.iter().find(|t| t.task_id == id).and_then(|t| t.started_ns);
    assert!(
        started("tsk-3") <= started("tsk-2"),
        "tsk-3 must run before tsk-2 after the reorder: {status:?}"
    );
}

/// Split-resume: a running raster displaced by a reorder suspends, the
/// investigate flies, the continuation finishes the raster — and the
/// search status shows every planned capture eventually fired.
#[tokio::test(start_paused = true)]
async fn split_resume_completes_all_raster_captures() {
    let agent = BenchAgent::new("wuas-01");
    let req = raster_req();
    let plan_total = {
        // The ack names the plan size; parse it back ("raster accepted:
        // N legs, M captures") instead of re-deriving the geometry here.
        let ack = agent.service.raster_search(req).await;
        assert!(ack.accepted, "detail: {}", ack.detail);
        let words: Vec<&str> = ack.detail.split_whitespace().collect();
        let captures_idx = words.iter().position(|w| w.starts_with("captures")).unwrap();
        words[captures_idx - 1].parse::<u64>().unwrap()
    };

    // Mid-sweep with captures already fired…
    assert!(
        wait_until(300.0, || agent
            .search_status()
            .is_some_and(|s| s.frames_captured >= 3 && s.state == "searching"))
        .await,
        "raster never got going"
    );
    // …queue an investigate and move it ahead of the running raster.
    assert_eq!(agent.service.investigate(investigate_req(50.0, 1.0)).await.code, "queued");
    let ack = agent
        .service
        .queue_reorder(QueueReorderRequest {
            ordered_task_ids: vec!["tsk-2".into(), "tsk-1".into()],
        })
        .await;
    assert!(ack.accepted && ack.detail.contains("split"), "{ack:?}");

    // The parent suspends within a control cycle; its terminal search
    // status carries the EXACT number of captures it fired before the
    // split (the investigate flies next, so the status stays put).
    assert!(
        wait_until(30.0, || agent.search_status().is_some_and(|s| s.state == "aborted")).await,
        "parent raster never suspended"
    );
    let fired_before_split = agent.search_status().unwrap().frames_captured;
    assert!(
        fired_before_split >= 3 && fired_before_split < plan_total,
        "split must land mid-raster ({fired_before_split}/{plan_total})"
    );

    // Everything completes: investigate, then the split continuation.
    assert!(
        wait_until(1200.0, || agent.busy().is_empty()
            && agent.task_state("tsk-2").as_deref() == Some(task_state::DONE)
            && agent.task_state("tsk-3").as_deref() == Some(task_state::DONE))
        .await,
        "split-resume never completed: {:?}",
        agent.queue()
    );
    let status = agent.queue();
    let continuation = status.tasks.iter().find(|t| t.task_id == "tsk-3").unwrap();
    assert_eq!(continuation.origin, task_origin::SPLIT);
    assert_eq!(continuation.parent.as_deref(), Some("tsk-1"));

    // The continuation's search finished DONE having fired exactly the
    // captures the parent had not (frames counters are per-flight).
    let resumed = agent.search_status().unwrap();
    assert_eq!(resumed.state, "done", "continuation ran the raster to completion");
    assert_eq!(
        fired_before_split + resumed.frames_captured,
        plan_total,
        "parent + continuation fire every planned capture exactly once"
    );
}
