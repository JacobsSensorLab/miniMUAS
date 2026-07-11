//! Mission state machine unit tests — the v2 semantics ported as scripted
//! detections: confirm-count, best-localized position, multi-target
//! multi-sensor dispatch, and the completion predicate.

use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};

use muas_contracts::services::Ack;
use muas_dashboard::mission::{
    Action, DetectOutcome, Detection, InvestigateOrder, JobResult, Mission, MissionConfig,
    RasterOrder,
};
use muas_dashboard::providers::{
    BoxFuture, CmdResult, Commander, ScriptedCommander, ScriptedDetector,
};
use muas_dashboard::{DashConfig, Dashboard};
use serde_json::{json, Value};

const LAT: f64 = 35.1208;
const LON: f64 = -89.9347;
/// ~1 m of latitude.
const M_LAT: f64 = 1.0 / 111_111.0;

fn machine(iuas: &[&str], confirm: u32) -> Mission {
    let mut cfg = MissionConfig::new("wuas-01", iuas.iter().map(|s| s.to_string()).collect());
    cfg.confirm_count = confirm;
    cfg.clock = Arc::new(|| 1_000.0);
    Mission::new(cfg)
}

fn params(sensors: &[&str]) -> Value {
    json!({
        "area": { "mode": "center", "center_lat": LAT, "center_lon": LON,
                  "width_m": 40.0, "height_m": 24.0 },
        "agl_m": 6.0, "leg_spacing_m": 5.0, "capture_every_m": 4.0,
        "speed_m_s": 2.0, "object_query": "tennis racket",
        "min_confidence": 0.3, "target_separation_m": 5.0,
        "orbit_agl_m": 8.0, "orbit_radius_m": 6.0, "orbit_count": 1.0,
        "max_duration_s": 600.0,
        "investigate_sensors": sensors,
    })
}

fn start(m: &mut Mission, sensors: &[&str]) -> Vec<Action> {
    m.start_mission(params(sensors))
}

fn hit(m: &mut Mission, frame: &str, lat: f64, lon: f64, conf: f64, offset: f64) -> Vec<Action> {
    let mut actions = m.on_new_frame(frame);
    actions.extend(m.on_detect_outcome(
        frame,
        DetectOutcome::Hit(Detection {
            object_id: "tennis racket".into(),
            confidence: conf,
            lat_deg: lat,
            lon_deg: lon,
            offset_m: offset,
        }),
    ));
    actions
}

fn kinds(actions: &[Action]) -> Vec<String> {
    actions
        .iter()
        .filter_map(|a| match a {
            Action::Emit(v) => v.get("kind").and_then(Value::as_str).map(str::to_string),
            _ => None,
        })
        .collect()
}

fn dispatches(actions: &[Action]) -> Vec<(usize, String, String)> {
    actions
        .iter()
        .filter_map(|a| match a {
            Action::Dispatch { target_index, sensor, vehicle, .. } => {
                Some((*target_index, sensor.clone(), vehicle.clone()))
            }
            _ => None,
        })
        .collect()
}

fn caps(m: &mut Mission, vid: &str, sensors: &[&str]) {
    let set: BTreeSet<String> = sensors.iter().map(|s| s.to_string()).collect();
    m.set_capabilities(vid, set, serde_json::Value::Null);
}

// ───────────────────────────── confirm-then-queue ───────────────────────────

#[test]
fn confirm_count_gates_promotion() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);

    // First hit: candidate only — no target, no dispatch.
    let a1 = hit(&mut m, "/f/1", LAT, LON, 0.9, 3.0);
    assert!(kinds(&a1).contains(&"detect.candidate".into()));
    assert!(!kinds(&a1).contains(&"mission.target_found".into()));
    assert!(m.targets.is_empty());

    // The SAME frame again reinforces nothing (hits are counted per frame).
    let a_dup = m.on_detect_outcome(
        "/f/1",
        DetectOutcome::Hit(Detection {
            object_id: "tennis racket".into(),
            confidence: 0.95,
            lat_deg: LAT,
            lon_deg: LON,
            offset_m: 2.0,
        }),
    );
    assert!(!kinds(&a_dup).contains(&"mission.target_found".into()));
    assert!(m.targets.is_empty(), "same-frame hits must not promote");

    // A second frame within target_separation_m promotes + dispatches.
    let a2 = hit(&mut m, "/f/2", LAT + M_LAT, LON, 0.8, 4.0);
    assert!(kinds(&a2).contains(&"mission.target_found".into()));
    assert_eq!(dispatches(&a2), vec![(0, "camera".into(), "iuas-01".into())]);
    assert_eq!(m.targets.len(), 1);
    assert_eq!(m.targets[0].status, "investigating");
}

#[test]
fn hits_below_min_confidence_never_reinforce() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    let a = hit(&mut m, "/f/1", LAT, LON, 0.1, 1.0);
    assert_eq!(kinds(&a), vec!["detect.sent", "detect.hit"]);
    assert!(m.targets.is_empty(), "sub-threshold hit must not promote");
}

#[test]
fn distant_hits_form_separate_candidates_and_targets() {
    let mut m = machine(&["iuas-01", "iuas-02"], 1);
    caps(&mut m, "iuas-02", &["camera"]);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    // 20 m north: beyond target_separation_m of 5 — a second object.
    let a = hit(&mut m, "/f/2", LAT + 20.0 * M_LAT, LON, 0.9, 1.0);
    assert_eq!(m.targets.len(), 2);
    assert_eq!(
        dispatches(&a),
        vec![(1, "camera".into(), "iuas-02".into())],
        "second target goes to the second idle IUAS"
    );
}

/// The 2026-07-10 field report read "the detect stream stops right after
/// the first dispatch". Detection must run continuously and INDEPENDENTLY
/// of dispatch/investigation lifecycles: with confirm-count 2 and job 1
/// held in flight for the whole test (accept ack only — under the
/// completion rework the job stays `investigating` until its vehicle goes
/// idle), every later frame still fans out `detect.sent` + a detector
/// request, and a second target confirms on two distinct frames and
/// dispatches to the other idle IUAS while job 1 is still flying.
#[test]
fn detection_outlives_dispatch_and_confirms_a_second_target() {
    let mut m = machine(&["iuas-01", "iuas-02"], 2);
    caps(&mut m, "iuas-01", &["camera"]);
    caps(&mut m, "iuas-02", &["camera"]);
    start(&mut m, &["camera"]);

    let sent_count =
        |a: &[Action]| kinds(a).iter().filter(|k| k.as_str() == "detect.sent").count();

    // Target 1 confirms on two distinct frames → dispatch #1.
    hit(&mut m, "/f/1", LAT, LON, 0.9, 2.0);
    let a = hit(&mut m, "/f/2", LAT + M_LAT, LON, 0.9, 2.5);
    assert_eq!(dispatches(&a), vec![(0, "camera".into(), "iuas-01".into())]);
    m.on_job_accepted(0, "camera", "carrot-orbit accepted");
    m.set_vehicle_busy("iuas-01", true); // job 1 stays in flight from here on

    // The raster keeps producing frames: every one must still emit
    // detect.sent and reach the detector (no slot/backpressure keyed on
    // the in-flight job), and outcomes must keep draining the counters.
    let mut sent_after_dispatch = 0;
    for i in 3..=8 {
        let frame = format!("/f/{i}");
        let a = m.on_new_frame(&frame);
        assert_eq!(sent_count(&a), 1, "frame {i} must fan out while job 1 flies");
        assert!(
            a.iter().any(|x| matches!(x, Action::Detect { .. })),
            "frame {i} must reach the detector"
        );
        sent_after_dispatch += sent_count(&a);
        m.on_detect_outcome(&frame, DetectOutcome::Miss(String::new()));
        assert_eq!(m.detects_pending, 0, "outcome {i} releases its pending slot");
    }
    assert_eq!(m.detects_done, 8);

    // Target 2 (20 m away) confirms on two distinct frames → dispatch #2
    // to the idle iuas-02 WHILE job 1 is still investigating.
    let lat2 = LAT + 20.0 * M_LAT;
    let a = hit(&mut m, "/f/9", lat2, LON, 0.9, 2.0);
    sent_after_dispatch += sent_count(&a);
    assert!(kinds(&a).contains(&"detect.candidate".into()));
    assert!(!kinds(&a).contains(&"mission.target_found".into()));
    let a = hit(&mut m, "/f/10", lat2 + M_LAT, LON, 0.9, 2.5);
    sent_after_dispatch += sent_count(&a);
    assert!(kinds(&a).contains(&"mission.target_found".into()));
    assert_eq!(dispatches(&a), vec![(1, "camera".into(), "iuas-02".into())]);

    assert_eq!(m.targets.len(), 2, "both targets promoted (2 hits each)");
    assert_eq!(
        m.targets[0].jobs[0].status, "investigating",
        "job 1 still in flight when target 2 dispatches"
    );
    assert_eq!(m.targets[1].jobs[0].status, "investigating");
    assert_eq!(
        sent_after_dispatch, 8,
        "detect.sent kept increasing after dispatch #1"
    );
    assert_eq!(m.detects_done + m.detects_pending, 10, "every frame accounted for");
}

// ───────────────────────────── best-localized ───────────────────────────────

#[test]
fn best_localized_sighting_wins_position() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    // Keep the target queued so refinement is allowed.
    m.set_enabled("iuas-01", false);

    // Edge-of-frame sighting first (offset 5), then a near-nadir one
    // (offset 2) at a slightly different position within separation.
    hit(&mut m, "/f/1", LAT, LON, 0.95, 5.0);
    let near_lat = LAT + 2.0 * M_LAT;
    hit(&mut m, "/f/2", near_lat, LON, 0.5, 2.0);
    assert_eq!(m.targets.len(), 1);
    let t = &m.targets[0];
    assert_eq!(t.lat, near_lat, "position from smallest offset, not highest confidence");
    assert_eq!(t.best_offset, 2.0);
    assert_eq!(t.confidence, 0.95, "confidence still tracks the max");
    assert_eq!(t.frame, "/f/2");

    // A better-localized later hit refines the queued target...
    let refine_lat = LAT + 1.0 * M_LAT;
    let a = hit(&mut m, "/f/3", refine_lat, LON, 0.4, 1.0);
    assert!(kinds(&a).contains(&"target.updated".into()));
    assert_eq!(m.targets[0].lat, refine_lat);

    // ...and a WORSE-localized one is absorbed without moving it.
    let a = hit(&mut m, "/f/4", LAT, LON, 0.99, 4.0);
    assert!(!kinds(&a).contains(&"target.updated".into()));
    assert_eq!(m.targets[0].lat, refine_lat);
    assert_eq!(m.targets[0].confidence, 0.99);
}

// ───────────────────────────── multi-sensor dispatch ────────────────────────

#[test]
fn jobs_split_across_capability_matching_vehicles() {
    let mut m = machine(&["iuas-01", "iuas-02"], 1);
    caps(&mut m, "iuas-01", &["camera"]);
    caps(&mut m, "iuas-02", &["audio"]);
    start(&mut m, &["camera", "audio"]);

    let a = hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    let d = dispatches(&a);
    assert_eq!(
        d,
        vec![
            (0, "camera".into(), "iuas-01".into()),
            (0, "audio".into(), "iuas-02".into()),
        ],
        "camera drone and microphone drone work the same target concurrently"
    );
    assert_eq!(m.targets[0].jobs.len(), 2);

    // Both jobs complete → target done; search done → mission completes.
    for sensor in ["camera", "audio"] {
        m.on_job_result(JobResult {
            target_index: 0,
            sensor: sensor.into(),
            ok: true,
            artifacts: vec![format!("/muas/v3/mission/m/x/{sensor}")],
            note: String::new(),
            artifact_items: vec![],
        });
    }
    assert_eq!(m.targets[0].status, "done");
    assert_eq!(m.targets[0].artifacts.len(), 2);
    let a = m.on_search_response(true, "done", 42, "");
    assert!(kinds(&a).contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
}

#[test]
fn dual_sensor_drone_flies_jobs_back_to_back() {
    let mut m = machine(&["iuas-01"], 1);
    caps(&mut m, "iuas-01", &["audio", "camera"]);
    start(&mut m, &["camera", "audio"]);

    // One idle dual-sensor drone: only ONE job dispatches at a time.
    let a = hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert_eq!(dispatches(&a), vec![(0, "camera".into(), "iuas-01".into())]);
    let queued: Vec<_> =
        m.targets[0].jobs.iter().filter(|j| j.status == "queued").collect();
    assert_eq!(queued.len(), 1);
    assert_eq!(queued[0].sensor, "audio");

    // First job lands → the second dispatches to the same drone.
    let a = m.on_job_result(JobResult {
        target_index: 0,
        sensor: "camera".into(),
        ok: true,
        artifacts: vec![],
        note: String::new(),
        artifact_items: vec![],
    });
    assert_eq!(dispatches(&a), vec![(0, "audio".into(), "iuas-01".into())]);
}

#[test]
fn reenabling_a_vehicle_pumps_queued_targets() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    m.set_enabled("iuas-01", false);
    let a = hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert!(dispatches(&a).is_empty(), "disabled vehicle must not launch");
    assert_eq!(m.targets[0].status, "queued");

    let a = m.set_enabled("iuas-01", true);
    assert!(kinds(&a).contains(&"vehicle.enabled".into()));
    assert_eq!(dispatches(&a), vec![(0, "camera".into(), "iuas-01".into())]);
}

// ───────────────────────────── completion predicate ─────────────────────────

#[test]
fn completion_waits_for_in_flight_jobs() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert_eq!(m.targets[0].jobs[0].status, "investigating");

    // Raster done, job in flight: investigating, NOT complete.
    let a = m.on_search_response(true, "done", 7, "");
    assert!(!kinds(&a).contains(&"mission.completed".into()));
    assert_eq!(m.state, "investigating");

    // Job lands: nothing in flight, nothing serviceable → complete.
    let a = m.on_job_result(JobResult {
        target_index: 0,
        sensor: "camera".into(),
        ok: true,
        artifacts: vec![],
        note: String::new(),
        artifact_items: vec![],
    });
    let k = kinds(&a);
    assert!(k.contains(&"target.completed".into()));
    assert!(k.contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
}

#[test]
fn unserviceable_jobs_do_not_hold_the_mission_open() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    m.set_enabled("iuas-01", false);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert_eq!(m.targets[0].jobs[0].status, "queued");

    // Raster done, the only capable vehicle disabled: the queued job can
    // never be served — the mission completes with the unserviceable note.
    let actions = m.on_search_response(true, "done", 3, "");
    let completed = actions
        .iter()
        .find_map(|a| match a {
            Action::Emit(v) if v.get("kind") == Some(&json!("mission.completed")) => Some(v),
            _ => None,
        })
        .expect("mission.completed emitted");
    assert_eq!(completed["note"], json!("unserviceable-jobs:1"));
    assert_eq!(m.state, "done");
}

/// The real completion fix: accept acks only mark jobs in flight; each job
/// completes on its assigned vehicle's busy→idle transition, so a raster
/// that finishes first leaves the mission open until BOTH investigations'
/// vehicles actually go idle.
#[test]
fn mission_stays_open_until_both_investigating_vehicles_go_idle() {
    let mut m = machine(&["iuas-01", "iuas-02"], 1);
    caps(&mut m, "iuas-01", &["camera"]);
    caps(&mut m, "iuas-02", &["camera"]);
    start(&mut m, &["camera"]);

    // Two targets, one job each, on the two inspectors.
    let a1 = hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert_eq!(dispatches(&a1), vec![(0, "camera".into(), "iuas-01".into())]);
    m.on_job_accepted(0, "camera", "carrot-orbit accepted");
    m.set_vehicle_busy("iuas-01", true);
    let a2 = hit(&mut m, "/f/2", LAT + 20.0 * M_LAT, LON, 0.9, 1.0);
    assert_eq!(dispatches(&a2), vec![(1, "camera".into(), "iuas-02".into())]);
    m.on_job_accepted(1, "camera", "carrot-orbit accepted");
    m.set_vehicle_busy("iuas-02", true);

    // Raster completes FIRST: both jobs still in flight — the mission must
    // stay open (the old accept-ack⇒done mapping completed it right here).
    let a = m.on_search_response(true, "done", 9, "");
    assert!(!kinds(&a).contains(&"mission.completed".into()));
    assert_eq!(m.state, "investigating");
    assert!(m
        .targets
        .iter()
        .all(|t| t.jobs[0].status == "investigating"));

    // First vehicle idles: its job completes, the mission is STILL open.
    let a = m.set_vehicle_busy("iuas-01", false);
    let k = kinds(&a);
    assert!(k.contains(&"target.job_completed".into()));
    assert!(!k.contains(&"mission.completed".into()), "one investigation still flying");
    assert_eq!(m.targets[0].jobs[0].status, "done");
    assert_eq!(m.targets[1].jobs[0].status, "investigating");

    // Second vehicle idles: now — and only now — the mission completes.
    let a = m.set_vehicle_busy("iuas-02", false);
    let k = kinds(&a);
    assert!(k.contains(&"target.job_completed".into()));
    assert!(k.contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
}

/// A busy hint that never saw the vehicle busy (telemetry lag right after
/// dispatch) must not complete the job: completion requires a REAL
/// busy→idle transition.
#[test]
fn idle_report_without_prior_busy_never_completes_a_job() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    m.on_job_accepted(0, "camera", "accepted");
    // Poller reports idle (stale sample from before the flight started).
    m.set_vehicle_busy("iuas-01", false);
    assert_eq!(m.targets[0].jobs[0].status, "investigating", "no prior busy: no completion");
    // The real flight shows up, then ends.
    m.set_vehicle_busy("iuas-01", true);
    let a = m.set_vehicle_busy("iuas-01", false);
    assert!(kinds(&a).contains(&"target.job_completed".into()));
    assert_eq!(m.targets[0].jobs[0].status, "done");
}

/// Single-target abort path: operator task_abort at the vehicle →
/// busy→idle → the job lands `done` with the aborted note → the mission
/// completes truthfully.
#[test]
fn task_abort_completes_the_job_with_an_aborted_note() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    m.on_job_accepted(0, "camera", "accepted");
    m.set_vehicle_busy("iuas-01", true);
    let a = m.on_search_response(true, "done", 3, "");
    assert!(!kinds(&a).contains(&"mission.completed".into()));

    // The dashboard sent task_abort("investigate") and the agent acked it;
    // the vehicle's busy label clears within one cycle.
    m.note_task_abort("iuas-01");
    let a = m.set_vehicle_busy("iuas-01", false);
    let completed = a
        .iter()
        .find_map(|x| match x {
            Action::Emit(v) if v.get("kind") == Some(&json!("target.job_completed")) => Some(v),
            _ => None,
        })
        .expect("job completes on busy→idle");
    assert!(
        completed["note"].as_str().unwrap().contains("aborted"),
        "outcome notes the operator abort: {}",
        completed["note"]
    );
    assert!(kinds(&a).contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
    assert_eq!(m.targets[0].jobs[0].status, "done");
}

/// Detection-panel ✕ on a QUEUED job: pure mission-machine cancellation —
/// the job leaves the queue (`cancelled`, `target.job_cancelled`) and no
/// longer blocks the completion predicate; non-queued jobs are untouched.
#[test]
fn queued_job_cancel_removes_it_from_queue_and_completion() {
    let mut m = machine(&["iuas-01"], 1);
    caps(&mut m, "iuas-01", &["audio", "camera"]);
    start(&mut m, &["camera", "audio"]);

    // One dual-sensor drone: camera flies, audio queues behind it.
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    m.on_job_accepted(0, "camera", "accepted");
    m.set_vehicle_busy("iuas-01", true);
    assert_eq!(m.targets[0].jobs[1].status, "queued");

    // Cancelling the IN-FLIGHT camera job here is a no-op (that path is
    // task_abort at the vehicle, not a queue edit).
    assert!(m.cancel_job(0, "camera").is_empty());
    assert_eq!(m.targets[0].jobs[0].status, "investigating");

    // Cancel the queued audio job: event + cancelled state.
    let a = m.cancel_job(0, "audio");
    assert!(kinds(&a).contains(&"target.job_cancelled".into()));
    assert_eq!(m.targets[0].jobs[1].status, "cancelled");
    // Idempotent: a second cancel finds nothing queued.
    assert!(m.cancel_job(0, "audio").is_empty());

    // Raster done + camera lands: the cancelled job neither dispatches
    // nor blocks completion (it was the only remaining queued work).
    m.on_search_response(true, "done", 2, "");
    let a = m.set_vehicle_busy("iuas-01", false);
    let k = kinds(&a);
    assert!(
        !a.iter().any(|x| matches!(x, Action::Dispatch { .. })),
        "a cancelled job must never dispatch"
    );
    assert!(k.contains(&"mission.completed".into()));
    assert_eq!(m.targets[0].jobs[0].status, "done");
    assert_eq!(m.targets[0].jobs[1].status, "cancelled");
    assert_eq!(m.targets[0].status, "done", "one done job carries the target");
}

#[test]
fn empty_mission_completes_on_search_end() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    let a = m.on_search_response(true, "done", 12, "");
    assert!(kinds(&a).contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
    // Idempotent: a repeated terminal status does nothing.
    assert!(m.on_search_response(true, "done", 12, "").is_empty());
}

// ───────────────────── unconfirmed candidates (end-of-raster) ───────────────

/// The geometry-starved scenario: confirm-count 2 with a camera footprint
/// narrower than the leg spacing — a real target is only ever seen on ONE
/// pass, so it ends the raster stuck as a 1-hit candidate. At search end it
/// must surface as `target.unconfirmed` (not auto-dispatched) while the
/// mission still completes.
#[test]
fn one_hit_candidate_surfaces_unconfirmed_and_mission_completes() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.7, 2.0); // 1/2 hits: candidate only
    assert!(m.targets.is_empty());

    let actions = m.on_search_response(true, "done", 5, "");
    let k = kinds(&actions);
    assert!(k.contains(&"target.unconfirmed".into()));
    assert!(
        k.contains(&"mission.completed".into()),
        "unconfirmed candidates must not block completion"
    );
    assert!(dispatches(&actions).is_empty(), "never auto-dispatched");
    assert_eq!(m.state, "done");
    assert!(m.targets.is_empty());

    // Event payload carries the disposition facts (and lat/lon for the map).
    let evt = actions
        .iter()
        .find_map(|a| match a {
            Action::Emit(v) if v.get("kind") == Some(&json!("target.unconfirmed")) => Some(v),
            _ => None,
        })
        .expect("target.unconfirmed emitted");
    assert_eq!(evt["index"], json!(0));
    assert_eq!(evt["hits"], json!(1));
    assert_eq!(evt["need"], json!(2));
    assert_eq!(evt["lat"], json!(LAT));
    assert_eq!(evt["lon"], json!(LON));
    assert!(evt.get("confidence").is_some());

    // Surfaced in the hello payload so a page refresh keeps the cards.
    let hello = m.hello_mission();
    assert_eq!(hello["unconfirmed"][0]["status"], json!("unconfirmed"));
    assert_eq!(hello["unconfirmed"][0]["hits"], json!(1));

    // Idempotent finish: a repeated terminal status re-emits nothing.
    assert!(m.on_search_response(true, "done", 5, "").is_empty());
    assert_eq!(m.unconfirmed.len(), 1);
}

/// "Investigate anyway": the promotion rides the NORMAL queue/dispatch
/// path, the completed mission reopens, and the completion predicate
/// re-runs against the re-armed job and closes the mission again.
#[test]
fn promote_unconfirmed_dispatches_and_recompletes_the_mission() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.7, 2.0);
    m.on_search_response(true, "done", 5, "");
    assert_eq!(m.state, "done");

    let actions = m.promote_unconfirmed(0);
    let k = kinds(&actions);
    assert!(k.contains(&"target.promoted".into()));
    assert!(k.contains(&"mission.target_found".into()));
    assert_eq!(
        dispatches(&actions),
        vec![(0, "camera".into(), "iuas-01".into())],
        "promotion flows through the normal dispatch pump"
    );
    assert_eq!(m.state, "investigating", "completed mission reopens");
    assert_eq!(m.targets.len(), 1);
    assert_eq!(m.targets[0].status, "investigating");
    // Provenance rides the target_found event; the honest hit count too.
    let found = actions
        .iter()
        .find_map(|a| match a {
            Action::Emit(v) if v.get("kind") == Some(&json!("mission.target_found")) => Some(v),
            _ => None,
        })
        .expect("target found");
    assert_eq!(found["promoted_from"], json!(0));
    assert_eq!(found["hits"], json!(1));

    // Promotion is one-shot.
    assert!(m.promote_unconfirmed(0).is_empty(), "second promote is a no-op");
    assert!(m.dismiss_unconfirmed(0).is_empty(), "promoted is past dismissal");

    // The re-armed job lands → the predicate completes the mission AGAIN.
    let actions = m.on_job_result(JobResult {
        target_index: 0,
        sensor: "camera".into(),
        ok: true,
        artifacts: vec![],
        note: String::new(),
        artifact_items: vec![],
    });
    let k = kinds(&actions);
    assert!(k.contains(&"target.completed".into()));
    assert!(k.contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
}

/// Dismiss is terminal: the candidate leaves the disposition pool for good
/// and the (already completed) mission is untouched.
#[test]
fn dismiss_unconfirmed_stays_terminal() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.7, 2.0);
    m.on_search_response(true, "done", 5, "");

    let actions = m.dismiss_unconfirmed(0);
    assert_eq!(kinds(&actions), vec!["target.dismissed"]);
    assert_eq!(m.state, "done", "dismissal never reopens the mission");
    assert!(m.targets.is_empty());
    assert_eq!(m.hello_mission()["unconfirmed"][0]["status"], json!("dismissed"));

    // Terminal: neither a second dismiss nor a late promote does anything.
    assert!(m.dismiss_unconfirmed(0).is_empty());
    assert!(m.promote_unconfirmed(0).is_empty(), "dismissed can never launch");
    assert!(m.targets.is_empty());
}

/// Promotion while another investigation is still flying (state
/// `investigating`, not `done`): the unconfirmed candidate queues into the
/// same pool and the mission completes only once EVERYTHING lands.
#[test]
fn promote_while_other_jobs_still_fly_joins_the_queue() {
    let mut m = machine(&["iuas-01"], 2);
    caps(&mut m, "iuas-01", &["camera"]);
    start(&mut m, &["camera"]);
    // Target 0 confirms (2 frames) and flies; a lone extra hit 20 m away
    // stays a 1-hit candidate.
    hit(&mut m, "/f/1", LAT, LON, 0.9, 2.0);
    hit(&mut m, "/f/2", LAT + M_LAT, LON, 0.9, 2.5);
    m.on_job_accepted(0, "camera", "accepted");
    m.set_vehicle_busy("iuas-01", true);
    hit(&mut m, "/f/3", LAT + 20.0 * M_LAT, LON, 0.8, 1.0);

    let actions = m.on_search_response(true, "done", 9, "");
    assert!(kinds(&actions).contains(&"target.unconfirmed".into()));
    assert_eq!(m.state, "investigating");

    // Promote mid-flight: the only capable vehicle is busy, so the new job
    // queues (no dispatch yet) — and the mission must NOT complete.
    let actions = m.promote_unconfirmed(0);
    assert!(kinds(&actions).contains(&"mission.target_found".into()));
    assert!(dispatches(&actions).is_empty(), "busy vehicle: job queues");
    assert_eq!(m.targets[1].jobs[0].status, "queued");

    // Vehicle idles: job 0 completes, the promoted job dispatches next —
    // the completion predicate treats the re-armed job like any other.
    let actions = m.set_vehicle_busy("iuas-01", false);
    let k = kinds(&actions);
    assert!(k.contains(&"target.job_completed".into()));
    assert_eq!(
        dispatches(&actions),
        vec![(1, "camera".into(), "iuas-01".into())]
    );
    assert!(!k.contains(&"mission.completed".into()), "promoted job in flight");

    m.set_vehicle_busy("iuas-01", true);
    let actions = m.set_vehicle_busy("iuas-01", false);
    assert!(kinds(&actions).contains(&"mission.completed".into()));
    assert_eq!(m.state, "done");
}

/// Confirmed targets and aborted missions never mint unconfirmed entries.
#[test]
fn confirmed_targets_and_aborts_never_surface_unconfirmed() {
    // confirm-count 1: the hit promotes immediately — nothing left over.
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    let actions = m.on_search_response(true, "done", 3, "");
    assert!(!kinds(&actions).contains(&"target.unconfirmed".into()));
    assert!(m.unconfirmed.is_empty());

    // Aborted mission: leftover candidates drop silently at search end.
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    m.note_all_command();
    let actions = m.on_search_response(true, "done", 3, "");
    assert!(!kinds(&actions).contains(&"target.unconfirmed".into()));
    assert!(m.unconfirmed.is_empty());
    assert!(m.promote_unconfirmed(0).is_empty());
}

/// A new mission clears the previous raster's disposition pool.
#[test]
fn start_mission_resets_unconfirmed() {
    let mut m = machine(&["iuas-01"], 2);
    start(&mut m, &["camera"]);
    hit(&mut m, "/f/1", LAT, LON, 0.7, 2.0);
    m.on_search_response(true, "done", 2, "");
    assert_eq!(m.unconfirmed.len(), 1);
    start(&mut m, &["camera"]);
    assert!(m.unconfirmed.is_empty());
    assert_eq!(m.hello_mission()["unconfirmed"], json!([]));
}

// ───────────────────────────── aborts & rejects ─────────────────────────────

#[test]
fn abort_stops_dispatch_and_start_is_rejected_mid_mission() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);

    // Starting again mid-search is rejected.
    let a = m.start_mission(params(&["camera"]));
    assert_eq!(kinds(&a), vec!["mission.rejected"]);

    // RTL-all aborts; a confirmed hit afterwards must not launch anything.
    m.note_all_command();
    assert_eq!(m.state, "aborted");
    let a = hit(&mut m, "/f/1", LAT, LON, 0.9, 1.0);
    assert!(dispatches(&a).is_empty(), "aborted mission must not dispatch");
    assert!(m.targets.is_empty(), "aborted mission must not confirm targets");
}

#[test]
fn wuas_rtl_during_search_aborts() {
    let mut m = machine(&["iuas-01"], 1);
    start(&mut m, &["camera"]);
    m.note_flight_command("iuas-01", "rtl");
    assert_eq!(m.state, "searching", "IUAS rtl does not abort the search");
    m.note_flight_command("wuas-01", "hold");
    assert_eq!(m.state, "searching", "hold does not abort");
    m.note_flight_command("wuas-01", "rtl");
    assert_eq!(m.state, "aborted");
}

// ───────────────────────────── async plumbing ───────────────────────────────

/// End to end through the Dashboard action executor: a scripted detection
/// flows detect→confirm→dispatch→job-result→completion with the commander
/// and detector both faked — no NDN anywhere.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn scripted_detection_drives_dispatch_through_the_executor() {
    let detector = Arc::new(ScriptedDetector::default());
    detector.script(
        "/muas/v3/mission/m/wuas-01/camera/cam0/frame/9/1",
        DetectOutcome::Hit(Detection {
            object_id: "tennis racket".into(),
            confidence: 0.9,
            lat_deg: LAT,
            lon_deg: LON,
            offset_m: 1.0,
        }),
    );
    let commander = Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok_detail(
        "accepted; execution stubbed",
    ))));
    let config = DashConfig {
        record_dir: None,
        confirm_count: 1,
        ..DashConfig::default()
    };
    let dash = Arc::new(Dashboard::new(config, detector, commander.clone()));

    let started = dash.handle_command(&json!({
        "cmd": "start_mission",
        "params": params(&["camera"]),
    }));
    assert!(started.is_none(), "start_mission has no direct reply");

    // The search poller would feed this frame; drive it directly.
    let actions = dash.with_mission(|m| {
        m.on_new_frame("/muas/v3/mission/m/wuas-01/camera/cam0/frame/9/1")
    });
    dash.apply_actions(actions);

    // Detect → hit → target → dispatch → ACCEPT ack. The job is now in
    // flight (`investigating`) — the accept ack is NOT completion.
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let dispatched = {
            let calls = commander.calls.lock().unwrap().clone();
            calls.contains(&("iuas-01".to_string(), "investigate".to_string()))
        };
        if dispatched {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "job never dispatched");
        tokio::time::sleep(std::time::Duration::from_millis(25)).await;
    }
    tokio::time::sleep(std::time::Duration::from_millis(50)).await; // ack lands
    assert_eq!(
        dash.with_mission(|m| m.targets[0].jobs[0].status.clone()),
        "investigating",
        "accept ack leaves the job in flight, not done"
    );

    // Completion rides the vehicle's busy→idle transition (the telemetry
    // poller feeds set_vehicle_busy in production).
    let actions = dash.with_mission(|m| m.set_vehicle_busy("iuas-01", true));
    dash.apply_actions(actions);
    let actions = dash.with_mission(|m| m.set_vehicle_busy("iuas-01", false));
    dash.apply_actions(actions);
    assert_eq!(dash.with_mission(|m| m.targets[0].status.clone()), "done");

    // Raster ends → completion predicate closes the mission.
    let actions = dash.with_mission(|m| m.on_search_response(true, "done", 1, ""));
    dash.apply_actions(actions);
    assert_eq!(dash.mission_state(), "done");

    let calls = commander.calls.lock().unwrap().clone();
    assert!(calls.contains(&("wuas-01".to_string(), "raster-search".to_string())));
    assert!(calls.contains(&("iuas-01".to_string(), "investigate".to_string())));
}

/// Worst-case transport: the investigate call (long-timeout client) never
/// answers. Dispatch is fire-and-forget in the action executor, so the
/// frame→detect loop must keep running at full rate while that future
/// hangs — and a second target must still confirm (2 distinct frames) and
/// dispatch to the other idle IUAS. This pins the "detection stalls after
/// the first dispatch" failure mode at the async layer.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn frames_keep_flowing_while_the_investigate_call_hangs() {
    /// Acks everything EXCEPT investigate, which never resolves.
    struct HangingInvestigate {
        calls: Mutex<Vec<(String, String)>>,
    }
    impl HangingInvestigate {
        fn log(&self, vehicle: String, op: &str) -> BoxFuture<CmdResult> {
            self.calls
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .push((vehicle, op.to_string()));
            Box::pin(async { CmdResult::Ack(Ack::ok_detail("accepted; execution stubbed")) })
        }
        fn investigated(&self, vehicle: &str) -> bool {
            self.calls
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .iter()
                .any(|(v, op)| v == vehicle && op == "investigate")
        }
    }
    impl Commander for HangingInvestigate {
        fn flight(&self, vehicle: String, command: String, _agl_m: Option<f64>)
            -> BoxFuture<CmdResult> {
            let op = format!("flight/{command}");
            self.log(vehicle, &op)
        }
        fn raster_search(&self, vehicle: String, _order: RasterOrder) -> BoxFuture<CmdResult> {
            self.log(vehicle, "raster-search")
        }
        fn investigate(&self, vehicle: String, _order: InvestigateOrder) -> BoxFuture<CmdResult> {
            self.calls
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .push((vehicle, "investigate".into()));
            Box::pin(std::future::pending())
        }
        fn task_abort(&self, vehicle: String, _label: String) -> BoxFuture<CmdResult> {
            self.log(vehicle, "task_abort")
        }
        fn queue_reorder(&self, vehicle: String, _ids: Vec<String>) -> BoxFuture<CmdResult> {
            self.log(vehicle, "queue_reorder")
        }
        fn sensor_capture(
            &self,
            vehicle: String,
            _request: muas_contracts::services::SensorRequest,
        ) -> BoxFuture<CmdResult> {
            self.log(vehicle, "sensor/capture")
        }
        fn video_control(
            &self,
            vehicle: String,
            _request: muas_contracts::services::VideoRequest,
        ) -> BoxFuture<CmdResult> {
            self.log(vehicle, "video/control")
        }
        fn system_shutdown(&self, vehicle: String, _confirm: String) -> BoxFuture<CmdResult> {
            self.log(vehicle, "system/shutdown")
        }
        fn rc_disengage(&self, vehicle: String) -> BoxFuture<CmdResult> {
            self.log(vehicle, "rc_disengage")
        }
    }

    let frame = |i: u32| format!("/muas/v3/mission/m/wuas-01/camera/cam0/frame/{}/{i}", 100 + i);
    let detector = Arc::new(ScriptedDetector::default());
    let lat2 = LAT + 20.0 * M_LAT;
    for (i, lat) in [(1, LAT), (2, LAT + M_LAT), (9, lat2), (10, lat2 + M_LAT)] {
        detector.script(
            frame(i),
            DetectOutcome::Hit(Detection {
                object_id: "tennis racket".into(),
                confidence: 0.9,
                lat_deg: lat,
                lon_deg: LON,
                offset_m: 2.0,
            }),
        );
    }
    let commander = Arc::new(HangingInvestigate { calls: Mutex::new(Vec::new()) });
    let config = DashConfig {
        record_dir: None,
        confirm_count: 2,
        iuas_ids: vec!["iuas-01".into(), "iuas-02".into()],
        ..DashConfig::default()
    };
    let dash = Arc::new(Dashboard::new(config, detector, commander.clone()));
    dash.handle_command(&json!({ "cmd": "start_mission", "params": params(&["camera"]) }));

    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(5);
    let settle = |dash: &Arc<Dashboard>, want_done: u64, why: &'static str| {
        let dash = dash.clone();
        async move {
            loop {
                if dash.detect_counters().1 >= want_done {
                    break;
                }
                assert!(tokio::time::Instant::now() < deadline, "{why}");
                tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            }
        }
    };

    // Target 1 confirms on frames 1+2 → dispatch #1; the investigate future
    // hangs forever from this point (the job stays `investigating`).
    for i in 1..=2 {
        let actions = dash.with_mission(|m| m.on_new_frame(&frame(i)));
        dash.apply_actions(actions);
    }
    settle(&dash, 2, "target-1 detections never resolved").await;
    loop {
        if commander.investigated("iuas-01") {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "job 1 never dispatched");
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }

    // Frames keep flowing while the dispatch future hangs: detects_done
    // keeps increasing — the loop is not awaiting the investigate call.
    for i in 3..=8 {
        let actions = dash.with_mission(|m| m.on_new_frame(&frame(i)));
        dash.apply_actions(actions);
    }
    settle(&dash, 8, "detect stream stalled behind the hanging dispatch").await;
    assert_eq!(
        dash.with_mission(|m| m.targets[0].jobs[0].status.clone()),
        "investigating",
        "job 1 still in flight (its ack never arrived)"
    );

    // Target 2 confirms on frames 9+10 → dispatch #2 to the idle iuas-02
    // while job 1 is still hanging.
    for i in 9..=10 {
        let actions = dash.with_mission(|m| m.on_new_frame(&frame(i)));
        dash.apply_actions(actions);
    }
    settle(&dash, 10, "target-2 detections never resolved").await;
    loop {
        if commander.investigated("iuas-02") {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "target 2 never dispatched");
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }
    assert_eq!(dash.with_mission(|m| m.targets.len()), 2, "both targets promoted");
    assert_eq!(
        dash.detect_counters(),
        (0, 10),
        "every frame's detection completed independently of dispatch"
    );
}

/// Recording sessions are mission-scoped: idle records nothing, mission
/// start arms `<run>-<mission>-<t>.jsonl`, RTL-all finalizes (capturing the
/// abort commands), and a later mission opens a NEW session.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn recording_sessions_are_mission_scoped() {
    let dir = std::env::temp_dir().join(format!("muas-dash-sess-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let commander = Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok())));
    let config = DashConfig {
        record_dir: Some(dir.clone()),
        run_name: "testrun".into(),
        confirm_count: 1,
        ..DashConfig::default()
    };
    let dash = Arc::new(Dashboard::new(
        config,
        Arc::new(ScriptedDetector::default()),
        commander,
    ));

    // Idle: nothing armed, nothing on disk.
    assert!(!dash.hub.is_recording());
    dash.hub.broadcast(&json!({ "type": "telemetry", "vehicle": "wuas-01" }));
    assert!(!dir.exists() || std::fs::read_dir(&dir).unwrap().next().is_none());

    // Mission start arms a session named by run + mission.
    dash.handle_command(&json!({ "cmd": "start_mission", "params": params(&["camera"]) }));
    let path = dash.hub.recording_path().expect("mission start arms the recorder");
    let name = path.file_name().unwrap().to_string_lossy().into_owned();
    assert!(name.starts_with("testrun-mission-"), "session name: {name}");

    // RTL-all aborts the mission and finalizes the session.
    dash.handle_command(&json!({ "cmd": "all", "command": "rtl" }));
    assert_eq!(dash.mission_state(), "aborted");
    assert!(!dash.hub.is_recording(), "RTL-all finalizes the recording");
    let text = std::fs::read_to_string(&path).expect("session file exists");
    assert!(text.contains("record.started"), "first line marks the arm");
    assert!(text.contains("mission.started"));
    assert!(text.contains("command.sent"), "the abort commands are captured");
    assert!(text.contains("record.stopped"), "last line marks the finalize");

    // Post-finalize idle traffic lands nowhere.
    dash.hub.broadcast(&json!({ "type": "telemetry", "vehicle": "wuas-01" }));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), text);

    // A new mission opens a NEW session file.
    dash.handle_command(&json!({ "cmd": "start_mission", "params": params(&["camera"]) }));
    let second = dash.hub.recording_path().expect("second session arms");
    assert_ne!(second, path, "each mission gets its own recording");
    let _ = std::fs::remove_dir_all(&dir);
}

/// `--gcs lat,lon` rides the hello message as `gcs:{lat,lon,source:"manual"}`
/// (the frontend's gcsLL() prefers it over NET.gcs and the first-fix
/// heuristic); without the flag the key is absent.
#[test]
fn hello_carries_the_surveyed_gcs_position() {
    let commander = Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok())));
    let config = DashConfig {
        record_dir: None,
        gcs: Some((-35.3635, 149.1652)),
        ..DashConfig::default()
    };
    let dash = Dashboard::new(config, Arc::new(ScriptedDetector::default()), commander.clone());
    let hello = dash.hello();
    assert_eq!(
        hello["gcs"],
        json!({ "lat": -35.3635, "lon": 149.1652, "source": "manual" })
    );

    let bare = Dashboard::new(
        DashConfig { record_dir: None, ..DashConfig::default() },
        Arc::new(ScriptedDetector::default()),
        commander,
    );
    assert!(bare.hello().get("gcs").is_none(), "no flag: no gcs key");
}

// ───────────────────────────── wire shapes ──────────────────────────────────

#[test]
fn events_and_targets_keep_the_v2_wire_shape() {
    let mut m = machine(&["iuas-01"], 1);
    let started = start(&mut m, &["camera"]);
    let Action::Emit(evt) = &started[0] else { panic!("first action is the event") };
    for key in ["type", "kind", "t", "mission_id", "vehicle", "agl_m"] {
        assert!(evt.get(key).is_some(), "mission.started missing {key}");
    }
    assert_eq!(evt["type"], json!("event"));

    let a = hit(&mut m, "/muas/v3/mission/m/wuas-01/camera/cam0/frame/123/7", LAT, LON, 0.9, 1.0);
    let found = a
        .iter()
        .find_map(|x| match x {
            Action::Emit(v) if v.get("kind") == Some(&json!("mission.target_found")) => Some(v),
            _ => None,
        })
        .expect("target found");
    for key in ["index", "object_id", "confidence", "lat", "lon", "frame", "hits", "sensors"] {
        assert!(found.get(key).is_some(), "mission.target_found missing {key}");
    }

    // detect.sent carries the frame's capture seq (v2 `_frame_seq`).
    let Action::Emit(sent) = &a[0] else { panic!("detect.sent first") };
    assert_eq!(sent["kind"], json!("detect.sent"));
    assert_eq!(sent["seq"], json!(7));

    // Targets serialize with the v2 dict keys (hello payload).
    let targets = m.targets_json();
    let t = &targets[0];
    for key in ["index", "object_id", "confidence", "lat", "lon", "frame",
                "best_offset", "status", "artifacts", "jobs"] {
        assert!(t.get(key).is_some(), "target dict missing {key}");
    }
    let j = &t["jobs"][0];
    for key in ["sensor", "vehicle", "status", "artifacts"] {
        assert!(j.get(key).is_some(), "job dict missing {key}");
    }
}

// ───────────────────────────── task-queue strip ─────────────────────────────

fn queue_fixture() -> Value {
    json!({
        "vehicle_id": "iuas-01",
        "gps_time_ns": 42u64,
        "depth_limit": 4,
        "tasks": [
            { "task_id": "tsk-3", "kind": "investigate",
              "params_digest": "orbit r=6m @ 8m [00c0ffee]",
              "state": "active", "origin": "dispatch", "enqueued_ns": 40u64,
              "started_ns": 41u64,
              "progress": { "pct": 40.0, "detail": "orbit 1/2", "eta_s": 33.0 },
              "est_duration_s": 60.0 },
            { "task_id": "tsk-5", "kind": "raster-search",
              "params_digest": "5 legs @ 6m [3fa2c81b]",
              "state": "pending", "origin": "split", "parent": "tsk-2",
              "enqueued_ns": 42u64, "eta_to_start_s": 33.0,
              "est_duration_s": 90.0 },
        ],
    })
}

/// The tasks/queue poller path: `on_task_queue` broadcasts the additive
/// `{"type":"task_queue",vehicle,status}` message, dedups unchanged
/// snapshots, and the latest copy rides the hello snapshot (mirrors the
/// coord-poller relay + hello patterns).
#[test]
fn task_queue_snapshots_broadcast_once_and_ride_hello() {
    let commander = Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok())));
    let config = DashConfig { record_dir: None, ..DashConfig::default() };
    let dash = Dashboard::new(config, Arc::new(ScriptedDetector::default()), commander);
    let mut rx = dash.hub.subscribe();

    let status = queue_fixture();
    assert!(dash.on_task_queue("iuas-01", status.clone()), "first snapshot broadcasts");
    let muas_dashboard::hub::Outbound::Text(text) = rx.try_recv().expect("broadcast out")
    else {
        panic!("task_queue rides a text frame")
    };
    let msg: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(msg["type"], json!("task_queue"));
    assert_eq!(msg["vehicle"], json!("iuas-01"));
    assert_eq!(msg["status"], status);

    // Unchanged queue: silent (the 1 Hz poll must not spam the WS).
    assert!(!dash.on_task_queue("iuas-01", status.clone()), "dedup on content");
    assert!(rx.try_recv().is_err(), "no rebroadcast");

    // New clients get the stored copy through hello.
    let hello = dash.hello();
    assert_eq!(hello["task_queues"]["iuas-01"], status);

    // A changed queue broadcasts again.
    let mut moved = status;
    moved["gps_time_ns"] = json!(43u64);
    assert!(dash.on_task_queue("iuas-01", moved));
}

/// The `queue_reorder` WS command: full ordered id list through the
/// commander, with the same command lifecycle (sent → acked) as
/// task_abort so the strip's optimistic reorder gets its toasts and its
/// refusal-revert signal.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn queue_reorder_command_calls_the_service_with_the_lifecycle() {
    let commander = Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok())));
    let config = DashConfig { record_dir: None, ..DashConfig::default() };
    let dash = Arc::new(Dashboard::new(
        config,
        Arc::new(ScriptedDetector::default()),
        commander.clone(),
    ));
    let mut rx = dash.hub.subscribe();
    dash.handle_command(&json!({
        "cmd": "queue_reorder",
        "vehicle": "iuas-01",
        "ordered_task_ids": ["tsk-5", "tsk-3", "tsk-4"],
    }));
    // Empty/absent id lists and unknown vehicles are ignored outright.
    dash.handle_command(&json!({ "cmd": "queue_reorder", "vehicle": "iuas-01" }));
    dash.handle_command(&json!({
        "cmd": "queue_reorder", "vehicle": "nope", "ordered_task_ids": ["tsk-1"],
    }));

    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let calls = commander.calls.lock().unwrap().clone();
        if !calls.is_empty() {
            assert_eq!(
                calls,
                vec![("iuas-01".to_string(), "queue_reorder/tsk-5,tsk-3,tsk-4".to_string())],
                "exactly one reorder reaches the service, ids in order"
            );
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "reorder never sent");
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }

    // Lifecycle events: command.sent then an acked command.result.
    let mut kinds = Vec::new();
    let event_deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(5);
    while kinds.len() < 2 {
        assert!(tokio::time::Instant::now() < event_deadline, "lifecycle incomplete: {kinds:?}");
        match rx.try_recv() {
            Ok(muas_dashboard::hub::Outbound::Text(text)) => {
                let v: Value = serde_json::from_str(&text).unwrap();
                if v["command"] == json!("queue_reorder") {
                    kinds.push((
                        v["kind"].as_str().unwrap_or("").to_string(),
                        v["ok"].clone(),
                    ));
                }
            }
            Ok(_) => {}
            Err(_) => tokio::time::sleep(std::time::Duration::from_millis(10)).await,
        }
    }
    assert_eq!(kinds[0], ("command.sent".to_string(), Value::Null));
    assert_eq!(kinds[1], ("command.result".to_string(), json!(true)));
}
