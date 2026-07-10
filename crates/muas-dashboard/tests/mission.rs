//! Mission state machine unit tests — the v2 semantics ported as scripted
//! detections: confirm-count, best-localized position, multi-target
//! multi-sensor dispatch, and the completion predicate.

use std::collections::BTreeSet;
use std::sync::Arc;

use muas_contracts::services::Ack;
use muas_dashboard::mission::{
    Action, DetectOutcome, Detection, JobResult, Mission, MissionConfig,
};
use muas_dashboard::providers::{CmdResult, ScriptedCommander, ScriptedDetector};
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
    m.set_capabilities(vid, set);
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

    // Detect → hit → target → dispatch → ack'd job → target done.
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let status = dash.with_mission(|m| m.targets.first().map(|t| t.status.clone()));
        if status.as_deref() == Some("done") {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "job never completed: {status:?}");
        tokio::time::sleep(std::time::Duration::from_millis(25)).await;
    }
    // Raster ends → completion predicate closes the mission.
    let actions = dash.with_mission(|m| m.on_search_response(true, "done", 1, ""));
    dash.apply_actions(actions);
    assert_eq!(dash.mission_state(), "done");

    let calls = commander.calls.lock().unwrap().clone();
    assert!(calls.contains(&("wuas-01".to_string(), "raster-search".to_string())));
    assert!(calls.contains(&("iuas-01".to_string(), "investigate".to_string())));
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
