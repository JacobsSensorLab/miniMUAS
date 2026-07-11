//! Preview parity: the `/preview` plan is built from uas-flight's OWN
//! raster geometry, so what the operator previews is what the agent flies.
//! Plus recorder JSONL parseability and the WS hello schema round-trip.

use std::sync::Arc;

use muas_dashboard::providers::{CmdResult, ScriptedCommander, StubDetector};
use muas_dashboard::{hub::Hub, raster, DashConfig, Dashboard};
use muas_contracts::services::Ack;
use serde_json::{json, Value};
use uas_console::Replayer;
use uas_flight::geo::Position;
use uas_flight::patterns::raster_targets;

const LAT: f64 = 35.1208;
const LON: f64 = -89.9347;

fn center_area(width_m: f64, height_m: f64) -> Value {
    json!({
        "mode": "center",
        "center_lat": LAT,
        "center_lon": LON,
        "width_m": width_m,
        "height_m": height_m,
    })
}

/// The preview's legs must be EXACTLY the uas-flight raster targets the
/// agent would fly — endpoint for endpoint.
#[test]
fn preview_legs_equal_uas_flight_raster_targets() {
    for (w, h) in [(40.0, 24.0), (24.0, 40.0), (15.0, 15.0)] {
        let area = center_area(w, h);
        let plan = raster::build_preview(&area, 5.0, 4.0).expect("plan builds");

        // The flight-side fixture: same bounds, same spacing, straight from
        // uas-flight.
        let path = raster::flight_path(LAT, LON, w, h, 5.0);
        let targets = raster_targets(&path).expect("targets build");
        assert_eq!(targets.len(), plan.legs.len() * 2, "two endpoints per leg");
        for (leg_index, leg) in plan.legs.iter().enumerate() {
            let start: &Position = &targets[leg_index * 2].position;
            let end: &Position = &targets[leg_index * 2 + 1].position;
            assert_eq!((leg[0].0, leg[0].1), (start.lat, start.lon), "leg {leg_index} start");
            assert_eq!((leg[1].0, leg[1].1), (end.lat, end.lon), "leg {leg_index} end");
        }

        // Serpentine: consecutive legs run in opposite directions.
        for pair in plan.legs.windows(2) {
            let d0 = (pair[0][1].0 - pair[0][0].0, pair[0][1].1 - pair[0][0].1);
            let d1 = (pair[1][1].0 - pair[1][0].0, pair[1][1].1 - pair[1][0].1);
            assert!(
                d0.0 * d1.0 + d0.1 * d1.1 < 0.0,
                "boustrophedon legs must alternate direction"
            );
        }

        // Captures land on every leg, endpoints included.
        assert!(plan.captures.iter().all(|c| c.3 < plan.legs.len()));
        let first_leg_caps: Vec<_> = plan.captures.iter().filter(|c| c.3 == 0).collect();
        assert!(first_leg_caps.len() >= 2, "leg endpoints captured");
    }
}

/// Corner mode resolves to the same rectangle (and therefore the same legs)
/// as the equivalent center-mode area.
#[test]
fn corners_mode_matches_center_mode() {
    let center = center_area(40.0, 24.0);
    let (clat, clon, w, h) = raster::resolve_area(&center);
    let corners_of = raster::area_corners(clat, clon, w, h);
    let area = json!({
        "mode": "corners",
        "corner_a": [corners_of[0].0, corners_of[0].1], // NW
        "corner_b": [corners_of[2].0, corners_of[2].1], // SE
    });
    let a = raster::build_preview(&center, 5.0, 4.0).unwrap();
    let b = raster::build_preview(&area, 5.0, 4.0).unwrap();
    assert_eq!(a.legs.len(), b.legs.len());
    for (la, lb) in a.legs.iter().zip(&b.legs) {
        assert!((la[0].0 - lb[0].0).abs() < 1e-9 && (la[0].1 - lb[0].1).abs() < 1e-9);
        assert!((la[1].0 - lb[1].0).abs() < 1e-9 && (la[1].1 - lb[1].1).abs() < 1e-9);
    }
}

/// The wire message keeps the v2 `raster_preview` shape.
#[test]
fn preview_message_keeps_v2_shape() {
    let m = raster::preview_message(&center_area(40.0, 24.0), 5.0, 4.0, 2.0);
    assert_eq!(m["type"], json!("raster_preview"));
    for key in ["center", "width_m", "height_m", "corners", "legs", "captures"] {
        assert!(m["plan"].get(key).is_some(), "plan missing {key}");
    }
    assert_eq!(m["plan"]["corners"].as_array().unwrap().len(), 4);
    assert!(m["estimate_s"].as_f64().unwrap() > 0.0);
    let cap = &m["plan"]["captures"][0];
    for key in ["lat", "lon", "heading_deg", "leg", "index"] {
        assert!(cap.get(key).is_some(), "capture missing {key}");
    }
    let leg = &m["plan"]["legs"][0];
    assert!(leg[0].get("lat").is_some() && leg[1].get("lon").is_some());
}

// ───────────────────────────── recorder ─────────────────────────────────────

/// Every broadcast during an ARMED session lands as a parseable uas-console
/// RecordedEvent line; idle broadcasts (before arming) land nowhere.
#[test]
fn recorder_jsonl_is_parseable_by_uas_console() {
    let dir = std::env::temp_dir().join(format!("muas-dash-rec-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let hub = Hub::new(Some(dir.clone()), "run-t");
    // Idle traffic before the session arms is NOT recorded.
    hub.broadcast(&json!({ "type": "telemetry", "vehicle": "wuas-01", "idle": true }));
    assert!(hub.recording_path().is_none(), "idle produces no recording");
    let name = hub.arm("mission-1").expect("session arms");
    assert!(name.starts_with("run-t-mission-1-") && name.ends_with(".jsonl"));
    let messages = [
        json!({ "type": "event", "kind": "mission.started", "t": 1.0, "mission_id": "m-1" }),
        json!({ "type": "telemetry", "vehicle": "wuas-01",
                "sample": { "lat_deg": LAT, "lon_deg": LON }, "age_s": 0.3, "skew_s": 0.0 }),
        json!({ "type": "sensor_data", "item": { "name": "/a/b", "sensor": "camera" } }),
    ];
    for m in &messages {
        hub.broadcast(m);
    }
    let path = hub.recording_path().expect("recording open while armed");
    hub.finalize();
    let replayer = Replayer::load(&path).expect("recording parses");
    assert_eq!(replayer.events().len(), messages.len());
    for (event, expected) in replayer.events().iter().zip(&messages) {
        assert_eq!(&event.event, expected, "recorded payload is the broadcast, verbatim");
        assert!(event.t_ns > 0);
    }
    let _ = std::fs::remove_dir_all(&dir);
}

// ───────────────────────────── WS schema ────────────────────────────────────

/// The hello message keeps the v2 shape and round-trips through JSON.
#[test]
fn hello_schema_round_trips() {
    let config = DashConfig {
        record_dir: None,
        iuas_ids: vec!["iuas-01".into(), "iuas-02".into()],
        ..DashConfig::default()
    };
    let dash = Arc::new(Dashboard::new(
        config,
        Arc::new(StubDetector),
        Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok()))),
    ));
    let hello = dash.hello();
    let text = hello.to_string();
    let parsed: Value = serde_json::from_str(&text).expect("hello round-trips");
    assert_eq!(parsed["type"], json!("hello"));
    assert_eq!(
        parsed["vehicles"],
        json!(["wuas-01", "iuas-01", "iuas-02"]),
        "WUAS first — the binary video frame index ordering"
    );
    assert_eq!(parsed["enabled"]["iuas-02"], json!(true));
    assert_eq!(parsed["mission"]["state"], json!("idle"));
    assert!(parsed["mission"]["targets"].as_array().unwrap().is_empty());
    assert!(parsed["sensor_data"].as_array().unwrap().is_empty());
    // RC pilot surface (RC-CONTROL R2): hello carries the rc snapshot map and
    // the RC-reachable target roster (empty on a plain config).
    assert_eq!(parsed["rc"], json!({}), "no RC status yet");
    assert_eq!(parsed["rc_targets"], json!([]), "no --rc-target configured");
}

/// The rc/status poller's broadcast shape + hello inclusion + content dedup —
/// mirrors the task-queue poller's `on_task_queue` contract.
#[test]
fn rc_status_broadcasts_and_rides_hello() {
    use muas_dashboard::hub::Outbound;
    let dash = Arc::new(Dashboard::new(
        DashConfig { record_dir: None, ..DashConfig::default() },
        Arc::new(StubDetector),
        Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok()))),
    ));
    let mut rx = dash.hub.subscribe();

    let status = json!({
        "vehicle_id": "iuas-01",
        "gps_time_ns": 42_u64,
        "engaged": true,
        "source": "listen:127.0.0.1:14650",
        "seq_gap_pct": 2.5,
        "age_ms": 18_u64,
        "failsafe_state": "manual",
    });
    assert!(dash.on_rc_status("iuas-01", status.clone()), "first sample broadcasts");
    let Ok(Outbound::Text(text)) = rx.try_recv() else { panic!("rc broadcast") };
    let msg: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(msg["type"], json!("rc"));
    assert_eq!(msg["vehicle"], json!("iuas-01"));
    assert_eq!(msg["status"]["failsafe_state"], json!("manual"));
    assert_eq!(msg["status"]["engaged"], json!(true));

    // Content dedup: an identical sample is silent (fresh clients get it via
    // hello instead).
    assert!(!dash.on_rc_status("iuas-01", status.clone()), "identical sample is silent");
    assert!(rx.try_recv().is_err(), "no second broadcast for an unchanged status");

    // The stored snapshot rides hello.
    let hello = dash.hello();
    assert_eq!(hello["rc"]["iuas-01"]["failsafe_state"], json!("manual"));
}

/// End-to-end honesty: a pilot's WS commands (`{"cmd":"rc",…}`) drive real
/// uas-rc frames onto a bound [`UdpRcReceiver`] — the exact receiver the
/// agent binds for `--rc listen:<addr>`. Nothing is short-circuited: the
/// command layer feeds the RcHost, whose sender puts frames on the wire.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pilot_ws_commands_drive_real_rc_frames() {
    use std::collections::BTreeMap;
    use std::time::{Duration, Instant};
    use uas_rc::{RcFlags, UdpRcReceiver};

    let mut rx = UdpRcReceiver::bind("127.0.0.1:0").unwrap();
    let addr = rx.local_addr().unwrap();
    let mut targets = BTreeMap::new();
    targets.insert("iuas-01".to_string(), addr);

    let config = DashConfig { record_dir: None, rc_targets: targets, ..DashConfig::default() };
    let dash = Arc::new(Dashboard::new(
        config,
        Arc::new(StubDetector),
        Arc::new(ScriptedCommander::answering(CmdResult::Ack(Ack::ok()))),
    ));
    // Wire the send host exactly as `start()` does.
    let host = muas_dashboard::rc::RcHost::new(&dash.config.rc_targets);
    dash.attach_rc(host.clone());

    // Engage + push a right-roll, arm-gesture held — all through the WS surface.
    dash.handle_command(&json!({ "cmd": "rc", "op": "engage", "target": "iuas-01" }));
    dash.handle_command(&json!({
        "cmd": "rc", "op": "input", "arm": true,
        "channels": [2000, 1500, 1600, 1500, 65535, 65535, 65535, 65535],
    }));
    host.tick(0);

    let deadline = Instant::now() + Duration::from_secs(2);
    let frame = loop {
        if let Some(f) = rx.poll().unwrap() {
            break f;
        }
        assert!(Instant::now() < deadline, "no frame reached the agent RC socket");
        std::thread::sleep(Duration::from_millis(2));
    };
    assert_eq!(frame.channels[0], 2000, "ch1 roll deflection arrived over the wire");
    assert_eq!(frame.channels[2], 1600, "ch3 throttle arrived");
    assert!(frame.flags.contains(RcFlags::ARM_GESTURE), "the held arm gesture rode the flag byte");
}
