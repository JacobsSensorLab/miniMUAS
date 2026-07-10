//! Headless smoke test: the full dashboard comes up with an NDN engine and
//! ZERO reachable vehicles — no peers required. Serves the embedded HTML,
//! answers the WS hello, runs the command path (recording a broadcast),
//! serves the preview endpoint, 404s a missing tile (grid fallback), and
//! exposes the uas-console instrument descriptor.

use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use muas_dashboard::providers::StubDetector;
use muas_dashboard::DashConfig;
use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;
use uas_console::Replayer;

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("muas-dash-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn headless_backend_serves_ui_ws_and_fallbacks() {
    let tiles = temp_dir("tiles");
    let replays = temp_dir("replays");
    let config = DashConfig {
        http_host: "127.0.0.1".into(),
        http_port: 0, // ephemeral
        tiles_dir: tiles.clone(),
        tile_upstream: String::new(), // pure offline: no proxying
        record_dir: Some(replays.clone()),
        ..DashConfig::default()
    };
    let running = muas_dashboard::start(config, Arc::new(StubDetector))
        .await
        .expect("dashboard up with zero vehicles");
    let addr = running.addr;
    let http = reqwest::Client::new();
    let base = format!("http://{addr}");

    // 1 — the embedded UI serves.
    let body = http.get(&base).send().await.expect("GET /").text().await.unwrap();
    assert!(body.contains("Mission Console"), "embedded dashboard.html serves");
    assert!(body.contains("cds-header"), "Carbon g100 shell present");
    assert!(
        body.contains("id=\"mapctl\"") && body.contains("id=\"followSeg\""),
        "map follow/orientation control cluster present"
    );
    assert!(
        body.contains("function applyFollow") && body.contains("function fleetFrame"),
        "follow-mode camera engine embedded"
    );

    // 2 — WS hello with the v2 schema.
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/ws"))
        .await
        .expect("ws connects");
    let hello: Value = match ws.next().await.expect("hello frame").expect("ws ok") {
        Message::Text(text) => serde_json::from_str(&text).expect("hello parses"),
        other => panic!("expected text hello, got {other:?}"),
    };
    assert_eq!(hello["type"], json!("hello"));
    assert_eq!(hello["vehicles"], json!(["wuas-01", "iuas-01"]));
    assert_eq!(hello["mission"]["state"], json!("idle"));

    // 3 — a command round-trips: disabling a vehicle broadcasts the event
    // (and lazily opens the recording).
    ws.send(Message::Text(
        json!({ "cmd": "set_enabled", "vehicle": "iuas-01", "enabled": false })
            .to_string()
            .into(),
    ))
    .await
    .expect("command sends");
    let event = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if let Message::Text(text) = ws.next().await.expect("ws open").expect("ws ok") {
                let v: Value = serde_json::from_str(&text).unwrap();
                if v["kind"] == json!("vehicle.disabled") {
                    return v;
                }
            }
        }
    })
    .await
    .expect("vehicle.disabled arrives");
    assert_eq!(event["vehicle"], json!("iuas-01"));

    // 4 — WS preview reply goes to the requesting client only (v2 path).
    ws.send(Message::Text(
        json!({
            "cmd": "preview_raster",
            "area": { "mode": "center", "center_lat": 35.1208,
                      "center_lon": -89.9347, "width_m": 40.0, "height_m": 24.0 },
            "leg_spacing_m": 5.0, "capture_every_m": 4.0, "speed_m_s": 2.0,
        })
        .to_string()
        .into(),
    ))
    .await
    .expect("preview sends");
    let preview = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if let Message::Text(text) = ws.next().await.expect("ws open").expect("ws ok") {
                let v: Value = serde_json::from_str(&text).unwrap();
                if v["type"] == json!("raster_preview") {
                    return v;
                }
            }
        }
    })
    .await
    .expect("raster_preview arrives");
    assert!(!preview["plan"]["legs"].as_array().unwrap().is_empty());

    // 5 — the HTTP preview endpoint answers the same shape.
    let posted: Value = http
        .post(format!("{base}/preview"))
        .json(&json!({
            "area": { "mode": "center", "center_lat": 35.1208,
                      "center_lon": -89.9347, "width_m": 40.0, "height_m": 24.0 },
            "leg_spacing_m": 5.0, "capture_every_m": 4.0, "speed_m_s": 2.0,
        }))
        .send()
        .await
        .expect("POST /preview")
        .json()
        .await
        .expect("preview json");
    assert_eq!(posted["type"], json!("raster_preview"));
    assert_eq!(posted["plan"]["legs"], preview["plan"]["legs"]);

    // 6 — tile fallback: empty cache + no upstream ⇒ 404 (UI grid kicks in);
    // junk coordinates ⇒ 400.
    let miss = http.get(format!("{base}/tiles/5/1/1")).send().await.unwrap();
    assert_eq!(miss.status(), reqwest::StatusCode::NOT_FOUND);
    let bad = http.get(format!("{base}/tiles/x/y/z")).send().await.unwrap();
    assert_eq!(bad.status(), reqwest::StatusCode::BAD_REQUEST);

    // 7 — the replay index lists the live recording, name-validated fetch
    // parses as uas-console JSONL, traversal names are rejected.
    let index: Value = http
        .get(format!("{base}/replays"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let replays_list = index["replays"].as_array().unwrap();
    assert_eq!(replays_list.len(), 1, "one live recording");
    assert_eq!(replays_list[0]["recording"], json!(true));
    let name = replays_list[0]["name"].as_str().unwrap().to_string();
    let jsonl = http
        .get(format!("{base}/replays/{name}"))
        .send()
        .await
        .unwrap()
        .text()
        .await
        .unwrap();
    assert!(jsonl.lines().any(|l| l.contains("vehicle.disabled")));
    let evil = http
        .get(format!("{base}/replays/..%2Fsecrets.jsonl"))
        .send()
        .await
        .unwrap();
    assert_ne!(evil.status(), reqwest::StatusCode::OK, "traversal name rejected");

    // 8 — the instrument descriptor exposes the console bundle, and /views
    // exists (empty until telemetry flows).
    let instrument: Value = http
        .get(format!("{base}/instrument.json"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(instrument["label"], json!("uas-console"));
    let renderers: Vec<&str> = instrument["renderers"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(Value::as_str)
        .collect();
    assert!(renderers.contains(&"console.track.live"));
    assert!(renderers.contains(&"console.panel.vehicle"));
    let namespaces = instrument["namespaces"].as_array().unwrap();
    assert!(namespaces
        .iter()
        .any(|n| n.as_str() == Some("/muas/v3/wuas-01/telemetry/live")));
    let views: Value = http
        .get(format!("{base}/views"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert!(views.is_object());

    // 9 — artifact fetch with no producers is an honest 404.
    let missing = http
        .get(format!("{base}/artifact?name=/muas/v3/mission/m/x/frame/1/1"))
        .send()
        .await
        .unwrap();
    assert_eq!(missing.status(), reqwest::StatusCode::NOT_FOUND);

    drop(ws);
    running.shutdown().await;

    // The recording survives shutdown and loads deterministically.
    let path = replays.join(&name);
    let replayer = Replayer::load(&path).expect("recording parses after shutdown");
    assert!(replayer
        .events()
        .iter()
        .any(|e| e.event["kind"] == json!("vehicle.disabled")));

    let _ = std::fs::remove_dir_all(&tiles);
    let _ = std::fs::remove_dir_all(&replays);
}
