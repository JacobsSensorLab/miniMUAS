//! The axum web layer: embedded UI, the single everything-WebSocket, the
//! preview endpoint, tile cache/proxy, replay index/fetch, artifact fetch,
//! and the uas-console instrument descriptor.

use std::path::PathBuf;
use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, Query, State};
use axum::http::{header, StatusCode};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde_json::{json, Value};
use tokio::sync::broadcast::error::RecvError;

use crate::hub::Outbound;
use crate::Dashboard;

/// The embedded single-page UI (ported v2 `dashboard.html`, Carbon g100).
const DASHBOARD_HTML: &str = include_str!("../assets/dashboard.html");

/// Build the dashboard router.
pub fn router(dash: Arc<Dashboard>) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/ws", get(ws_upgrade))
        .route("/preview", post(preview))
        .route("/tiles/{z}/{x}/{y}", get(tile))
        .route("/replays", get(replays_index))
        .route("/replays/{name}", get(replay_file))
        .route("/artifact", get(artifact))
        .route("/instrument.json", get(instrument))
        .route("/views", get(views))
        .route("/catalog.json", get(catalog))
        .with_state(dash)
}

async fn index() -> Html<&'static str> {
    Html(DASHBOARD_HTML)
}

// ───────────────────────────── websocket ────────────────────────────────────

async fn ws_upgrade(State(dash): State<Arc<Dashboard>>, ws: WebSocketUpgrade) -> Response {
    ws.on_upgrade(move |socket| session(dash, socket))
}

/// One WS session: hello, then fan-in commands / fan-out broadcasts on a
/// single task (no split needed — select over both directions).
async fn session(dash: Arc<Dashboard>, mut socket: WebSocket) {
    if socket
        .send(Message::Text(dash.hello().to_string().into()))
        .await
        .is_err()
    {
        return;
    }
    let mut rx = dash.hub.subscribe();
    loop {
        tokio::select! {
            out = rx.recv() => match out {
                Ok(Outbound::Text(text)) => {
                    if socket.send(Message::Text(text.as_str().into())).await.is_err() {
                        break;
                    }
                }
                Ok(Outbound::Binary(bytes)) => {
                    if socket
                        .send(Message::Binary(bytes.as_slice().to_vec().into()))
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
                Err(RecvError::Lagged(skipped)) => {
                    tracing::debug!(skipped, "ws client lagged; frames dropped");
                }
                Err(RecvError::Closed) => break,
            },
            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Text(text))) => {
                    let Ok(parsed) = serde_json::from_str::<Value>(&text) else { continue };
                    if let Some(reply) = dash.handle_command(&parsed) {
                        if socket.send(Message::Text(reply.to_string().into())).await.is_err() {
                            break;
                        }
                    }
                }
                Some(Ok(Message::Close(_))) | None => break,
                Some(Ok(_)) => {}
                Some(Err(_)) => break,
            },
        }
    }
}

// ───────────────────────────── preview ──────────────────────────────────────

/// `/preview`: the raster the WUAS would fly, from uas-flight's own
/// geometry (preview == flight).
async fn preview(Json(body): Json<Value>) -> Json<Value> {
    let f = |k: &str, d: f64| body.get(k).and_then(Value::as_f64).unwrap_or(d);
    Json(crate::raster::preview_message(
        body.get("area").unwrap_or(&Value::Null),
        f("leg_spacing_m", 5.0),
        f("capture_every_m", 4.0),
        f("speed_m_s", 2.0),
    ))
}

// ───────────────────────────── tiles ────────────────────────────────────────

/// `/tiles/{z}/{x}/{y}`: local cache first, then (with the `tile-proxy`
/// feature and a configured upstream) proxy-and-cache; a miss 404s and the
/// UI falls back to the 10 m grid.
async fn tile(
    State(dash): State<Arc<Dashboard>>,
    Path((z, x, y)): Path<(String, String, String)>,
) -> Response {
    let (Ok(z), Ok(x), Ok(y)) = (z.parse::<u32>(), x.parse::<u32>(), y.parse::<u32>()) else {
        return StatusCode::BAD_REQUEST.into_response();
    };
    if z > 20 {
        return StatusCode::BAD_REQUEST.into_response();
    }
    let path = dash
        .config
        .tiles_dir
        .join(z.to_string())
        .join(x.to_string())
        .join(format!("{y}.jpg"));
    if let Ok(body) = tokio::fs::read(&path).await {
        return jpeg_response(body);
    }
    #[cfg(feature = "tile-proxy")]
    if !dash.config.tile_upstream.is_empty() {
        if let Some(body) = fetch_upstream_tile(&dash.config.tile_upstream, z, x, y).await {
            // Cache what we fetched: bench panning warms the cache the
            // offline field deployment serves from. Write failure isn't
            // fatal.
            if let Some(parent) = path.parent() {
                let _ = tokio::fs::create_dir_all(parent).await;
            }
            let _ = tokio::fs::write(&path, &body).await;
            return jpeg_response(body);
        }
    }
    StatusCode::NOT_FOUND.into_response()
}

fn jpeg_response(body: Vec<u8>) -> Response {
    (
        [
            (header::CONTENT_TYPE, "image/jpeg"),
            (header::CACHE_CONTROL, "max-age=86400"),
        ],
        body,
    )
        .into_response()
}

#[cfg(feature = "tile-proxy")]
async fn fetch_upstream_tile(template: &str, z: u32, x: u32, y: u32) -> Option<Vec<u8>> {
    let url = template
        .replace("{z}", &z.to_string())
        .replace("{x}", &x.to_string())
        .replace("{y}", &y.to_string());
    let client = reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(4))
        .build()
        .ok()?;
    let response = client.get(url).send().await.ok()?;
    if !response.status().is_success() {
        return None;
    }
    Some(response.bytes().await.ok()?.to_vec())
}

// ───────────────────────────── replays ──────────────────────────────────────

/// `/replays`: the recording index (name, size, mtime, live flag).
async fn replays_index(State(dash): State<Arc<Dashboard>>) -> Json<Value> {
    let mut items = Vec::new();
    if let Some(dir) = dash.config.record_dir.clone() {
        let live = dash.hub.recording_path();
        let mut names: Vec<(String, PathBuf)> = std::fs::read_dir(&dir)
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().to_string();
                name.ends_with(".jsonl").then(|| (name, e.path()))
            })
            .collect();
        names.sort_by(|a, b| b.0.cmp(&a.0));
        for (name, path) in names {
            let Ok(meta) = path.metadata() else { continue };
            let mtime = meta
                .modified()
                .ok()
                .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0);
            items.push(json!({
                "name": name,
                "bytes": meta.len(),
                "mtime": mtime,
                "recording": live.as_deref() == Some(path.as_path()),
            }));
        }
    }
    Json(json!({ "replays": items }))
}

/// Strict v2 name validation: `[A-Za-z0-9._-]+\.jsonl`, no separators.
fn valid_replay_name(name: &str) -> bool {
    name.ends_with(".jsonl")
        && name.len() > ".jsonl".len()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-')
}

async fn replay_file(
    State(dash): State<Arc<Dashboard>>,
    Path(name): Path<String>,
) -> Response {
    if !valid_replay_name(&name) {
        return (StatusCode::BAD_REQUEST, "bad replay name").into_response();
    }
    let Some(dir) = dash.config.record_dir.clone() else {
        return (StatusCode::NOT_FOUND, "recording disabled").into_response();
    };
    let path = dir.join(&name);
    if dash.hub.recording_path().as_deref() == Some(path.as_path()) {
        dash.hub.sync(); // replaying the live recording: complete it
    }
    match tokio::fs::read(&path).await {
        Ok(body) => (
            [(header::CONTENT_TYPE, "application/x-ndjson")],
            body,
        )
            .into_response(),
        Err(_) => (StatusCode::NOT_FOUND, "no such replay").into_response(),
    }
}

// ───────────────────────────── artifacts ────────────────────────────────────

#[derive(serde::Deserialize)]
struct ArtifactQuery {
    #[serde(default)]
    name: String,
}

/// `/artifact?name=`: fetch the named object over NDN, split the
/// `MUASFRAME1` container, serve the body under its declared kind.
async fn artifact(
    State(dash): State<Arc<Dashboard>>,
    Query(query): Query<ArtifactQuery>,
) -> Response {
    let Some(consumer) = dash.consumer() else {
        return (StatusCode::NOT_FOUND, "artifact unavailable").into_response();
    };
    match crate::ndn::fetch_artifact(consumer, &query.name).await {
        Some((body, kind)) => {
            let content_type = if kind.contains('/') { kind } else { "image/jpeg".to_string() };
            ([(header::CONTENT_TYPE, content_type)], body).into_response()
        }
        None => (StatusCode::NOT_FOUND, "artifact unavailable").into_response(),
    }
}

// ───────────────────────────── console dogfood ──────────────────────────────

/// `/instrument.json`: the uas-console adoption bundle (namespaces +
/// three-layer manifest set + renderer ids).
async fn instrument(State(dash): State<Arc<Dashboard>>) -> Response {
    (
        [(header::CONTENT_TYPE, "application/json")],
        dash.lens.instrument_json(),
    )
        .into_response()
}

/// `/views`: the latest Binder-rendered track + tile outputs per vehicle.
async fn views(State(dash): State<Arc<Dashboard>>) -> Json<Value> {
    Json(dash.lens.views())
}

/// `/catalog.json`: the surface catalog (ROUND-3 §3½) — this dashboard's
/// typed self-description: understood WS kinds, surface-native widgets,
/// and the uas-console render contracts it binds through. Assembled
/// server-side from the registries in [`crate::catalog`].
async fn catalog(State(dash): State<Arc<Dashboard>>) -> Json<Value> {
    Json(crate::catalog::catalog(&dash.vehicles()))
}
