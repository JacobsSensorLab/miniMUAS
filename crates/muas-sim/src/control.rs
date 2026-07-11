//! The virtual deployment's control endpoint: a small HTTP API through
//! which OUTSIDE actors (the dashboard's anomaly tool via its `SimControl`
//! seam, `--verify` scripting, ad-hoc curl) mutate and observe simulation
//! truth. Simulation truth itself stays in [`crate::anomaly::AnomalyField`]
//! — this is only the door.
//!
//! Routes:
//! - `GET  /anomalies`       → current ground truth (JSON list)
//! - `POST /anomalies`       → place one anomaly (body: tagged `Anomaly`,
//!   empty id = assign); returns the stored value
//! - `DELETE /anomalies/{id}`→ remove one
//! - `DELETE /anomalies`     → clear all
//! - `GET  /netstats`        → the latest 1 Hz network snapshot (per-link
//!   ndn-sim face counters + the active link profile)
//!
//! Also here: [`http_json`], a dependency-free localhost HTTP/1.1 JSON
//! client (the dashboard's tile proxy is the only reqwest user and is
//! feature-gated, so the control path stays independent), and
//! [`HttpSimControl`], the `muas_dashboard::providers::SimControl` impl
//! that routes the dashboard's WS `sim` commands through this endpoint —
//! WS → control endpoint → AnomalyField, no dashboard-side truth.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::routing::{delete, get};
use axum::{Json, Router};
use muas_contracts::anomaly::Anomaly;
use muas_dashboard::providers::{BoxFuture, SimControl};
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_util::sync::CancellationToken;

use crate::anomaly::{AnomalyField, AnomalySource};

/// Shared endpoint state.
#[derive(Clone)]
struct ControlState {
    field: Arc<AnomalyField>,
    /// Latest net snapshot, written at 1 Hz by the deployment's exporter.
    net: Arc<Mutex<Value>>,
}

/// Latest-net-snapshot handle (exporter writes, `GET /netstats` reads).
pub type NetSnapshot = Arc<Mutex<Value>>;

/// Start the control endpoint on `127.0.0.1:port` (0 = ephemeral).
/// Returns the bound address; the server stops when `cancel` fires.
pub async fn serve_control(
    port: u16,
    field: Arc<AnomalyField>,
    net: NetSnapshot,
    cancel: CancellationToken,
) -> Result<SocketAddr, String> {
    let state = ControlState { field, net };
    let router = Router::new()
        .route("/anomalies", get(list_anomalies).post(place_anomaly).delete(clear_anomalies))
        .route("/anomalies/{id}", delete(remove_anomaly))
        .route("/netstats", get(netstats))
        .with_state(state);
    let listener = tokio::net::TcpListener::bind(("127.0.0.1", port))
        .await
        .map_err(|e| format!("control endpoint bind 127.0.0.1:{port}: {e}"))?;
    let addr = listener.local_addr().map_err(|e| format!("control addr: {e}"))?;
    tokio::spawn(async move {
        let serve = axum::serve(listener, router)
            .with_graceful_shutdown(async move { cancel.cancelled().await });
        if let Err(err) = serve.await {
            tracing::warn!(%err, "control endpoint ended");
        }
    });
    Ok(addr)
}

async fn list_anomalies(State(state): State<ControlState>) -> Json<Value> {
    Json(json!({ "anomalies": state.field.snapshot() }))
}

async fn place_anomaly(
    State(state): State<ControlState>,
    Json(body): Json<Value>,
) -> Result<Json<Value>, (StatusCode, String)> {
    let anomaly: Anomaly = serde_json::from_value(body)
        .map_err(|e| (StatusCode::BAD_REQUEST, format!("bad anomaly: {e}")))?;
    let placed = state.field.place(anomaly);
    Ok(Json(json!({ "placed": placed })))
}

async fn remove_anomaly(
    State(state): State<ControlState>,
    Path(id): Path<String>,
) -> Result<Json<Value>, (StatusCode, String)> {
    if state.field.remove(&id) {
        Ok(Json(json!({ "removed": id })))
    } else {
        Err((StatusCode::NOT_FOUND, format!("no anomaly '{id}'")))
    }
}

async fn clear_anomalies(State(state): State<ControlState>) -> Json<Value> {
    Json(json!({ "cleared": state.field.clear() }))
}

/// Assemble the deployment's 1 Hz network snapshot — the ONE document that
/// is both broadcast to the dashboard as the WS `net` message and served
/// verbatim at `GET /netstats`. `gcs` is the ground-station position the
/// network layer anchors its GCS node to. `prefixes` is the per-node
/// per-prefix rate table from the bridge taps ([`crate::nettap`]) — the
/// namespace lens's feed; empty when the deployment measures none. Both
/// keys are additive: the phase-1 renderer ignores keys it does not know.
///
/// Positioning note: with the current STATIC link profiles this position
/// is visualization truth only — no propagation model consumes it. When
/// geometry-based propagation lands, this same exported value feeds it,
/// so the flag/plumbing shape stays put.
pub fn net_snapshot(
    t_s: f64,
    profile: &Value,
    gcs: (f64, f64),
    links: Vec<Value>,
    prefixes: Vec<Value>,
) -> Value {
    json!({
        "type": "net",
        "t": t_s,
        "profile": profile,
        "gcs": { "lat": gcs.0, "lon": gcs.1 },
        "links": links,
        "prefixes": prefixes,
        // Measurability provenance for the lens UI: names are read at the
        // deployment's UDP bridge taps, never synthesized (nettap docs).
        "prefix_source": "udp-bridge-taps",
    })
}

async fn netstats(State(state): State<ControlState>) -> Json<Value> {
    Json(
        state
            .net
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone(),
    )
}

// ───────────────────────────── tiny HTTP client ─────────────────────────────

/// Minimal HTTP/1.1 JSON exchange over one TCP connection — enough for the
/// localhost control endpoint (verify scripting + the dashboard seam)
/// without pulling an HTTP client into this crate.
pub async fn http_json(method: &str, url: &str, body: Option<&Value>) -> Result<Value, String> {
    let rest = url
        .strip_prefix("http://")
        .ok_or_else(|| format!("only http:// URLs supported: {url}"))?;
    let (host, path) = match rest.split_once('/') {
        Some((host, path)) => (host, format!("/{path}")),
        None => (rest, "/".to_string()),
    };
    let payload = body.map(|b| b.to_string()).unwrap_or_default();
    let request = format!(
        "{method} {path} HTTP/1.1\r\nHost: {host}\r\nContent-Type: application/json\r\n\
         Content-Length: {}\r\nConnection: close\r\n\r\n{payload}",
        payload.len(),
    );
    let mut stream = tokio::net::TcpStream::connect(host)
        .await
        .map_err(|e| format!("connect {host}: {e}"))?;
    stream
        .write_all(request.as_bytes())
        .await
        .map_err(|e| format!("send: {e}"))?;
    let mut response = Vec::new();
    stream
        .read_to_end(&mut response)
        .await
        .map_err(|e| format!("recv: {e}"))?;
    let text = String::from_utf8_lossy(&response);
    let (head, body) = text
        .split_once("\r\n\r\n")
        .ok_or_else(|| "malformed HTTP response".to_string())?;
    let status: u16 = head
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| "malformed status line".to_string())?;
    if !(200..300).contains(&status) {
        return Err(format!("HTTP {status}: {}", body.trim()));
    }
    if body.trim().is_empty() {
        return Ok(Value::Null);
    }
    serde_json::from_str(body.trim()).map_err(|e| format!("response JSON: {e}"))
}

// ───────────────────────────── dashboard seam ───────────────────────────────

/// The dashboard's [`SimControl`] provider: forwards WS `sim` commands to
/// the deployment control endpoint over HTTP, so the write path is
/// operator click → WS → THIS endpoint → `AnomalyField` (one door for
/// dashboards and scripts alike).
pub struct HttpSimControl {
    base: String,
}

impl HttpSimControl {
    /// `base` like `http://127.0.0.1:8081` (no trailing slash).
    pub fn new(base: impl Into<String>) -> Self {
        Self { base: base.into() }
    }
}

impl SimControl for HttpSimControl {
    fn call(&self, op: String, params: Value) -> BoxFuture<Result<Value, String>> {
        let base = self.base.clone();
        Box::pin(async move {
            match op.as_str() {
                "place_anomaly" => {
                    http_json("POST", &format!("{base}/anomalies"), Some(&params)).await
                }
                "remove_anomaly" => {
                    let id = params.get("id").and_then(Value::as_str).unwrap_or("");
                    http_json("DELETE", &format!("{base}/anomalies/{id}"), None).await
                }
                "clear_anomalies" => {
                    http_json("DELETE", &format!("{base}/anomalies"), None).await
                }
                other => Err(format!("unknown sim op '{other}'")),
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread")]
    async fn control_endpoint_round_trips_through_the_http_door() {
        let field = Arc::new(AnomalyField::new());
        let net: NetSnapshot = Arc::new(Mutex::new(json!({ "links": [] })));
        let cancel = CancellationToken::new();
        let addr = serve_control(0, field.clone(), net.clone(), cancel.clone())
            .await
            .expect("endpoint up");
        let base = format!("http://{addr}");

        // Place through the door (empty id → assigned).
        let placed = http_json(
            "POST",
            &format!("{base}/anomalies"),
            Some(&json!({
                "kind": "visual", "lat_deg": 35.0, "lon_deg": 149.0,
                "size_m": 4.0, "signature": "red",
            })),
        )
        .await
        .expect("place");
        let id = placed["placed"]["id"].as_str().expect("id assigned").to_string();
        assert!(id.starts_with("anom-"));
        assert_eq!(field.snapshot().len(), 1, "the field IS the truth");

        // The dashboard seam goes through the same door.
        let control = HttpSimControl::new(base.clone());
        control
            .call(
                "place_anomaly".into(),
                json!({ "kind": "audio", "lat_deg": 35.0, "lon_deg": 149.0, "loudness_db": 80.0 }),
            )
            .await
            .expect("seam place");
        assert_eq!(field.snapshot().len(), 2);

        let listed = http_json("GET", &format!("{base}/anomalies"), None).await.unwrap();
        assert_eq!(listed["anomalies"].as_array().unwrap().len(), 2);

        // The deployment's snapshot (net_snapshot) carries the GCS anchor
        // through /netstats verbatim — the network layer's position source.
        *net.lock().unwrap() = net_snapshot(
            12.5,
            &json!({ "name": "apsta" }),
            (-35.3635, 149.1652),
            vec![json!({ "from": "a", "to": "b" })],
            vec![json!({ "node": "a", "prefix": "/muas/v3/a/telemetry" })],
        );
        let stats = http_json("GET", &format!("{base}/netstats"), None).await.unwrap();
        assert_eq!(stats["links"].as_array().unwrap().len(), 1);
        assert_eq!(stats["gcs"], json!({ "lat": -35.3635, "lon": 149.1652 }));
        assert_eq!(stats["type"], json!("net"));
        // The namespace-lens feed rides the same document (additive keys).
        assert_eq!(stats["prefixes"].as_array().unwrap().len(), 1);
        assert_eq!(stats["prefix_source"], json!("udp-bridge-taps"));

        // Single-anomaly removal through the dashboard seam (the UI's ✕):
        // remove_anomaly(id) → DELETE /anomalies/{id} → field truth.
        let removed = control
            .call("remove_anomaly".into(), json!({ "id": id }))
            .await
            .expect("seam remove");
        assert_eq!(removed["removed"], json!(id));
        assert_eq!(field.snapshot().len(), 1, "only the named anomaly went");
        // Unknown ids surface a typed 404, not a silent no-op.
        let err = control
            .call("remove_anomaly".into(), json!({ "id": "anom-nope" }))
            .await
            .unwrap_err();
        assert!(err.contains("404"), "{err}");
        let cleared = control.call("clear_anomalies".into(), json!({})).await.unwrap();
        assert_eq!(cleared["cleared"], 1);
        assert!(field.snapshot().is_empty());

        // Bad placements are typed 4xx errors, not silent drops.
        let err = http_json("POST", &format!("{base}/anomalies"), Some(&json!({"kind":"nope"})))
            .await
            .unwrap_err();
        assert!(err.contains("400"), "{err}");
        cancel.cancel();
    }
}
