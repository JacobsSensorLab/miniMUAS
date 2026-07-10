//! The resolver layer: one [`Mission`] = the resolved Block set of one run.
//!
//! Two transports, ONE data identity:
//!
//! - **Live (`--bootstrap`)**: bring up a real NDN engine + UDP faces and
//!   fetch every Block of the configured chains *by name* over NDN via
//!   `ndf_apps::AppRuntime` (`follow` → `resolve_trusted`). No file reads —
//!   the only file is the bootstrap itself (endpoint + chain addresses +
//!   identity key).
//! - **Offline (`--from-journal`)**: read the power-loss-safe JSONL journals
//!   and *republish them through the identical `AppRuntime::publish` path*
//!   into in-process chains, then resolve those. The Block hashing rules are
//!   byte-identical (deterministic header + deterministic Ed25519), so the
//!   provenance math is the same — **the transport differs, the data
//!   identity doesn't**. (An offline writer key is derived per chain, so
//!   offline hashes are stable across re-runs of the same journals; a live
//!   agent's own boot key produces the live hashes.)
//!
//! Also here: [`mirror_lines_into_chain`], the standalone chain-mirror the
//! dashboard does not yet grow natively — it ingests a recording JSONL into
//! an `AppRuntime` chain exactly the way `muas-agent`'s journal mirror
//! batches its events (2-second windows). The dashboard should eventually do
//! this itself at record time; until then this helper makes recordings
//! first-class named data too.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use ed25519_dalek::SigningKey;
use ndf_apps::{make_reachable, AppRuntime, ChainAddress, Follow, Identity, PublishReceipt};
use ndf_core::hash_of;
use ndf_policy::presence::{AttachmentIntent, DeviceDimensions, UsageClass};
use ndn_engine::builder::{EngineBuilder, EngineConfig};
use ndn_engine::ShutdownHandle;
use ndn_face::UdpFace;
use serde::Deserialize;
use tokio_util::sync::CancellationToken;
use tracing::{debug, info};

use crate::dataset::{from_hex, hex, BlockRef, MissionDataset};

/// What a chain holds (selects the JSONL decoder).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChainRole {
    /// A vehicle agent's journal chain (`muas-agent --journal-chain`).
    VehicleJournal,
    /// A dashboard recording mirrored into a chain.
    DashboardRecording,
}

/// Journal batching window — matches `muas-agent`'s chain mirror
/// (`CHAIN_BATCH_WINDOW`), so offline re-publication cuts Blocks on the same
/// boundaries the live mirror uses.
pub const BATCH_WINDOW_NS: u64 = 2_000_000_000;

// ───────────────────────────── bootstrap config ─────────────────────────────

/// One point-to-point UDP face in the bootstrap.
#[derive(Clone, Debug, Deserialize)]
pub struct LinkSpec {
    /// Local bind address.
    pub local: std::net::SocketAddr,
    /// Remote peer address.
    pub remote: std::net::SocketAddr,
}

/// This process's NDN identity (the *reader*, not the chain writers).
#[derive(Clone, Debug, Deserialize)]
pub struct IdentitySpec {
    /// Principal namespace, e.g. `/muas/v3/gcs`.
    pub principal: String,
    /// Device name, e.g. `artifacts`.
    pub device: String,
    /// 32-byte Ed25519 seed, hex.
    pub key_seed_hex: String,
}

/// One chain to resolve.
#[derive(Clone, Debug, Deserialize)]
pub struct ChainCfg {
    /// `vehicle-journal` or `dashboard-recording`.
    pub role: String,
    /// Chain root name.
    pub root: String,
    /// Writer SVS name (e.g. `/muas/v3/iuas-01/companion`).
    pub writer: String,
    /// Writer's pinned Ed25519 public key, hex — trust is this pin.
    pub writer_key_hex: String,
}

fn default_settle_ms() -> u64 {
    300
}
fn default_max_rounds() -> usize {
    240
}
fn default_quiet_rounds() -> usize {
    3
}
fn default_step_timeout_ms() -> u64 {
    1_000
}

/// The live resolver's bootstrap: the ONLY file the live path reads.
#[derive(Clone, Debug, Deserialize)]
pub struct Bootstrap {
    /// Our identity.
    pub identity: IdentitySpec,
    /// UDP faces toward the fleet / forwarder.
    #[serde(default)]
    pub links: Vec<LinkSpec>,
    /// Chains to resolve.
    pub chains: Vec<ChainCfg>,
    /// Face settle time before following (ms).
    #[serde(default = "default_settle_ms")]
    pub settle_ms: u64,
    /// Per-chain follow round budget.
    #[serde(default = "default_max_rounds")]
    pub max_rounds: usize,
    /// Consecutive silent rounds that end a chain's catch-up.
    #[serde(default = "default_quiet_rounds")]
    pub quiet_rounds: usize,
    /// Bound on the *wait* for the next sync update per round (ms). Bounds
    /// the wait, never the work (NS-9) — raise it for very lossy links.
    #[serde(default = "default_step_timeout_ms")]
    pub step_timeout_ms: u64,
}

impl Bootstrap {
    /// Parse from JSON.
    pub fn from_json(text: &str) -> Result<Self, String> {
        serde_json::from_str(text).map_err(|e| format!("bootstrap: {e}"))
    }
}

fn chain_spec(cfg: &ChainCfg) -> Result<(ChainRole, ChainAddress), String> {
    let role = match cfg.role.as_str() {
        "vehicle-journal" => ChainRole::VehicleJournal,
        "dashboard-recording" => ChainRole::DashboardRecording,
        other => return Err(format!("chain role '{other}': expected vehicle-journal|dashboard-recording")),
    };
    let writer_key = from_hex::<32>(&cfg.writer_key_hex)
        .map_err(|e| format!("chain {}: writer_key_hex: {e}", cfg.root))?;
    Ok((role, ChainAddress { root: cfg.root.clone(), writer: cfg.writer.clone(), writer_key }))
}

fn reader_dims() -> DeviceDimensions {
    DeviceDimensions {
        battery_pct: 100,
        metered_network: false,
        capacity_tight: false,
        attachment: AttachmentIntent::Fixed,
    }
}

// ───────────────────────────── the mission ──────────────────────────────────

/// One audit re-fetch: a Block cold-resolved from its chain and re-hashed.
/// Keyed by `stored` (the content hash IS the identity) — chain names may
/// legitimately repeat across runs of the same fleet.
#[derive(Clone, Debug)]
pub struct RefetchedBlock {
    /// Chain root the Block was re-fetched from.
    pub chain: String,
    /// Chain seq.
    pub seq: u64,
    /// The chain node's stored content hash (hex).
    pub stored: String,
    /// SHA-256 of the re-fetched signed packet (hex).
    pub rehashed: String,
}

/// One run's resolved chains: the runtimes stay attached so provenance can
/// be re-verified (audit = re-fetch from the verified store + re-hash).
pub struct Mission {
    /// The one dataset every artifact renders from.
    pub dataset: Arc<MissionDataset>,
    chains: Vec<(ChainRole, ChainAddress, usize)>,
    runtimes: Vec<AppRuntime>,
    shutdowns: Vec<ShutdownHandle>,
    cancel: CancellationToken,
}

impl Mission {
    /// The chains this mission resolved (role, address).
    pub fn chains(&self) -> impl Iterator<Item = (&ChainRole, &ChainAddress)> {
        self.chains.iter().map(|(role, addr, _)| (role, addr))
    }

    /// Cold re-fetch + re-hash of every Block on every chain. `resolve`
    /// (not `resolve_trusted`) re-verifies each Block's envelope against
    /// the pinned writer key on the way out.
    pub fn refetch_all(&self) -> Result<Vec<RefetchedBlock>, String> {
        let mut out = Vec::new();
        for (_, addr, idx) in &self.chains {
            let blocks = self.runtimes[*idx]
                .resolve(addr)
                .map_err(|e| format!("audit re-fetch {}: {e:?}", addr.root))?;
            for rb in blocks {
                out.push(RefetchedBlock {
                    chain: addr.root.clone(),
                    seq: rb.node.seq,
                    stored: hex(&rb.node.content_hash),
                    rehashed: hex(&hash_of(&rb.packet)),
                });
            }
        }
        Ok(out)
    }

    /// Tear down engines and tasks.
    pub async fn shutdown(self) {
        self.cancel.cancel();
        for s in self.shutdowns {
            s.shutdown().await;
        }
    }

    fn build_dataset(
        runtimes: &[AppRuntime],
        chains: &[(ChainRole, ChainAddress, usize)],
    ) -> Result<MissionDataset, String> {
        let mut ds = MissionDataset::new();
        for (role, addr, idx) in chains {
            let blocks = runtimes[*idx]
                .resolve_trusted(addr)
                .map_err(|e| format!("resolve {}: {e:?}", addr.root))?;
            for rb in blocks {
                let r = BlockRef {
                    hash: rb.node.content_hash,
                    chain: addr.root.clone(),
                    seq: rb.node.seq,
                };
                let payload = rb
                    .payload
                    .ok_or_else(|| format!("block {} seq {} has no payload", addr.root, rb.node.seq))?;
                match role {
                    ChainRole::VehicleJournal => ds.add_journal_block(r, &payload),
                    ChainRole::DashboardRecording => ds.add_recording_block(r, &payload),
                }
            }
        }
        ds.finish();
        Ok(ds)
    }
}

// ───────────────────────────── live resolver ────────────────────────────────

/// Drive one follow until the chain stops advancing: `quiet` consecutive
/// silent step windows after at least one replication event (or an already
/// non-empty store) end the catch-up. Budgeted — never an unbounded wait.
async fn drain(follow: &mut Follow, max_rounds: usize, quiet: usize) -> Result<usize, String> {
    follow.attach().await.map_err(|e| format!("follow attach: {e:?}"))?;
    let mut fetched = 0usize;
    let mut silent = 0usize;
    for _ in 0..max_rounds {
        match follow.step().await {
            Ok(Some((_update, events))) => {
                fetched += events.len();
                silent = 0;
            }
            Ok(None) => {
                silent += 1;
                if silent >= quiet && fetched > 0 {
                    return Ok(fetched);
                }
            }
            Err(e) => return Err(format!("follow step: {e:?}")),
        }
    }
    Ok(fetched)
}

/// Resolve a run live over NDN. Mission data arrives ONLY as named Blocks
/// through the engine — this path never opens a journal file.
pub async fn resolve_live(bootstrap: &Bootstrap) -> Result<Mission, String> {
    let seed = from_hex::<32>(&bootstrap.identity.key_seed_hex)
        .map_err(|e| format!("identity.key_seed_hex: {e}"))?;
    let cancel = CancellationToken::new();
    let (engine, shutdown) = EngineBuilder::new(EngineConfig::default())
        .build()
        .await
        .map_err(|e| format!("engine: {e}"))?;

    let mut face_ids = Vec::new();
    for link in &bootstrap.links {
        let id = engine.faces().alloc_id();
        let face = UdpFace::bind(link.local, link.remote, id)
            .await
            .map_err(|e| format!("udp face {} -> {}: {e}", link.local, link.remote))?;
        engine.add_face(face, cancel.child_token());
        face_ids.push(id);
        info!(local = %link.local, remote = %link.remote, "artifact resolver face up");
    }

    let mut chains = Vec::new();
    for cfg in &bootstrap.chains {
        let (role, addr) = chain_spec(cfg)?;
        for id in &face_ids {
            make_reachable(&engine, &addr, *id).map_err(|e| format!("route {}: {e:?}", addr.root))?;
        }
        chains.push((role, addr, 0usize));
    }
    if !face_ids.is_empty() {
        tokio::time::sleep(std::time::Duration::from_millis(bootstrap.settle_ms)).await;
    }

    let identity = Identity::new(
        &bootstrap.identity.principal,
        &bootstrap.identity.device,
        SigningKey::from_bytes(&seed),
    );
    let mut runtime = AppRuntime::attach(engine.clone(), identity, cancel.child_token());
    runtime.set_follow_config(ndf_apps::FollowConfig {
        step_timeout: std::time::Duration::from_millis(bootstrap.step_timeout_ms),
        ..Default::default()
    });

    for (_, addr, _) in &chains {
        let mut follow = runtime
            .follow(addr.clone(), &reader_dims(), UsageClass::Active)
            .await
            .map_err(|e| format!("follow {}: {e:?}", addr.root))?;
        let fetched = drain(&mut follow, bootstrap.max_rounds, bootstrap.quiet_rounds).await?;
        drop(follow);
        let head = runtime
            .head(addr)
            .map_err(|e| format!("head {}: {e:?}", addr.root))?
            .ok_or_else(|| format!("chain {} never converged (no blocks fetched)", addr.root))?;
        debug!(chain = %addr.root, fetched, head_seq = head.seq, "chain drained");
    }

    let runtimes = vec![runtime];
    let dataset = Mission::build_dataset(&runtimes, &chains)?;
    Ok(Mission {
        dataset: Arc::new(dataset),
        chains,
        runtimes,
        shutdowns: vec![shutdown],
        cancel,
    })
}

// ───────────────────────────── chain mirror ─────────────────────────────────

/// Mirror timestamped JSONL lines into an `AppRuntime` chain, one Block per
/// [`BATCH_WINDOW_NS`] window (epoch-aligned), `application/x-ndjson` — the
/// same batching contract as `muas-agent`'s live journal mirror.
///
/// This is the standalone "chain-mirror" the dashboard should grow natively
/// for its recordings: until it does, this ingests a recording after the
/// fact and gives every recorded line a named, content-addressed home.
pub async fn mirror_lines_into_chain(
    runtime: &mut AppRuntime,
    address: &ChainAddress,
    lines: &[(u64, String)],
) -> Result<Vec<PublishReceipt>, String> {
    let mut receipts = Vec::new();
    let mut batch = String::new();
    let mut window: Option<u64> = None;
    for (t_ns, line) in lines {
        let w = t_ns / BATCH_WINDOW_NS;
        if window.is_some_and(|cur| cur != w) && !batch.is_empty() {
            receipts.push(publish_batch(runtime, address, &mut batch).await?);
        }
        window = Some(w);
        batch.push_str(line);
        batch.push('\n');
    }
    if !batch.is_empty() {
        receipts.push(publish_batch(runtime, address, &mut batch).await?);
    }
    Ok(receipts)
}

async fn publish_batch(
    runtime: &mut AppRuntime,
    address: &ChainAddress,
    batch: &mut String,
) -> Result<PublishReceipt, String> {
    let payload = std::mem::take(batch);
    runtime
        .publish(address, "application/x-ndjson", payload.as_bytes())
        .await
        .map_err(|e| format!("publish on {}: {e:?}", address.root))
}

// ───────────────────────────── journal fallback ─────────────────────────────

struct SourceFile {
    role: ChainRole,
    principal: String,
    device: String,
    app: String,
    lines: Vec<(u64, String)>,
}

fn sniff_source(path: &Path) -> Result<Option<SourceFile>, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("read {}: {e}", path.display()))?;
    let mut lines = Vec::new();
    let mut role: Option<ChainRole> = None;
    let mut vehicle: Option<String> = None;
    for raw in text.lines() {
        if raw.trim().is_empty() {
            continue;
        }
        let Ok(v) = serde_json::from_str::<serde_json::Value>(raw) else { continue };
        let (t, this_role) = if v.get("ts_ns").is_some() && v.get("kind").is_some() {
            if vehicle.is_none() {
                vehicle = v["vehicle_id"].as_str().map(str::to_string);
            }
            (v["ts_ns"].as_u64().unwrap_or(0), ChainRole::VehicleJournal)
        } else if v.get("t_ns").is_some() && v.get("event").is_some() {
            (v["t_ns"].as_u64().unwrap_or(0), ChainRole::DashboardRecording)
        } else {
            continue;
        };
        role.get_or_insert(this_role);
        lines.push((t, raw.to_string()));
    }
    let Some(role) = role else { return Ok(None) };
    let stem = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("source")
        .replace(|c: char| !c.is_ascii_alphanumeric() && c != '-', "-");
    Ok(Some(match role {
        ChainRole::VehicleJournal => {
            // Mirror the live agent's chain layout:
            // root = /muas/v3/<vid>/journal/companion.
            let vid = vehicle.unwrap_or_else(|| stem.clone());
            SourceFile {
                role,
                principal: muas_contracts::names::vehicle_prefix(&vid),
                device: "companion".into(),
                app: "journal".into(),
                lines,
            }
        }
        ChainRole::DashboardRecording => SourceFile {
            role,
            principal: "/muas/v3/gcs".into(),
            device: stem,
            app: "recording".into(),
            lines,
        },
    }))
}

/// Deterministic offline writer key for a chain: stable across re-runs of
/// the same journals, so offline provenance hashes are reproducible.
fn offline_seed(principal: &str, device: &str) -> [u8; 32] {
    hash_of(format!("muas-artifacts/offline-writer:{principal}/{device}").as_bytes())
}

/// Build a run's mission from a directory of JSONL files — agent journals
/// (`{"kind","ts_ns",..}`) and dashboard recordings (`{"t_ns","event"}`),
/// sniffed by shape. Each file is republished through the real
/// `AppRuntime::publish` path (verify + gate + store), then resolved exactly
/// like the live path — so every datum carries a true Block hash.
pub async fn from_journal_dir(dir: &Path) -> Result<Mission, String> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("read dir {}: {e}", dir.display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "jsonl"))
        .collect();
    paths.sort();
    if paths.is_empty() {
        return Err(format!("{}: no .jsonl files", dir.display()));
    }

    let cancel = CancellationToken::new();
    let (engine, shutdown) = EngineBuilder::new(EngineConfig::default())
        .build()
        .await
        .map_err(|e| format!("engine: {e}"))?;

    let mut runtimes = Vec::new();
    let mut chains = Vec::new();
    for path in &paths {
        let Some(source) = sniff_source(path)? else {
            continue;
        };
        let identity = Identity::new(
            &source.principal,
            &source.device,
            SigningKey::from_bytes(&offline_seed(&source.principal, &source.device)),
        );
        let mut runtime = AppRuntime::attach(engine.clone(), identity, cancel.child_token());
        let address = runtime.identity().chain(&source.app);
        let receipts = mirror_lines_into_chain(&mut runtime, &address, &source.lines).await?;
        debug!(file = %path.display(), chain = %address.root, blocks = receipts.len(),
               "journal republished into chain");
        chains.push((source.role, address, runtimes.len()));
        runtimes.push(runtime);
    }
    if chains.is_empty() {
        return Err(format!("{}: no journal or recording lines found", dir.display()));
    }

    let dataset = Mission::build_dataset(&runtimes, &chains)?;
    Ok(Mission {
        dataset: Arc::new(dataset),
        chains,
        runtimes,
        shutdowns: vec![shutdown],
        cancel,
    })
}
