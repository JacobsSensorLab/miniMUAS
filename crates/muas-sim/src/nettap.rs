//! Per-prefix traffic accounting at the deployment's UDP bridge seams —
//! the measurement source of the dashboard's **namespace lens**
//! (`docs/v3/NETWORK-VIZ.md` Revision 2, R2a).
//!
//! # Why the taps live here (measurability, documented)
//!
//! The fabric itself exports only per-face totals
//! ([`ndn_sim::FaceStats`] — counters, no names), and `SimLink`s shuttle
//! opaque frames. The one seam where every packet's NAME is visible
//! without modifying sibling crates is the UDP bridge [`crate::FleetSim`]
//! builds itself: agent-engine ↔ fabric-node (one per vehicle) and
//! dashboard ↔ console node. A tap is a tiny loopback UDP relay
//! interposed on that seam: it forwards each datagram untouched and, on
//! the side, decodes the L3 name (NDNLPv2-aware) and counts
//! bytes/packets per `(node, prefix)`.
//!
//! Coverage: every fabric traversal starts or ends at a tapped seam, so
//! per-node per-prefix emission ("out") and delivery ("in") are complete
//! — vehicle↔vehicle traffic is seen at both vehicles' taps, GCS traffic
//! at the console tap. NOT visible here: hop-by-hop paths inside the
//! fabric (that is R2b's span feed) and content a node's own engine
//! answers from cache (it never crosses a bridge). Radio-layer truths
//! stay out of scope entirely (the never-synthesize rule).
//!
//! # Prefix grouping
//!
//! Names under the app tree keep FOUR components —
//! `/muas/v3/<subject>/<stream-head>` — because the fourth component is
//! the semantic namespace the lens exists for (telemetry vs coord vs
//! video vs tasks). Everything else keeps the spec's default of three.
//! Undecodable datagrams (LP continuation fragments, junk) count under
//! `"(unparsed)"` so bytes never silently vanish from the totals.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use ndn_packet::lp::LpPacket;
use ndn_packet::{Data, Interest};
use tokio_util::sync::CancellationToken;

/// Cumulative counters for one `(node, prefix)` pair. `out_*` = emitted
/// by the labeled node toward the fabric; `in_*` = delivered to it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize)]
pub struct PrefixCounters {
    pub out_bytes: u64,
    pub in_bytes: u64,
    pub out_interests: u64,
    pub out_data: u64,
    pub in_interests: u64,
    pub in_data: u64,
}

/// One snapshot row: a node's cumulative counters for one name prefix.
#[derive(Clone, Debug, serde::Serialize)]
pub struct PrefixSample {
    pub node: String,
    pub prefix: String,
    #[serde(flatten)]
    pub counters: PrefixCounters,
}

/// Shared per-prefix counter table, written by every tap, read at 1 Hz by
/// the deployment's net exporter.
#[derive(Default)]
pub struct PrefixStats {
    inner: Mutex<HashMap<(String, String), PrefixCounters>>,
}

impl PrefixStats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    fn count(&self, node: &str, prefix: &str, outbound: bool, kind: WireKind, bytes: usize) {
        let mut map = self.inner.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        let c = map.entry((node.to_string(), prefix.to_string())).or_default();
        if outbound {
            c.out_bytes += bytes as u64;
            match kind {
                WireKind::Interest => c.out_interests += 1,
                WireKind::Data => c.out_data += 1,
                WireKind::Other => {}
            }
        } else {
            c.in_bytes += bytes as u64;
            match kind {
                WireKind::Interest => c.in_interests += 1,
                WireKind::Data => c.in_data += 1,
                WireKind::Other => {}
            }
        }
    }

    /// Deterministically ordered snapshot of every `(node, prefix)` row.
    pub fn snapshot(&self) -> Vec<PrefixSample> {
        let map = self.inner.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut rows: Vec<PrefixSample> = map
            .iter()
            .map(|((node, prefix), counters)| PrefixSample {
                node: node.clone(),
                prefix: prefix.clone(),
                counters: *counters,
            })
            .collect();
        rows.sort_by(|a, b| (&a.node, &a.prefix).cmp(&(&b.node, &b.prefix)));
        rows
    }
}

/// L3 packet classification for the counters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WireKind {
    Interest,
    Data,
    Other,
}

/// Decode one datagram far enough to learn its kind and name. LP packets
/// are unwrapped one level: the first fragment carries the inner L3
/// header (name included); continuations are unattributable by design.
fn classify(wire: &[u8]) -> (WireKind, Option<String>) {
    match wire.first() {
        Some(0x05) => match Interest::decode(bytes::Bytes::copy_from_slice(wire)) {
            Ok(i) => (WireKind::Interest, Some(i.name.to_string())),
            Err(_) => (WireKind::Other, None),
        },
        Some(0x06) => match Data::decode(bytes::Bytes::copy_from_slice(wire)) {
            Ok(d) => (WireKind::Data, Some(d.name.to_string())),
            Err(_) => (WireKind::Other, None),
        },
        Some(0x64) => match LpPacket::decode(bytes::Bytes::copy_from_slice(wire)) {
            // Only the FIRST fragment carries the inner header; later
            // fragments count as unparsed bytes (never dropped from the
            // totals, never mis-attributed to a name).
            Ok(lp) if lp.frag_index.unwrap_or(0) == 0 => match lp.fragment {
                Some(inner) => classify(&inner),
                None => (WireKind::Other, None),
            },
            _ => (WireKind::Other, None),
        },
        _ => (WireKind::Other, None),
    }
}

/// Group a name URI into its accounting prefix: 4 components under the
/// `/muas/...` app tree (`/muas/v3/<subject>/<stream-head>` — the
/// semantic namespace), 3 elsewhere (the R2a default).
pub fn group_prefix(uri: &str) -> String {
    let comps: Vec<&str> = uri.split('/').filter(|c| !c.is_empty()).collect();
    if comps.is_empty() {
        return "/".into();
    }
    let take = if comps[0] == "muas" { comps.len().min(4) } else { comps.len().min(3) };
    format!("/{}", comps[..take].join("/"))
}

/// Interpose a counting relay between `node_side` (an agent/dashboard
/// engine's UDP face) and `fabric_side` (the ndn-sim bridge socket).
/// Returns the tap's own address: point BOTH peers at it — datagrams are
/// routed by source address and forwarded byte-for-byte.
pub async fn spawn_tap(
    label: impl Into<String>,
    stats: Arc<PrefixStats>,
    node_side: SocketAddr,
    fabric_side: SocketAddr,
    cancel: CancellationToken,
) -> Result<SocketAddr, String> {
    let socket = tokio::net::UdpSocket::bind("127.0.0.1:0")
        .await
        .map_err(|e| format!("net tap bind: {e}"))?;
    let addr = socket.local_addr().map_err(|e| format!("net tap addr: {e}"))?;
    let label = label.into();
    tokio::spawn(async move {
        let mut buf = vec![0u8; 65_535];
        loop {
            tokio::select! {
                () = cancel.cancelled() => break,
                received = socket.recv_from(&mut buf) => {
                    let Ok((n, src)) = received else { break };
                    // Route by source: the node side's datagrams are that
                    // node's emissions; the fabric side's are deliveries.
                    let (dst, outbound) = if src == node_side {
                        (fabric_side, true)
                    } else if src == fabric_side {
                        (node_side, false)
                    } else {
                        continue; // stray datagram: not ours to relay
                    };
                    let (kind, name) = classify(&buf[..n]);
                    let prefix = name
                        .as_deref()
                        .map(group_prefix)
                        .unwrap_or_else(|| "(unparsed)".into());
                    stats.count(&label, &prefix, outbound, kind, n);
                    let _ = socket.send_to(&buf[..n], dst).await;
                }
            }
        }
    });
    Ok(addr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndn_packet::encode::{encode_data_unsigned, encode_interest};
    use ndn_packet::Name;

    #[test]
    fn prefix_grouping_keeps_the_semantic_component_under_the_app_tree() {
        // App names keep 4 components: the stream head IS the namespace.
        assert_eq!(
            group_prefix("/muas/v3/iuas-01/telemetry/live"),
            "/muas/v3/iuas-01/telemetry"
        );
        assert_eq!(
            group_prefix("/muas/v3/wuas-01/video/live/7"),
            "/muas/v3/wuas-01/video"
        );
        assert_eq!(group_prefix("/muas/v3/sim/anomalies"), "/muas/v3/sim/anomalies");
        // Short app names never panic or pad.
        assert_eq!(group_prefix("/muas/v3"), "/muas/v3");
        // Foreign trees keep the R2a default of 3 components.
        assert_eq!(group_prefix("/edu/ucla/data/seg/0"), "/edu/ucla/data");
        assert_eq!(group_prefix("/a"), "/a");
        assert_eq!(group_prefix("/"), "/");
    }

    #[test]
    fn classify_reads_names_through_lp_wrapping() {
        let name: Name = "/muas/v3/iuas-01/telemetry/live".parse().expect("name");
        let interest = encode_interest(&name, None);
        let (kind, got) = classify(&interest);
        assert_eq!(kind, WireKind::Interest);
        assert_eq!(got.as_deref(), Some("/muas/v3/iuas-01/telemetry/live"));

        let data = encode_data_unsigned(&name, b"payload");
        let (kind, got) = classify(&data);
        assert_eq!(kind, WireKind::Data);
        assert_eq!(got.as_deref(), Some("/muas/v3/iuas-01/telemetry/live"));

        // Hand-rolled LPv2 wrap: LP_PACKET(0x64) { Fragment(0x50) { data } }.
        let mut lp = vec![0x64, (data.len() + 2) as u8, 0x50, data.len() as u8];
        lp.extend_from_slice(&data);
        let (kind, got) = classify(&lp);
        assert_eq!(kind, WireKind::Data);
        assert_eq!(got.as_deref(), Some("/muas/v3/iuas-01/telemetry/live"));

        // Junk counts as unattributable, never panics.
        assert_eq!(classify(&[0xff, 0x00]).0, WireKind::Other);
        assert_eq!(classify(&[]).0, WireKind::Other);
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn tap_relays_bytes_verbatim_and_counts_per_prefix() {
        let node = tokio::net::UdpSocket::bind("127.0.0.1:0").await.expect("node sock");
        let fabric = tokio::net::UdpSocket::bind("127.0.0.1:0").await.expect("fabric sock");
        let node_addr = node.local_addr().unwrap();
        let fabric_addr = fabric.local_addr().unwrap();
        let stats = PrefixStats::new();
        let cancel = CancellationToken::new();
        let tap = spawn_tap("iuas-01", stats.clone(), node_addr, fabric_addr, cancel.clone())
            .await
            .expect("tap up");

        // Node → fabric: an interest, counted as the node's emission.
        let name: Name = "/muas/v3/wuas-01/telemetry/live".parse().unwrap();
        let interest = encode_interest(&name, None);
        node.send_to(&interest, tap).await.expect("send interest");
        let mut buf = [0u8; 2048];
        let (n, from) = tokio::time::timeout(
            std::time::Duration::from_secs(5),
            fabric.recv_from(&mut buf),
        )
        .await
        .expect("interest forwarded")
        .expect("recv ok");
        assert_eq!(from, tap, "relay preserves the tap as the visible peer");
        assert_eq!(&buf[..n], &interest[..], "bytes forwarded verbatim");

        // Fabric → node: the data back, counted as a delivery.
        let data = encode_data_unsigned(&name, b"sample");
        fabric.send_to(&data, tap).await.expect("send data");
        let (n, _) = tokio::time::timeout(
            std::time::Duration::from_secs(5),
            node.recv_from(&mut buf),
        )
        .await
        .expect("data forwarded")
        .expect("recv ok");
        assert_eq!(&buf[..n], &data[..]);

        let rows = stats.snapshot();
        assert_eq!(rows.len(), 1, "one (node, prefix) row: {rows:?}");
        let row = &rows[0];
        assert_eq!(row.node, "iuas-01");
        assert_eq!(row.prefix, "/muas/v3/wuas-01/telemetry");
        assert_eq!(row.counters.out_interests, 1);
        assert_eq!(row.counters.in_data, 1);
        assert_eq!(row.counters.out_bytes, interest.len() as u64);
        assert_eq!(row.counters.in_bytes, data.len() as u64);
        cancel.cancel();
    }
}
