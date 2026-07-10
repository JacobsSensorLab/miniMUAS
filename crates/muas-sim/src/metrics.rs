//! Measurement helpers: percentile summaries, the spark-lane receiver, and
//! the UDP impairment relay that subjects the (non-NDN) spark lane to the
//! same link profile as the fabric's SimLinks.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use tokio::net::UdpSocket;
use tokio_util::sync::CancellationToken;

/// Nearest-rank percentile over an unsorted sample set (destructive sort).
pub fn percentile(samples: &mut [f64], p: f64) -> f64 {
    if samples.is_empty() {
        return f64::NAN;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let rank = ((p / 100.0) * samples.len() as f64).ceil().max(1.0) as usize;
    samples[rank.min(samples.len()) - 1]
}

/// p50/p95 summary of a sample set (milliseconds by convention).
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Summary {
    pub p50: f64,
    pub p95: f64,
    pub n: usize,
}

/// Summarize a sample set (NaN percentiles when empty).
pub fn summarize(samples: &mut [f64]) -> Summary {
    Summary {
        p50: percentile(samples, 50.0),
        p95: percentile(samples, 95.0),
        n: samples.len(),
    }
}

/// Spark-lane receiver counters, updated live by the receiver task.
#[derive(Debug, Default)]
pub struct SparkStats {
    pub received: u64,
    pub decode_errors: u64,
    pub min_seq: Option<u64>,
    pub max_seq: Option<u64>,
}

impl SparkStats {
    /// Sequence span covered by what arrived (`max - min + 1`).
    pub fn span(&self) -> u64 {
        match (self.min_seq, self.max_seq) {
            (Some(min), Some(max)) => max - min + 1,
            _ => 0,
        }
    }

    /// Frame loss estimated from sequence gaps; `None` before any frame.
    pub fn loss_rate(&self) -> Option<f64> {
        let span = self.span();
        (span > 0).then(|| 1.0 - (self.received as f64 / span as f64))
    }
}

/// Receive spark datagrams on `socket`, decode [`ndf_spark::SparkPayload`],
/// and track sequence coverage in `stats` until `cancel`.
pub fn spawn_spark_receiver(
    socket: UdpSocket,
    stats: Arc<Mutex<SparkStats>>,
    cancel: CancellationToken,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let mut buf = vec![0u8; 65536];
        loop {
            let received = tokio::select! {
                _ = cancel.cancelled() => break,
                r = socket.recv_from(&mut buf) => r,
            };
            let Ok((len, _)) = received else { break };
            let mut guard = stats.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
            match ndf_spark::SparkPayload::decode(&buf[..len]) {
                Ok(payload) => {
                    guard.received += 1;
                    guard.min_seq = Some(guard.min_seq.map_or(payload.seq, |m| m.min(payload.seq)));
                    guard.max_seq = Some(guard.max_seq.map_or(payload.seq, |m| m.max(payload.seq)));
                }
                Err(_) => guard.decode_errors += 1,
            }
        }
    })
}

/// One-way UDP impairment relay: datagrams arriving on `listen` are dropped
/// with probability `loss_rate`, else forwarded to `dest` after
/// `delay + U[0, jitter]`. This applies the *same* `LinkConfig` numbers the
/// fabric's SimLinks use to the raw-UDP spark lane, which cannot ride a
/// SimLink (ndn-sim carries NDN frames between engines, not foreign flows).
pub fn spawn_impairment_relay(
    listen: UdpSocket,
    dest: SocketAddr,
    loss_rate: f64,
    delay: Duration,
    jitter: Duration,
    cancel: CancellationToken,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let out = match UdpSocket::bind("127.0.0.1:0").await {
            Ok(socket) => Arc::new(socket),
            Err(err) => {
                tracing::warn!(%err, "impairment relay: bind failed");
                return;
            }
        };
        let mut buf = vec![0u8; 65536];
        loop {
            let received = tokio::select! {
                _ = cancel.cancelled() => break,
                r = listen.recv_from(&mut buf) => r,
            };
            let Ok((len, _)) = received else { break };
            if fastrand::f64() < loss_rate {
                continue;
            }
            let wait = delay + jitter.mul_f64(fastrand::f64());
            let bytes = buf[..len].to_vec();
            let out = out.clone();
            tokio::spawn(async move {
                tokio::time::sleep(wait).await;
                let _ = out.send_to(&bytes, dest).await;
            });
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentile_nearest_rank() {
        let mut samples: Vec<f64> = (1..=100).map(f64::from).collect();
        assert_eq!(percentile(&mut samples, 50.0), 50.0);
        assert_eq!(percentile(&mut samples, 95.0), 95.0);
        let mut one = vec![7.0];
        assert_eq!(percentile(&mut one, 50.0), 7.0);
        assert_eq!(percentile(&mut one, 95.0), 7.0);
        let mut none: Vec<f64> = Vec::new();
        assert!(percentile(&mut none, 50.0).is_nan());
    }

    #[test]
    fn spark_stats_loss_from_seq_span() {
        let stats = SparkStats {
            received: 90,
            decode_errors: 0,
            min_seq: Some(0),
            max_seq: Some(99),
        };
        assert_eq!(stats.span(), 100);
        let loss = stats.loss_rate().unwrap();
        assert!((loss - 0.10).abs() < 1e-9, "loss {loss}");
        assert!(SparkStats::default().loss_rate().is_none());
    }
}
