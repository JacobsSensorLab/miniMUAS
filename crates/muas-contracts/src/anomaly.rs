//! Simulation-anomaly wire types: the ground truth a virtual deployment
//! owns (`muas-sim`'s `AnomalyField`) and the agents' synthetic sensors
//! query OVER THE NETWORK (latest-wins fetch of
//! [`crate::names::sim_stream`]`("anomalies")`).
//!
//! Lives in `muas-contracts` because both sides of that wire (`muas-sim`
//! serves, `muas-agent` consumes) must agree on the bytes, and the agent
//! cannot depend on the sim crate.
//!
//! Pluggability: the enum is serde-tagged (`"kind"`), so new anomaly types
//! are additive — old consumers skip entries they cannot parse (fetchers
//! decode entry-by-entry via [`decode_lossy`]).

use serde::{Deserialize, Serialize};

/// One placed anomaly. `id` is assigned by the owning field ("" on a
/// placement request means "assign one"); `created_ns` is the owner's clock.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Anomaly {
    /// A ground blob a nadir camera can see.
    Visual {
        #[serde(default)]
        id: String,
        lat_deg: f64,
        lon_deg: f64,
        /// Blob diameter, metres.
        size_m: f64,
        /// Visual signature (color name the renderer/detector agree on:
        /// `"red"`, `"orange"`, `"blue"`, `"magenta"`, `"yellow"`).
        #[serde(default)]
        signature: String,
        #[serde(default)]
        created_ns: u64,
    },
    /// A point sound source a microphone can hear.
    Audio {
        #[serde(default)]
        id: String,
        lat_deg: f64,
        lon_deg: f64,
        /// Source level, dB (drives tone energy in synthetic WAVs).
        loudness_db: f64,
        /// Audio signature (tone family; hashed to a frequency).
        #[serde(default)]
        signature: String,
        #[serde(default)]
        created_ns: u64,
    },
}

impl Anomaly {
    pub fn id(&self) -> &str {
        match self {
            Self::Visual { id, .. } | Self::Audio { id, .. } => id,
        }
    }

    pub fn set_id(&mut self, new_id: String) {
        match self {
            Self::Visual { id, .. } | Self::Audio { id, .. } => *id = new_id,
        }
    }

    pub fn set_created_ns(&mut self, ns: u64) {
        match self {
            Self::Visual { created_ns, .. } | Self::Audio { created_ns, .. } => *created_ns = ns,
        }
    }

    /// `(lat_deg, lon_deg)`.
    pub fn position(&self) -> (f64, f64) {
        match self {
            Self::Visual { lat_deg, lon_deg, .. } | Self::Audio { lat_deg, lon_deg, .. } => {
                (*lat_deg, *lon_deg)
            }
        }
    }

    /// The serde tag (`"visual"` / `"audio"`).
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Visual { .. } => "visual",
            Self::Audio { .. } => "audio",
        }
    }
}

/// Decode an anomaly-list JSON document entry-by-entry, skipping entries of
/// unknown kinds — the forward-compatibility contract of the tagged enum.
pub fn decode_lossy(bytes: &[u8]) -> Vec<Anomaly> {
    let Ok(values) = serde_json::from_slice::<Vec<serde_json::Value>>(bytes) else {
        return Vec::new();
    };
    values
        .into_iter()
        .filter_map(|v| serde_json::from_value(v).ok())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tagged_round_trip() {
        let list = vec![
            Anomaly::Visual {
                id: "a-1".into(),
                lat_deg: 35.0,
                lon_deg: 149.0,
                size_m: 4.0,
                signature: "red".into(),
                created_ns: 7,
            },
            Anomaly::Audio {
                id: "a-2".into(),
                lat_deg: 35.001,
                lon_deg: 149.001,
                loudness_db: 80.0,
                signature: "siren".into(),
                created_ns: 8,
            },
        ];
        let bytes = serde_json::to_vec(&list).unwrap();
        assert_eq!(decode_lossy(&bytes), list);
        let json = String::from_utf8(bytes).unwrap();
        assert!(json.contains(r#""kind":"visual""#), "serde tag on the wire: {json}");
    }

    #[test]
    fn unknown_kinds_are_skipped_not_fatal() {
        let bytes = br#"[
            {"kind":"thermal","id":"x","lat_deg":1.0,"lon_deg":2.0,"peak_c":90.0},
            {"kind":"visual","id":"a","lat_deg":35.0,"lon_deg":149.0,"size_m":2.0}
        ]"#;
        let decoded = decode_lossy(bytes);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].id(), "a");
        assert_eq!(decoded[0].kind(), "visual");
    }
}
