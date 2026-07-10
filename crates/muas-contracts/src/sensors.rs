//! Sensor metadata riding the capability profile — the dashboard's sensor
//! visualization layer (FoV quads, DRI rings, audio reach) renders from
//! THESE facts, not hard-coded airframe knowledge.
//!
//! # Schema position (additive only)
//!
//! `uas_fleet_data::kinds::CapabilityProfile` is a pinned v2 wire type in a
//! sibling repo this increment does not touch. `SensorMeta` therefore rides
//! the profile JSON as one ADDITIVE top-level key (`"sensor_meta"`): the
//! agent serializes its profile, merges the key in
//! ([`merge_into_profile`]), and consumers that predate the key ignore it
//! (serde tolerates unknown fields; the v2 Python readers used `.get`).
//! Consumers that know it read it back with [`from_profile_json`].
//!
//! Everything is `#[serde(default)]` so partial advertisements parse; a
//! vehicle with no camera simply omits `camera`.

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Camera facts for FoV/DRI rendering and nadir projection.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CameraMeta {
    /// Horizontal field of view, degrees.
    pub hfov_deg: f64,
    /// Detection / recognition / identification ranges, metres
    /// (`[d, r, i]`; missing entries mean "not characterized").
    #[serde(default)]
    pub dri_m: Vec<f64>,
    /// Native frame width, pixels (0 = unknown).
    #[serde(default)]
    pub width_px: u32,
    /// Native frame height, pixels (0 = unknown).
    #[serde(default)]
    pub height_px: u32,
}

/// One directional beamforming lobe (arrays that report them).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AudioLobe {
    /// Lobe center bearing relative to the vehicle nose, degrees.
    pub bearing_deg: f64,
    /// Full lobe width, degrees.
    pub width_deg: f64,
    /// Useful range along the lobe, metres.
    pub range_m: f64,
}

/// Microphone facts: beamforming lobes when the array reports them, else
/// an omnidirectional confidence radius.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AudioMeta {
    /// Omnidirectional useful range, metres (the fallback circle).
    #[serde(default)]
    pub omni_range_m: f64,
    /// Beamforming lobes; empty = render the omni circle.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lobes: Vec<AudioLobe>,
}

/// The additive `sensor_meta` capability-profile key. Sensors are dynamic:
/// every field is optional and unknown future keys must be tolerated.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct SensorMeta {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<CameraMeta>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub audio: Option<AudioMeta>,
}

impl SensorMeta {
    /// True when nothing is advertised (skip the merge).
    pub fn is_empty(&self) -> bool {
        self.camera.is_none() && self.audio.is_none()
    }
}

/// Merge `meta` into a serialized capability profile as the additive
/// `"sensor_meta"` key. No-op on an empty meta or a non-object profile.
pub fn merge_into_profile(profile_json: &mut Value, meta: &SensorMeta) {
    if meta.is_empty() {
        return;
    }
    if let (Some(map), Ok(value)) = (profile_json.as_object_mut(), serde_json::to_value(meta)) {
        map.insert("sensor_meta".to_string(), value);
    }
}

/// Read the `"sensor_meta"` key back out of a profile JSON document.
/// `None` when absent or unparseable (legacy profiles).
pub fn from_profile_json(profile_json: &Value) -> Option<SensorMeta> {
    serde_json::from_value(profile_json.get("sensor_meta")?.clone()).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn merge_is_additive_and_round_trips() {
        // A legacy-shaped profile document (only the fields matter).
        let mut profile = json!({
            "vehicle_id": "wuas-01",
            "extras": ["raster", "camera"],
            "gps_time_ns": 1u64,
        });
        let meta = SensorMeta {
            camera: Some(CameraMeta {
                hfov_deg: 66.0,
                dri_m: vec![40.0, 20.0, 8.0],
                width_px: 320,
                height_px: 240,
            }),
            audio: Some(AudioMeta {
                omni_range_m: 30.0,
                lobes: Vec::new(),
            }),
        };
        merge_into_profile(&mut profile, &meta);
        // Legacy fields untouched (additive rule).
        assert_eq!(profile["vehicle_id"], "wuas-01");
        assert_eq!(profile["extras"][1], "camera");
        let back = from_profile_json(&profile).expect("sensor_meta present");
        assert_eq!(back, meta);
    }

    #[test]
    fn empty_meta_does_not_pollute_the_profile() {
        let mut profile = json!({ "vehicle_id": "iuas-01" });
        merge_into_profile(&mut profile, &SensorMeta::default());
        assert!(profile.get("sensor_meta").is_none());
        assert!(from_profile_json(&profile).is_none());
    }

    #[test]
    fn partial_advertisements_parse_with_defaults() {
        let meta: SensorMeta =
            serde_json::from_value(json!({ "audio": { "omni_range_m": 25.0 } })).unwrap();
        assert!(meta.camera.is_none());
        let audio = meta.audio.unwrap();
        assert_eq!(audio.omni_range_m, 25.0);
        assert!(audio.lobes.is_empty(), "missing lobes default empty");
    }
}
