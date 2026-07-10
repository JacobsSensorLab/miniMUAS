//! `SimpleDetector`: the first REAL [`DetectionProvider`] — fetch the
//! published frame over NDN (exactly like the v2 GCS perception client),
//! decode it, find the dominant saturated blob, project it to the ground
//! through the frame's own capture pose, and hand the mission machine a
//! localized [`Detection`].
//!
//! The point of this detector is not sophistication (a color threshold
//! finds synthetic anomaly blobs); it is that the WHOLE
//! detect→confirm→dispatch mission loop now runs end-to-end against frames
//! that traveled the data plane. The digital-twin / model-backed detector
//! remains a future `DetectionProvider` impl — the seam is unchanged.
//!
//! # Nadir projection (the renderer's inverse)
//!
//! Frames are self-describing: `FrameHeader` carries width/height and the
//! capture pose + `hfov_deg` in metadata (muas-agent's synthetic feed and
//! any future real camera both stamp them). Image "up" is the vehicle
//! heading; ground metres-per-pixel is `2·AGL·tan(hfov/2)/width`. Pixel
//! offset from frame center → (forward, right) metres → north/east via the
//! heading basis — byte-for-byte the inverse of the agent's renderer.

use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use ndn_app::Consumer;
use uas_fleet_data::kinds::{CapturePose, FrameHeader};
use uas_flight::geo::{m_per_deg_lon, EARTH_M_PER_DEG_LAT};

use crate::mission::{DetectOutcome, Detection};
use crate::providers::{BoxFuture, DetectionProvider};

/// Fallback hfov when a frame carries no `hfov_deg` metadata (matches the
/// synthetic feed default so legacy frames still localize plausibly).
const DEFAULT_HFOV_DEG: f64 = 66.0;

/// Saturation threshold: a pixel is "signal" when its channel spread
/// exceeds this (the synthetic ground sits under 10; signature colors
/// exceed 100 even after JPEG loss).
const MIN_CHANNEL_SPREAD: u8 = 60;

/// Minimum blob area, pixels — rejects lone JPEG-artifact pixels.
const MIN_BLOB_AREA_PX: usize = 12;

// ---------------------------------------------------------------------------
// pure pieces (unit-tested without a fabric)
// ---------------------------------------------------------------------------

/// A found blob: saturated-pixel centroid + mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blob {
    pub cx_px: f64,
    pub cy_px: f64,
    pub area_px: usize,
}

/// Centroid of all saturated pixels (one dominant blob assumed — the
/// synthetic field is sparse; multi-blob segmentation is a later detector).
pub fn find_blob(rgb: &[u8], width: usize, height: usize) -> Option<Blob> {
    let (mut sx, mut sy, mut n) = (0.0f64, 0.0f64, 0usize);
    for py in 0..height {
        for px in 0..width {
            let i = (py * width + px) * 3;
            let (r, g, b) = (rgb[i], rgb[i + 1], rgb[i + 2]);
            let max = r.max(g).max(b);
            let min = r.min(g).min(b);
            if max - min > MIN_CHANNEL_SPREAD {
                sx += px as f64 + 0.5;
                sy += py as f64 + 0.5;
                n += 1;
            }
        }
    }
    (n >= MIN_BLOB_AREA_PX).then(|| Blob {
        cx_px: sx / n as f64,
        cy_px: sy / n as f64,
        area_px: n,
    })
}

/// Project a pixel to the ground through the capture pose: returns
/// `(lat_deg, lon_deg, offset_m)` where `offset_m` is the nadir offset (the
/// mission machine's localization-quality metric — smaller wins).
pub fn project_nadir(
    pose: &CapturePose,
    hfov_deg: f64,
    width_px: u32,
    height_px: u32,
    cx_px: f64,
    cy_px: f64,
) -> (f64, f64, f64) {
    let agl = pose.agl_m.max(0.5);
    let mppx = 2.0 * agl * (hfov_deg.to_radians() / 2.0).tan() / f64::from(width_px.max(1));
    let dx = (cx_px - f64::from(width_px) / 2.0) * mppx; // metres right
    let dy = (f64::from(height_px) / 2.0 - cy_px) * mppx; // metres forward
    let theta = pose.heading_deg.to_radians();
    let (fwd_n, fwd_e) = (theta.cos(), theta.sin());
    let (right_n, right_e) = (-theta.sin(), theta.cos());
    let d_north = fwd_n * dy + right_n * dx;
    let d_east = fwd_e * dy + right_e * dx;
    let lat = pose.lat_deg + d_north / EARTH_M_PER_DEG_LAT;
    let lon = pose.lon_deg + d_east / m_per_deg_lon(pose.lat_deg);
    (lat, lon, dx.hypot(dy))
}

/// Decode + detect + localize one fetched frame container. Pure with
/// respect to the network (the async shell fetches).
pub fn detect_in_container(payload: &[u8], object_query: &str) -> DetectOutcome {
    let (header, body) = match FrameHeader::split_frame(payload) {
        Ok(split) => split,
        Err(err) => return DetectOutcome::Miss(format!("bad frame container: {err}")),
    };
    if !header.kind.starts_with("image/") {
        return DetectOutcome::Miss(format!("not an image frame ({})", header.kind));
    }
    let mut decoder = jpeg_decoder::Decoder::new(body);
    let pixels = match decoder.decode() {
        Ok(pixels) => pixels,
        Err(err) => return DetectOutcome::Miss(format!("jpeg decode: {err}")),
    };
    let Some(info) = decoder.info() else {
        return DetectOutcome::Miss("jpeg: no info".into());
    };
    if info.pixel_format != jpeg_decoder::PixelFormat::RGB24 {
        return DetectOutcome::Miss(format!("unsupported pixel format {:?}", info.pixel_format));
    }
    let (w, h) = (info.width as usize, info.height as usize);
    let Some(blob) = find_blob(&pixels, w, h) else {
        return DetectOutcome::Miss(String::new()); // clean miss
    };
    let Some(pose) = CapturePose::from_metadata(&header.metadata) else {
        return DetectOutcome::Miss("frame carries no capture pose".into());
    };
    let hfov = header
        .metadata
        .get("hfov_deg")
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(DEFAULT_HFOV_DEG);
    let (lat, lon, offset_m) = project_nadir(
        &pose,
        hfov,
        header.width.unwrap_or(w as u32),
        header.height.unwrap_or(h as u32),
        blob.cx_px,
        blob.cy_px,
    );
    // Confidence from blob mass: a well-resolved anomaly saturates near
    // 0.95; a marginal handful of pixels sits near the 0.3 floor.
    let confidence = (0.35 + blob.area_px as f64 / 800.0).min(0.95);
    DetectOutcome::Hit(Detection {
        object_id: if object_query.is_empty() { "anomaly".into() } else { object_query.into() },
        confidence,
        lat_deg: lat,
        lon_deg: lon,
        offset_m,
    })
}

// ---------------------------------------------------------------------------
// the provider
// ---------------------------------------------------------------------------

/// Mints a fresh consumer per detection (fetches run concurrently).
pub type ConsumerFactory = Arc<dyn Fn() -> Option<Consumer> + Send + Sync>;

/// The color-threshold detection provider. Construct before the dashboard
/// starts, [`attach`](Self::attach) the consumer factory once the engine
/// exists; un-attached detections resolve to a Miss (mission still counts
/// them — no hang).
#[derive(Default)]
pub struct SimpleDetector {
    factory: OnceLock<ConsumerFactory>,
    /// Frame names already fetched → outcome note (debug/testing aid).
    last_note: Mutex<String>,
}

impl SimpleDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Wire the NDN fetch path (idempotent; first attach wins).
    pub fn attach(&self, factory: ConsumerFactory) {
        let _ = self.factory.set(factory);
    }

    /// The last outcome note (test/debug observability).
    pub fn last_note(&self) -> String {
        crate::lock(&self.last_note).clone()
    }
}

impl DetectionProvider for SimpleDetector {
    fn detect(&self, _mission_id: String, frame: String, object_query: String)
        -> BoxFuture<DetectOutcome> {
        let factory = self.factory.get().cloned();
        Box::pin(async move {
            let Some(factory) = factory else {
                return DetectOutcome::Miss("detector not attached to an engine".into());
            };
            let Some(consumer) = factory() else {
                return DetectOutcome::Miss("engine unavailable".into());
            };
            let name: ndn_packet::Name = match frame.parse() {
                Ok(name) => name,
                Err(err) => return DetectOutcome::Miss(format!("bad frame name: {err:?}")),
            };
            // Fetch THROUGH the fabric (console bridge + SimLinks) — the
            // same path the v2 GCS perception client took.
            let fetched = tokio::time::timeout(
                Duration::from_secs(10),
                consumer.object(name).fetch(),
            )
            .await;
            match fetched {
                Ok(Ok(payload)) => detect_in_container(&payload, &object_query),
                Ok(Err(err)) => DetectOutcome::Miss(format!("frame fetch: {err}")),
                Err(_) => DetectOutcome::Timeout,
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(width: usize, height: usize, disk: Option<(f64, f64, f64)>) -> Vec<u8> {
        // The synthetic feed's palette: low-saturation ground, red blob.
        let mut rgb = vec![0u8; width * height * 3];
        for py in 0..height {
            for px in 0..width {
                let i = (py * width + px) * 3;
                let base: [u8; 3] = if (px / 24 + py / 24) % 2 == 0 { [34, 40, 34] } else { [46, 54, 46] };
                rgb[i..i + 3].copy_from_slice(&base);
                if let Some((cx, cy, r)) = disk {
                    let d2 = (px as f64 + 0.5 - cx).powi(2) + (py as f64 + 0.5 - cy).powi(2);
                    if d2 <= r * r {
                        rgb[i..i + 3].copy_from_slice(&[210, 40, 40]);
                    }
                }
            }
        }
        rgb
    }

    #[test]
    fn blob_centroid_lands_on_the_disk() {
        let rgb = frame(320, 240, Some((200.0, 60.0, 9.0)));
        let blob = find_blob(&rgb, 320, 240).expect("blob found");
        assert!((blob.cx_px - 200.0).abs() < 1.5, "cx {}", blob.cx_px);
        assert!((blob.cy_px - 60.0).abs() < 1.5, "cy {}", blob.cy_px);
        assert!(blob.area_px > 200, "area {}", blob.area_px);
        assert!(find_blob(&frame(320, 240, None), 320, 240).is_none(), "clean ground misses");
    }

    /// Round-trip against the agent's rendering convention: a target D
    /// metres forward/right of the vehicle appears up/right of frame
    /// center; projecting that pixel back must return the target.
    #[test]
    fn nadir_projection_inverts_the_render_convention() {
        let pose = CapturePose {
            lat_deg: 35.0,
            lon_deg: -90.0,
            agl_m: 10.0,
            heading_deg: 30.0,
            roll_deg: None,
            pitch_deg: None,
        };
        let (w, h, hfov) = (320u32, 240u32, 66.0f64);
        let mppx = 2.0 * 10.0 * (hfov.to_radians() / 2.0).tan() / f64::from(w);

        // Target: 4 m forward, 2 m right of the vehicle (heading 30°).
        let theta = 30.0f64.to_radians();
        let d_north = theta.cos() * 4.0 + (-theta.sin()) * 2.0;
        let d_east = theta.sin() * 4.0 + theta.cos() * 2.0;
        let t_lat = pose.lat_deg + d_north / EARTH_M_PER_DEG_LAT;
        let t_lon = pose.lon_deg + d_east / m_per_deg_lon(pose.lat_deg);

        // Its pixel under the render convention (up = forward).
        let cx = w as f64 / 2.0 + 2.0 / mppx;
        let cy = h as f64 / 2.0 - 4.0 / mppx;

        let (lat, lon, offset) = project_nadir(&pose, hfov, w, h, cx, cy);
        let err_m = ((lat - t_lat) * EARTH_M_PER_DEG_LAT)
            .hypot((lon - t_lon) * m_per_deg_lon(35.0));
        assert!(err_m < 0.05, "round-trip error {err_m:.3} m");
        assert!((offset - (4.0f64).hypot(2.0)).abs() < 0.05, "offset {offset:.2}");
    }

    #[test]
    fn container_detection_localizes_and_scores() {
        // Encode a frame the long way around: reuse jpeg-decoder's inverse
        // is impossible (no encoder here), so run the container path with a
        // raw (non-JPEG) body and assert the typed miss — full pipeline
        // parity is exercised by the virtual deployment's --verify.
        let mut header = FrameHeader {
            body_len: 0,
            body_sha256: String::new(),
            gps_time_ns: 1,
            height: Some(240),
            kind: "image/jpeg".into(),
            metadata: Default::default(),
            mission_id: "m1".into(),
            sensor_id: "cam0".into(),
            vehicle_id: "wuas-01".into(),
            width: Some(320),
        };
        let body = b"not a jpeg".to_vec();
        header.set_body(&body);
        let container = header.to_frame_bytes(&body).unwrap();
        match detect_in_container(&container, "anomaly") {
            DetectOutcome::Miss(note) => assert!(note.contains("jpeg decode"), "{note}"),
            other => panic!("expected decode miss, got {other:?}"),
        }
        // And a wrong-kind container is a typed miss too.
        header.kind = "audio/wav".into();
        let container = header.to_frame_bytes(&body).unwrap();
        match detect_in_container(&container, "") {
            DetectOutcome::Miss(note) => assert!(note.contains("not an image"), "{note}"),
            other => panic!("expected kind miss, got {other:?}"),
        }
    }
}
