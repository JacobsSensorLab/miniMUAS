//! The `/muas/v3` name tree.
//!
//! Same shape as v2 (services, latest-wins data, mission objects) so parity
//! audits map one-to-one; only the version segment changes.

/// Application prefix for everything miniMUAS v3 publishes or serves.
pub const APP_PREFIX: &str = "/muas/v3";

/// Group prefix for service-layer sync. Deployment note carried from v2:
/// this prefix must run a multicast forwarding strategy — best-route
/// silently breaks group sync.
pub const GROUP_PREFIX: &str = "/muas/v3/group";

/// Service name under a vehicle, e.g. `flight/rtl`, `sensor/capture`,
/// `video/control`, `system/shutdown`.
pub fn vehicle_service(vehicle_id: &str, service: &str) -> String {
    format!("{APP_PREFIX}/{vehicle_id}/{service}")
}

/// Latest-wins data stream under a vehicle, e.g. `telemetry/live`,
/// `telemetry/state`, `search/status`, `coord/status`, `video/live`.
pub fn vehicle_stream(vehicle_id: &str, stream: &str) -> String {
    format!("{APP_PREFIX}/{vehicle_id}/{stream}")
}

/// Mission-scoped object name, e.g.
/// `/muas/v3/mission/<mid>/<vid>/camera/<cam>/frame/<gps_ns>/<seq>`.
pub fn mission_object(mission_id: &str, vehicle_id: &str, rest: &str) -> String {
    format!("{APP_PREFIX}/mission/{mission_id}/{vehicle_id}/{rest}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name_shapes_match_v2_tree() {
        assert_eq!(
            vehicle_service("iuas-01", "flight/investigate"),
            "/muas/v3/iuas-01/flight/investigate"
        );
        assert_eq!(
            vehicle_stream("wuas-01", "telemetry/live"),
            "/muas/v3/wuas-01/telemetry/live"
        );
        assert_eq!(
            mission_object("m1", "wuas-01", "camera/cam0/frame/123/7"),
            "/muas/v3/mission/m1/wuas-01/camera/cam0/frame/123/7"
        );
    }
}
