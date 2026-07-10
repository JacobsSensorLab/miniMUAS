//! In-proc smoke tests: two full agents on two engines linked by localhost
//! UDP faces, sim backends — the fleet-coordination convergence path and the
//! service round-trip path, end to end through the real NDN data plane.

use std::time::Duration;

use muas_agent::{Agent, AgentConfig, BackendExt, Endpoint, UdpLink};
use muas_contracts::names;
use muas_contracts::services::{TakeoffRequest, VehicleServiceClient};
use ndn_app::EngineAppExt;
use ndn_engine::builder::{EngineBuilder, EngineConfig};
use ndn_face::UdpFace;
use ndn_packet::Name;
use ndn_rpc::FaceRpcCarrier;
use ndn_service_core::ServiceId;
use tokio_util::sync::CancellationToken;
use uas_fleet_node::coordination::{CoordEntry, EARTH_M_PER_DEG_LAT};

const ORIGIN: (f64, f64) = (35.0, -90.0);

fn north_of(lat: f64, metres: f64) -> f64 {
    lat + metres / EARTH_M_PER_DEG_LAT
}

fn sim_config(vehicle_id: &str, origin: (f64, f64), links: Vec<UdpLink>) -> AgentConfig {
    AgentConfig {
        vehicle_id: vehicle_id.to_string(),
        endpoint: Endpoint::Sim {
            lat_deg: origin.0,
            lon_deg: origin.1,
        },
        links,
        ..AgentConfig::default()
    }
}

fn link(local: u16, remote: u16, route: Option<String>) -> UdpLink {
    UdpLink {
        local: format!("127.0.0.1:{local}").parse().unwrap(),
        remote: format!("127.0.0.1:{remote}").parse().unwrap(),
        route,
    }
}

/// A test-side client engine with one UDP face toward `agent_port`, routing
/// the agent's vehicle prefix out of it.
async fn client_engine(
    local_port: u16,
    agent_port: u16,
    vehicle_id: &str,
) -> (
    ndn_engine::ForwarderEngine,
    ndn_engine::ShutdownHandle,
    VehicleServiceClient<FaceRpcCarrier>,
) {
    let (engine, shutdown) = EngineBuilder::new(EngineConfig::default())
        .build()
        .await
        .expect("client engine");
    let face_id = engine.faces().alloc_id();
    let face = UdpFace::bind(
        format!("127.0.0.1:{local_port}").parse().unwrap(),
        format!("127.0.0.1:{agent_port}").parse().unwrap(),
        face_id,
    )
    .await
    .expect("client udp face");
    engine.add_face(face, CancellationToken::new());
    let prefix: Name = names::vehicle_prefix(vehicle_id).parse().unwrap();
    engine.fib().add_nexthop(&prefix, face_id, 0);
    tokio::time::sleep(Duration::from_millis(150)).await; // faces settle

    let consumer = engine.app_consumer(CancellationToken::new());
    let client = VehicleServiceClient::new(
        FaceRpcCarrier::client(consumer).with_timeout(Duration::from_secs(4)),
        ServiceId::new(prefix),
    );
    (engine, shutdown, client)
}

/// Service round-trip over the engine + UDP face: policy rejects an
/// out-of-range AGL at ack, accepts a valid takeoff and flies it; the
/// shutdown double-authorization refuses a bad phrase and an armed vehicle.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn service_round_trip_policy_gates_over_the_wire() {
    let vid = "iuas-91";
    // Agent listens for the test client on 48611; client binds 48610.
    let agent = Agent::start(sim_config(vid, ORIGIN, vec![link(48611, 48610, None)]))
        .await
        .expect("agent up");
    let (_engine, client_shutdown, client) = client_engine(48610, 48611, vid).await;

    // Out-of-range AGL is rejected at ack with the typed policy code.
    let ack = client
        .flight_takeoff(TakeoffRequest { agl_m: 50.0 })
        .await
        .expect("call transported");
    assert!(!ack.accepted, "50 m AGL must be rejected: {ack:?}");
    assert_eq!(ack.code, "agl-out-of-bounds");

    // Shutdown with the wrong confirm phrase is rejected (first gate).
    let ack = client
        .system_shutdown("not-the-vehicle".to_string())
        .await
        .expect("call transported");
    assert_eq!(ack.code, "shutdown-confirm-mismatch");

    // A valid takeoff is accepted and actually flown by the sim backend.
    let ack = client
        .flight_takeoff(TakeoffRequest { agl_m: 5.0 })
        .await
        .expect("call transported");
    assert!(ack.accepted, "valid takeoff refused: {ack:?}");
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    loop {
        let t = agent.shared.backend.lock().unwrap().as_dyn_ref().telemetry();
        if t.armed && (t.agl_m - 5.0).abs() < 0.5 {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "takeoff did not settle: armed={} agl={}",
            t.armed,
            t.agl_m
        );
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // Armed now: shutdown with the RIGHT phrase is still refused (v2 rail).
    let ack = client
        .system_shutdown(vid.to_string())
        .await
        .expect("call transported");
    assert_eq!(ack.code, "shutdown-while-armed");

    agent.shutdown().await;
    client_shutdown.shutdown().await;
}

/// Two agents, two engines, one UDP link: converging gotos trigger the
/// deterministic symmetric pair plan on both sides — both publish coord
/// entries, confirm to `coop` (proving telemetry AND coord flow over the
/// wire), and the altitude-bias overlay is applied to both backends.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn converging_agents_coordinate_over_the_wire() {
    let (vid_a, vid_b) = ("iuas-92", "wuas-92");
    let fleet: Vec<String> = vec![vid_a.to_string(), vid_b.to_string()];
    let origin_b = (north_of(ORIGIN.0, 150.0), ORIGIN.1);

    let mut config_a = sim_config(
        vid_a,
        ORIGIN,
        vec![link(48621, 48622, Some(names::vehicle_prefix(vid_b)))],
    );
    config_a.fleet_ids = fleet.clone();
    // Spark lane coverage: agent A mirrors telemetry onto a UDP spark stream.
    let spark_rx = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
    config_a.spark_udp = Some(spark_rx.local_addr().unwrap());

    let mut config_b = sim_config(
        vid_b,
        origin_b,
        vec![link(48622, 48621, Some(names::vehicle_prefix(vid_a)))],
    );
    config_b.fleet_ids = fleet;

    let agent_a = Agent::start(config_a).await.expect("agent a up");
    let agent_b = Agent::start(config_b).await.expect("agent b up");

    // Converging gotos: A (10 m AGL) flies north toward B; B (8 m AGL) flies
    // south toward A; 150 m apart closing at ~10 m/s → CPA inside the 20 s
    // horizon with zero miss distance and only 2 m of vertical separation.
    {
        let mut backend = agent_a.shared.backend.lock().unwrap();
        let b = backend.as_dyn();
        assert!(b.ensure_airborne(10.0));
        b.set_cruise_speed(5.0);
        b.goto(origin_b.0, origin_b.1, 10.0, None);
    }
    {
        let mut backend = agent_b.shared.backend.lock().unwrap();
        let b = backend.as_dyn();
        assert!(b.ensure_airborne(8.0));
        b.set_cruise_speed(5.0);
        b.goto(ORIGIN.0, ORIGIN.1, 8.0, None);
    }

    // Wait for both sides to engage, publish, and confirm cooperation.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(40);
    let (mut coop_a, mut coop_b) = (false, false);
    while !(coop_a && coop_b) {
        assert!(
            tokio::time::Instant::now() < deadline,
            "coordination did not converge: a={coop_a} b={coop_b}"
        );
        tokio::time::sleep(Duration::from_millis(250)).await;
        let entries_a: Vec<CoordEntry> =
            serde_json::from_slice(&agent_a.shared.latest_coord.lock().unwrap().clone())
                .unwrap_or_default();
        let entries_b: Vec<CoordEntry> =
            serde_json::from_slice(&agent_b.shared.latest_coord.lock().unwrap().clone())
                .unwrap_or_default();
        coop_a = entries_a
            .iter()
            .any(|e| e.from_id == vid_a && e.to_id == vid_b && e.mode == "coop");
        coop_b = entries_b
            .iter()
            .any(|e| e.from_id == vid_b && e.to_id == vid_a && e.mode == "coop");
    }

    // The deterministic symmetric pair plan applied its biases through the
    // altitude overlay: higher A climbs (+3), lower B gives way (−2).
    let bias_a = agent_a.shared.backend.lock().unwrap().as_dyn_ref().avoid_bias();
    let bias_b = agent_b.shared.backend.lock().unwrap().as_dyn_ref().avoid_bias();
    assert!(bias_a > 0.0, "climber bias applied: {bias_a}");
    assert!(bias_b < 0.0, "descender bias applied: {bias_b}");
    assert!((bias_a - 3.0).abs() < 1e-9, "pair-plan climb bias: {bias_a}");
    assert!((bias_b + 2.0).abs() < 1e-9, "pair-plan descent bias: {bias_b}");

    // The spark lane carried real telemetry sparks.
    let mut buf = vec![0u8; 65536];
    let (len, _) = tokio::time::timeout(Duration::from_secs(5), spark_rx.recv_from(&mut buf))
        .await
        .expect("a spark datagram arrives")
        .expect("spark recv");
    let spark = ndf_spark::SparkPayload::decode(&buf[..len]).expect("valid spark payload");
    let sample: serde_json::Value = serde_json::from_slice(&spark.data).expect("sample json");
    assert_eq!(sample["vehicle_id"], vid_a);

    agent_a.shutdown().await;
    agent_b.shutdown().await;
}
