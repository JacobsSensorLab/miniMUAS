//! `FleetSim`: N unmodified miniMUAS agents bridged onto an ndn-sim fabric.
//!
//! Topology (per vehicle *i*):
//!
//! ```text
//! agent_i engine ──UDP loopback (lossless)── node_i ══SimLink (lossy)══ node_j ── ... agent_j
//! ```
//!
//! - the agent's single UDP face routes the whole `/muas/v3` tree toward its
//!   fabric node (its own served prefixes win by longest-prefix match on the
//!   app faces, so only *outbound* peer traffic takes the bridge);
//! - `node_i` routes `/muas/v3/<vid_i>` down the bridge face and every peer
//!   prefix across the matching SimLink;
//! - an optional **sink** node, linked with `loss_rate = 1.0`, lets a test
//!   blackhole one name subtree (e.g. a peer's `coord/status`) while the
//!   rest of the traffic flows — the "coord publications blackholed"
//!   escalation scenario, data-plane only.

use std::net::SocketAddr;
use std::sync::{Mutex, MutexGuard};
use std::time::Duration;

use muas_agent::{Agent, AgentConfig, AgentHandle, BackendExt, Endpoint, UdpLink};
use muas_contracts::names;
use muas_contracts::services::VehicleServiceClient;
use ndn_app::{Consumer, EngineAppExt};
use ndn_engine::builder::EngineConfig;
use ndn_packet::Name;
use ndn_rpc::FaceRpcCarrier;
use ndn_service_core::ServiceId;
use ndn_sim::{LinkConfig, NodeId, RunningSimulation, Simulation};
use uas_fleet_node::coordination::CoordEntry;
use uas_fleet_node::flight_backend::FlightBackend;
use uas_mavlink::BackendTelemetry;

/// Lock a mutex, recovering from poisoning (same posture as the agent:
/// a panicked task must not wedge the harness).
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// One vehicle in a scenario.
#[derive(Debug, Clone)]
pub struct VehicleSpec {
    pub vehicle_id: String,
    /// Sim backend start position `(lat_deg, lon_deg)`.
    pub origin: (f64, f64),
}

impl VehicleSpec {
    pub fn new(vehicle_id: &str, origin: (f64, f64)) -> Self {
        Self {
            vehicle_id: vehicle_id.to_string(),
            origin,
        }
    }
}

/// Reserve a free loopback UDP port by binding an ephemeral socket and
/// dropping it. The agent re-binds the port moments later; the tiny reuse
/// race is accepted (documented) — the OS ephemeral allocator does not
/// hand the same port straight back under normal test load.
fn reserve_udp_port() -> Result<u16, String> {
    let socket = std::net::UdpSocket::bind(("127.0.0.1", 0))
        .map_err(|e| format!("reserve port: {e}"))?;
    socket
        .local_addr()
        .map(|a| a.port())
        .map_err(|e| format!("reserve port addr: {e}"))
}

/// A running fleet: the fabric, its nodes, and one real agent per vehicle.
pub struct FleetSim {
    pub fabric: RunningSimulation,
    pub nodes: Vec<NodeId>,
    /// Blackhole node (all links to it drop 100%), when built with one.
    pub sink: Option<NodeId>,
    pub agents: Vec<AgentHandle>,
    pub vehicle_ids: Vec<String>,
}

impl FleetSim {
    /// Build the fabric (full mesh of `link`-configured SimLinks), bridge
    /// one agent per spec onto it, and start the agents. `configure` mutates
    /// each vehicle's [`AgentConfig`] after the defaults (fleet ids, sim
    /// endpoint, bridge link) are set.
    pub async fn start(
        specs: &[VehicleSpec],
        link: LinkConfig,
        with_sink: bool,
        seed: u64,
        configure: impl Fn(usize, &mut AgentConfig),
    ) -> Result<Self, String> {
        let vehicle_ids: Vec<String> = specs.iter().map(|s| s.vehicle_id.clone()).collect();

        // -- fabric ----------------------------------------------------------
        let mut sim = Simulation::new().seed(seed); // default WallClockKernel
        let nodes: Vec<NodeId> = specs
            .iter()
            .map(|_| sim.add_node(EngineConfig::default()))
            .collect();
        for i in 0..nodes.len() {
            for j in (i + 1)..nodes.len() {
                sim.link(nodes[i], nodes[j], link.clone());
            }
        }
        let sink = if with_sink {
            let sink = sim.add_node(EngineConfig::default());
            for &node in &nodes {
                sim.link(
                    node,
                    sink,
                    LinkConfig {
                        loss_rate: 1.0,
                        ..LinkConfig::default()
                    },
                );
            }
            Some(sink)
        } else {
            None
        };
        // Inter-node vehicle routes: node_i sends /muas/v3/<vid_j> to node_j.
        for i in 0..specs.len() {
            for (j, spec) in specs.iter().enumerate() {
                if i != j {
                    sim.add_route(nodes[i], &names::vehicle_prefix(&spec.vehicle_id), nodes[j]);
                }
            }
        }
        let fabric = sim.start().await.map_err(|e| format!("fabric start: {e}"))?;

        // -- bridges (fabric edge, lossless loopback UDP) ----------------------
        let mut agent_links = Vec::with_capacity(specs.len());
        for (i, spec) in specs.iter().enumerate() {
            let agent_port = reserve_udp_port()?;
            let agent_addr: SocketAddr = format!("127.0.0.1:{agent_port}")
                .parse()
                .map_err(|e| format!("agent addr: {e}"))?;
            let bridge_socket = tokio::net::UdpSocket::bind("127.0.0.1:0")
                .await
                .map_err(|e| format!("bridge socket: {e}"))?;
            let bridge_addr = bridge_socket
                .local_addr()
                .map_err(|e| format!("bridge addr: {e}"))?;
            let bridge_face = fabric
                .bridge_udp_socket(nodes[i], bridge_socket, agent_addr)
                .map_err(|e| format!("bridge {}: {e}", spec.vehicle_id))?;
            let own_prefix: Name = names::vehicle_prefix(&spec.vehicle_id)
                .parse()
                .map_err(|e| format!("vehicle prefix: {e:?}"))?;
            fabric
                .engine_of(nodes[i])
                .ok_or_else(|| format!("no engine for node {i}"))?
                .fib()
                .add_nexthop(&own_prefix, bridge_face, 0);
            agent_links.push(UdpLink {
                local: agent_addr,
                remote: bridge_addr,
                route: Some(names::APP_PREFIX.to_string()),
            });
        }

        // -- agents (the real thing, unmodified) -------------------------------
        let mut agents = Vec::with_capacity(specs.len());
        for (i, spec) in specs.iter().enumerate() {
            let mut config = AgentConfig {
                vehicle_id: spec.vehicle_id.clone(),
                fleet_ids: vehicle_ids.clone(),
                endpoint: Endpoint::Sim {
                    lat_deg: spec.origin.0,
                    lon_deg: spec.origin.1,
                },
                links: vec![agent_links[i].clone()],
                ..AgentConfig::default()
            };
            configure(i, &mut config);
            let agent = Agent::start(config)
                .await
                .map_err(|e| format!("agent {} start: {e}", spec.vehicle_id))?;
            agents.push(agent);
        }

        Ok(Self {
            fabric,
            nodes,
            sink,
            agents,
            vehicle_ids,
        })
    }

    /// Blackhole `target`'s `coord/status` name as seen from `viewer`'s
    /// fabric node: a longest-prefix-match route sends exactly that subtree
    /// over the 100%-loss link to the sink, while telemetry (and everything
    /// else under the peer's prefix) keeps flowing.
    pub fn blackhole_coord(&self, viewer: usize, target: usize) -> Result<(), String> {
        let sink = self.sink.ok_or("fleet built without a sink node")?;
        let face = self
            .fabric
            .face_between(self.nodes[viewer], sink)
            .ok_or("no link to the sink node")?;
        let name: Name = names::vehicle_stream(&self.vehicle_ids[target], "coord/status")
            .parse()
            .map_err(|e| format!("coord name: {e:?}"))?;
        self.fabric
            .engine_of(self.nodes[viewer])
            .ok_or("no engine for viewer node")?
            .fib()
            .add_nexthop(&name, face, 0);
        Ok(())
    }

    /// Run `f` against vehicle `i`'s flight backend (the same shared
    /// backend the agent's services and coordination thread fly).
    pub fn with_backend<R>(&self, i: usize, f: impl FnOnce(&mut dyn FlightBackend) -> R) -> R {
        let mut guard = lock(&self.agents[i].shared.backend);
        f(guard.as_dyn())
    }

    /// Freshest backend telemetry snapshot for vehicle `i`.
    pub fn telemetry_of(&self, i: usize) -> BackendTelemetry {
        lock(&self.agents[i].shared.backend).as_dyn_ref().telemetry()
    }

    /// Vehicle `i`'s currently published coordination entries.
    pub fn coord_entries(&self, i: usize) -> Vec<CoordEntry> {
        let bytes = lock(&self.agents[i].shared.latest_coord).clone();
        serde_json::from_slice(&bytes).unwrap_or_default()
    }

    /// True when vehicle `from` currently publishes an entry to `to` in
    /// `mode` ("coop-pending" / "coop" / "unco").
    pub fn publishes_mode(&self, from: usize, to: usize, mode: &str) -> bool {
        let to_id = &self.vehicle_ids[to];
        self.coord_entries(from)
            .iter()
            .any(|e| &e.to_id == to_id && e.mode == mode)
    }

    /// An NDN consumer on vehicle `i`'s own engine (fetches ride the bridge
    /// + fabric exactly like the agent's own peer fetcher).
    pub fn consumer(&self, i: usize) -> Consumer {
        self.agents[i]
            .engine
            .app_consumer(self.agents[i].cancel.child_token())
    }

    /// A typed vehicle-service client on `from`'s engine targeting `to`.
    pub fn service_client(
        &self,
        from: usize,
        to: usize,
        timeout: Duration,
    ) -> Result<VehicleServiceClient<FaceRpcCarrier>, String> {
        let prefix: Name = names::vehicle_prefix(&self.vehicle_ids[to])
            .parse()
            .map_err(|e| format!("service prefix: {e:?}"))?;
        let carrier = FaceRpcCarrier::client(self.consumer(from)).with_timeout(timeout);
        Ok(VehicleServiceClient::new(carrier, ServiceId::new(prefix)))
    }

    /// Stop every agent, then the fabric.
    pub async fn shutdown(self) {
        for agent in self.agents {
            agent.shutdown().await;
        }
        self.fabric.shutdown().await;
    }
}
