//! Co-simulation and verification harness (M3/M5): real miniMUAS agents on
//! an ndn-sim (ndn-lab) fabric with lossy links, driving the v2 SITL
//! validation set as scripted regression scenarios plus the radio-mode
//! comparison matrix.
//!
//! # Embedding approach (documented choice): UDP bridges at the fabric edge
//!
//! Two candidate embeddings were evaluated against the actual code:
//!
//! 1. **`fabric.engine_of(node)`** — attach app code directly to a sim
//!    node's engine (the ndf-apps bootstrap pattern). Rejected here because
//!    [`muas_agent::Agent::start`] constructs its own engine internally
//!    (there is no engine-injection seam on `AgentConfig`, and this crate
//!    must not modify `muas-agent`), and because the agent is *not*
//!    virtual-time-safe anyway: it spawns a dedicated OS coordination
//!    thread ticking on the wall clock (`coord.rs`) and multi-thread tokio
//!    loops, while ndn-sim's `VirtualKernel` owns a paused single-thread
//!    runtime.
//! 2. **UDP interop bridges** (`RunningSimulation::bridge_udp_socket`,
//!    ndn-lab slice 9) — each *unmodified* agent runs exactly as deployed
//!    (own engine, one UDP face) and peers with a dedicated fabric node
//!    over a loopback UDP hop; all vehicle-to-vehicle traffic then crosses
//!    the fabric's lossy `SimLink`s. **Chosen**: it exercises the real
//!    agent end to end (same code path as the field deployment), and the
//!    bridge is the mechanism ndn-sim explicitly supports for external
//!    forwarders (`src/bridge.rs`, `tests/bridge.rs`).
//!
//! Consequence (documented per the M3 brief): the bridge is wall-clock
//! only, so scenarios run on the default `WallClockKernel` in real time
//! with **compressed parameters** — 5 m/s+ cruise speeds, 120–150 m
//! engagement ranges, `grace_s` 1.5 s in the comparison harness — keeping
//! the parity tests tens of seconds each and the full three-profile
//! comparison under five minutes (profiles run concurrently).
//!
//! Loss/delay/jitter/bandwidth live on the inter-node `SimLink`s
//! ([`ndn_sim::LinkConfig`]); the loopback agent↔node hop is lossless by
//! construction. The spark telemetry lane is raw UDP *outside* the NDN
//! fabric, so the comparison harness pushes it through a
//! [`metrics::spawn_impairment_relay`] parameterised with the same link
//! profile (ndn-sim has no way to carry a foreign UDP flow over a SimLink
//! — see the friction notes in the M5 report).

pub mod fleet;
pub mod metrics;
pub mod verdict;

pub use fleet::{FleetSim, VehicleSpec};
pub use metrics::{Summary, summarize};
pub use verdict::Verdict;
