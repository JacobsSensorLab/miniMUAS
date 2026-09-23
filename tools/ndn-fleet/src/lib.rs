//! ndn-fleet: the miniMUAS fleet deploy + measurement protocol (see PROTOCOL.md).
//!
//! The MCP server (`ndn-fleet mcp`) and the CLI are thin fronts over the same operations; the
//! operations enforce the protocol's invariants, so neither front can skip a step.

pub mod cells;
pub mod config;
pub mod counters;
pub mod deploy;
pub mod jobs;
pub mod mcp;
pub mod measure;
pub mod remote;
pub mod results;
pub mod state;
pub mod status;
