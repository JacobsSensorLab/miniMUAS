//! miniMUAS v3 contracts: the name tree, wire types, L1 semantic manifests,
//! and `#[ndn_service]` service definitions.
//!
//! Carries the v2 name-tree shape forward under `/muas/v3` (see
//! `docs/v3/surveys/minimuas-v2.md` §NDN usage). Service definitions ride
//! ndn-service-core's contract⇄carrier seam so backends (ndn-rpc, ndn-ndnsf,
//! ndn-nacabe) stay pluggable and comparable.

pub mod anomaly;
pub mod names;
pub mod policy;
pub mod rc;
pub mod sensors;
pub mod services;
pub mod strategy;
pub mod tasks;
