//! Transport-neutral flight/motion primitives for multi-UAS systems.
//!
//! Extraction and refactor of UAS-IPBRC's `relay.flight` layer (see
//! `docs/v3/surveys/uas-ipbrc.md`): primitives emit intent as typed
//! [`FlightCommand`]s and carry all state in an explicit serializable
//! blackboard — no MAVLink, no NDN, no hidden state, no sleeps. Adapters
//! (muas-mavlink) translate intent to a vehicle; runtimes (muas-agent) drive
//! the tick loop.
//!
//! Port order (each module lands with tests translated from the UAS-IPBRC
//! `tests/test_flight_*.py` oracle):
//! 1. `geo` — flat-earth position math (done)
//! 2. `deconflict` — CPA, cooperative avoidance, RTL slots (pure math)
//! 3. `patterns` / `placement` — orbit/raster generators, mast-follow
//! 4. `primitives` / `motion` / `sequence` / `runtime` — the tick core
//! 5. `lifecycle`, `orbit` (mode ladder), `constraints`

pub mod geo;
