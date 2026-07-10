# miniMUAS v3 — Vision & Requirements

Captured 2026-07-09 from the project owner's directive. This is the source-of-truth
requirements document for v3; ARCHITECTURE.md maps these onto concrete designs.

## What v3 is

A ground-up Rust port of miniMUAS v2, built on **ndn-workspace** and the **NDF
refounding (ndf-rs)**, with lossless feature parity as the floor and deliberate
upgrades on top. v2 stays intact on `main`; v3 lives on this branch.

## Core requirements

### 1. Flight/motion primitives → standalone Rust library
- Extract the flight/motion primitives from **UAS-IPBRC** into a reusable Rust
  library (working name: `muas-flight`), refactoring and upgrading in the process
  — not a line-by-line translation.
- MAVLink is the lingua franca **wherever UAS interaction exists**.

### 2. Dashboard
- Built from **flotilla semantic manifests + render contracts** and the NDF
  refounding.
- **Lossless feature parity** with the v2 dashboard, then upgraded.

### 3. Pluggable service backends
- The C++ **NDNSF** (NDN_Service_Framework) is consumed via its Rust port in
  **ndn-ext**, but only as *one of several* pluggable backends behind
  **ndn-service** — so backends can be compared head-to-head.

### 4. Observability
- **tracing** spans + **OpenTelemetry** throughout the rebuild. No other NDN app
  has this — make it a showcase.

### 5. Simulation
- **ndn-sim** is a first-class asset during and after development for testing and
  verification (alongside ArduPilot SITL for the flight side).

### 6. Security & wireless networking (revamped emphasis)
- **Named data radio** (monitor-mode wifi) is now available: each UAS node has
  two rtl8812eu interfaces; adding one or more ESP32s per UAS is trivial.
- Wireless must stay **flexible and comparable**: e.g. wifi AP/STA mode vs named
  data radio must both run so modes can be evaluated against each other.

### 7. RC subsumption — 1-to-many control
- Eliminate the per-UAS RC transmitter. One USB game controller on the dashboard
  drives the fleet over named data radio (wifi and/or LoRa).
- Inspiration: **ExpressLRS/CRSF on ESP32**, **DroneBridge**, **ESP-NOW RC
  transmitter** projects.
- Consider **MAVLink over NDN** (NDF Sparkstreams, or something lighter if
  needed) as the transport design.

### 8. Field deployment quality of life
- Physical tools + software that make field deployment simpler and faster while
  granting more capability. Example: an **ESP32 CYD** (cheap yellow display) as
  an NDN node with monitor-mode wifi — quick fleet stats view and management in
  the palm of your hand.

## The two-way street

This project is a live test drive of fresh frameworks; the framework maintainers
get real-user feedback in return.

- Track **friction, pain points, bugs, ideas, and constructive criticism** for
  the maintainers of ndn-workspace and ndf (refounding) → `docs/v3/FEEDBACK.md`.
- ndn-workspace has features unique to it, unavailable in the mainline NDN
  ecosystem — use them aggressively and report on them.
- The **ndn waterline suite** (Anchor, Capstan, Sextant — WIP fleet-management
  apps) wants input on fleet-management UI/UX to shape their flotilla models →
  `docs/v3/WATERLINE-INPUT.md`. Context lives in `~/Downloads/latest`.
- Requests for work from other sessions: write a file with the request + context
  under `docs/v3/requests/` and flag it to the owner.

## Non-negotiables (carried from v2 hard-won lessons)

- ArduCopter silently drops `goto` below 3 m AGL right after takeoff → keep the
  3.5 m command floor.
- Fleet coordination semantics: data-plane symmetric avoidance, altitude-bias
  overlay, smart RTL slots, fleet flight floor plumbed into cooperative planning.
- Power-loss-safe journals, mission replay, authorized companion shutdown.
- Deployment flow: config repo branch `mini-muas-v2` (v3 will get its own),
  minimuas-src flake input pinned by rev — push before flake bump.
