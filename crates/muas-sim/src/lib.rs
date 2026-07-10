//! Co-simulation and verification harness (first scenarios land at M3):
//! real ForwarderEngines in ndn-sim (`mavlink` + `geometry` features) with
//! ArduPilot SITL mobility, OTLP traces as the assertion substrate. The v2
//! SITL validations (goto floor, avoidance-bias lead cap, smart RTL slots,
//! cooperative grace/escalation) become scripted regression scenarios here.
