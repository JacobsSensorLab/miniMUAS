//! RC subsumption (milestone M6): one USB game controller on the dashboard
//! drives the fleet over named data radio — RC_CHANNELS_OVERRIDE /
//! MANUAL_CONTROL framed as ndf-spark streams (`/muas/v3/<vid>/rc`), loss
//! honest, failsafe by stream silence. Inspiration: ExpressLRS/CRSF on
//! ESP32, DroneBridge, ESP-NOW RC. See docs/v3/ARCHITECTURE.md §RC.
