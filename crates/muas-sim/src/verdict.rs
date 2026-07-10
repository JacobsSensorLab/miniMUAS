//! JSON verdicts: one line per parity assertion, machine-greppable
//! (`PARITY_VERDICT {...}`) from test output.

use serde::Serialize;

/// One assertion's verdict, emitted as a JSON line.
#[derive(Debug, Clone, Serialize)]
pub struct Verdict {
    /// Scenario name (e.g. "coordination-parity").
    pub scenario: &'static str,
    /// Assertion name (e.g. "symmetric-coop").
    pub assertion: &'static str,
    pub pass: bool,
    /// Free-form evidence (biases, modes, slot altitudes, timings).
    pub details: serde_json::Value,
}

impl Verdict {
    pub fn new(
        scenario: &'static str,
        assertion: &'static str,
        pass: bool,
        details: serde_json::Value,
    ) -> Self {
        Self {
            scenario,
            assertion,
            pass,
            details,
        }
    }

    /// Print the verdict as one JSON line (stdout, captured by the test
    /// harness; shown with `--nocapture` or on failure).
    pub fn emit(&self) {
        match serde_json::to_string(self) {
            Ok(json) => println!("PARITY_VERDICT {json}"),
            Err(err) => println!("PARITY_VERDICT {{\"error\":\"{err}\"}}"),
        }
    }
}
