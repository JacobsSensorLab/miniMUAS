//! The lenses: four renderers over ONE dataset (or one run-set), registered
//! as `Via::Native` ids and reachable only through the uas-console `Binder`
//! (match → authorize → instantiate). The closures capture a shared
//! [`RunSet`] — the same `Arc<MissionDataset>` feeds every renderer, and no
//! renderer keeps a transformed copy. Provenance rides progressive
//! disclosure: association (settings ↔ outcomes) up front, Block hashes in
//! drawers/tooltips, the full list in the audit surface.

use std::collections::BTreeMap;
use std::sync::Arc;

use serde_json::json;
use uas_console::registry::{RenderCtx, RenderError, RenderOutput, RendererRegistry};
use uas_console::view::{RawInspectView, ViewModel};
use uas_console::{Binder, Budget};

use crate::contracts::{artifact_pack, intent, renderer};
use crate::dataset::{Citation, MissionDataset, RunConfig};

pub mod compare;
pub mod deck;
pub mod demo;
pub mod report;

/// The set of runs the renderers lens over. Single-run artifacts read
/// `runs[0]`; the comparison reads all of them.
pub struct RunSet {
    /// The datasets, one per run, in the order given.
    pub runs: Vec<Arc<MissionDataset>>,
}

/// One rendered artifact: the HTML bytes and the citations it consumed.
#[derive(Clone, Debug)]
pub struct RenderedArtifact {
    /// Self-contained HTML.
    pub html: String,
    /// Every `(hash, chain, seq)` the artifact was derived from.
    pub citations: Vec<Citation>,
}

// ───────────────────────── registry (instantiate side) ──────────────────────

const VM_DATASET: &str = "mission-dataset";
const VM_RUN_SET: &str = "run-set";

fn expect_vm(vm: &ViewModel, id: &'static str, want: &str) -> Result<(), RenderError> {
    match vm {
        ViewModel::Raw(raw) if raw.kind == want => Ok(()),
        ViewModel::Raw(raw) => Err(RenderError::Unrenderable {
            renderer: id,
            reason: format!("expected a {want} handle, got kind `{}`", raw.kind),
        }),
        other => Err(RenderError::WrongViewModel { renderer: id, expected: "raw", got: other.kind() }),
    }
}

fn output(id: &str, lens: &str, art: RenderedArtifact) -> Result<RenderOutput, RenderError> {
    Ok(RenderOutput {
        renderer: id.into(),
        body: json!({
            "lens": lens,
            "html": art.html,
            "citations": art.citations,
        }),
    })
}

/// The artifact renderer registry: four `Via::Native` ids, every closure
/// borrowing the SAME `RunSet` — the sharing the thesis claims, visible in
/// the capture list.
pub fn registry_for(set: Arc<RunSet>) -> RendererRegistry {
    let mut r = RendererRegistry::new();

    let s = Arc::clone(&set);
    r.register(renderer::REPORT, move |vm, _ctx| {
        expect_vm(vm, renderer::REPORT, VM_DATASET)?;
        let ds = first_run(&s, renderer::REPORT)?;
        output(renderer::REPORT, intent::REPORT, report::render(ds))
    })
    .expect("fresh registry");

    let s = Arc::clone(&set);
    r.register(renderer::DECK, move |vm, _ctx| {
        expect_vm(vm, renderer::DECK, VM_DATASET)?;
        let ds = first_run(&s, renderer::DECK)?;
        output(renderer::DECK, intent::DECK, deck::render(ds))
    })
    .expect("fresh registry");

    let s = Arc::clone(&set);
    r.register(renderer::DEMO, move |vm, _ctx| {
        expect_vm(vm, renderer::DEMO, VM_DATASET)?;
        let ds = first_run(&s, renderer::DEMO)?;
        output(renderer::DEMO, intent::DEMO, demo::render(ds))
    })
    .expect("fresh registry");

    let s = Arc::clone(&set);
    r.register(renderer::COMPARE, move |vm, _ctx| {
        expect_vm(vm, renderer::COMPARE, VM_RUN_SET)?;
        if s.runs.len() < 2 {
            return Err(RenderError::Unrenderable {
                renderer: renderer::COMPARE,
                reason: format!("comparison needs >= 2 runs, got {}", s.runs.len()),
            });
        }
        output(renderer::COMPARE, intent::COMPARE, compare::render(&s.runs))
    })
    .expect("fresh registry");

    r
}

fn first_run<'a>(s: &'a RunSet, id: &'static str) -> Result<&'a MissionDataset, RenderError> {
    s.runs
        .first()
        .map(Arc::as_ref)
        .ok_or(RenderError::Unrenderable { renderer: id, reason: "empty run set".into() })
}

/// The view-model handle a bound renderer receives: the dataset named, not
/// copied (the closure already shares the data by `Arc`).
pub fn dataset_view(set: &RunSet, which: &str) -> ViewModel {
    let (kind, name, size) = if which == intent::COMPARE {
        (VM_RUN_SET, "/muas/v3/artifacts/run-set".to_string(), set.runs.len() as u64)
    } else {
        let label = set.runs.first().map(|r| r.run_label()).unwrap_or_default();
        (
            VM_DATASET,
            format!("/muas/v3/artifacts/run/{label}"),
            set.runs.first().map(|r| r.data.len() as u64).unwrap_or(0),
        )
    };
    ViewModel::Raw(RawInspectView {
        name,
        media_type: "application/x-mission-dataset".into(),
        size,
        kind: kind.into(),
    })
}

/// File name each intent renders to.
pub fn artifact_filename(which: &str) -> &'static str {
    match which {
        intent::REPORT => "report.html",
        intent::DECK => "deck.html",
        intent::DEMO => "demo.html",
        intent::COMPARE => "compare.html",
        _ => "artifact.html",
    }
}

/// Produce artifacts through the full flotilla path: author the instance
/// manifest, run the real matcher, authorize, instantiate the registered
/// renderer, render. Returns `filename -> artifact`.
pub fn produce(set: &Arc<RunSet>, intents: &[&str]) -> Result<BTreeMap<String, RenderedArtifact>, String> {
    let mut pack = artifact_pack();
    let single_subject = format!(
        "/muas/v3/artifacts/run/{}",
        set.runs.first().map(|r| r.run_label()).unwrap_or_default()
    );
    let dataset_manifest = pack
        .publish_instance(pack.terms.mission_dataset, &single_subject)
        .map_err(|e| format!("instance manifest: {e:?}"))?;
    let run_set_manifest = pack
        .publish_instance(pack.terms.run_set, "/muas/v3/artifacts/run-set")
        .map_err(|e| format!("run-set manifest: {e:?}"))?;

    let registry = registry_for(Arc::clone(set));
    let binder = Binder::new(&registry);
    let contracts = pack.contracts();
    let frontier = pack.frontier();
    let now_ns = set.runs.iter().map(|r| r.t1()).max().unwrap_or(0);

    let mut out = BTreeMap::new();
    for which in intents {
        let manifest =
            if *which == intent::COMPARE { run_set_manifest } else { dataset_manifest };
        let binding = binder
            .bind(&pack.dag, manifest, &contracts, which, &frontier, Budget::generous())
            .map_err(|e| format!("bind {which}: {e}"))?;
        let rendered = binding
            .render(&dataset_view(set, which), &RenderCtx { now_ns })
            .map_err(|e| format!("render {which}: {e}"))?;
        let html = rendered.body["html"]
            .as_str()
            .ok_or_else(|| format!("{which}: renderer produced no html"))?
            .to_string();
        let citations: Vec<Citation> =
            serde_json::from_value(rendered.body["citations"].clone())
                .map_err(|e| format!("{which}: citations: {e}"))?;
        tracing::info!(
            intent = which,
            renderer = %binding.renderer(),
            verdict = ?binding.verdict(),
            citations = citations.len(),
            "artifact bound + rendered"
        );
        out.insert(artifact_filename(which).to_string(), RenderedArtifact { html, citations });
    }
    Ok(out)
}

// ───────────────────────────── shared HTML bits ─────────────────────────────

/// HTML-escape.
pub(crate) fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// `mm:ss` relative to the run start.
pub(crate) fn t_rel(t_ns: u64, t0_ns: u64) -> String {
    let s = t_ns.saturating_sub(t0_ns) / 1_000_000_000;
    format!("{:02}:{:02}", s / 60, s % 60)
}

/// First 10 hex chars of a citation hash — enough to eyeball, full hash in
/// the tooltip and the audit surface.
pub(crate) fn hash_short(c: &Citation) -> String {
    c.hash.chars().take(10).collect()
}

/// The provenance drawer: progressive disclosure of the Block hashes a
/// section was derived from. Summary line first; full table on demand.
pub(crate) fn prov_drawer(label: &str, citations: &[Citation]) -> String {
    if citations.is_empty() {
        return String::new();
    }
    let mut rows = String::new();
    for c in citations {
        rows.push_str(&format!(
            "<tr><td class=\"mono\" title=\"{h}\">{s}</td><td class=\"mono\">{ch}</td><td>{q}</td></tr>",
            h = esc(&c.hash),
            s = hash_short(c),
            ch = esc(&c.chain),
            q = c.seq
        ));
    }
    format!(
        "<details class=\"prov\"><summary>provenance — {n} block{pl} · {label}</summary>\
         <table><thead><tr><th>block hash</th><th>chain</th><th>seq</th></tr></thead>\
         <tbody>{rows}</tbody></table></details>",
        n = citations.len(),
        pl = if citations.len() == 1 { "" } else { "s" },
        label = esc(label),
    )
}

/// The key settings the association layer leads with:
/// `(flattened key, human label, unit suffix)`.
pub(crate) const KEY_SETTINGS: &[(&str, &str, &str)] = &[
    ("link_profile.name", "link profile", ""),
    ("link_profile.loss_pct", "link loss", " %"),
    ("link_profile.delay_ms", "link delay", " ms"),
    ("link_profile.jitter_ms", "link jitter", " ms"),
    ("link_profile.bandwidth_mbps", "bandwidth", " Mbps"),
    ("coord.grace_s", "coop grace", " s"),
    ("coord.hsep_m", "h-sep", " m"),
    ("coord.vsep_m", "v-sep", " m"),
    ("coord.floor_agl_m", "flight floor", " m"),
    ("coord.rtl_base_agl_m", "RTL base", " m"),
    ("carrier", "carrier", ""),
    ("fleet_ids", "fleet", ""),
    ("telemetry_hz", "telemetry", " Hz"),
];

/// One compact association line: the subset of `keys` the run knows,
/// rendered `label value·unit`, joined with `·`. Unknown settings are
/// listed as unknown rather than dropped silently when `show_unknown`.
pub(crate) fn assoc_line(run: &RunConfig, keys: &[&str], show_unknown: bool) -> String {
    let mut parts = Vec::new();
    for key in keys {
        let Some((_, label, unit)) = KEY_SETTINGS.iter().find(|(k, _, _)| k == key) else {
            continue;
        };
        match run.display(key) {
            Some(v) => parts.push(format!("{label} {}{unit}", esc(&v))),
            None if show_unknown => parts.push(format!("{label} ?")),
            None => {}
        }
    }
    parts.join(" · ")
}

/// The "Run configuration" panel: the settings that mattered as a grid,
/// the full flattened config behind a drawer, and an honest banner when the
/// config was synthesized (no `run.config` record in the journals).
pub(crate) fn config_panel(run: &RunConfig) -> String {
    let mut cells = String::new();
    for (key, label, unit) in KEY_SETTINGS {
        let value = match run.display(key) {
            Some(v) => format!("{}{}", esc(&v), unit),
            None => "<span class=\"unknown\">unknown</span>".to_string(),
        };
        cells.push_str(&format!(
            "<div class=\"cfg-cell\"><div class=\"cfg-label\">{label}</div>\
             <div class=\"cfg-value\">{value}</div></div>"
        ));
    }
    let mut full = String::new();
    for (k, v) in &run.settings {
        full.push_str(&format!(
            "<tr><td class=\"mono\">{}</td><td class=\"mono\">{}</td></tr>",
            esc(k),
            esc(&crate::dataset::display_value(v))
        ));
    }
    let note = if run.synthesized {
        "<p class=\"note\">No <code>run.config</code> record in these journals — settings \
         below were inferred from <code>agent.up</code>; unlisted settings are unknown, \
         not defaulted.</p>"
    } else {
        ""
    };
    format!(
        "<section class=\"card\"><h2>Run configuration</h2>{note}\
         <div class=\"cfg-grid\">{cells}</div>\
         <details><summary>full configuration ({n} settings)</summary>\
         <table><thead><tr><th>setting</th><th>value</th></tr></thead><tbody>{full}</tbody></table>\
         </details></section>",
        n = run.settings.len()
    )
}

/// Shared stylesheet (dataviz reference palette, light + dark selected).
pub(crate) const STYLE: &str = r#"
:root {
  --surface:#fcfcfb; --page:#f9f9f7; --ink:#0b0b0b; --ink2:#52514e;
  --muted:#898781; --grid:#e1e0d9; --line:#c3c2b7;
  --s1:#2a78d6; --s2:#1baf7a; --s3:#eda100; --s4:#008300; --s5:#4a3aa7; --s6:#e34948;
  --good:#0ca30c; --warn:#fab219; --crit:#d03b3b;
  --border:rgba(11,11,11,0.10);
}
@media (prefers-color-scheme: dark) {
  :root {
    --surface:#1a1a19; --page:#0d0d0d; --ink:#ffffff; --ink2:#c3c2b7;
    --grid:#2c2c2a; --line:#383835;
    --s1:#3987e5; --s2:#199e70; --s3:#c98500; --s5:#9085e9; --s6:#e66767;
    --border:rgba(255,255,255,0.10);
  }
}
* { box-sizing: border-box; }
body { margin:0; background:var(--page); color:var(--ink);
       font:15px/1.55 system-ui,-apple-system,"Segoe UI",sans-serif; }
main { max-width: 920px; margin: 0 auto; padding: 24px 20px 64px; }
h1 { font-size: 26px; margin: 8px 0 2px; }
h2 { font-size: 17px; margin: 0 0 12px; }
.sub { color: var(--ink2); margin: 0 0 20px; }
.card { background: var(--surface); border: 1px solid var(--border);
        border-radius: 8px; padding: 16px 18px; margin: 16px 0; }
.mono { font-family: ui-monospace,SFMono-Regular,Menlo,monospace; font-size: 12.5px; }
a { color: var(--s1); }
table { border-collapse: collapse; width: 100%; margin: 8px 0; }
th { text-align: left; color: var(--muted); font-weight: 500; font-size: 12.5px; }
th, td { padding: 5px 10px 5px 0; border-bottom: 1px solid var(--grid); }
td { font-variant-numeric: tabular-nums; }
.unknown { color: var(--muted); font-style: italic; }
.note { color: var(--ink2); font-size: 13.5px; }
.assoc { color: var(--ink2); font-size: 13px; margin: 2px 0 10px; }
.assoc::before { content: "associated settings — "; color: var(--muted); }
details.prov { margin-top: 10px; font-size: 12.5px; color: var(--ink2); }
details.prov summary, details summary { cursor: pointer; color: var(--muted); }
.cfg-grid { display: grid; grid-template-columns: repeat(auto-fill,minmax(150px,1fr));
            gap: 10px 16px; margin: 10px 0; }
.cfg-label { color: var(--muted); font-size: 12px; }
.cfg-value { font-size: 15px; font-variant-numeric: tabular-nums; }
.footer { color: var(--muted); font-size: 12.5px; margin-top: 28px; }
.overflow { overflow-x: auto; }
"#;

/// Wrap a body in the standard page shell.
pub(crate) fn page(title: &str, extra_style: &str, body: &str) -> String {
    format!(
        "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">\
         <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
         <title>{t}</title><style>{STYLE}{extra_style}</style></head>\
         <body>{body}</body></html>",
        t = esc(title),
    )
}
