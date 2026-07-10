//! The inter-run comparison lens: input→output association across runs.
//! Settings that never changed collapse into a shared header; the settings
//! that DID change become highlighted columns beside the outcome columns,
//! so "only grace changed between run A and B; coop went 90→60" is a
//! one-glance read. Labeled *associated settings* — the lens shows the
//! association, it does not claim causality.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use serde_json::json;

use crate::dataset::{display_value, Citation, DatumKind, MissionDataset};
use crate::metrics::{metrics, outcome_columns, MetricCell};
use crate::render::{esc, page, prov_drawer, RenderedArtifact};

const COMPARE_STYLE: &str = r#"
th.sortable { cursor: pointer; user-select: none; }
th.sortable:hover { color: var(--ink); }
th.delta, td.delta { border-left: 2px solid var(--s1); padding-left: 10px; }
thead .group th { border-bottom: none; color: var(--ink2); font-size: 12px; }
canvas { width: 100%; height: 380px; display: block; border-radius: 4px; }
.map-wrap { background: var(--surface); border: 1px solid var(--border);
            border-radius: 8px; padding: 10px; }
.legend { display: flex; flex-wrap: wrap; gap: 8px 22px; font-size: 13px;
          color: var(--ink2); margin-top: 8px; }
.key-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%;
           vertical-align: middle; margin-right: 6px; }
.legend .delta-note { color: var(--muted); }
.csv { font-size: 13px; }
"#;

const SORT_JS: &str = r#"
const table = document.getElementById('cmp');
const dirs = {};
table.querySelectorAll('thead tr.cols th').forEach((th, i) => {
  th.classList.add('sortable');
  th.title = 'click to sort';
  th.addEventListener('click', () => {
    dirs[i] = -(dirs[i] || -1);
    const tb = table.tBodies[0];
    [...tb.rows]
      .sort((a, b) => {
        const x = a.cells[i], y = b.cells[i];
        const nx = parseFloat(x.dataset.num), ny = parseFloat(y.dataset.num);
        const c = (!isNaN(nx) && !isNaN(ny)) ? nx - ny
                : x.textContent.localeCompare(y.textContent);
        return c * dirs[i];
      })
      .forEach(r => tb.appendChild(r));
  });
});
"#;

const OVERLAY_JS: &str = r#"
const cv = document.getElementById('overlay'), ctx = cv.getContext('2d');
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const SERIES = ['--s1','--s2','--s3','--s5','--s6','--s4'];
const MLAT = 111320;
let b = null;
for (const run of RUNS) for (const tr of run.tracks) for (const p of tr) {
  if (!b) b = { la0: p[0], la1: p[0], lo0: p[1], lo1: p[1] };
  b.la0 = Math.min(b.la0, p[0]); b.la1 = Math.max(b.la1, p[0]);
  b.lo0 = Math.min(b.lo0, p[1]); b.lo1 = Math.max(b.lo1, p[1]);
}
if (!b) b = { la0: 0, la1: 0.001, lo0: 0, lo1: 0.001 };
const clat = (b.la0 + b.la1) / 2, clon = (b.lo0 + b.lo1) / 2;
const mlon = MLAT * Math.cos(clat * Math.PI / 180);
function draw() {
  const r = cv.getBoundingClientRect();
  cv.width = r.width * devicePixelRatio; cv.height = r.height * devicePixelRatio;
  const wm = Math.max((b.la1 - b.la0) * MLAT, (b.lo1 - b.lo0) * mlon, 20);
  const mppx = wm * 1.3 / Math.min(cv.width, cv.height);
  const proj = (lat, lon) => [cv.width / 2 + (lon - clon) * mlon / mppx,
                              cv.height / 2 - (lat - clat) * MLAT / mppx];
  ctx.fillStyle = css('--surface'); ctx.fillRect(0, 0, cv.width, cv.height);
  const step = 50 / mppx;
  ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1;
  const [ox, oy] = proj(clat, clon);
  for (let x = ox % step; x < cv.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cv.height); ctx.stroke(); }
  for (let y = oy % step; y < cv.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cv.width, y); ctx.stroke(); }
  ctx.font = (11 * devicePixelRatio) + 'px system-ui';
  RUNS.forEach((run, ri) => {
    const color = css(SERIES[ri % SERIES.length]);
    ctx.strokeStyle = color; ctx.lineWidth = 2 * devicePixelRatio;
    ctx.lineJoin = ctx.lineCap = 'round';
    run.tracks.forEach((tr, vi) => {
      ctx.beginPath();
      tr.forEach((p, i) => {
        const [x, y] = proj(p[0], p[1]);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      const p = tr[tr.length - 1];
      if (p) {
        const [x, y] = proj(p[0], p[1]);
        ctx.beginPath(); ctx.arc(x, y, 5 * devicePixelRatio, 0, 7);
        ctx.fillStyle = color; ctx.fill();
        ctx.lineWidth = 2 * devicePixelRatio; ctx.strokeStyle = css('--surface'); ctx.stroke();
        ctx.fillStyle = css('--ink2');
        ctx.fillText(run.vehicles[vi] || '', x + 9 * devicePixelRatio, y - 7 * devicePixelRatio);
      }
    });
  });
  ctx.fillStyle = css('--muted');
  ctx.fillText('grid 50 m', 10 * devicePixelRatio, cv.height - 10 * devicePixelRatio);
}
addEventListener('resize', draw);
draw();
"#;

fn percent_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' | b'/' | b':' => {
                out.push(b as char)
            }
            b' ' => out.push_str("%20"),
            other => out.push_str(&format!("%{other:02X}")),
        }
    }
    out
}

struct RunView {
    label: String,
    settings: BTreeMap<String, String>,
    outcomes: BTreeMap<String, MetricCell>,
}

/// Render `compare.html` from two or more run datasets.
pub fn render(runs: &[Arc<MissionDataset>]) -> RenderedArtifact {
    let views: Vec<RunView> = runs
        .iter()
        .map(|ds| RunView {
            label: ds.run_label(),
            settings: ds
                .run
                .settings
                .iter()
                .map(|(k, v)| (k.clone(), display_value(v)))
                .collect(),
            outcomes: outcome_columns(&metrics(ds)),
        })
        .collect();

    // Partition settings: shared (same everywhere) vs deltas (differ or
    // missing somewhere) — the deltas are the association the page leads with.
    let all_keys: BTreeSet<&String> = views.iter().flat_map(|v| v.settings.keys()).collect();
    let mut shared: Vec<(String, String)> = Vec::new();
    let mut delta_keys: Vec<String> = Vec::new();
    for key in all_keys {
        let values: Vec<Option<&String>> = views.iter().map(|v| v.settings.get(key)).collect();
        match values[0] {
            Some(first) if values.iter().all(|v| *v == Some(first)) => {
                shared.push((key.clone(), first.clone()));
            }
            _ => delta_keys.push(key.clone()),
        }
    }
    let outcome_keys: Vec<String> = {
        let mut keys: Vec<String> = Vec::new();
        for v in &views {
            for k in v.outcomes.keys() {
                if !keys.contains(k) {
                    keys.push(k.clone());
                }
            }
        }
        keys
    };

    // ── the table (+ CSV twin) ─────────────────────────────────────────
    let mut group_row = format!(
        "<tr class=\"group\"><th></th><th colspan=\"{}\" class=\"delta\">settings that changed</th>\
         <th colspan=\"{}\">outcomes</th></tr>",
        delta_keys.len().max(1),
        outcome_keys.len().max(1)
    );
    if delta_keys.is_empty() {
        group_row = format!(
            "<tr class=\"group\"><th></th><th colspan=\"{}\">outcomes (no settings differ)</th></tr>",
            outcome_keys.len().max(1)
        );
    }
    let mut header = String::from("<tr class=\"cols\"><th>run</th>");
    let mut csv = String::from("run");
    for k in &delta_keys {
        header.push_str(&format!("<th class=\"delta\">{}</th>", esc(k)));
        csv.push_str(&format!(",{k}"));
    }
    for k in &outcome_keys {
        header.push_str(&format!("<th>{}</th>", esc(k)));
        csv.push_str(&format!(",{k}"));
    }
    header.push_str("</tr>");
    csv.push('\n');

    let mut rows = String::new();
    for v in &views {
        rows.push_str(&format!("<tr><td>{}</td>", esc(&v.label)));
        csv.push_str(&v.label.replace(',', ";"));
        for k in &delta_keys {
            let val = v.settings.get(k).cloned().unwrap_or_else(|| "—".into());
            rows.push_str(&format!("<td class=\"delta\">{}</td>", esc(&val)));
            csv.push_str(&format!(",{}", val.replace(',', ";")));
        }
        for k in &outcome_keys {
            match v.outcomes.get(k) {
                Some(cell) => {
                    let num = cell.num.map(|n| format!(" data-num=\"{n}\"")).unwrap_or_default();
                    rows.push_str(&format!("<td{num}>{}</td>", esc(&cell.display)));
                    csv.push_str(&format!(",{}", cell.display.replace(',', ";")));
                }
                None => {
                    rows.push_str("<td>—</td>");
                    csv.push(',');
                }
            }
        }
        rows.push_str("</tr>");
        csv.push('\n');
    }

    // ── shared settings header ─────────────────────────────────────────
    let mut shared_rows = String::new();
    for (k, v) in &shared {
        shared_rows.push_str(&format!(
            "<tr><td class=\"mono\">{}</td><td class=\"mono\">{}</td></tr>",
            esc(k),
            esc(v)
        ));
    }
    let delta_summary = if delta_keys.is_empty() {
        "No settings differ between these runs — outcome differences are run-to-run \
         variation, not configuration."
            .to_string()
    } else {
        format!(
            "Only <strong>{n}</strong> setting{pl} differ{v} between these runs: \
             <code>{keys}</code>. Outcome differences are <em>associated</em> with \
             those settings — association shown, causality not claimed.",
            n = delta_keys.len(),
            pl = if delta_keys.len() == 1 { "" } else { "s" },
            v = if delta_keys.len() == 1 { "s" } else { "" },
            keys = delta_keys.iter().map(|k| esc(k)).collect::<Vec<_>>().join("</code>, <code>"),
        )
    };

    // ── overlay data ───────────────────────────────────────────────────
    let overlay_runs: Vec<serde_json::Value> = runs
        .iter()
        .map(|ds| {
            let vehicles = ds.vehicles();
            let mut tracks: Vec<Vec<serde_json::Value>> = vec![Vec::new(); vehicles.len()];
            for d in ds.of_kind(DatumKind::Telemetry) {
                let Some(i) = d
                    .vehicle
                    .as_deref()
                    .and_then(|v| vehicles.iter().position(|x| x == v))
                else {
                    continue;
                };
                let s = &d.body["sample"];
                if let (Some(lat), Some(lon)) = (s["lat_deg"].as_f64(), s["lon_deg"].as_f64()) {
                    tracks[i].push(json!([lat, lon]));
                }
            }
            json!({ "label": ds.run_label(), "vehicles": vehicles, "tracks": tracks })
        })
        .collect();

    let series_vars = ["--s1", "--s2", "--s3", "--s5", "--s6", "--s4"];
    let mut legend = String::from("<div class=\"legend\">");
    for (i, v) in views.iter().enumerate() {
        let deltas: Vec<String> = delta_keys
            .iter()
            .filter_map(|k| v.settings.get(k).map(|val| format!("{k} {val}")))
            .collect();
        legend.push_str(&format!(
            "<span><span class=\"key-dot\" style=\"background:var({var})\"></span>\
             <strong>{label}</strong> <span class=\"delta-note\">{note}</span></span>",
            var = series_vars[i % series_vars.len()],
            label = esc(&v.label),
            note = esc(&if deltas.is_empty() { "(identical settings)".into() } else { deltas.join(" · ") }),
        ));
    }
    legend.push_str("</div>");

    // ── provenance ─────────────────────────────────────────────────────
    let mut all: Vec<Citation> = runs.iter().flat_map(|ds| ds.all_citations()).collect();
    all.sort();
    all.dedup();
    let mut prov = String::new();
    for ds in runs {
        prov.push_str(&prov_drawer(&format!("run {}", ds.run_label()), &ds.all_citations()));
    }

    let body = format!(
        "<main><h1>Run comparison</h1>\
         <p class=\"sub\">{n} runs · same lens machinery, one dataset per run · \
         {blocks} blocks total</p>\
         <section class=\"card\"><h2>What changed between runs</h2>\
         <p class=\"note\">{delta_summary}</p>\
         <details><summary>shared settings ({ns} identical across all runs)</summary>\
         <table><thead><tr><th>setting</th><th>value</th></tr></thead>\
         <tbody>{shared_rows}</tbody></table></details></section>\
         <section class=\"card\"><h2>Settings ↔ outcomes</h2>\
         <p class=\"note\">Click a column header to sort. \
         <a class=\"csv\" download=\"run-comparison.csv\" href=\"data:text/csv;charset=utf-8,{csv_uri}\">\
         download CSV</a></p>\
         <div class=\"overflow\"><table id=\"cmp\"><thead>{group_row}{header}</thead>\
         <tbody>{rows}</tbody></table></div></section>\
         <section class=\"card\"><h2>Tracks, overlaid</h2>\
         <p class=\"note\">Every run's ground tracks on one map, color = run. The legend \
         ties each color to the settings that distinguish that run.</p>\
         <div class=\"map-wrap\"><canvas id=\"overlay\"></canvas>{legend}</div></section>\
         <section class=\"card\"><h2>Provenance</h2>\
         <p class=\"note\">Each run cites its own chains; a datum shared with the \
         single-run artifacts carries the identical Block hash there.</p>{prov}</section>\
         <p class=\"footer\">Rendered through a matched, authorized console binding \
         (artifact.compare) over the run set — associated settings, not claimed causality.</p>\
         </main>\
         <script>const RUNS = {overlay};</script>\
         <script>{SORT_JS}</script><script>{OVERLAY_JS}</script>",
        n = runs.len(),
        blocks = all.len(),
        ns = shared.len(),
        csv_uri = percent_encode(&csv),
        overlay = serde_json::to_string(&overlay_runs).expect("overlay data serializes"),
    );

    RenderedArtifact {
        html: page("Run comparison", COMPARE_STYLE, &body),
        citations: all,
    }
}
