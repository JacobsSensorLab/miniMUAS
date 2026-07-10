//! The stakeholder lens: 7 slides, high level, arrow-key navigation. Same
//! dataset, same Block citations as the report — just less of them shown.

use crate::dataset::{DatumKind, MissionDataset};
use crate::metrics::metrics;
use crate::render::{assoc_line, esc, hash_short, page, t_rel, RenderedArtifact, KEY_SETTINGS};

const DECK_STYLE: &str = r#"
html { scroll-snap-type: y mandatory; }
.slide { min-height: 100vh; scroll-snap-align: start; display: flex;
         flex-direction: column; justify-content: center; align-items: center;
         padding: 48px 24px; position: relative; }
.slide-inner { max-width: 780px; width: 100%; }
.slide h1 { font-size: 40px; margin: 0 0 8px; }
.slide h2 { font-size: 28px; margin: 0 0 18px; }
.hero { font-size: 84px; font-weight: 600; line-height: 1.05; margin: 12px 0; }
.hero-label { color: var(--ink2); font-size: 17px; }
.slide-footer { position: absolute; bottom: 16px; left: 0; right: 0; text-align: center;
                color: var(--muted); font-size: 12px; }
.bullets { font-size: 19px; line-height: 1.8; }
.kv { display: grid; grid-template-columns: auto auto; gap: 6px 28px;
      font-size: 19px; justify-content: center; }
.kv .k { color: var(--ink2); text-align: right; }
.kv .v { font-variant-numeric: tabular-nums; }
.nav-hint { color: var(--muted); font-size: 13px; margin-top: 32px; }
.counter { position: fixed; right: 18px; bottom: 14px; color: var(--muted);
           font-size: 12.5px; font-variant-numeric: tabular-nums; z-index: 2; }
.legend { display: flex; gap: 18px; justify-content: center; font-size: 13.5px;
          color: var(--ink2); margin-top: 6px; }
.key { display: inline-block; width: 18px; height: 3px; border-radius: 2px;
       vertical-align: middle; margin-right: 6px; }
svg text { fill: var(--muted); font-size: 11px; }
"#;

const DECK_JS: &str = r#"
const slides = [...document.querySelectorAll('.slide')];
let cur = 0;
const counter = document.getElementById('counter');
function go(i) {
  cur = Math.max(0, Math.min(slides.length - 1, i));
  slides[cur].scrollIntoView({ behavior: 'smooth' });
  counter.textContent = (cur + 1) + ' / ' + slides.length;
}
addEventListener('keydown', (e) => {
  if (e.key === 'ArrowRight' || e.key === 'PageDown' || e.key === ' ') { e.preventDefault(); go(cur + 1); }
  if (e.key === 'ArrowLeft' || e.key === 'PageUp') { e.preventDefault(); go(cur - 1); }
  if (e.key === 'Home') go(0);
  if (e.key === 'End') go(slides.length - 1);
});
let ticking = false;
addEventListener('scroll', () => {
  if (ticking) return; ticking = true;
  requestAnimationFrame(() => {
    ticking = false;
    const mid = scrollY + innerHeight / 2;
    slides.forEach((s, i) => { if (s.offsetTop <= mid && mid < s.offsetTop + s.offsetHeight) cur = i; });
    counter.textContent = (cur + 1) + ' / ' + slides.length;
  });
});
counter.textContent = '1 / ' + slides.length;
"#;

/// Palette series slots (light-mode hex lives in CSS vars; here we only
/// pick var names so dark mode stays selected, not flipped).
const SERIES_VARS: &[&str] = &["--s1", "--s2", "--s3", "--s5", "--s6", "--s4"];

fn slide(body: &str, footer: &str) -> String {
    format!(
        "<section class=\"slide\"><div class=\"slide-inner\">{body}</div>\
         <div class=\"slide-footer\">{footer}</div></section>"
    )
}

/// A bias-over-time step chart from the coordination events (SVG, 2px
/// lines, hairline grid, legend + end dots). Returns `None` when the run
/// had no bias data — the slide says so instead of drawing an empty frame.
fn bias_chart(ds: &MissionDataset) -> Option<String> {
    let t0 = ds.t0();
    let t1 = ds.t1().max(t0 + 1);
    let vehicles = ds.vehicles();
    let mut series: Vec<(String, Vec<(f64, f64)>)> = Vec::new();
    for vid in &vehicles {
        let mut pts = Vec::new();
        for d in ds.of_kind(DatumKind::Coord) {
            if d.vehicle.as_deref() != Some(vid) {
                continue;
            }
            if let Some(b) = d.body["bias_m"].as_f64() {
                let x = (d.t_ns.saturating_sub(t0)) as f64 / (t1 - t0) as f64;
                pts.push((x, b));
            }
        }
        if !pts.is_empty() {
            series.push((vid.clone(), pts));
        }
    }
    if series.is_empty() {
        return None;
    }
    let (w, h, ml, mr, mt, mb) = (680.0, 260.0, 44.0, 16.0, 12.0, 26.0);
    let (pw, ph) = (w - ml - mr, h - mt - mb);
    let ymax = series
        .iter()
        .flat_map(|(_, pts)| pts.iter().map(|p| p.1.abs()))
        .fold(1.0f64, f64::max)
        .ceil();
    let sx = |x: f64| ml + x * pw;
    let sy = |y: f64| mt + ph - (y / ymax).clamp(0.0, 1.0) * ph;
    let mut svg = format!(
        "<svg viewBox=\"0 0 {w} {h}\" role=\"img\" aria-label=\"avoidance bias over time\">"
    );
    // Hairline grid + y labels (clean steps).
    for i in 0..=2 {
        let yv = ymax * i as f64 / 2.0;
        let y = sy(yv);
        svg.push_str(&format!(
            "<line x1=\"{ml}\" y1=\"{y:.1}\" x2=\"{x2}\" y2=\"{y:.1}\" stroke=\"var(--grid)\" stroke-width=\"1\"/>\
             <text x=\"{tx}\" y=\"{ty:.1}\" text-anchor=\"end\">{yv:.0} m</text>",
            x2 = w - mr,
            tx = ml - 6.0,
            ty = y + 3.5,
        ));
    }
    // Step lines per vehicle.
    for (i, (_, pts)) in series.iter().enumerate() {
        let color = format!("var({})", SERIES_VARS[i % SERIES_VARS.len()]);
        let mut path = String::new();
        let mut last_y = sy(0.0);
        path.push_str(&format!("M {ml:.1} {last_y:.1}"));
        for (x, b) in pts {
            let (px, py) = (sx(*x), sy(b.abs()));
            path.push_str(&format!(" L {px:.1} {last_y:.1} L {px:.1} {py:.1}"));
            last_y = py;
        }
        path.push_str(&format!(" L {x:.1} {last_y:.1}", x = w - mr));
        svg.push_str(&format!(
            "<path d=\"{path}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"2\" \
             stroke-linejoin=\"round\" stroke-linecap=\"round\"/>"
        ));
        if let Some((x, b)) = pts.last() {
            svg.push_str(&format!(
                "<circle cx=\"{cx:.1}\" cy=\"{cy:.1}\" r=\"4\" fill=\"{color}\" \
                 stroke=\"var(--surface)\" stroke-width=\"2\"/>",
                cx = sx(*x),
                cy = sy(b.abs()),
            ));
        }
    }
    // x labels: start/end.
    svg.push_str(&format!(
        "<text x=\"{ml}\" y=\"{y}\" text-anchor=\"start\">{a}</text>\
         <text x=\"{x2}\" y=\"{y}\" text-anchor=\"end\">{b}</text></svg>",
        y = h - 8.0,
        x2 = w - mr,
        a = t_rel(t0, t0),
        b = t_rel(t1, t0),
    ));
    let mut legend = String::from("<div class=\"legend\">");
    for (i, (vid, _)) in series.iter().enumerate() {
        legend.push_str(&format!(
            "<span><span class=\"key\" style=\"background:var({})\"></span>{}</span>",
            SERIES_VARS[i % SERIES_VARS.len()],
            esc(vid)
        ));
    }
    legend.push_str("</div>");
    Some(format!("{svg}{legend}"))
}

/// Render `deck.html` from the one dataset.
pub fn render(ds: &MissionDataset) -> RenderedArtifact {
    let m = metrics(ds);
    let label = ds.run_label();
    let all = ds.all_citations();
    let short: Vec<String> = all.iter().take(4).map(hash_short).collect();
    let footer = format!(
        "<span title=\"{tip}\">run {label} · derived from {n} content-addressed blocks</span>",
        tip = esc(&format!("{}{}", short.join(", "), if all.len() > 4 { ", …" } else { "" })),
        label = esc(&label),
        n = all.len(),
    );

    let mut slides = String::new();

    // 1 — title.
    slides.push_str(&slide(
        &format!(
            "<h1>{label}</h1><p class=\"sub\" style=\"font-size:20px\">miniMUAS mission — \
             {nveh} vehicles, {dur:.0} s</p>\
             <p class=\"bullets\">One mission dataset. Every artifact you will see — this \
             deck, the engineer's report, the live demo — reads the <em>same named, \
             content-addressed data</em>. Nothing was exported, copied, or re-keyed.</p>\
             <p class=\"nav-hint\">→ / ← to navigate</p>",
            label = esc(&label),
            nveh = ds.vehicles().len(),
            dur = m.duration_s,
        ),
        &footer,
    ));

    // 2 — run configuration (the inputs).
    let mut kv = String::new();
    for (key, name, unit) in KEY_SETTINGS {
        if let Some(v) = ds.run.display(key) {
            kv.push_str(&format!(
                "<div class=\"k\">{name}</div><div class=\"v\">{}{unit}</div>",
                esc(&v)
            ));
        }
    }
    if kv.is_empty() {
        kv = "<div class=\"k\">configuration</div><div class=\"v unknown\">unknown (no run.config record)</div>".into();
    }
    slides.push_str(&slide(
        &format!(
            "<h2>The inputs — run configuration</h2><div class=\"kv\">{kv}</div>\
             {note}",
            note = if ds.run.synthesized {
                "<p class=\"note\" style=\"text-align:center;margin-top:18px\">inferred from \
                 journals — no run.config record; unlisted settings unknown</p>"
            } else {
                ""
            }
        ),
        &footer,
    ));

    // 3 — outcome hero.
    let (hero, hero_label) = match m.coop_rate_pct() {
        Some(rate) => (
            format!("{rate:.0}%"),
            format!(
                "cooperative avoidance success ({}/{} episodes)",
                m.coop,
                m.coop + m.unco
            ),
        ),
        None => (
            format!("{}/{}", m.takeoff_ok, m.takeoff_attempts.max(1)),
            "takeoffs completed".to_string(),
        ),
    };
    slides.push_str(&slide(
        &format!(
            "<h2>The outcome</h2><div class=\"hero\">{hero}</div>\
             <div class=\"hero-label\">{hl}</div>\
             <p class=\"bullets\">{tok}/{tatt} takeoffs airborne · RTL {rtl} · \
             {ops} service calls ({rej} rejected)</p>\
             <p class=\"assoc\" style=\"text-align:center\">{assoc}</p>",
            hl = esc(&hero_label),
            tok = m.takeoff_ok,
            tatt = m.takeoff_attempts,
            rtl = m.rtl_outcome.as_deref().map(esc).unwrap_or_else(|| "not engaged".into()),
            ops = m.service_ops,
            rej = m.service_rejected,
            assoc = assoc_line(&ds.run, &["coord.grace_s", "link_profile.loss_pct", "carrier"], false),
        ),
        &footer,
    ));

    // 4 — coordination highlight + chart.
    let chart = bias_chart(ds).unwrap_or_else(|| {
        "<p class=\"note\">No coordination episodes in this run.</p>".to_string()
    });
    slides.push_str(&slide(
        &format!(
            "<h2>Fleet coordination</h2>\
             <p class=\"note\">Vertical avoidance bias applied over the mission — each \
             confirmed cooperative episode splits the separation; an unconfirmed peer \
             pushes the whole burden up.</p>{chart}\
             <p class=\"assoc\" style=\"text-align:center\">{assoc}</p>",
            assoc = assoc_line(
                &ds.run,
                &["coord.grace_s", "coord.hsep_m", "coord.vsep_m", "coord.floor_agl_m"],
                false
            ),
        ),
        &footer,
    ));

    // 5 — radio / link.
    let mut lrows = String::new();
    for (vid, v) in &m.vehicles {
        lrows.push_str(&format!(
            "<tr><td>{}</td><td>{}</td></tr>",
            esc(vid),
            v.interarrival_ms
                .as_ref()
                .map(|p| format!("{:.0} / {:.0} ms", p.p50, p.p95))
                .unwrap_or_else(|| "—".into()),
        ));
    }
    let mut link_facts = Vec::new();
    if let Some(p) = &m.service_rtt_ms {
        link_facts.push(format!("service RTT {:.0}/{:.0} ms", p.p50, p.p95));
    }
    if let Some(loss) = m.spark_loss_pct {
        link_facts.push(format!("spark loss {loss:.1}%"));
    }
    if m.stale_marks > 0 {
        link_facts.push(format!("{} stale marks", m.stale_marks));
    }
    slides.push_str(&slide(
        &format!(
            "<h2>The radio, measured</h2>\
             <p class=\"assoc\" style=\"text-align:center\">{assoc}</p>\
             <table><thead><tr><th>vehicle</th><th>telemetry inter-arrival p50 / p95</th></tr></thead>\
             <tbody>{lrows}</tbody></table><p class=\"note\">{facts}</p>",
            assoc = assoc_line(
                &ds.run,
                &["link_profile.name", "link_profile.loss_pct", "link_profile.delay_ms",
                  "link_profile.bandwidth_mbps"],
                true
            ),
            facts = esc(&if link_facts.is_empty() {
                "no service/spark link data in this run".to_string()
            } else {
                link_facts.join(" · ")
            }),
        ),
        &footer,
    ));

    // 6 — provenance.
    let mut prov_rows = String::new();
    for c in all.iter().take(8) {
        prov_rows.push_str(&format!(
            "<tr><td class=\"mono\" title=\"{}\">{}</td><td class=\"mono\">{}</td><td>{}</td></tr>",
            esc(&c.hash),
            hash_short(c),
            esc(&c.chain),
            c.seq
        ));
    }
    slides.push_str(&slide(
        &format!(
            "<h2>Where these numbers come from</h2>\
             <p class=\"bullets\">Every figure in this deck was read from {n} signed \
             Blocks on the vehicles' own journal chains and the ground station's \
             recording chain. The engineer's report and the replay demo cite \
             <em>the same hashes</em> — same data, different lens.</p>\
             <details><summary>sample of the block set</summary>\
             <table><thead><tr><th>hash</th><th>chain</th><th>seq</th></tr></thead>\
             <tbody>{prov_rows}</tbody></table></details>\
             <p class=\"note\">Full verification: <code>muas-artifacts --audit</code> \
             re-fetches and re-hashes every cited Block.</p>",
            n = all.len(),
        ),
        &footer,
    ));

    // 7 — closing.
    slides.push_str(&slide(
        "<h1>Same data.<br>As much — or as little — as each audience needs.</h1>\
         <p class=\"bullets\">Report · deck · demo · comparison: four lenses, one \
         dataset, zero copies. Provenance is the Block hash, not a file path.</p>",
        &footer,
    ));

    let body = format!(
        "{slides}<div class=\"counter\" id=\"counter\"></div><script>{DECK_JS}</script>"
    );
    RenderedArtifact {
        html: page(&format!("Mission deck — {label}"), DECK_STYLE, &body),
        citations: all,
    }
}
