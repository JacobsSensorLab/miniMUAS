//! The demo lens: a self-contained replayable mini-map (canvas), animating
//! the mission from the same telemetry the report summarizes — the
//! dashboard's local-ENU track drawing approach in miniature.

use serde_json::{json, Value};

use crate::dataset::{DatumKind, MissionDataset};
use crate::render::{assoc_line, esc, page, prov_drawer, RenderedArtifact};

const DEMO_STYLE: &str = r#"
.map-wrap { background: var(--surface); border: 1px solid var(--border);
            border-radius: 8px; padding: 10px; }
canvas { width: 100%; height: 420px; display: block; border-radius: 4px; }
.controls { display: flex; gap: 10px; align-items: center; margin-top: 10px;
            flex-wrap: wrap; }
.controls button, .controls select {
  font: inherit; background: var(--page); color: var(--ink);
  border: 1px solid var(--line); border-radius: 6px; padding: 4px 12px; cursor: pointer; }
.controls input[type=range] { flex: 1; min-width: 160px; accent-color: var(--s1); }
.clock { font-variant-numeric: tabular-nums; color: var(--ink2); min-width: 88px; }
.ticker { margin-top: 10px; font-size: 13px; color: var(--ink2); min-height: 7.5em; }
.ticker div { border-bottom: 1px solid var(--grid); padding: 2px 0; }
.legend { display: flex; gap: 16px; font-size: 13px; color: var(--ink2); margin-top: 8px; }
.key-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%;
           vertical-align: middle; margin-right: 5px; }
"#;

/// The replay data embedded in the page — built straight off the dataset,
/// serialized once (serde_json maps are ordered, so bytes are stable).
fn replay_data(ds: &MissionDataset) -> Value {
    let t0 = ds.t0();
    let vehicles = ds.vehicles();
    let vidx = |v: &Option<String>| -> i64 {
        v.as_deref()
            .and_then(|v| vehicles.iter().position(|x| x == v))
            .map(|i| i as i64)
            .unwrap_or(-1)
    };
    let mut tracks: Vec<Vec<Value>> = vec![Vec::new(); vehicles.len()];
    for d in ds.of_kind(DatumKind::Telemetry) {
        let i = vidx(&d.vehicle);
        if i < 0 {
            continue;
        }
        let s = &d.body["sample"];
        let (Some(lat), Some(lon)) = (s["lat_deg"].as_f64(), s["lon_deg"].as_f64()) else {
            continue;
        };
        let t = (d.t_ns.saturating_sub(t0)) as f64 / 1e9;
        let agl = s["agl_m"].as_f64().unwrap_or(0.0);
        tracks[i as usize].push(json!([t, lat, lon, agl]));
    }
    let mut events = Vec::new();
    for d in &ds.data {
        if !matches!(d.kind, DatumKind::Coord | DatumKind::Service | DatumKind::Rtl | DatumKind::Event)
            || d.label == "run.config"
        {
            continue;
        }
        let t = (d.t_ns.saturating_sub(t0)) as f64 / 1e9;
        events.push(json!([t, vidx(&d.vehicle), d.label]));
    }
    json!({
        "dur": (ds.t1().saturating_sub(t0)) as f64 / 1e9,
        "vehicles": vehicles,
        "tracks": tracks,
        "events": events,
    })
}

const DEMO_JS: &str = r#"
const cv = document.getElementById('map'), ctx = cv.getContext('2d');
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const SERIES = ['--s1','--s2','--s3','--s5','--s6','--s4'];
const MLAT = 111320;
let bounds = null;
for (const tr of DATA.tracks) for (const p of tr) {
  if (!bounds) bounds = { la0: p[1], la1: p[1], lo0: p[2], lo1: p[2] };
  bounds.la0 = Math.min(bounds.la0, p[1]); bounds.la1 = Math.max(bounds.la1, p[1]);
  bounds.lo0 = Math.min(bounds.lo0, p[2]); bounds.lo1 = Math.max(bounds.lo1, p[2]);
}
if (!bounds) bounds = { la0: 0, la1: 0.001, lo0: 0, lo1: 0.001 };
const clat = (bounds.la0 + bounds.la1) / 2, clon = (bounds.lo0 + bounds.lo1) / 2;
const mlon = MLAT * Math.cos(clat * Math.PI / 180);
function resize() {
  const r = cv.getBoundingClientRect();
  cv.width = r.width * devicePixelRatio; cv.height = r.height * devicePixelRatio;
}
addEventListener('resize', () => { resize(); draw(); });
resize();
function proj(lat, lon) {
  const wm = Math.max((bounds.la1 - bounds.la0) * MLAT, (bounds.lo1 - bounds.lo0) * mlon, 20);
  const mppx = wm * 1.3 / Math.min(cv.width, cv.height);
  return [cv.width / 2 + (lon - clon) * mlon / mppx,
          cv.height / 2 - (lat - clat) * MLAT / mppx, mppx];
}
let clock = 0, playing = false, speed = 1, last = null;
const dur = Math.max(DATA.dur, 1);
const fmt = (s) => String(Math.floor(s / 60)).padStart(2, '0') + ':' + String(Math.floor(s % 60)).padStart(2, '0');
function draw() {
  ctx.fillStyle = css('--surface'); ctx.fillRect(0, 0, cv.width, cv.height);
  // 50 m grid.
  const [ox, oy, mppx] = proj(clat, clon);
  const step = 50 / mppx;
  ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1;
  for (let x = ox % step; x < cv.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cv.height); ctx.stroke(); }
  for (let y = oy % step; y < cv.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cv.width, y); ctx.stroke(); }
  ctx.fillStyle = css('--muted');
  ctx.font = (11 * devicePixelRatio) + 'px system-ui';
  ctx.fillText('grid 50 m', 10 * devicePixelRatio, cv.height - 10 * devicePixelRatio);
  DATA.tracks.forEach((tr, i) => {
    const color = css(SERIES[i % SERIES.length]);
    ctx.strokeStyle = color; ctx.lineWidth = 2 * devicePixelRatio;
    ctx.lineJoin = ctx.lineCap = 'round';
    ctx.beginPath();
    let lastP = null;
    for (const p of tr) {
      if (p[0] > clock) break;
      const [x, y] = proj(p[1], p[2]);
      if (lastP === null) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      lastP = p;
    }
    ctx.stroke();
    if (lastP) {
      const [x, y] = proj(lastP[1], lastP[2]);
      ctx.beginPath(); ctx.arc(x, y, 6 * devicePixelRatio, 0, 7);
      ctx.fillStyle = color; ctx.fill();
      ctx.lineWidth = 2 * devicePixelRatio; ctx.strokeStyle = css('--surface'); ctx.stroke();
      ctx.fillStyle = css('--ink2');
      ctx.fillText(DATA.vehicles[i] + ' · ' + lastP[3].toFixed(1) + ' m',
                   x + 10 * devicePixelRatio, y - 8 * devicePixelRatio);
    }
  });
}
function tick(ts) {
  if (playing) {
    if (last !== null) clock = Math.min(dur, clock + (ts - last) / 1000 * speed);
    last = ts;
    if (clock >= dur) { playing = false; btn.textContent = 'play'; }
  } else last = ts;
  scrub.value = clock / dur * 1000;
  clockEl.textContent = fmt(clock) + ' / ' + fmt(dur);
  const seen = DATA.events.filter(e => e[0] <= clock).slice(-6);
  ticker.innerHTML = seen.map(e =>
    '<div>' + fmt(e[0]) + ' · ' + (e[1] >= 0 ? DATA.vehicles[e[1]] : 'fleet') + ' · ' + e[2] + '</div>'
  ).join('');
  draw();
  requestAnimationFrame(tick);
}
const btn = document.getElementById('play'), scrub = document.getElementById('scrub'),
      clockEl = document.getElementById('clock'), ticker = document.getElementById('ticker'),
      speedSel = document.getElementById('speed');
btn.onclick = () => { if (clock >= dur) clock = 0; playing = !playing; btn.textContent = playing ? 'pause' : 'play'; };
speedSel.onchange = () => speed = Number(speedSel.value);
scrub.oninput = () => { clock = scrub.value / 1000 * dur; };
const legend = document.getElementById('legend');
legend.innerHTML = DATA.vehicles.map((v, i) =>
  '<span><span class="key-dot" style="background:var(' + SERIES[i % SERIES.length] + ')"></span>' + v + '</span>'
).join('');
requestAnimationFrame(tick);
"#;

/// Render `demo.html` from the one dataset.
pub fn render(ds: &MissionDataset) -> RenderedArtifact {
    let label = ds.run_label();
    let all = ds.all_citations();
    let data = replay_data(ds);
    let assoc = assoc_line(
        &ds.run,
        &["link_profile.name", "coord.grace_s", "coord.floor_agl_m", "fleet_ids"],
        false,
    );
    let body = format!(
        "<main><h1>Mission replay — {label}</h1>\
         <p class=\"sub\">Animated from the same telemetry Blocks the report and deck \
         summarize — the third lens over the one dataset.</p>\
         {assoc_html}\
         <div class=\"map-wrap\"><canvas id=\"map\"></canvas>\
         <div class=\"controls\">\
           <button id=\"play\">play</button>\
           <select id=\"speed\"><option value=\"1\">1×</option>\
           <option value=\"4\" selected>4×</option><option value=\"16\">16×</option></select>\
           <input id=\"scrub\" type=\"range\" min=\"0\" max=\"1000\" value=\"0\">\
           <span class=\"clock\" id=\"clock\"></span>\
         </div>\
         <div class=\"legend\" id=\"legend\"></div>\
         <div class=\"ticker\" id=\"ticker\"></div></div>\
         {prov}\
         <p class=\"footer\">Rendered through a matched, authorized console binding \
         (artifact.demo). Provenance: hover nothing — open the drawer; every track \
         point cites its recording Block.</p></main>\
         <script>const DATA = {data};</script><script>{DEMO_JS}</script>",
        label = esc(&label),
        assoc_html = if assoc.is_empty() {
            String::new()
        } else {
            format!("<p class=\"assoc\">{assoc}</p>")
        },
        prov = prov_drawer("replay source blocks", &all),
        data = serde_json::to_string(&data).expect("replay data serializes"),
    );
    RenderedArtifact {
        html: page(&format!("Mission replay — {label}"), DEMO_STYLE, &body),
        citations: all,
    }
}
