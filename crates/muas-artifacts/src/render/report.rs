//! The operator/engineer lens: association first (which settings shaped
//! which outcomes), detail second, Block-hash provenance behind drawers and
//! row tooltips — never inline hash spam.

use crate::dataset::{DatumKind, MissionDataset};
use crate::metrics::{metrics, Pctl};
use crate::render::{assoc_line, config_panel, esc, page, prov_drawer, t_rel, RenderedArtifact};

fn fmt_pctl(p: &Pctl) -> String {
    format!("{:.1} / {:.1} ms (n={})", p.p50, p.p95, p.n)
}

/// Render `report.html` from the one dataset.
pub fn render(ds: &MissionDataset) -> RenderedArtifact {
    let m = metrics(ds);
    let t0 = ds.t0();
    let label = ds.run_label();

    // ── header ──────────────────────────────────────────────────────────
    let mut body = String::new();
    body.push_str("<main>");
    body.push_str(&format!(
        "<h1>Mission report — {}</h1>\
         <p class=\"sub\">{} vehicle(s) · {:.0} s window · {} data · {} blocks across {} chains</p>",
        esc(&label),
        ds.vehicles().len(),
        m.duration_s,
        ds.data.len(),
        ds.blocks.len(),
        ds.blocks.iter().map(|b| &b.chain).collect::<std::collections::BTreeSet<_>>().len(),
    ));

    // ── the association layer leads ─────────────────────────────────────
    body.push_str(&config_panel(&ds.run));

    // ── coordination ────────────────────────────────────────────────────
    let coop_rate = m
        .coop_rate_pct()
        .map(|r| format!("{r:.0}%"))
        .unwrap_or_else(|| "—".to_string());
    let mut coord_rows = String::new();
    for d in ds.of_kind(DatumKind::Coord) {
        let c = ds.citation_of(d);
        coord_rows.push_str(&format!(
            "<tr title=\"block {h} · {ch} seq {q}\"><td>{t}</td><td>{v}</td><td>{k}</td>\
             <td>{peer}</td><td>{bias}</td></tr>",
            h = esc(&c.hash),
            ch = esc(&c.chain),
            q = c.seq,
            t = t_rel(d.t_ns, t0),
            v = esc(d.vehicle.as_deref().unwrap_or("—")),
            k = esc(&d.label),
            peer = esc(d.body["peer"].as_str().unwrap_or("—")),
            bias = d.body["bias_m"].as_f64().map(|b| format!("{b:+.1} m")).unwrap_or_else(|| "—".into()),
        ));
    }
    body.push_str(&format!(
        "<section class=\"card\"><h2>Coordination</h2>\
         <p class=\"assoc\">{assoc}</p>\
         <p>Cooperative avoidance succeeded in <strong>{coop}</strong> of \
         <strong>{total}</strong> episodes ({rate}); max vertical bias \
         {bias}. Smart RTL: {rtl}.</p>\
         <div class=\"overflow\"><table><thead><tr><th>t</th><th>vehicle</th><th>event</th>\
         <th>peer</th><th>bias</th></tr></thead><tbody>{coord_rows}</tbody></table></div>{prov}\
         </section>",
        assoc = assoc_line(
            &ds.run,
            &["coord.grace_s", "coord.hsep_m", "coord.vsep_m", "coord.floor_agl_m",
              "link_profile.loss_pct", "link_profile.delay_ms"],
            true
        ),
        coop = m.coop,
        total = m.coop + m.unco,
        rate = coop_rate,
        bias = m.max_bias_m.map(|b| format!("{b:.1} m")).unwrap_or_else(|| "—".into()),
        rtl = m.rtl_outcome.as_deref().map(esc).unwrap_or_else(|| "not engaged".into()),
        prov = prov_drawer("coordination events", &ds.citations_where(|d| d.kind == DatumKind::Coord)),
    ));

    // ── mission timeline (services, flight results, RTL, events) ───────
    let mut rows = String::new();
    for d in &ds.data {
        if !matches!(d.kind, DatumKind::Service | DatumKind::Rtl | DatumKind::Event) {
            continue;
        }
        if d.label == "run.config" {
            continue; // rendered as the config panel above
        }
        let detail = if d.label.starts_with("service.") {
            let ok = match d.body["accepted"].as_bool() {
                Some(true) => "accepted",
                Some(false) => "rejected",
                None => "",
            };
            let rtt = d.body["rtt_ms"].as_f64().map(|r| format!(" · {r:.0} ms")).unwrap_or_default();
            format!("{ok}{rtt}")
        } else if d.label == "flight.takeoff.result" {
            if d.body["airborne"].as_bool() == Some(true) { "airborne".into() } else { "failed".into() }
        } else if d.label == "rtl.done" {
            d.body["outcome"].as_str().unwrap_or("").to_string()
        } else {
            String::new()
        };
        let c = ds.citation_of(d);
        rows.push_str(&format!(
            "<tr title=\"block {h} · {ch} seq {q}\"><td>{t}</td><td>{v}</td><td>{k}</td><td>{detail}</td></tr>",
            h = esc(&c.hash),
            ch = esc(&c.chain),
            q = c.seq,
            t = t_rel(d.t_ns, t0),
            v = esc(d.vehicle.as_deref().unwrap_or("—")),
            k = esc(&d.label),
            detail = esc(&detail),
        ));
    }
    body.push_str(&format!(
        "<section class=\"card\"><h2>Mission timeline</h2>\
         <p class=\"note\">Services journaled: {ops} ({rej} rejected) · takeoffs {tok}/{tatt}. \
         Hover a row for its source Block.</p>\
         <div class=\"overflow\"><table><thead><tr><th>t</th><th>vehicle</th><th>event</th><th>detail</th></tr>\
         </thead><tbody>{rows}</tbody></table></div>{prov}</section>",
        ops = m.service_ops,
        rej = m.service_rejected,
        tok = m.takeoff_ok,
        tatt = m.takeoff_attempts,
        prov = prov_drawer(
            "timeline events",
            &ds.citations_where(|d| matches!(d.kind, DatumKind::Service | DatumKind::Rtl | DatumKind::Event)),
        ),
    ));

    // ── per-vehicle flight summary ──────────────────────────────────────
    let mut vrows = String::new();
    for (vid, v) in &m.vehicles {
        vrows.push_str(&format!(
            "<tr><td>{vid}</td><td>{n}</td><td>{agl:.1} m</td><td>{spd:.1} m/s</td>\
             <td>{bat}</td><td>{modes}</td></tr>",
            vid = esc(vid),
            n = v.samples,
            agl = v.max_agl_m,
            spd = v.max_speed_m_s,
            bat = v
                .battery_pct
                .map(|(a, b)| format!("{a:.0}% → {b:.0}%"))
                .unwrap_or_else(|| "—".into()),
            modes = esc(&v.modes.join(", ")),
        ));
    }
    body.push_str(&format!(
        "<section class=\"card\"><h2>Flight summary</h2>\
         <p class=\"assoc\">{assoc}</p>\
         <div class=\"overflow\"><table><thead><tr><th>vehicle</th><th>samples</th><th>max AGL</th>\
         <th>max speed</th><th>battery</th><th>modes</th></tr></thead><tbody>{vrows}</tbody></table></div>\
         {prov}</section>",
        assoc = assoc_line(&ds.run, &["coord.floor_agl_m", "telemetry_hz", "fleet_ids"], true),
        prov = prov_drawer("telemetry samples", &ds.citations_where(|d| d.kind == DatumKind::Telemetry)),
    ));

    // ── network / link ──────────────────────────────────────────────────
    let mut lrows = String::new();
    for (vid, v) in &m.vehicles {
        lrows.push_str(&format!(
            "<tr><td>{vid}</td><td>{ia}</td></tr>",
            vid = esc(vid),
            ia = v.interarrival_ms.as_ref().map(fmt_pctl).unwrap_or_else(|| "—".into()),
        ));
    }
    let mut link_lines = Vec::new();
    if let Some(p) = &m.service_rtt_ms {
        link_lines.push(format!("Service RTT p50/p95: <strong>{}</strong>.", fmt_pctl(p)));
    }
    if m.stale_marks > 0 {
        link_lines.push(format!(
            "The dashboard flagged <strong>{}</strong> stale-telemetry mark(s).",
            m.stale_marks
        ));
    }
    if let Some(loss) = m.spark_loss_pct {
        link_lines.push(format!("Spark telemetry-lane frame loss: <strong>{loss:.1}%</strong>."));
    }
    if let Some(kbps) = m.video_kbps_mean {
        link_lines.push(format!("Mean video bitrate: <strong>{kbps:.0} kbps</strong>."));
    }
    if link_lines.is_empty() {
        link_lines.push("No service-RTT, spark, or video data in this run.".into());
    }
    body.push_str(&format!(
        "<section class=\"card\"><h2>Network &amp; link</h2>\
         <p class=\"assoc\">{assoc}</p>\
         <p>{lines}</p>\
         <div class=\"overflow\"><table><thead><tr><th>vehicle</th><th>telemetry inter-arrival p50/p95</th></tr>\
         </thead><tbody>{lrows}</tbody></table></div>{prov}</section>",
        assoc = assoc_line(
            &ds.run,
            &["link_profile.name", "link_profile.loss_pct", "link_profile.delay_ms",
              "link_profile.jitter_ms", "link_profile.bandwidth_mbps", "carrier"],
            true
        ),
        lines = link_lines.join(" "),
        prov = prov_drawer(
            "telemetry + link data",
            &ds.citations_where(|d| matches!(d.kind, DatumKind::Telemetry | DatumKind::Link)),
        ),
    ));

    // ── audit appendix ─────────────────────────────────────────────────
    let all = ds.all_citations();
    body.push_str(&format!(
        "<section class=\"card\"><h2>Provenance appendix</h2>\
         <p class=\"note\">Every number above was derived from the {n} signed, \
         content-addressed Blocks below — the same Blocks the deck, the demo, and \
         the comparison cite. Verify with <code>muas-artifacts --audit</code>.</p>{prov}\
         </section>",
        n = all.len(),
        prov = prov_drawer("entire artifact", &all),
    ));
    body.push_str(
        "<p class=\"footer\">Rendered through a matched, authorized console binding \
         (artifact.report) over the one mission dataset — a lens, not a copy.</p></main>",
    );

    RenderedArtifact { html: page(&format!("Mission report — {label}"), "", &body), citations: all }
}
