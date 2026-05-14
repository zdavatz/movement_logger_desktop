//! Plotly-based HTML report. Same visual layout as
//! `Utilities/scripts/visualize_combined.py`: map (coloured by nose angle),
//! then nose angle / height / speed time series rows.
//!
//! The JS library is loaded from the Plotly CDN, identical to the Python
//! `to_html(include_plotlyjs='cdn')` path. 1.2 MB HTML files worked fine
//! there and continue to work here.

use crate::gps::Ride;
use crate::gps::TICKS_PER_SEC;
use crate::io::GpsRow;
use chrono::{Duration, NaiveDate, NaiveDateTime, NaiveTime};
use serde_json::json;
use std::fmt::Write as _;

pub struct PanelData<'a> {
    pub t_sensor_s: &'a [f64],
    pub nose_deg: &'a [f64],
    pub height_m: &'a [f64],
    /// Baro + accelerometer complementary-filter height, aligned to
    /// `t_sensor_s`. Makes the 1 Hz pump oscillation visible at cm scale
    /// while inheriting the baro's absolute level (and its thermal drift).
    pub fused_height_m: &'a [f64],
    pub gps: &'a [GpsRow],
    pub gps_speed_kmh: &'a [f64],
    pub gps_t_s: &'a [f64],
    /// GPS altitude, aligned 1:1 with `gps`. Plotted as a second trace
    /// in the height panel after subtracting the stationary-period
    /// median so it's directly comparable to the baro height.
    /// MAX-M10S vertical accuracy is ~5–10 m, so individual pump strokes
    /// (0.8 m) are invisible — the trace is useful mostly for spotting
    /// GPS dropouts when the board lies flat on the water and the
    /// antenna loses fix quality.
    pub gps_height_m: &'a [f64],
    pub rides: &'a [Ride],
    pub title: &'a str,
    /// Date (in UTC) anchoring the x-axis. Combined with `utc_at_t0_sec`
    /// gives the absolute UTC datetime corresponding to sensor t = 0.
    pub date_utc: NaiveDate,
    /// UTC seconds-since-midnight at sensor t = 0, back-extrapolated
    /// from the first GPS fix. Allows mapping `t_sensor_s` (seconds
    /// since first sensor sample) to absolute wall-clock time.
    pub utc_at_t0_sec: f64,
    /// Local-time offset from UTC in hours (e.g. 3 for Greek summer).
    /// Added to UTC for x-axis tick labels and hover values.
    pub tz_offset_h: f64,
}

/// Convert a seconds-since-sensor-t0 value to an ISO datetime string
/// in local time, suitable for Plotly's date axis.
fn t_to_local_iso(t: f64, base_utc: NaiveDateTime, tz_offset_h: f64) -> String {
    if !t.is_finite() {
        // Plotly tolerates an empty string as a missing-date marker.
        return String::new();
    }
    let total_ms = (t + tz_offset_h * 3600.0) * 1000.0;
    let dt = base_utc + Duration::milliseconds(total_ms.round() as i64);
    // Plotly accepts "YYYY-MM-DD HH:MM:SS.fff"; the .fff resolution
    // matters for the 100 Hz nose-angle trace where binning lands
    // points 100 ms apart.
    dt.format("%Y-%m-%d %H:%M:%S%.3f").to_string()
}

/// Produce the full HTML for one combined report.
pub fn render(data: &PanelData) -> String {
    let have_map = !data.gps.is_empty();
    // Map occupies the top 45 % of the plot when present. Hoisted to
    // the top of this function so the map-colorbar positioning
    // expressions can use it.
    let map_frac_f: f64 = if have_map { 0.45 } else { 0.0 };

    // Wall-clock anchor for the x-axis. All sensor / GPS / ride time
    // values are mapped to ISO datetime strings so Plotly renders the
    // axis as real local time (HH:MM:SS) rather than seconds-since-start.
    let base_utc = NaiveDateTime::new(data.date_utc, NaiveTime::from_hms_opt(0, 0, 0).unwrap())
        + Duration::milliseconds((data.utc_at_t0_sec * 1000.0).round() as i64);
    let to_iso = |t: f64| t_to_local_iso(t, base_utc, data.tz_offset_h);

    let t_sensor_iso: Vec<String> = data.t_sensor_s.iter().copied().map(to_iso).collect();
    let gps_t_iso: Vec<String> = data.gps_t_s.iter().copied().map(to_iso).collect();

    // --- Map trace (Scattermap, coloured by nose-angle at each fix) ---
    // To colour by nose angle at each GPS moment, we interpolate the
    // sensor-side nose series at each GPS-sample time.
    let nose_at_gps = interp(data.gps_t_s, data.t_sensor_s, data.nose_deg);
    let height_at_gps = interp(data.gps_t_s, data.t_sensor_s, data.height_m);

    let (lat_vals, lon_vals): (Vec<f64>, Vec<f64>) =
        data.gps.iter().map(|g| (g.lat, g.lon)).unzip();

    let hover: Vec<String> = (0..data.gps.len())
        .map(|i| {
            format!(
                "UTC {utc}<br>speed={sp:.1} km/h · nose={na:+.1}° · height={h:+.2} m",
                utc = data.gps[i].utc,
                sp = data.gps_speed_kmh.get(i).copied().unwrap_or(0.0),
                na = nose_at_gps.get(i).copied().unwrap_or(0.0),
                h = height_at_gps.get(i).copied().unwrap_or(0.0),
            )
        })
        .collect();

    // Centre the map on the median lat/lon, zoom to fit ride extent.
    let (centre_lat, centre_lon, zoom) = if have_map {
        let mut lat_sorted = lat_vals.clone();
        lat_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut lon_sorted = lon_vals.clone();
        lon_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let c_lat = lat_sorted[lat_sorted.len() / 2];
        let c_lon = lon_sorted[lon_sorted.len() / 2];
        let lat_span = lat_sorted.last().unwrap_or(&0.0) - lat_sorted.first().unwrap_or(&0.0);
        let lon_span = lon_sorted.last().unwrap_or(&0.0) - lon_sorted.first().unwrap_or(&0.0);
        let span = lat_span.max(lon_span).max(0.005);
        // Rough zoom from span (empirical, matches Python's auto-fit)
        let zoom = (14.0 - (span * 200.0).log2()).clamp(10.0, 18.0);
        (c_lat, c_lon, zoom)
    } else {
        (0.0, 0.0, 2.0)
    };

    // --- Traces ---
    let mut traces = Vec::<serde_json::Value>::new();

    // One map trace per ride. The "All rides" view is intentionally
    // dropped — on-shore and between-ride points dominate the map and
    // the full-track x-axis range made the time-series panels extend
    // past where any ride actually happened.
    //
    // The ride buttons (built further below) toggle visibility so only
    // one ride's trace is on the map at a time. The first ride is
    // visible by default; the rest start hidden.
    let mut map_trace_indices: Vec<usize> = Vec::new();
    if have_map {
        for (ri, r) in data.rides.iter().enumerate() {
            let end = r.gps_end.min(data.gps.len());
            if end <= r.gps_start { continue; }
            let lat_slice: Vec<f64> = data.gps[r.gps_start..end].iter().map(|g| g.lat).collect();
            let lon_slice: Vec<f64> = data.gps[r.gps_start..end].iter().map(|g| g.lon).collect();
            let speed_slice: Vec<f64> = data.gps_speed_kmh
                .get(r.gps_start..end).map(|s| s.to_vec()).unwrap_or_default();
            let hover_slice: Vec<String> = hover
                .get(r.gps_start..end).map(|s| s.to_vec()).unwrap_or_default();
            let is_first = ri == 0;
            map_trace_indices.push(traces.len());
            traces.push(json!({
                "type": "scattermap",
                "mode": "markers+lines",
                "lat": lat_slice,
                "lon": lon_slice,
                "text": hover_slice,
                "hoverinfo": "text",
                "marker": {
                    "size": 6,
                    "color": speed_slice,
                    "colorscale": "Viridis",
                    "cmin": 0.0,
                    "cmax": 25.0,
                    // Aligned with the map's vertical extent: map
                    // domain is y=[1-map_frac, 1.0], so its centre is
                    // at 1 - map_frac/2 in paper coords. Colorbar
                    // sits at that same centre with len ≈ map_frac × 0.9
                    // so the top and bottom of the bar line up with
                    // the top and bottom of the map.
                    "colorbar": {
                        "title": {"text": "km/h", "side": "top", "font": {"size": 11}},
                        "thickness": 8,
                        "x": 0.99,
                        "xanchor": "right",
                        "y": 1.0 - map_frac_f / 2.0,
                        "yanchor": "middle",
                        "len": map_frac_f * 0.9,
                        "tickfont": {"size": 10},
                        "bgcolor": "rgba(255,255,255,0.9)",
                        "bordercolor": "#888",
                        "borderwidth": 1,
                    },
                    "showscale": is_first,
                },
                "line": {"color": "rgba(80,80,80,0.45)", "width": 2},
                "subplot": "map",
                "name": format!("Ride {}", ri + 1),
                "showlegend": false,
                "visible": is_first,
            }));
        }
    }

    // Scattermap doesn't consume a y-axis number, so the first time-series
    // panel goes to y1 regardless of whether the map is present.
    traces.push(json!({
        "type": "scatter",
        "mode": "lines",
        "x": t_sensor_iso,
        "y": data.nose_deg,
        "line": {"color": "#1f77b4", "width": 1},
        "name": "Nose [°]",
        "xaxis": "x",
        "yaxis": "y",
        "hovertemplate": "%{x|%H:%M:%S}<br>nose=%{y:+.2f}°<extra></extra>",
        "showlegend": false,
    }));

    // Baro height — GPS-anchored water reference + temperature-
    // compensated. Single primary trace; earlier α-β fusion with
    // accelerometer is stashed away because the user found its
    // aggressive detrending hid the slow component that correlates
    // with the nose angle.
    traces.push(json!({
        "type": "scatter",
        "mode": "lines",
        "x": t_sensor_iso,
        "y": data.height_m,
        "line": {"color": "#2ca02c", "width": 1.5},
        "name": "Baro [m]",
        "xaxis": "x",
        "yaxis": "y2",
        "hovertemplate": "%{x|%H:%M:%S}<br>baro=%{y:.2f} m<extra></extra>",
        "showlegend": false,
    }));

    // Optional fused trace (blue) — drawn only when fused_height_m is
    // non-empty. Call sites that aren't using fusion leave it empty.
    if !data.fused_height_m.is_empty() {
        traces.push(json!({
            "type": "scatter",
            "mode": "lines",
            "x": t_sensor_iso,
            "y": data.fused_height_m,
            "line": {"color": "#1f77b4", "width": 1.0},
            "name": "Fused [m]",
            "xaxis": "x",
            "yaxis": "y2",
            "hovertemplate": "%{x|%H:%M:%S}<br>fused=%{y:.2f} m<extra></extra>",
            "showlegend": false,
        }));
    }

    // Mast reference line at 0.80 m
    if !t_sensor_iso.is_empty() {
        let x_ends = vec![t_sensor_iso.first().cloned().unwrap_or_default(),
                          t_sensor_iso.last().cloned().unwrap_or_default()];
        traces.push(json!({
            "type": "scatter",
            "mode": "lines",
            "x": x_ends,
            "y": [0.80, 0.80],
            "line": {"color": "rgba(200,0,0,0.5)", "width": 1, "dash": "dot"},
            "hoverinfo": "skip",
            "xaxis": "x",
            "yaxis": "y2",
            "name": "0.80 m mast",
            "showlegend": false,
        }));
    }

    // GPS altitude overlay in the height panel (when available). Drawn
    // as orange dots so the reader can distinguish the 1–10 Hz GPS
    // samples from the dense baro trace. Dropouts (= missing points /
    // sparse stretches) are visible at a glance.
    if !data.gps_height_m.is_empty() && have_map {
        traces.push(json!({
            "type": "scatter",
            "mode": "markers",
            "x": gps_t_iso,
            "y": data.gps_height_m,
            "marker": {"color": "#ff7f0e", "size": 4, "opacity": 0.8},
            "name": "GPS alt",
            "xaxis": "x",
            "yaxis": "y2",
            "hovertemplate": "%{x|%H:%M:%S}<br>GPS alt=%{y:+.2f} m<extra></extra>",
            "showlegend": false,
        }));
    }

    // Speed time-series (only when GPS is present → third panel)
    if have_map {
        traces.push(json!({
            "type": "scatter",
            "mode": "lines",
            "x": gps_t_iso,
            "y": data.gps_speed_kmh,
            "line": {"color": "#d62728", "width": 1},
            "name": "Speed [km/h]",
            "xaxis": "x",
            "yaxis": "y3",
            "hovertemplate": "%{x|%H:%M:%S}<br>speed=%{y:.1f} km/h<extra></extra>",
            "showlegend": false,
        }));
    }

    // --- Layout ---
    // Rows: map (if have_map) + nose + height + speed (if have_map).
    // All time-series rows get equal height. Each has a small title
    // strip above it for the horizontal panel caption.
    let n_ts_rows: usize = if have_map { 3 } else { 2 };
    let map_frac = map_frac_f;
    let title_h = 0.03;
    let ts_total = 1.0 - map_frac;
    let row_heights: Vec<f64> = (0..n_ts_rows)
        .map(|_| ts_total / n_ts_rows as f64 - title_h)
        .collect();

    // Per-ride x-axis ranges (relative to sensor time) for the zoom buttons.
    // Rides carry GPS row indices, which we translate into the sensor-time
    // base that the time-series x-axis uses.
    let sensor_base = data.t_sensor_s.first().copied().unwrap_or(0.0);
    let gps_base = data.gps.first().map(|g| g.ticks).unwrap_or(0.0);
    let ride_windows_s: Vec<(f64, f64)> = data.rides.iter().map(|r| {
        let start_ticks = data.gps.get(r.gps_start).map(|g| g.ticks).unwrap_or(0.0);
        let end_idx = r.gps_end.saturating_sub(1).min(data.gps.len().saturating_sub(1));
        let end_ticks = data.gps.get(end_idx).map(|g| g.ticks).unwrap_or(0.0);
        ((start_ticks - gps_base) / TICKS_PER_SEC + sensor_base,
         (end_ticks - gps_base) / TICKS_PER_SEC + sensor_base)
    }).collect();
    // ISO datetime equivalents for the ride buttons' xaxis.range.
    let ride_windows: Vec<(String, String)> = ride_windows_s.iter()
        .map(|&(a, b)| (to_iso(a), to_iso(b)))
        .collect();

    let tz_label = if data.tz_offset_h == 0.0 {
        "UTC".to_string()
    } else if data.tz_offset_h.fract() == 0.0 {
        format!("UTC{:+}", data.tz_offset_h as i64)
    } else {
        format!("UTC{:+}", data.tz_offset_h)
    };

    // Build one button per ride — each update does three things via the
    // "update" method: (1) toggle trace visibility so only this ride's
    // map trace is on, (2) relayout x-axis to the ride's time window,
    // (3) recentre the map on the ride's extent. There's deliberately
    // no "All rides" button: showing every ride's track on the map at
    // once was busy and the full-session x-axis made time-series panels
    // extend past where anything interesting happens.
    // All traces (map-per-ride + nose + height + mast + speed) are already
    // pushed at this point, so `traces.len()` is the final count.
    let n_traces = traces.len();
    let mut buttons: Vec<serde_json::Value> = Vec::new();
    for (i, win) in ride_windows.iter().enumerate() {
        // Per-ride visibility array: only the matching map trace is on,
        // all non-map traces (nose / height / mast / speed) stay visible.
        let mut visible = vec![serde_json::Value::Bool(true); n_traces];
        for (mi, &trace_idx) in map_trace_indices.iter().enumerate() {
            visible[trace_idx] = serde_json::Value::Bool(mi == i);
        }
        // Also drive the colourbar onto the selected trace only.
        // Per-ride map centre
        let r = &data.rides[i];
        let end = r.gps_end.min(data.gps.len());
        let (lat_slice, lon_slice): (Vec<f64>, Vec<f64>) = data.gps[r.gps_start..end]
            .iter().map(|g| (g.lat, g.lon)).unzip();
        let (c_lat, c_lon, z) = if lat_slice.is_empty() {
            (centre_lat, centre_lon, zoom)
        } else {
            let mut ls = lat_slice.clone(); ls.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mut os = lon_slice.clone(); os.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let cl = ls[ls.len() / 2];
            let co = os[os.len() / 2];
            let span = (ls.last().unwrap() - ls.first().unwrap())
                .max(os.last().unwrap() - os.first().unwrap()).max(0.001);
            let z = (14.0 - (span * 200.0).log2()).clamp(11.0, 18.0);
            (cl, co, z)
        };
        buttons.push(json!({
            "label": format!("Ride {}", i + 1),
            "method": "update",
            "args": [
                {"visible": visible},
                {
                    "xaxis.range": [&win.0, &win.1],
                    "map.center": {"lat": c_lat, "lon": c_lon},
                    "map.zoom": z,
                }
            ]
        }));
    }

    // Initial view = first ride, matching the map trace that starts
    // visible. Without this the x-axis would autoscale to the full
    // session and the time-series panels would be zoomed out.
    let (init_x_lo, init_x_hi) = ride_windows.first().cloned()
        .unwrap_or_else(|| {
            (t_sensor_iso.first().cloned().unwrap_or_default(),
             t_sensor_iso.last().cloned().unwrap_or_default())
        });
    let (init_c_lat, init_c_lon, init_zoom) = if let Some(r) = data.rides.first() {
        let end = r.gps_end.min(data.gps.len());
        let (mut ls, mut os): (Vec<f64>, Vec<f64>) = data.gps[r.gps_start..end]
            .iter().map(|g| (g.lat, g.lon)).unzip();
        if ls.is_empty() {
            (centre_lat, centre_lon, zoom)
        } else {
            ls.sort_by(|a, b| a.partial_cmp(b).unwrap());
            os.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let span = (ls.last().unwrap() - ls.first().unwrap())
                .max(os.last().unwrap() - os.first().unwrap()).max(0.001);
            let z = (14.0 - (span * 200.0).log2()).clamp(11.0, 18.0);
            (ls[ls.len() / 2], os[os.len() / 2], z)
        }
    } else {
        (centre_lat, centre_lon, zoom)
    };

    let mut layout = json!({
        "title": {"text": data.title, "x": 0.5, "xanchor": "center"},
        "height": if have_map { 1000 } else { 600 },
        "margin": {"t": 90, "r": 40, "b": 40, "l": 90},
        // "x unified" groups hover tooltips across all traces at the same
        // x — one mouse position shows nose/height/speed values together.
        // "closest" remains for the map (lat/lon space).
        "hovermode": "x unified",
        "showlegend": false,
        "updatemenus": [{
            "type": "buttons",
            "direction": "right",
            "buttons": buttons,
            "x": 0.0, "xanchor": "left",
            "y": 1.04, "yanchor": "bottom",
            "pad": {"r": 8, "t": 4},
            "showactive": true,
            "bgcolor": "rgba(255,255,255,0.9)",
            "active": 0,
        }]
    });

    // Build subplot domain/axis definitions
    let layout_obj = layout.as_object_mut().unwrap();

    if have_map {
        layout_obj.insert("map".into(), json!({
            "center": {"lat": init_c_lat, "lon": init_c_lon},
            "zoom": init_zoom,
            "style": "carto-positron",
            "domain": {"x": [0.0, 1.0], "y": [1.0 - map_frac, 1.0]},
        }));
    }

    // Axis stacks: y1..yN from top to bottom under the map.
    // Panel titles go above each panel as horizontal annotations (see
    // the annotation loop below) rather than rotated y-axis titles —
    // the panels are ~180 px tall and the long "Board height above
    // water · baro temperature-drift limited" caption can't fit rotated.
    let panel_titles: Vec<&str> = if have_map {
        vec![
            "Board nose angle to water [°]",
            "Board height above water [m] — green = baro (TC + GPS-anchored), orange = GPS altitude · mast = 0.80 m, but baro thermal drift can push values beyond that over a ride",
            "Speed [km/h] (position-derived, 5 s median)",
        ]
    } else {
        vec![
            "Board nose angle to water [°]",
            "Board height above water [m]",
        ]
    };

    let mut annotations: Vec<serde_json::Value> = Vec::new();

    // Walk down from the top of the time-series area (just under the
    // map), stacking each row's title strip + panel.
    let mut cursor = 1.0 - map_frac;
    for r in 0..n_ts_rows {
        let slot_top = cursor;
        let panel_top = slot_top - title_h;
        let panel_bot = panel_top - row_heights[r];
        let title_y = panel_top + title_h * 0.5;
        cursor = panel_bot; // next slot starts here

        let idx = r + 1; // y1, y2, y3
        let axis_name = if idx == 1 { "yaxis".to_string() } else { format!("yaxis{}", idx) };
        let unit_label = match (have_map, r) {
            (true, 0) => "°",
            (true, 1) => "m",
            (true, 2) => "km/h",
            (false, 0) => "°",
            (false, 1) => "m",
            _ => "",
        };
        let mut ax = json!({
            "domain": [panel_bot.max(0.0), panel_top.min(1.0)],
            "anchor": "x",
            "title": {"text": unit_label, "standoff": 4},
            "automargin": true,
        });
        // Height range matches the physical mast: 80 cm. Show a
        // little headroom and a small negative margin for noise
        // (the board can't really go below water, but baro noise
        // near 0 would otherwise clip the signal). Labels every
        // 10 cm. Speed 0–30 km/h.
        let height_panel = (have_map && r == 1) || (!have_map && r == 1);
        if height_panel {
            ax["range"] = json!([-0.1, 0.9]);
            ax["dtick"] = json!(0.10);
            ax["tickformat"] = json!(".1f");
        }
        if r == n_ts_rows - 1 && have_map {
            ax["range"] = json!([0, 30]);
        }
        layout_obj.insert(axis_name, ax);

        // Horizontal panel caption centred in the title strip.
        let title = panel_titles.get(r).copied().unwrap_or("");
        annotations.push(json!({
            "text": title,
            "xref": "paper",
            "yref": "paper",
            "x": 0.5,
            "y": title_y,
            "xanchor": "center",
            "yanchor": "middle",
            "showarrow": false,
            "font": {"size": 13, "color": "#333"},
        }));
    }
    // Shared x-axis across all TS rows, label only at the bottom.
    // Initial range = first ride's window, matching the default visible
    // map trace; user switches via the ride buttons.
    //
    // Spike lines draw a vertical rule through every subplot at the
    // cursor x — combined with hovermode "x unified" this gives
    // cross-panel correlation at a glance.
    layout_obj.insert("xaxis".into(), json!({
        "domain": [0.0, 1.0],
        "anchor": format!("y{}", n_ts_rows),
        "type": "date",
        "title": {"text": format!("Local time ({})", tz_label)},
        "range": [&init_x_lo, &init_x_hi],
        "tickformat": "%H:%M:%S",
        "showspikes": true,
        "spikemode": "across",
        "spikesnap": "cursor",
        "spikethickness": 1,
        "spikecolor": "rgba(120,120,120,0.6)",
        "spikedash": "solid",
    }));

    // Attach the panel annotations to the layout.
    layout_obj.insert("annotations".into(), serde_json::Value::Array(annotations));

    // --- Ride summary block (plain HTML TOC above the chart) ---
    let mut ride_toc = String::from(
        "<div style=\"font-family: sans-serif; font-size: 13px; margin: 10px 20px;\">"
    );
    write!(ride_toc,
        "<b>{n} ride{s} detected over water</b>",
        n = data.rides.len(),
        s = if data.rides.len() == 1 { "" } else { "s" }).unwrap();
    ride_toc.push_str("<ul style=\"margin: 4px 0;\">");
    for (i, r) in data.rides.iter().enumerate() {
        let dur_min = (r.duration_s / 60.0) as u32;
        let dur_sec = ((r.duration_s - dur_min as f64 * 60.0) as u32).min(59);
        // Format the start time in the same local zone as the x-axis.
        let local_start = ride_windows_s.get(i)
            .map(|&(t, _)| {
                let total_ms = (t + data.tz_offset_h * 3600.0) * 1000.0;
                let dt = base_utc + Duration::milliseconds(total_ms.round() as i64);
                dt.format("%H:%M:%S").to_string()
            })
            .unwrap_or_else(|| r.utc_start.clone());
        write!(ride_toc,
            "<li>Session {}: {tz} {time}, duration {m:02}:{s:02}, p90 {p:.1} km/h</li>",
            i + 1, tz = tz_label, time = local_start,
            m = dur_min, s = dur_sec, p = r.p90_kmh).unwrap();
    }
    ride_toc.push_str("</ul></div>");

    let traces_json = serde_json::to_string(&traces).unwrap();
    let layout_json = serde_json::to_string(&layout).unwrap();

    // Paper-y extent of the combined time-series region — click markers
    // span from the x-axis at the bottom up to just below the map.
    let ts_top_paper = 1.0 - if have_map { 0.45 } else { 0.0 };

    format!(
        r#"<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{title}</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
</head>
<body style="margin: 0; background: #fafafa;">
  {toc}
  <div style="margin: 4px 20px;">
    <button id="clearMarks" style="
        font: 12px sans-serif;
        padding: 4px 10px;
        border: 1px solid #888;
        background: #fff;
        border-radius: 3px;
        cursor: pointer;">Clear click-marks</button>
    <span style="font-size: 12px; color: #666; margin-left: 10px;">
      Tip: hover = vertical line + unified tooltip across all panels.
      Click a time-series point to pin a numbered marker.
    </span>
  </div>
  <div id="plot" style="width: 100%; height: 1040px;"></div>
  <script>
    const data = {traces};
    const layout = {layout};
    Plotly.newPlot('plot', data, layout, {{responsive: true, scrollZoom: false}});

    // Cross-panel click markers: clicking a point on any time-series trace
    // pins a vertical dashed red line through every time-series panel at
    // that x-coordinate, with a numbered label at the top. Map clicks are
    // ignored because scattermap uses lat/lon, not the time axis.
    const plotEl = document.getElementById('plot');
    const tsTop = {ts_top_paper};
    const baseAnnotCount = (plotEl.layout.annotations || []).length;
    let clickCount = 0;

    plotEl.on('plotly_click', function(ev) {{
      const pt = ev.points && ev.points[0];
      if (!pt) return;
      if (pt.data && pt.data.type === 'scattermap') return;

      clickCount++;
      const x = pt.x;

      const shapes = (plotEl.layout.shapes || []).slice();
      shapes.push({{
        type: 'line',
        xref: 'x', yref: 'paper',
        x0: x, x1: x,
        y0: 0.0, y1: tsTop - 0.005,
        line: {{color: '#e22', width: 1.5, dash: 'dash'}},
        name: 'click-mark',
      }});

      const annotations = (plotEl.layout.annotations || []).slice();
      annotations.push({{
        x: x, xref: 'x',
        y: tsTop - 0.015, yref: 'paper',
        text: String(clickCount),
        showarrow: false,
        font: {{size: 13, color: '#e22'}},
        bgcolor: 'rgba(255,255,255,0.95)',
        bordercolor: '#e22',
        borderwidth: 1,
        borderpad: 3,
        name: 'click-mark',
      }});

      Plotly.relayout(plotEl, {{shapes: shapes, annotations: annotations}});
    }});

    document.getElementById('clearMarks').addEventListener('click', function() {{
      clickCount = 0;
      const annotations = (plotEl.layout.annotations || [])
        .slice(0, baseAnnotCount);  // drop the click-marks we appended
      Plotly.relayout(plotEl, {{shapes: [], annotations: annotations}});
    }});
  </script>
</body>
</html>
"#,
        title = data.title,
        toc = ride_toc,
        traces = traces_json,
        layout = layout_json,
        ts_top_paper = ts_top_paper,
    )
}

fn interp(x_new: &[f64], x: &[f64], y: &[f64]) -> Vec<f64> {
    crate::baro::interp_linear(x_new, x, y)
}
