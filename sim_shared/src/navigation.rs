//! Piecewise-linear path in meters: arc-length parameterization.

/// Total length of a polyline (m).
pub fn polyline_length_m(points: &[(f32, f32)]) -> f32 {
    if points.len() < 2 {
        return 0.0;
    }
    let mut sum = 0.0f32;
    for w in points.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        sum += ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
    }
    sum
}

/// Position and tangent angle at distance `s` from start along the polyline (clamped).
pub fn position_at_s(points: &[(f32, f32)], s: f32) -> (f32, f32, f32) {
    if points.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    if points.len() == 1 {
        return (points[0].0, points[0].1, 0.0);
    }
    let mut remaining = s.max(0.0);
    for w in points.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        let seg = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
        if seg < 1e-8 {
            continue;
        }
        if remaining <= seg {
            let t = remaining / seg;
            let x = x0 + t * (x1 - x0);
            let y = y0 + t * (y1 - y0);
            let theta = f32::atan2(y1 - y0, x1 - x0);
            return (x, y, theta);
        }
        remaining -= seg;
    }
    let last = *points.last().unwrap();
    let prev = points[points.len() - 2];
    let theta = f32::atan2(last.1 - prev.1, last.0 - prev.0);
    (last.0, last.1, theta)
}

/// Arc length along the polyline to the closest point to `(px, py)` (for entering a new edge).
pub fn closest_s_on_polyline(px: f32, py: f32, poly: &[(f32, f32)]) -> f32 {
    if poly.len() < 2 {
        return 0.0;
    }
    let mut best_s = 0.0f32;
    let mut min_d2 = f32::MAX;
    let mut acc = 0.0f32;
    for w in poly.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        let dx = x1 - x0;
        let dy = y1 - y0;
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-12 {
            0.0
        } else {
            ((px - x0) * dx + (py - y0) * dy) / len2
        }
        .clamp(0.0, 1.0);
        let qx = x0 + t * dx;
        let qy = y0 + t * dy;
        let d2 = (px - qx).powi(2) + (py - qy).powi(2);
        if d2 < min_d2 {
            min_d2 = d2;
            best_s = acc + t * len2.sqrt();
        }
        acc += len2.sqrt();
    }
    best_s
}
