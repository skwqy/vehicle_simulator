//! Map-backed route chain (segment ids → polylines). Shared by motion and downlink handlers.

use crate::map::MapModel;
use crate::navigation::polyline_length_m;

pub(super) fn point_keys_match(map: &MapModel, a: &str, b: &str) -> bool {
    if a == b {
        return true;
    }
    match (map.resolve_point_key(a), map.resolve_point_key(b)) {
        (Some(ka), Some(kb)) => ka == kb,
        _ => false,
    }
}

pub(super) struct RouteSeg {
    pub edge_id: i32,
    pub poly: Vec<(f32, f32)>,
    pub len_m: f32,
    /// OpenTCS point name at the end of this segment (travel direction; matches `path.dest` or reversed `path.source`).
    pub end_open_tcs_point: String,
}

pub(super) struct ActiveRoute {
    pub segments: Vec<RouteSeg>,
    pub edge_idx: usize,
    pub s_m: f32,
    /// OpenTCS point name at the end of this chain (set when the route is built).
    pub end_open_tcs_point: String,
}

impl ActiveRoute {
    pub(super) fn current_edge_id(&self) -> i32 {
        self.segments
            .get(self.edge_idx)
            .map(|s| s.edge_id)
            .unwrap_or(0)
    }

    pub(super) fn seg_distance_mm(&self) -> i32 {
        let s = self.s_m.max(0.0);
        let mm = (s * 1000.0f32).round();
        mm.clamp(0.0, i32::MAX as f32) as i32
    }
}

pub(super) fn build_route_segments(
    map: &MapModel,
    start_point: &str,
    segment_ids: &[i32],
) -> Option<(Vec<RouteSeg>, String)> {
    let mut out = Vec::new();
    let mut expected = start_point.to_string();
    for &sid in segment_ids {
        let path = map.path_by_edge_id(sid)?;
        let rev = if point_keys_match(map, &expected, &path.source) {
            false
        } else if point_keys_match(map, &expected, &path.dest) {
            true
        } else {
            return None;
        };
        let mut pts: Vec<(f32, f32)> = path
            .polyline_world_m
            .iter()
            .map(|&(a, b)| (a as f32, b as f32))
            .collect();
        if pts.len() < 2 {
            return None;
        }
        if rev {
            pts.reverse();
        }
        let len_m = polyline_length_m(&pts);
        let end_open_tcs_point = if rev {
            path.source.clone()
        } else {
            path.dest.clone()
        };
        out.push(RouteSeg {
            edge_id: sid,
            poly: pts,
            len_m,
            end_open_tcs_point,
        });
        expected = if rev {
            path.source.clone()
        } else {
            path.dest.clone()
        };
    }
    Some((out, expected))
}

pub(super) fn reported_point_int(map: &MapModel, open_tcs_point_name: &str) -> i32 {
    let trimmed = open_tcs_point_name.trim();
    if trimmed.is_empty() {
        return 0;
    }
    let v = if map.name_prefixes.apply_stripping {
        let p = &map.name_prefixes.point_prefix;
        if !p.is_empty() && trimmed.starts_with(p) {
            trimmed[p.len()..].to_string()
        } else {
            trimmed.to_string()
        }
    } else {
        trimmed.to_string()
    };
    if let Ok(n) = v.trim().parse::<i32>() {
        return n;
    }
    trailing_digits_i32(&v)
}

fn trailing_digits_i32(s: &str) -> i32 {
    let t = s.trim();
    let mut start = t.len();
    let end = t.len();
    for (i, c) in t.char_indices().rev() {
        if c.is_ascii_digit() {
            start = i;
        } else {
            break;
        }
    }
    if start < end {
        t[start..end].parse().unwrap_or(0)
    } else {
        0
    }
}
