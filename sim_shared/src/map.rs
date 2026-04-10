use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use quick_xml::events::Event;
use quick_xml::Reader;
use thiserror::Error;

use crate::common::MapNamePrefixes;

#[derive(Debug, Error)]
pub enum MapError {
    #[error("XML: {0}")]
    Xml(String),
    #[error("IO: {0}")]
    Io(String),
}

#[derive(Debug, Clone)]
pub struct MapPoint {
    pub x_m: f64,
    pub y_m: f64,
}

#[derive(Debug, Clone)]
pub struct MapPath {
    pub name: String,
    pub source: String,
    pub dest: String,
    pub length_mm: f64,
    pub max_velocity_mm_s: f64,
    pub polyline_world_m: Vec<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct MapModel {
    pub points: HashMap<String, MapPoint>,
    pub paths: HashMap<String, MapPath>,
    pub name_prefixes: MapNamePrefixes,
}

impl Default for MapModel {
    fn default() -> Self {
        Self {
            points: HashMap::new(),
            paths: HashMap::new(),
            name_prefixes: MapNamePrefixes::default(),
        }
    }
}

impl MapModel {
    pub fn resolve_point_key(&self, name: &str) -> Option<String> {
        if self.points.contains_key(name) {
            return Some(name.to_string());
        }
        if !self.name_prefixes.apply_stripping {
            return None;
        }
        let id = strip_prefix_and_parse_int(name, &self.name_prefixes.point_prefix);
        if id <= 0 {
            return None;
        }
        let canonical = format!("{}{}", self.name_prefixes.point_prefix, id);
        if self.points.contains_key(&canonical) {
            Some(canonical)
        } else {
            None
        }
    }

    fn resolve_path_key(&self, name: &str) -> Option<String> {
        if self.paths.contains_key(name) {
            return Some(name.to_string());
        }
        if !self.name_prefixes.apply_stripping {
            return None;
        }
        let id = strip_prefix_and_parse_int(name, &self.name_prefixes.path_prefix);
        if id <= 0 {
            return None;
        }
        let canonical = format!("{}{}", self.name_prefixes.path_prefix, id);
        if self.paths.contains_key(&canonical) {
            Some(canonical)
        } else {
            None
        }
    }

    pub fn point_world(&self, name: &str) -> Option<(f64, f64)> {
        let key = self.resolve_point_key(name)?;
        self.points.get(&key).map(|p| (p.x_m, p.y_m))
    }

    pub fn path_for_edge(&self, edge_id: &str, start_node_id: &str, end_node_id: &str) -> Option<&MapPath> {
        if let Some(k) = self.resolve_path_key(edge_id) {
            if let Some(p) = self.paths.get(&k) {
                return Some(p);
            }
        }
        let start = self.resolve_point_key(start_node_id)?;
        let end = self.resolve_point_key(end_node_id)?;
        self.paths.values().find(|p| p.source == start && p.dest == end)
    }

    pub fn path_by_edge_id(&self, edge_id: i32) -> Option<&MapPath> {
        let key = edge_id.to_string();
        let k = self.resolve_path_key(&key)?;
        self.paths.get(&k)
    }
}

pub fn validate_map_name_prefixes(p: &MapNamePrefixes) -> Result<(), MapError> {
    if !p.apply_stripping {
        return Ok(());
    }
    if !p.point_prefix.ends_with('_') {
        return Err(MapError::Io(format!(
            "map.name_prefixes.point_prefix must end with '_', got: {}",
            p.point_prefix
        )));
    }
    if !p.path_prefix.ends_with('_') {
        return Err(MapError::Io(format!(
            "map.name_prefixes.path_prefix must end with '_', got: {}",
            p.path_prefix
        )));
    }
    Ok(())
}

pub fn load_map(
    xml_path: &str,
    layout_scale_mm: f64,
    layout_flip_y: bool,
    name_prefixes: &MapNamePrefixes,
) -> Result<MapModel, MapError> {
    validate_map_name_prefixes(name_prefixes)?;
    let text = fs::read_to_string(xml_path).map_err(|e| MapError::Io(format!("{xml_path}: {e}")))?;
    let mut model = parse_opentcs_model_xml(&text, layout_scale_mm, layout_flip_y)?;
    model.name_prefixes = name_prefixes.clone();
    Ok(model)
}

pub fn load_map_arc(
    xml_path: &str,
    layout_scale_mm: f64,
    layout_flip_y: bool,
    name_prefixes: &MapNamePrefixes,
) -> Result<Arc<MapModel>, MapError> {
    load_map(xml_path, layout_scale_mm, layout_flip_y, name_prefixes).map(Arc::new)
}

#[derive(Default)]
struct PathBuild {
    name: String,
    source: String,
    dest: String,
    length_mm: f64,
    max_velocity_mm_s: f64,
    layout_cps: Vec<(f64, f64)>,
}

pub fn parse_opentcs_model_xml(
    xml: &str,
    layout_scale: f64,
    layout_flip_y: bool,
) -> Result<MapModel, MapError> {
    let mut reader = Reader::from_reader(xml.as_bytes());
    reader.config_mut().trim_text(true);

    let mut buf = Vec::new();
    let mut points: HashMap<String, MapPoint> = HashMap::new();
    let mut paths: HashMap<String, MapPath> = HashMap::new();

    let mut inside_point = false;
    let mut point_name = String::new();
    let mut px: f64 = 0.0;
    let mut py: f64 = 0.0;

    let mut inside_path = false;
    let mut path_build = PathBuild::default();
    let mut inside_path_layout = false;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => match e.name().as_ref() {
                b"point" => {
                    inside_point = true;
                    read_point_attrs(e, &mut point_name, &mut px, &mut py);
                }
                b"path" => {
                    inside_path = true;
                    path_build = PathBuild::default();
                    read_path_attrs(e, &mut path_build);
                }
                b"pathLayout" => {
                    if inside_path {
                        inside_path_layout = true;
                        path_build.layout_cps.clear();
                    }
                }
                b"controlPoint" if inside_path_layout => {
                    let (cx, cy) = read_control_point_attrs(e);
                    let (lx, ly) = preprocess_layout_xy(cx, cy, layout_scale, layout_flip_y);
                    path_build.layout_cps.push((lx, ly));
                }
                _ => {}
            },
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"point" => {
                    let mut n = String::new();
                    let mut x = 0.0f64;
                    let mut y = 0.0f64;
                    read_point_attrs(e, &mut n, &mut x, &mut y);
                    if !n.is_empty() {
                        points.insert(
                            n,
                            MapPoint {
                                x_m: x * 1e-3,
                                y_m: y * 1e-3,
                            },
                        );
                    }
                }
                b"controlPoint" if inside_path_layout => {
                    let (cx, cy) = read_control_point_attrs(e);
                    let (lx, ly) = preprocess_layout_xy(cx, cy, layout_scale, layout_flip_y);
                    path_build.layout_cps.push((lx, ly));
                }
                _ => {}
            },
            Ok(Event::End(ref e)) => match e.name().as_ref() {
                b"point" => {
                    if inside_point && !point_name.is_empty() {
                        points.insert(
                            point_name.clone(),
                            MapPoint {
                                x_m: px * 1e-3,
                                y_m: py * 1e-3,
                            },
                        );
                    }
                    inside_point = false;
                }
                b"pathLayout" => inside_path_layout = false,
                b"path" => {
                    if inside_path {
                        let pb = std::mem::take(&mut path_build);
                        let polyline = finish_path_polyline(&points, &pb);
                        if !pb.name.is_empty() && polyline.len() >= 2 {
                            paths.insert(
                                pb.name.clone(),
                                MapPath {
                                    name: pb.name,
                                    source: pb.source,
                                    dest: pb.dest,
                                    length_mm: pb.length_mm,
                                    max_velocity_mm_s: pb.max_velocity_mm_s,
                                    polyline_world_m: polyline,
                                },
                            );
                        }
                    }
                    inside_path = false;
                }
                _ => {}
            },
            Ok(Event::Eof) => break,
            Err(e) => return Err(MapError::Xml(e.to_string())),
            _ => {}
        }
        buf.clear();
    }

    Ok(MapModel {
        points,
        paths,
        name_prefixes: MapNamePrefixes::default(),
    })
}

fn read_point_attrs(
    e: &quick_xml::events::BytesStart<'_>,
    name: &mut String,
    px: &mut f64,
    py: &mut f64,
) {
    name.clear();
    for a in e.attributes().flatten() {
        let k = String::from_utf8_lossy(a.key.as_ref());
        let v = String::from_utf8_lossy(&a.value);
        match k.as_ref() {
            "name" => *name = v.into_owned(),
            "positionX" => *px = v.parse().unwrap_or(0.0),
            "positionY" => *py = v.parse().unwrap_or(0.0),
            _ => {}
        }
    }
}

fn read_path_attrs(e: &quick_xml::events::BytesStart<'_>, pb: &mut PathBuild) {
    for a in e.attributes().flatten() {
        let k = String::from_utf8_lossy(a.key.as_ref());
        let v = String::from_utf8_lossy(&a.value);
        match k.as_ref() {
            "name" => pb.name = v.into_owned(),
            "sourcePoint" => pb.source = v.into_owned(),
            "destinationPoint" => pb.dest = v.into_owned(),
            "length" => pb.length_mm = v.parse().unwrap_or(0.0),
            "maxVelocity" => pb.max_velocity_mm_s = v.parse().unwrap_or(0.0),
            _ => {}
        }
    }
}

fn read_control_point_attrs(e: &quick_xml::events::BytesStart<'_>) -> (f64, f64) {
    let mut cx = 0.0f64;
    let mut cy = 0.0f64;
    for a in e.attributes().flatten() {
        let k = String::from_utf8_lossy(a.key.as_ref());
        let v = String::from_utf8_lossy(&a.value);
        match k.as_ref() {
            "x" => cx = v.parse().unwrap_or(0.0),
            "y" => cy = v.parse().unwrap_or(0.0),
            _ => {}
        }
    }
    (cx, cy)
}

fn finish_path_polyline(points: &HashMap<String, MapPoint>, pb: &PathBuild) -> Vec<(f64, f64)> {
    let Some(s_pt) = points.get(&pb.source) else {
        return Vec::new();
    };
    let Some(d_pt) = points.get(&pb.dest) else {
        return Vec::new();
    };
    let s_world = (s_pt.x_m, s_pt.y_m);
    let d_world = (d_pt.x_m, d_pt.y_m);

    if pb.layout_cps.len() >= 2 {
        polyline_from_layout(&pb.layout_cps, s_world, d_world)
    } else {
        vec![s_world, d_world]
    }
}

fn strip_prefix_and_parse_int(value: &str, prefix: &str) -> i32 {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return 0;
    }
    let after_prefix = if !prefix.is_empty() && trimmed.starts_with(prefix) {
        &trimmed[prefix.len()..]
    } else {
        trimmed
    };
    if let Ok(n) = after_prefix.parse::<i32>() {
        return n;
    }
    if let Some(digits) = trailing_digits_at_end(after_prefix) {
        if let Ok(n) = digits.parse::<i32>() {
            return n;
        }
    }
    0
}

fn trailing_digits_at_end(s: &str) -> Option<&str> {
    let s = s.trim_end();
    let mut start_idx = None;
    for (i, c) in s.char_indices().rev() {
        if c.is_ascii_digit() {
            start_idx = Some(i);
        } else {
            break;
        }
    }
    let i = start_idx?;
    Some(&s[i..])
}

fn preprocess_layout_xy(x: f64, y: f64, scale: f64, flip_y: bool) -> (f64, f64) {
    let y = if flip_y { -y } else { y };
    (x * scale, y * scale)
}

fn layout_to_world_xy(
    lx: f64,
    ly: f64,
    l_first: (f64, f64),
    l_last: (f64, f64),
    s_world: (f64, f64),
    d_world: (f64, f64),
) -> (f64, f64) {
    let (lfx, lfy) = l_first;
    let (llx, lly) = l_last;
    let (sx, sy) = s_world;
    let (dx, dy) = d_world;

    let wx = if (llx - lfx).abs() > 1e-9 {
        sx + (lx - lfx) / (llx - lfx) * (dx - sx)
    } else {
        sx
    };
    let wy = if (lly - lfy).abs() > 1e-9 {
        sy + (ly - lfy) / (lly - lfy) * (dy - sy)
    } else {
        sy
    };
    (wx, wy)
}

fn polyline_from_layout(
    layout_points: &[(f64, f64)],
    s_world: (f64, f64),
    d_world: (f64, f64),
) -> Vec<(f64, f64)> {
    if layout_points.len() < 2 {
        return vec![s_world, d_world];
    }
    let first = layout_points[0];
    let last = *layout_points.last().unwrap();
    layout_points
        .iter()
        .map(|&(lx, ly)| layout_to_world_xy(lx, ly, first, last, s_world, d_world))
        .collect()
}
