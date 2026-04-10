//! PLC behaviour aligned with FV2 `PlcSimulationEngine`: internal pose in **metres**, `StatusMsg` **x/y mm**, **heading** degrees.
//!
//! Prost types live in `crate::scheduling_system`; per-message logic is split into submodules (`new_task`, `segment`, …).

mod encode;
mod estop;
mod new_task;
mod operation;
mod plc_io;
mod route;
mod segment;
mod status_uplink;

use std::sync::Arc;
use std::time::{Duration, Instant};

use log::{info, warn};

use crate::config::{Config, SimulationConfig};
use crate::map::MapModel;
use crate::navigation::{closest_s_on_polyline, position_at_s};

use route::{build_route_segments, reported_point_int, ActiveRoute};

pub struct PlcProtobufEngine {
    pub config: Config,
    pub agv_id: i32,
    /// Fleet index for `map.initial_point_names[robot_index]`.
    robot_index: u32,
    map: Option<Arc<MapModel>>,
    uplink_frame_seq: u64,

    x_m: f64,
    y_m: f64,
    heading_deg: f64,
    last_open_tcs_point: String,

    task_id: i32,
    target_point: i32,
    current_point: i32,
    current_segment_edge_id: i32,

    active_route: Option<ActiveRoute>,
    pending_segment_ids: Vec<i32>,

    moving: bool,
    fallback_remaining_s: f32,
    move_result: i32,
    operation_result: i32,
    host_estop: bool,

    /// `tracing` target for this vehicle
    log_target: String,
    /// Throttle `StatusMsg` cadence (FV2 `PlcSimulationEngine.shouldPublishStatus`).
    last_status_publish: Instant,
}

impl PlcProtobufEngine {
    pub fn new(
        config: Config,
        agv_id: i32,
        robot_index: u32,
        map: Option<Arc<MapModel>>,
        log_target: String,
    ) -> Self {
        let mut eng = Self {
            config: config.clone(),
            agv_id,
            robot_index,
            map: map.clone(),
            uplink_frame_seq: 1,
            x_m: 0.0,
            y_m: 0.0,
            heading_deg: 0.0,
            last_open_tcs_point: String::new(),
            task_id: 0,
            target_point: 0,
            current_point: 0,
            current_segment_edge_id: 0,
            active_route: None,
            pending_segment_ids: Vec::new(),
            moving: false,
            fallback_remaining_s: 0.0,
            move_result: 0,
            operation_result: -1,
            host_estop: false,
            log_target,
            last_status_publish: {
                let now = Instant::now();
                let ms = config.simulation.status_interval_ms.max(50);
                now.checked_sub(Duration::from_millis(ms))
                    .unwrap_or(now)
            },
        };
        eng.apply_initial_pose();
        eng
    }

    pub(crate) fn pb(&self) -> &SimulationConfig {
        &self.config.simulation
    }

    pub(crate) fn lt(&self) -> &str {
        self.log_target.as_str()
    }

    fn apply_initial_pose(&mut self) {
        let pb = self.pb().clone();
        if let (Some(m), Some(name)) = (
            self.map.as_ref(),
            self.config.map.initial_point_name_for_robot(self.robot_index),
        ) {
            if let Some((xm, ym)) = m.point_world(&name) {
                self.x_m = xm;
                self.y_m = ym;
                self.last_open_tcs_point = name.clone();
                self.current_point = reported_point_int(m, &name);
                self.heading_deg = pb.initial_heading_deg;
                info!(
                    target: self.lt(),
                    "Protobuf initial pose from map: point={} x_mm={:.3} y_mm={:.3}",
                    self.current_point,
                    self.x_m * 1000.0,
                    self.y_m * 1000.0
                );
                return;
            }
            warn!(
                target: self.lt(),
                "map.initial_point_names[{}] {:?} not found; using protobuf initial x/y mm",
                self.robot_index,
                name
            );
        }
        self.x_m = pb.initial_x_mm / 1000.0;
        self.y_m = pb.initial_y_mm / 1000.0;
        self.heading_deg = pb.initial_heading_deg;
        self.current_point = pb.initial_point;
        self.last_open_tcs_point.clear();
    }

    pub fn tick(&mut self, dt_seconds: f32) {
        let dt = dt_seconds.max(1e-6);
        if self.active_route.is_some() {
            let speed_mps = self.pb().default_linear_speed_mm_s as f32 / 1000.0;
            let mut dist = speed_mps * dt;
            let done = {
                let Some(route) = self.active_route.as_mut() else {
                    return;
                };
                while dist > 0.0 {
                    let Some(seg) = route.segments.get(route.edge_idx) else {
                        break;
                    };
                    let rem = (seg.len_m - route.s_m).max(0.0);
                    if rem <= 1e-6 {
                        route.edge_idx += 1;
                        route.s_m = 0.0;
                        if route.edge_idx >= route.segments.len() {
                            break;
                        }
                        continue;
                    }
                    if dist < rem {
                        route.s_m += dist;
                        dist = 0.0;
                    } else {
                        dist -= rem;
                        route.edge_idx += 1;
                        route.s_m = 0.0;
                        if route.edge_idx >= route.segments.len() {
                            break;
                        }
                    }
                }
                route.edge_idx >= route.segments.len()
            };
            if done {
                self.complete_plant_route();
                return;
            }
            self.sync_pose_from_route();
            if let Some(route) = self.active_route.as_ref() {
                self.current_segment_edge_id = route.current_edge_id();
            }
            self.moving = true;
            return;
        }

        if self.moving && self.fallback_remaining_s > 0.0 {
            self.fallback_remaining_s -= dt;
            if self.fallback_remaining_s <= 0.0 {
                self.moving = false;
                self.fallback_remaining_s = 0.0;
                self.current_point = self.target_point;
                self.move_result = 1;
                info!(
                    target: self.lt(),
                    "Fallback segment finished at target_point={}",
                    self.target_point
                );
            }
        }
    }

    fn complete_plant_route(&mut self) {
        let Some(route) = self.active_route.take() else {
            return;
        };
        if let Some(last) = route.segments.last() {
            if let Some(p) = last.poly.last() {
                self.x_m = p.0 as f64;
                self.y_m = p.1 as f64;
            }
            let (_, _, th) = position_at_s(&last.poly, last.len_m);
            self.heading_deg = th.to_degrees() as f64;
        }
        self.last_open_tcs_point = route.end_open_tcs_point;
        if let Some(m) = self.map.as_ref() {
            if !self.last_open_tcs_point.is_empty() {
                self.current_point = reported_point_int(m, &self.last_open_tcs_point);
            }
        }
        self.moving = false;
        self.move_result = 1;
        self.current_segment_edge_id = 0;
        info!(
            target: self.lt(),
            "Plant route finished at reported_point={} openTcs={}",
            self.current_point,
            self.last_open_tcs_point
        );
        self.start_pending_route_if_any();
    }

    fn sync_pose_from_route(&mut self) {
        let Some(route) = self.active_route.as_ref() else {
            return;
        };
        let Some(seg) = route.segments.get(route.edge_idx) else {
            return;
        };
        let (x, y, theta) = position_at_s(&seg.poly, route.s_m);
        self.x_m = x as f64;
        self.y_m = y as f64;
        self.heading_deg = theta.to_degrees() as f64;
    }

    fn start_pending_route_if_any(&mut self) {
        if self.pending_segment_ids.is_empty() {
            return;
        }
        let ids = std::mem::take(&mut self.pending_segment_ids);
        if !self.try_start_route(&ids) {
            self.moving = true;
            self.fallback_remaining_s = (self.pb().segment_travel_ms as f32) / 1000.0;
            warn!(
                target: self.lt(),
                "Queued segments {:?} could not start from lastOpenTcs={} — fallback timer",
                ids,
                self.last_open_tcs_point
            );
        }
    }

    pub(crate) fn try_start_route(&mut self, segment_ids: &[i32]) -> bool {
        let Some(map) = self.map.as_ref() else {
            return false;
        };
        if self.last_open_tcs_point.is_empty() {
            warn!(
                target: self.lt(),
                "Cannot build plant route: last_open_tcs_point empty; set map.initial_point_names"
            );
            return false;
        }
        let Some((segs, end_point)) =
            build_route_segments(map, &self.last_open_tcs_point, segment_ids)
        else {
            return false;
        };
        let mut route = ActiveRoute {
            segments: segs,
            edge_idx: 0,
            s_m: 0.0,
            end_open_tcs_point: end_point,
        };
        if let Some(first) = route.segments.first() {
            route.s_m = closest_s_on_polyline(
                self.x_m as f32,
                self.y_m as f32,
                &first.poly,
            )
            .min(first.len_m);
        }
        self.active_route = Some(route);
        self.moving = true;
        self.move_result = 0;
        self.sync_pose_from_route();
        self.current_segment_edge_id = self
            .active_route
            .as_ref()
            .map(|r| r.current_edge_id())
            .unwrap_or(0);
        true
    }

    pub fn handle_downlink(&mut self, normalized_suffix: &str, payload: &[u8]) -> Vec<(String, Vec<u8>)> {
        let mut out: Vec<(String, Vec<u8>)> = Vec::new();
        match normalized_suffix {
            "NewTaskMsg" => {
                if let Some(v) = new_task::handle(self, payload) {
                    out = v;
                }
            }
            "SegmentMsg" => {
                if let Some(v) = segment::handle(self, payload) {
                    out = v;
                }
            }
            "OperationMsg" => {
                if let Some(v) = operation::handle(self, payload) {
                    out = v;
                }
            }
            "EstopMsg" => {
                let _ = estop::handle(self, payload);
            }
            "PLCWriteMsg" => {
                if let Some(v) = plc_io::handle_write(self, payload) {
                    out = v;
                }
            }
            "PLCReadMsg" => {
                if let Some(v) = plc_io::handle_read(self, payload) {
                    out = v;
                }
            }
            _ => {
                warn!(
                    target: self.lt(),
                    "MC -> AGV: unhandled protobuf downlink type: {}",
                    normalized_suffix
                );
            }
        }
        out
    }

    pub fn build_status_payload(&mut self) -> Vec<u8> {
        status_uplink::build_payload(self)
    }

    pub fn is_moving(&self) -> bool {
        self.active_route.is_some() || (self.moving && self.fallback_remaining_s > 0.0)
    }

    pub fn should_publish_status(&self) -> bool {
        let interval_ms = self.status_interval_ms(self.is_moving()).max(50);
        let min = Duration::from_millis(interval_ms);
        Instant::now().duration_since(self.last_status_publish) >= min
    }

    pub fn after_status_published(&mut self) {
        self.last_status_publish = Instant::now();
    }

    pub fn status_interval_ms(&self, moving: bool) -> u64 {
        let pb = self.pb();
        let base = pb.status_interval_ms.max(50);
        if moving && pb.status_interval_moving_ms > 0 {
            base.min(pb.status_interval_moving_ms.max(50))
        } else {
            base
        }
    }
}
