//! Downlink: `scheduling_system.SegmentMsg`.

use log::{info, warn};
use prost::Message;

use super::encode;
use super::PlcProtobufEngine;
use crate::scheduling_system::SegmentMsg;

pub(super) fn handle(eng: &mut PlcProtobufEngine, payload: &[u8]) -> Option<Vec<(String, Vec<u8>)>> {
    let msg = SegmentMsg::decode(payload).ok()?;
    eng.task_id = msg.task_id;
    eng.target_point = msg.target_point;
    eng.move_result = 0;
    let incoming: Vec<i32> = msg.segments.clone();
    if incoming.is_empty() {
        eng.moving = false;
        warn!(
            target: eng.lt(),
            "MC -> AGV (SegmentMsg): empty segments, frame={}",
            msg.frame_id
        );
        return Some(vec![("AckMsg".into(), encode::ack(eng.agv_id, msg.frame_id))]);
    }
    if eng.active_route.is_some() {
        eng.pending_segment_ids.extend(incoming.iter().copied());
        return Some(vec![("AckMsg".into(), encode::ack(eng.agv_id, msg.frame_id))]);
    }
    if eng.try_start_route(&incoming) {
        info!(
            target: eng.lt(),
            "MC -> AGV (SegmentMsg): plant route segments={:?} targetPoint={} frame={}",
            incoming,
            eng.target_point,
            msg.frame_id
        );
    } else {
        eng.moving = true;
        eng.fallback_remaining_s = (eng.pb().segment_travel_ms as f32) / 1000.0;
        info!(
            target: eng.lt(),
            "MC -> AGV (SegmentMsg): fallback timer segments={:?} targetPoint={} frame={}",
            incoming,
            eng.target_point,
            msg.frame_id
        );
    }
    Some(vec![("AckMsg".into(), encode::ack(eng.agv_id, msg.frame_id))])
}
