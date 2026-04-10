//! Downlink: `scheduling_system.NewTaskMsg`.

use log::info;
use prost::Message;

use super::encode;
use super::PlcProtobufEngine;
use crate::scheduling_system::NewTaskMsg;

pub(super) fn handle(eng: &mut PlcProtobufEngine, payload: &[u8]) -> Option<Vec<(String, Vec<u8>)>> {
    let msg = NewTaskMsg::decode(payload).ok()?;
    eng.task_id = msg.task_id;
    eng.target_point = msg.target_point;
    eng.move_result = 0;
    eng.active_route = None;
    eng.pending_segment_ids.clear();
    eng.moving = false;
    let out = vec![("AckMsg".into(), encode::ack(eng.agv_id, msg.frame_id))];
    info!(
        target: eng.lt(),
        "MC -> AGV (NewTaskMsg): taskId={} targetPoint={} frame={}",
        eng.task_id,
        eng.target_point,
        msg.frame_id
    );
    Some(out)
}
