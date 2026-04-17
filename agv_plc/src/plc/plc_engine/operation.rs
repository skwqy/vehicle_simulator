//! Downlink: `scheduling_system.OperationMsg`.

use log::info;
use prost::Message;

use super::encode;
use super::PlcProtobufEngine;
use crate::scheduling_system::OperationMsg;

pub(super) fn handle(eng: &mut PlcProtobufEngine, payload: &[u8]) -> Option<Vec<(String, Vec<u8>)>> {
    let msg = OperationMsg::decode(payload).ok()?;
    eng.on_operation_msg_received(msg.op_code);
    info!(
        target: eng.lt(),
        "MC -> AGV (OperationMsg): frame={} op={}",
        msg.frame_id,
        msg.op_code
    );
    Some(vec![("AckMsg".into(), encode::ack(eng.agv_id, msg.frame_id))])
}
