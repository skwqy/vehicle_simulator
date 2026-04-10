//! Small prost encoders for uplink replies (not the `StatusMsg` body — see `status_uplink`).

use prost::Message;

use crate::scheduling_system::{AckMsg, PlcResponseMsg};

pub(super) fn ack(agv_id: i32, frame_id: i32) -> Vec<u8> {
    let m = AckMsg { agv_id, frame_id };
    let mut v = Vec::new();
    m.encode(&mut v).unwrap();
    v
}

pub(super) fn plc_response(
    frame_id: i32,
    agv_id: i32,
    task_id: i32,
    type_: i32,
    result: i32,
    items: &[crate::scheduling_system::PlcItem],
) -> Vec<u8> {
    let m = PlcResponseMsg {
        frame_id,
        agv_id,
        task_id,
        r#type: type_,
        result,
        items: items.iter().copied().collect(),
    };
    let mut v = Vec::new();
    m.encode(&mut v).unwrap();
    v
}
