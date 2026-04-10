//! Downlink: `scheduling_system.EstopMsg`.

use log::info;
use prost::Message;

use super::PlcProtobufEngine;
use crate::scheduling_system::{EstopMsg, EstopStatus};

pub(super) fn handle(eng: &mut PlcProtobufEngine, payload: &[u8]) -> Option<()> {
    let msg = EstopMsg::decode(payload).ok()?;
    eng.host_estop = msg.estop_status == EstopStatus::Activateestop as i32;
    info!(
        target: eng.lt(),
        "MC -> AGV (EstopMsg): activate={}",
        eng.host_estop
    );
    Some(())
}
