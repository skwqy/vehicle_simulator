//! Uplink: `scheduling_system.StatusMsg` payload bytes for TCP framing.

use prost::Message;

use super::PlcProtobufEngine;
use crate::scheduling_system::{
    BitStatus, StatusMsg, VehicleMode, VehicleType,
};

pub(super) fn build_payload(eng: &mut PlcProtobufEngine) -> Vec<u8> {
    let fid = eng.uplink_frame_seq;
    eng.uplink_frame_seq = eng.uplink_frame_seq.wrapping_add(1);

    let pb = eng.pb();
    let battery_pct = pb.battery_percent.clamp(0, 100);

    let seg_dist_mm = eng
        .active_route
        .as_ref()
        .map(|r| r.seg_distance_mm())
        .unwrap_or(0);

    let linear_speed = if eng.moving {
        pb.default_linear_speed_mm_s()
    } else {
        0
    };

    let mut status_bits: Vec<i32> = vec![BitStatus::Inserted as i32];
    if eng.moving {
        status_bits.push(BitStatus::Moving as i32);
    }
    if eng.host_estop {
        status_bits.push(BitStatus::Estop as i32);
    }
    if pb.charging {
        status_bits.push(BitStatus::Charging as i32);
    }
    let low_th = pb.battery_low_threshold_percent;
    if low_th >= 0 && battery_pct <= low_th {
        status_bits.push(BitStatus::Needcharge as i32);
    }

    let b = StatusMsg {
        frame_id: fid as i32,
        agv_id: eng.agv_id,
        vehicle_type: VehicleType::VehicleIi as i32,
        mode: VehicleMode::Master as i32,
        task_id: eng.task_id,
        point: eng.current_point,
        segment: eng.current_segment_edge_id,
        x: eng.x_m * 1000.0,
        y: eng.y_m * 1000.0,
        heading: eng.heading_deg,
        direction: 1,
        seg_distance: seg_dist_mm,
        status: status_bits,
        linear_speed,
        angular_speed: 0,
        move_result: eng.move_result,
        operation_result: eng.operation_result,
        battery_percent: Some(battery_pct),
        error_code: 0,
        error_message: String::new(),
        op_code: eng.status_op_code,
        faultmsg: vec![],
        lift_mileage: 0,
        travel_odometer: 0,
        subvehicle_mileage: 0,
        child_chargestate: if pb.charging { 1 } else { 0 },
        warnmsg: vec![],
    };

    let mut v = Vec::new();
    b.encode(&mut v).expect("encode StatusMsg");
    v
}
