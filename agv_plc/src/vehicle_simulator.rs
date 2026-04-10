//! One simulated AGV: PLC engine bound to `vehicle` + `agv_id` + optional map.
//! Same idea as `vda5050_vehicle_simulator`'s `VehicleSimulator` — one instance per robot.

use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use crate::config::Config;
use crate::map::MapModel;
use crate::plc::PlcProtobufEngine;

pub struct VehicleSimulator {
    inner: PlcProtobufEngine,
}

impl VehicleSimulator {
    pub fn new(
        config: Config,
        agv_id: i32,
        robot_index: u32,
        map: Option<Arc<MapModel>>,
        log_target: String,
    ) -> Self {
        Self {
            inner: PlcProtobufEngine::new(config, agv_id, robot_index, map, log_target),
        }
    }
}

impl Deref for VehicleSimulator {
    type Target = PlcProtobufEngine;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl DerefMut for VehicleSimulator {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}
