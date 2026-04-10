pub mod config;
pub mod log;
pub mod map;
pub mod navigation;
pub mod socket;
pub mod vehicle_simulator;

pub mod scheduling_system {
    include!(concat!(env!("OUT_DIR"), "/scheduling_system.rs"));
}

pub mod plc;
