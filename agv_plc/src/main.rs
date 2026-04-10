use std::sync::Arc;
use std::time::Duration;

use agv_plc::{config, log, map, socket};
use sim_shared::common::fleet_serial;

#[tokio::main]
async fn main() {
    let config = config::get_config();
    let _log_guards = log::init_from_config(&config).expect("failed to initialize logging");

    let map_model = if config.map.enabled {
        match map::load_map_arc(&config.map) {
            Ok(m) => {
                tracing::info!(
                    target: "bootstrap",
                    "Map loaded: {} points, {} paths from {}",
                    m.points.len(),
                    m.paths.len(),
                    config.map.xml_path
                );
                Some(m)
            }
            Err(e) => {
                tracing::warn!(
                    target: "bootstrap",
                    "Map load failed (running without map geometry): {e}"
                );
                None
            }
        }
    } else {
        None
    };

    for robot_index in 0..config.settings.robot_count {
        spawn_vehicle_simulator(config.clone(), robot_index, map_model.clone()).await;
    }

    loop {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

/// One `VehicleSimulator` + TCP session to the scheduler (same spawn pattern as `agv_vda5050`).
async fn spawn_vehicle_simulator(
    config: config::Config,
    robot_index: u32,
    map_model: Option<std::sync::Arc<map::MapModel>>,
) {
    let mut vehicle_config = config.clone();
    vehicle_config.vehicle.serial_number = fleet_serial(
        &config.vehicle.serial_number,
        config.settings.serial_suffix_start,
        robot_index,
    );

    let agv_id = config::agv_id_for_robot(&config, robot_index);
    let log_target = log::vehicle_log_target(&vehicle_config.vehicle.serial_number);

    tracing::info!(
        target: "bootstrap",
        "PLC simulator instance: agv_id={} serial_number={} robot_index={} -> scheduler {}:{}",
        agv_id,
        vehicle_config.vehicle.serial_number,
        robot_index,
        config.socket.host,
        config.socket.port
    );

    let cfg = Arc::new(vehicle_config);
    let map = map_model.clone();
    let lt = log_target;
    tokio::spawn(async move {
        socket::tcp_client::connection_loop(cfg, agv_id, robot_index, map, lt).await;
    });
}
