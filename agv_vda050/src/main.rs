use std::sync::Arc;
use std::time::Duration;
use sim_shared::common::fleet_serial;
use tokio::sync::Mutex;

use agv_vda050::{
    config, logging, map, mqtt_handler, vehicle_simulator::VehicleSimulator,
};

use mqtt_handler::{publish_vda_messages, subscribe_vda_messages};

#[tokio::main]
async fn main() {
    let config = config::get_config();
    let _log_guards = logging::init_from_config(&config).expect("failed to initialize logging");

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

    // Keep the main thread alive
    loop {
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

async fn spawn_vehicle_simulator(
    config: config::Config,
    robot_index: u32,
    map_model: Option<std::sync::Arc<map::MapModel>>,
) {
    // Create vehicle-specific configuration
    let mut vehicle_config = config.clone();
    vehicle_config.vehicle.serial_number = fleet_serial(
        &config.vehicle.serial_number,
        config.settings.serial_suffix_start,
        robot_index,
    );

    let log_target = logging::vehicle_log_target(&vehicle_config.vehicle.serial_number);

    // Create and share vehicle simulator
    let vehicle_simulator = VehicleSimulator::new(vehicle_config.clone(), robot_index, map_model);
    let shared_simulator = Arc::new(Mutex::new(vehicle_simulator));
    
    // Clone for async tasks
    let simulator_for_subscribe = Arc::clone(&shared_simulator);
    let simulator_for_publish = Arc::clone(&shared_simulator);

    // Spawn MQTT subscription task
    tokio::spawn(subscribe_vda_messages(
        vehicle_config,
        simulator_for_subscribe,
        log_target.clone(),
    ));

    // Spawn MQTT publishing task (state: event + heartbeat per docs/vda5050-state-publish-design.md)
    tokio::spawn(publish_vda_messages(
        simulator_for_publish,
        config.settings.state_max_interval_secs,
        config.map.sim_dt_seconds,
        config.settings.visualization_frequency,
        log_target.clone(),
    ));
}
