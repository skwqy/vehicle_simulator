//! Per-vehicle file logging wrapper over `sim_shared::logging`.

use crate::config::Config;
use sim_shared::common::fleet_serial;
pub use sim_shared::logging::{vehicle_log_target, LogGuards};
use sim_shared::logging::{init_logging, LoggingInitConfig};

pub fn init_from_config(config: &Config) -> Result<LogGuards, String> {
    let serials = (0..config.settings.robot_count)
        .map(|i| fleet_serial(&config.vehicle.serial_number, config.settings.serial_suffix_start, i))
        .collect();

    init_logging(LoggingInitConfig {
        serials,
        log_root: ["logs"].iter().collect(),
        max_bytes: config.settings.log_max_file_bytes,
        max_files: config.settings.log_max_files,
        enable_visualization_log: false,
    })
}
