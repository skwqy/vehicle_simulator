use config_file::FromConfigFile;
use serde::Deserialize;
use sim_shared::common::{fleet_numeric_id, resolve_config_path, MapNamePrefixes};

pub fn get_config() -> Config {
    let path = resolve_config_path();
    Config::from_config_file(&path).unwrap_or_else(|e| {
        panic!(
            "Failed to load config from {}: {e}",
            path.display()
        )
    })
}

/// Wire vehicle id for protobuf (`StatusMsg.agv_id`, `AckMsg`, …).
///
/// Same rule as the serial suffix in `main`: `settings.serial_suffix_start + robot_index`.
pub fn agv_id_for_robot(config: &Config, robot_index: u32) -> i32 {
    fleet_numeric_id(config.settings.serial_suffix_start, robot_index) as i32
}

fn default_connect_timeout_ms() -> u64 {
    10_000
}

fn default_max_frame_length() -> usize {
    65536
}

fn default_uplink_has_crc() -> bool {
    true
}

fn default_reconnect_enabled() -> bool {
    true
}

fn default_reconnect_interval_ms() -> u64 {
    5000
}

/// TCP to YFAOS `AgvSocketServer` (FV2 `SocketPlcBridge`).
#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct SocketConfig {
    pub host: String,
    pub port: u16,
    #[serde(default = "default_connect_timeout_ms")]
    pub connect_timeout_ms: u64,
    /// 0 = no read timeout (blocking read on half, same as Java `readTimeoutMs: 0`).
    #[serde(default)]
    pub read_timeout_ms: u64,
    #[serde(default = "default_max_frame_length")]
    pub max_frame_length: usize,
    #[serde(default = "default_uplink_has_crc")]
    pub uplink_has_crc: bool,
    #[serde(default = "default_reconnect_enabled")]
    pub reconnect_enabled: bool,
    #[serde(default = "default_reconnect_interval_ms")]
    pub reconnect_interval_ms: u64,
}

impl Default for SocketConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 8008,
            connect_timeout_ms: default_connect_timeout_ms(),
            read_timeout_ms: 0,
            max_frame_length: default_max_frame_length(),
            uplink_has_crc: default_uplink_has_crc(),
            reconnect_enabled: default_reconnect_enabled(),
            reconnect_interval_ms: default_reconnect_interval_ms(),
        }
    }
}

#[derive(Deserialize, Clone)]
pub struct VehicleConfig {
    pub manufacturer: String,
    pub serial_number: String,
}

fn default_serial_suffix_start() -> u32 {
    1
}

fn default_log_max_file_bytes() -> u64 {
    10 * 1024 * 1024
}

fn default_log_max_files() -> usize {
    10
}

#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct Settings {
    pub robot_count: u32,
    #[serde(default = "default_serial_suffix_start")]
    pub serial_suffix_start: u32,
    /// Optional map id string (same field name as `vda5050_vehicle_simulator` for shared config).
    pub map_id: String,
    #[serde(default = "default_log_max_file_bytes")]
    pub log_max_file_bytes: u64,
    #[serde(default = "default_log_max_files")]
    pub log_max_files: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            robot_count: 1,
            serial_suffix_start: default_serial_suffix_start(),
            map_id: String::new(),
            log_max_file_bytes: default_log_max_file_bytes(),
            log_max_files: default_log_max_files(),
        }
    }
}

/// OpenTCS plant XML and layout options. Omitted `[map]` in TOML → defaults (disabled).
#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct MapConfig {
    pub enabled: bool,
    #[serde(alias = "xmlPath")]
    pub xml_path: String,
    #[serde(alias = "layoutScaleMm")]
    pub layout_scale_mm: f64,
    #[serde(alias = "layoutFlipY")]
    pub layout_flip_y: bool,
    pub sim_dt_seconds: f32,
    /// One OpenTCS point name per fleet index `0,1,…` (single vehicle: one element, e.g. `["Point_1"]`).
    #[serde(default, alias = "initialPointNames")]
    pub initial_point_names: Vec<String>,
    #[serde(default)]
    pub name_prefixes: MapNamePrefixes,
}

impl Default for MapConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            xml_path: "maps/youle-final-4.xml".into(),
            layout_scale_mm: 1.0,
            layout_flip_y: false,
            sim_dt_seconds: 0.05,
            initial_point_names: Vec::new(),
            name_prefixes: MapNamePrefixes::default(),
        }
    }
}

impl MapConfig {
    pub fn initial_point_name_for_robot(&self, robot_index: u32) -> Option<String> {
        self.initial_point_names
            .get(robot_index as usize)
            .map(|s| s.trim())
            .filter(|s| !s.is_empty())
            .map(|s| s.to_string())
    }
}

fn default_status_interval_ms() -> u64 {
    200
}

fn default_status_interval_moving_ms() -> u64 {
    100
}

fn default_segment_travel_ms() -> u64 {
    2000
}

fn default_default_linear_speed_mm_s() -> i32 {
    500
}

fn default_battery_percent() -> i32 {
    100
}

fn default_battery_low_threshold() -> i32 {
    -1
}

/// PLC protobuf behaviour parameters — same meaning as FV2 `application.yaml` → `simulation:`.
#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct SimulationConfig {
    #[serde(default = "default_status_interval_ms", alias = "statusIntervalMs")]
    pub status_interval_ms: u64,
    #[serde(default = "default_status_interval_moving_ms", alias = "statusIntervalMovingMs")]
    pub status_interval_moving_ms: u64,
    #[serde(default = "default_segment_travel_ms", alias = "segmentTravelMs")]
    pub segment_travel_ms: u64,
    #[serde(
        default = "default_default_linear_speed_mm_s",
        alias = "defaultLinearSpeed"
    )]
    pub default_linear_speed_mm_s: i32,
    #[serde(default = "default_battery_percent", alias = "batteryPercent")]
    pub battery_percent: i32,
    #[serde(
        default = "default_battery_low_threshold",
        alias = "batteryLowThresholdPercent"
    )]
    pub battery_low_threshold_percent: i32,
    #[serde(default)]
    pub charging: bool,
    #[serde(default, alias = "initialPoint")]
    pub initial_point: i32,
    #[serde(default, alias = "initialX")]
    pub initial_x_mm: f64,
    #[serde(default, alias = "initialY")]
    pub initial_y_mm: f64,
    #[serde(default, alias = "initialHeading")]
    pub initial_heading_deg: f64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            status_interval_ms: default_status_interval_ms(),
            status_interval_moving_ms: default_status_interval_moving_ms(),
            segment_travel_ms: default_segment_travel_ms(),
            default_linear_speed_mm_s: default_default_linear_speed_mm_s(),
            battery_percent: default_battery_percent(),
            battery_low_threshold_percent: default_battery_low_threshold(),
            charging: false,
            initial_point: 0,
            initial_x_mm: 0.0,
            initial_y_mm: 0.0,
            initial_heading_deg: 0.0,
        }
    }
}

#[derive(Deserialize, Clone)]
pub struct Config {
    #[serde(default)]
    pub socket: SocketConfig,
    pub vehicle: VehicleConfig,
    #[serde(default)]
    pub settings: Settings,
    #[serde(default)]
    pub map: MapConfig,
    #[serde(default)]
    pub simulation: SimulationConfig,
}
