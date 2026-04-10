use config_file::FromConfigFile;
use serde::Deserialize;
use sim_shared::common::{resolve_config_path, MapNamePrefixes};

pub fn get_config() -> Config {
    let path = resolve_config_path();
    Config::from_config_file(&path).unwrap_or_else(|e| {
        panic!(
            "Failed to load config from {}: {e}",
            path.display()
        )
    })
}

#[derive(Deserialize, Clone)]
pub struct MqttBrokerConfig {
    pub host: String,
    pub port: String,
    pub vda_interface: String,
    pub username: Option<String>,
    pub password: Option<String>,
}

#[derive(Deserialize, Clone)]
pub struct VehicleConfig {
    pub manufacturer: String,
    pub serial_number: String,
    pub vda_version: String,
    pub vda_full_version: String,
}

fn default_serial_suffix_start() -> u32 {
    1
}

fn default_state_max_interval_secs() -> u64 {
    30
}

fn default_log_max_file_bytes() -> u64 {
    10 * 1024 * 1024
}

fn default_log_max_files() -> usize {
    10
}

#[derive(Deserialize, Clone)]
pub struct Settings {
    pub action_time: f32,
    pub speed: f32,
    pub robot_count: u32,
    /// Retained for existing `config.toml`; periodic `state` uses `state_max_interval_secs`.
    #[allow(dead_code)]
    pub state_frequency: u64,
    pub visualization_frequency: u64,
    pub map_id: String,
    /// Maximum time between `state` publishes when no event triggers (default 30s, VDA product baseline).
    #[serde(default = "default_state_max_interval_secs")]
    pub state_max_interval_secs: u64,
    /// First robot uses this numeric suffix appended to `vehicle.serial_number`; each additional
    /// robot increments by 1 (e.g. start 2 → …2, …3, …).
    #[serde(default = "default_serial_suffix_start")]
    pub serial_suffix_start: u32,
    /// When true, log outgoing `visualization` JSON to `logs/<serial>/visualization.log` (separate from main vehicle log).
    #[serde(default)]
    pub log_visualization_messages: bool,
    /// Max size per log file before rotation (bytes). Default 10 MiB.
    #[serde(default = "default_log_max_file_bytes")]
    pub log_max_file_bytes: u64,
    /// Max number of files per log stream (active + numbered backups); oldest is dropped when rotating.
    #[serde(default = "default_log_max_files")]
    pub log_max_files: usize,
}

/// OpenTCS plant XML and layout options. Omitted `[map]` in TOML → defaults (disabled).
#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct MapConfig {
    pub enabled: bool,
    pub xml_path: String,
    pub layout_scale_mm: f64,
    pub layout_flip_y: bool,
    /// Stop when this close to target node (m).
    pub arrival_threshold_m: f32,
    /// Simulation time step for motion integration (s); should match publish tick (~0.05).
    pub sim_dt_seconds: f32,
    /// One OpenTCS point name per fleet index `0,1,…` (single vehicle: one element). Requires
    /// `enabled` and a successful map load.
    #[serde(default, alias = "initialPointNames")]
    pub initial_point_names: Vec<String>,
    /// Aligns with `aos-backend` `yeefungagv.plc.name-prefixes` + `aos.vda5050.map-name-prefixes`
    /// (see `YeefungAgvPlcPrefixProperties` / `application-dev.yml`): resolve VDA `nodeId` / `edgeId`
    /// that omit `Point_` / `Path_` to OpenTCS plant names.
    #[serde(default)]
    pub name_prefixes: MapNamePrefixes,
}

/// Same semantics as Java `YeefungAgvPlcPrefixProperties` / `AgvPlcNamePrefixHelper` for point and path
/// names (vehicle prefix is PLC-only; not used for map lookup here).
impl Default for MapConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            xml_path: "maps/youle-final-4.xml".into(),
            layout_scale_mm: 1.0,
            layout_flip_y: false,
            arrival_threshold_m: 0.08,
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

#[derive(Deserialize, Clone)]
pub struct Config {
    pub mqtt_broker: MqttBrokerConfig,
    pub vehicle: VehicleConfig,
    pub settings: Settings,
    #[serde(default)]
    pub map: MapConfig,
}
