use config_file::FromConfigFile;
use serde::Deserialize;
use sim_shared::common::{resolve_config_path, MapNamePrefixes};

pub fn get_config() -> Config {
    let path = resolve_config_path();
    let mut cfg = Config::from_config_file(&path).unwrap_or_else(|e| {
        panic!(
            "Failed to load config from {}: {e}",
            path.display()
        )
    });
    // Backward compatibility: accept legacy runtime keys under [settings].
    if let Some(v) = cfg.settings.legacy_action_time {
        cfg.simulation.action_time = v;
    }
    if let Some(v) = cfg.settings.legacy_speed_m_s {
        cfg.simulation.speed_m_s = v;
    }
    if let Some(v) = cfg.settings.legacy_state_frequency {
        cfg.simulation.state_frequency = v;
    }
    if let Some(v) = cfg.settings.legacy_visualization_frequency {
        cfg.simulation.visualization_frequency = v;
    }
    if let Some(v) = cfg.settings.legacy_state_max_interval_secs {
        cfg.simulation.state_max_interval_secs = v;
    }
    cfg
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

fn default_action_time() -> f32 {
    1.0
}

fn default_speed_m_s() -> f32 {
    1.0
}

fn default_state_frequency() -> u64 {
    1
}

fn default_visualization_frequency() -> u64 {
    1
}

#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct Settings {
    pub robot_count: u32,
    pub map_id: String,
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
    #[serde(default, alias = "action_time")]
    pub legacy_action_time: Option<f32>,
    #[serde(default, alias = "speed_m_s", alias = "speed")]
    pub legacy_speed_m_s: Option<f32>,
    #[serde(default, alias = "state_frequency")]
    pub legacy_state_frequency: Option<u64>,
    #[serde(default, alias = "visualization_frequency")]
    pub legacy_visualization_frequency: Option<u64>,
    #[serde(default, alias = "state_max_interval_secs")]
    pub legacy_state_max_interval_secs: Option<u64>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            robot_count: 1,
            map_id: String::new(),
            serial_suffix_start: default_serial_suffix_start(),
            log_visualization_messages: false,
            log_max_file_bytes: default_log_max_file_bytes(),
            log_max_files: default_log_max_files(),
            legacy_action_time: None,
            legacy_speed_m_s: None,
            legacy_state_frequency: None,
            legacy_visualization_frequency: None,
            legacy_state_max_interval_secs: None,
        }
    }
}

#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct SimulationConfig {
    #[serde(default = "default_action_time")]
    pub action_time: f32,
    #[serde(default = "default_speed_m_s", alias = "speed")]
    pub speed_m_s: f32,
    /// Retained for compatibility with existing config style.
    #[allow(dead_code)]
    #[serde(default = "default_state_frequency")]
    pub state_frequency: u64,
    #[serde(default = "default_visualization_frequency")]
    pub visualization_frequency: u64,
    /// Maximum time between `state` publishes when no event triggers (default 30s, VDA product baseline).
    #[serde(default = "default_state_max_interval_secs")]
    pub state_max_interval_secs: u64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            action_time: default_action_time(),
            speed_m_s: default_speed_m_s(),
            state_frequency: default_state_frequency(),
            visualization_frequency: default_visualization_frequency(),
            state_max_interval_secs: default_state_max_interval_secs(),
        }
    }
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
    #[serde(default)]
    pub settings: Settings,
    #[serde(default)]
    pub simulation: SimulationConfig,
    #[serde(default)]
    pub map: MapConfig,
}
