use std::path::PathBuf;

use serde::Deserialize;

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct MapNamePrefixes {
    #[serde(alias = "apply-stripping")]
    pub apply_stripping: bool,
    #[serde(alias = "pointPrefix")]
    pub point_prefix: String,
    #[serde(alias = "pathPrefix")]
    pub path_prefix: String,
}

impl Default for MapNamePrefixes {
    fn default() -> Self {
        Self {
            apply_stripping: false,
            point_prefix: "Point_".into(),
            path_prefix: "Path_".into(),
        }
    }
}

pub fn resolve_config_path() -> PathBuf {
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let beside = dir.join("config.toml");
            if beside.is_file() {
                return beside;
            }
        }
    }
    PathBuf::from("config.toml")
}

pub fn fleet_serial(prefix: &str, serial_suffix_start: u32, robot_index: u32) -> String {
    format!("{prefix}{}", serial_suffix_start + robot_index)
}

pub fn fleet_numeric_id(serial_suffix_start: u32, robot_index: u32) -> u32 {
    serial_suffix_start + robot_index
}
