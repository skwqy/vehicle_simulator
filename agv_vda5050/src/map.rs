pub use sim_shared::map::{parse_opentcs_model_xml, MapError, MapModel, MapPoint};
use sim_shared::map;

use crate::config::MapConfig;

pub fn load_map_from_config(cfg: &MapConfig) -> Result<MapModel, MapError> {
    map::load_map(
        &cfg.xml_path,
        cfg.layout_scale_mm,
        cfg.layout_flip_y,
        &cfg.name_prefixes,
    )
}

pub fn load_map_arc(cfg: &MapConfig) -> Result<std::sync::Arc<MapModel>, MapError> {
    map::load_map_arc(
        &cfg.xml_path,
        cfg.layout_scale_mm,
        cfg.layout_flip_y,
        &cfg.name_prefixes,
    )
}
