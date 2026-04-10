use crate::protocol::vda_2_0_0::vda5050_2_0_0_order::Edge;

/// Meters advanced per `update_state()` call (simulation tick).
///
/// - `cfg_speed_per_tick`: legacy `settings.speed` — meters per tick (not m/s).
/// - Order `max_speed` is VDA m/s -> multiplied by `dt_seconds`.
/// - Map path `max_velocity` is mm/s -> m/s then x `dt_seconds`.
pub fn resolve_distance_per_tick(
    edge: Option<&Edge>,
    map_path_max_velocity_m_s: Option<f32>,
    cfg_speed_per_tick: f32,
    dt_seconds: f32,
) -> f32 {
    let mut step = cfg_speed_per_tick.max(0.0);
    if let Some(e) = edge {
        if let Some(ms) = e.max_speed {
            if ms > 0.0 {
                step = step.min(ms * dt_seconds);
            }
        }
    }
    if let Some(pv) = map_path_max_velocity_m_s {
        if pv > 0.0 {
            step = step.min(pv * dt_seconds);
        }
    }
    step
}
