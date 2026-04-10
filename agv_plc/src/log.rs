//! Per-vehicle file logging (`tracing`) and size-based log rotation.

mod logging;

pub use logging::{init_from_config, vehicle_log_target, LogGuards};
