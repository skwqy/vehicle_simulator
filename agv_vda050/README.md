# agv_vda050

基于 Rust 的 MQTT/VDA5050 仿真器。

## 功能说明

- 按 VDA5050 协议模拟一个或多个 AGV
- 通过 MQTT 进行状态/指令交互
- 支持 OpenTCS XML 地图驱动运动（可选）
- 支持可选的 visualization 日志独立落盘

## 关键文件

- `config.toml`：运行配置
- `src/vehicle_simulator.rs`：核心状态机与行为
- `src/mqtt_handler.rs`：发布/订阅流程
- `src/protocol/`：VDA 协议数据结构
- `src/main.rs`：启动与多车实例化

## 本地运行

```bat
cd /d d:\workspace\simulator\vehicle_simulator\agv_vda050
cargo run --release
```

## 推荐打包方式（仓库根目录）

```bat
cd /d d:\workspace\simulator\vehicle_simulator
build_agv_vda050.bat
```

输出目录：

- `dist/agv_vda050/agv_vda050.exe`
- `dist/agv_vda050/config.toml`
- `dist/agv_vda050/maps/`（若存在）

## 配置分区

- `[mqtt_broker]`：MQTT 连接和接口配置
- `[vehicle]`：序列号前缀、厂商、VDA 版本
- `[settings]`：频率、速度、多车、日志参数
- `[map]`：地图与名称前缀规则

常用参数：

- `settings.state_max_interval_secs`
- `settings.visualization_frequency`
- `settings.log_visualization_messages`
- `settings.log_max_file_bytes`
- `settings.log_max_files`

## 日志

- 日志基础设施来自 `sim_shared::logging`
- 每车日志：
  - `logs/<serial>/vehicle.log`
  - `logs/<serial>/visualization.log`（可选）
- 可通过 `RUST_LOG` 控制终端日志级别
