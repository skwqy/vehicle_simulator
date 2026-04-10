# agv_plc

基于 Rust 的 TCP/FV2 PLC 仿真器。

## 功能说明

- 与调度系统建立 TCP 连接并收发 protobuf 帧（`AgvMsgProtocol.proto`）
- 单进程支持多车仿真
- 支持基于 OpenTCS XML 的地图/路径驱动
- 支持按车辆分文件滚动日志

## 关键文件

- `config.toml`：运行配置
- `proto/AgvMsgProtocol.proto`：协议定义
- `build.rs`：protobuf 代码生成入口（`prost`）
- `src/plc/`：PLC 行为逻辑
- `src/socket/`：TCP 连接与帧处理

## 本地运行

```bat
cd /d d:\workspace\simulator\vehicle_simulator\agv_plc
cargo run --release
```

## 推荐打包方式（仓库根目录）

```bat
cd /d d:\workspace\simulator\vehicle_simulator
build_agv_plc.bat
```

输出目录：

- `dist/agv_plc/agv_plc.exe`
- `dist/agv_plc/config.toml`
- `dist/agv_plc/maps/`（若存在）

## 配置分区

- `[socket]`：调度端地址、帧参数、重连参数
- `[vehicle]`：序列号前缀、制造商
- `[settings]`：车辆数、序列号后缀起点、日志轮转
- `[map]`：地图路径及前缀解析规则
- `[simulation]`：状态发布与运动默认参数

## 日志

- 日志基础设施来自 `sim_shared::logging`
- 每车主日志：`logs/<serial>/vehicle.log`
- 轮转参数：
  - `settings.log_max_file_bytes`
  - `settings.log_max_files`
