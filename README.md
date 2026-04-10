# Vehicle Simulator 单仓说明

该仓库统一管理两个仿真器：

- `agv_plc`：基于 TCP/FV2 的 PLC 仿真器
- `agv_vda5050`：基于 MQTT/VDA5050 的仿真器

## 目录结构

```text
vehicle_simulator/
  Cargo.toml
  Cargo.lock
  sim_shared/                # 公共模块（map/navigation/logging/common）
  agv_plc/
  agv_vda5050/
  build_agv_plc.bat          # 打包到 dist/agv_plc
  build_agv_vda5050.bat      # 打包到 dist/agv_vda5050
  README.md
```

## 构建与运行

在仓库根目录执行：

```bat
cargo check --workspace
cargo run --release -p agv_plc
cargo run --release -p agv_vda5050
```

## 打包发布

在仓库根目录执行：

```bat
build_agv_plc.bat
build_agv_vda5050.bat
```

输出目录：

- `dist/agv_plc/`
- `dist/agv_vda5050/`

每个输出目录包含：

- 可执行文件（`.exe`）
- `config.toml`
- `maps/`（若源工程存在）

## 配置文件

- `agv_plc/config.toml`
- `agv_vda5050/config.toml`

## 公共模块

两个仿真器都使用 `sim_shared`：

- `sim_shared::map`
- `sim_shared::navigation`
- `sim_shared::logging`
- `sim_shared::size_rotating`
- `sim_shared::common`

## 备注

- Docker 相关文件已移除。
- 子工程旧的 `package-release` 脚本已移除，请使用根目录打包脚本。
