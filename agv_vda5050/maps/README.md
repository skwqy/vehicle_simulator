# 地图目录（与 AOS / vda5050-simulator 一致）

本目录用于放置 **OpenTCS 驾驶层（driving course）XML**，与 **AOS / 内核当前使用的植物模型** 为同一份文件时，本模拟器使用相同的 **点坐标（毫米）**、**路径几何** 与 **layout→世界变换**（`layoutScaleMm` / `layoutFlipY`），处理方式与 `vda5050-simulator` 一致。

## 目录约定

| 路径（相对工程根或分发包根 `plcv2-simulator/`） | 用途 |
|-----------------------------------|------|
| `maps/youle-final-1.xml` | 示例地图（从 vda5050-simulator 拷贝）。`application.yaml` 里 `map.xmlPath` 默认可指向此文件。 |
| `maps/active.xml`（可选） | 可自行将当前启用地图命名为 `active.xml` 并把 `map.xmlPath` 设为 `maps/active.xml`。 |
| `maps/README.md` | 本说明。 |

## 与 AOS 保持一致

1. 在 AOS / Model Editor 中导出植物模型，或直接使用内核加载的 **同一份** `.xml`（OpenTCS 6 `model` 格式）。
2. 将文件放到本目录（或任意路径），在 `application.yaml` 的 `map.xmlPath` 中写相对工作目录或绝对路径。
3. 若使用 `simulation.initialPointName`（如 `Point_1`），模拟器从植物模型解析该点的 `positionX` / `positionY`（毫米），作为 `StatusMsg` 的初始 `x` / `y`（同为毫米）。

**本仓库 Rust 模拟器**：在 `config.toml` 的 `[map]` 中设置 `enabled = true`、正确 `xml_path`，并设置 `initial_point_name = "Point_1"`（与 OpenTCS `point` 的 `name` 一致），启动时会从地图读取世界坐标（米）写入 `agvPosition`，`lastNodeId` 设为该点名，`positionInitialized = true`。`lastNodeSequenceId` 仍为 `0`（无订单上下文）。

## 切换地图

- **方式 A**：用新文件覆盖 `maps/youle-final-1.xml`（或 `active.xml`），重启模拟器。
- **方式 B**：保留多份 XML，修改 `map.xmlPath` 指向例如 `maps/other.xml`，或通过 `-Dplc.simulator.config=...` 指定另一套 YAML。

## Git

若地图含现场数据、体积大，可在本目录使用 `.gitignore` 忽略 `*.xml`，仅本地保留所需文件。
