# 地图驱动运动与状态上报 — 设计方案

> **状态**：待确认（确认后再编码）  
> **关联**：本仓库 Rust 模拟器 `vda5050_vehicle_simulator`；地图为 OpenTCS 6 `model` XML（见 `maps/`）。  
> **说明**：你提到的 Java 版 VDA5050 模拟器（如 `vda5050-simulator` / `application.yaml` + `map.xmlPath`）不在本仓库内；本方案按 **OpenTCS 植物模型语义** 与 **VDA5050 2.0 订单/状态** 对齐，并在实现阶段尽量与你在 Java 侧已验证的 **坐标单位、layout 变换** 保持一致（参见 `maps/README.md` 中的 `layoutScaleMm` / `layoutFlipY` 等约定）。

---

## 1. 目标与范围

### 1.1 目标

在 **接收 VDA5050 Order** 后：

1. 将订单中的 **节点（node）/边（edge）** 与 **OpenTCS 地图** 中的 **点（point）/路径（path）** 关联（可配置映射规则）。
2. 根据 **地图几何**（点坐标、路径长度、可选路径控制点）与 **速度约束**（订单 `maxSpeed`、地图 `maxVelocity` 等）在仿真循环中 **实时积分位置**。
3. 按 VDA5050 要求更新并 **上报** `state` / `visualization`（`agvPosition`、`velocity`、`driving`、`distanceSinceLastNode`、`lastNodeId`、`nodeStates` / `edgeStates` 等与运动相关的字段）。

### 1.2 非目标（首期可不实现）

- 多楼层 / 多 `map_id` 切换与电梯场景（可在数据结构中预留）。
- 完整的 OpenTCS 全要素（区块、车辆类型、锁定策略等）仿真。
- 与真实控制器一致的加减速 jerk 模型（可用分段匀速或简单梯形速度曲线作为二期）。

### 1.3 与当前代码的关系

当前 `VehicleSimulator` 已具备：

- 订单接受、`node_states` / `edge_states` 填充、`last_node_sequence_id` 推进。
- 沿 **订单中 `nodePosition`（若存在）** 向下一节点插值；若边带 **VDA5050 `trajectory`（NURBS）** 则走 `iterate_position_with_trajectory`，否则直线 `iterate_position`。
- **全局速度** 使用 `config.settings.speed`（m/s 量级），未读取地图 XML。

本方案在 **不破坏现有 MQTT/协议结构** 的前提下，增加 **地图加载与几何/速度来源**，使运动轨迹与 OpenTCS 一致或可追溯。

---

## 2. 地图数据模型（从 OpenTCS XML 解析）

### 2.1 输入格式

- 文件：OpenTCS 6 `<model version="6.0.0" ...>`。
- **点 `point`**：`name`，`positionX` / `positionY` / `positionZ`（XML 中为 **毫米**，整数或浮点以解析为准）。
- **路径 `path`**：`name`，`sourcePoint`，`destinationPoint`，`length`（**毫米**），`maxVelocity` / `maxReverseVelocity`（OpenTCS 常用 **毫米/秒**，以你现场导出为准，实现时集中换算）。
- **几何 `pathLayout`**：`connectionType`（如 `POLYPATH`、`BEZIER`、`DIRECT` 等），`controlPoint` 列表（`x`,`y` — 通常为 **布局/示意坐标**，需经 **layout→世界** 变换后与点坐标一致）。

### 2.2 内存结构（建议）

- `MapModel`：
  - `points: HashMap<String, Point>` — 世界坐标建议统一为 **米**（`f64`），便于与 VDA5050 一致。
  - `paths: HashMap<String, Path>` — 含起止点引用、`length_mm`、`max_velocity_mm_s`、可选 **折线/样条控制点序列**（已变换到世界坐标米）。
- **有向图**：由 `path` 的 `sourcePoint` → `destinationPoint` 建立邻接表，用于校验订单边是否与地图拓扑一致（可选：首期仅做几何插值，拓扑校验为警告）。

### 2.3 坐标与 layout 变换

与 `maps/README.md` 及 Java 模拟器对齐：

- **世界坐标**：OpenTCS 点坐标（mm）→ 米：`x_m = positionX * 1e-3`（若需与 Java 一致，可配置 `layoutScaleMm`、`layoutFlipY` 对 **pathLayout 控制点** 做变换后再与世界坐标拼接）。
- **配置项（建议写入 `config.toml` 新段 `[map]`）**：
  - `xml_path`：如 `maps/youle-final-4.xml`。
  - `layout_scale_mm` / `layout_flip_y`（及若 Java 侧还有平移/原点偏移，一并参数化）。
  - `coordinate_frame`：文档化「与 MC 下发的 `nodePosition` 是否同一坐标系」。

**原则**：同一套 XML 在 Java 与 Rust 中算出的 **同一点世界坐标** 应一致（便于联调）。

---

## 3. 订单与地图的关联策略

### 3.1 标识映射

MC 下发的 `order.nodes[].nodeId` / `order.edges[].edgeId` 与 OpenTCS 的 `point.name` / `path.name` 可能 **字符串不一致**。

建议支持 **可配置映射**（首期可实现一种默认）：

| 模式 | 说明 |
|------|------|
| **A. 直接同名** | `nodeId == point.name`，`edgeId == path.name`。 |
| **B. 前缀/规则** | 如 `nodeId = "Point_12"` 与地图 `Point_12` 一致（可配置前缀剥离）。 |
| **C. 外部表** | 可选 `map_id_mapping.toml`（`vdaNodeId` → `opentcsPointName`），二期再做。 |

默认 **A**，并在日志中打印无法解析的 id。

### 3.2 当订单节点缺少 `nodePosition` 时

- 从 `MapModel` 按 `nodeId` 查 **点坐标**，填充到内部使用的「目标点」或仅用于校验 MC 给出的坐标与地图差值。
- 若订单 **带 `nodePosition`**：优先以订单为准做运动目标，**可选** 与地图坐标做容差校验（超差打日志或拒绝订单 — 可配置）。

### 3.3 边的几何来源优先级

对每条 `order` 中的 `Edge`（连接 `startNodeId` → `endNodeId`）：

1. 若边上有 VDA5050 **`trajectory`（NURBS）**：**保持现有逻辑**（与 MC 规划一致）。
2. 否则若地图中存在对应 `path` 且带 **`pathLayout` 控制点**：沿 **折线或贝塞尔离散后的折线** 插值（见 §4）。
3. 否则：**起点/终点直线**（与当前 `iterate_position` 一致），长度可用地图 `length` 或欧氏距离校验。

---

## 4. 运动学与时间步进

### 4.1 仿真时钟

- 继续基于现有 `publish_vda_messages` 中的 **固定 tick**（如 50 ms）调用 `update_state()`。
- 引入 **速度上限**：
  - `v_order = edge.max_speed`（VDA5050，**米/秒**）；
  - `v_map = path.max_velocity_mm_s * 1e-3`（换算为 m/s）；
  - `v_cfg = config.settings.speed`；
  - 实际 `v = min(v_order, v_map, v_cfg)`（若某项缺失则忽略该项）。

### 4.2 沿路径积分

- 维护 **标量弧长进度** `s`（米）：当前在哪条边的几何上、从起点累积。
- 每步：`delta_s = v * dt`（`dt` 为 tick 秒数）。
- 根据几何类型求 **当前 (x, y, theta)**：
  - **直线段**：线性插值，theta = `atan2(dy, dx)`。
  - **折线**：分段累加长度，落在对应线段内插值。
  - **贝塞尔**：将曲线按弧长离散为折线近似（可调采样步数），或解析求最近点（实现复杂度更高，首期推荐离散折线）。

### 4.3 节点到达判定

- 与现实现类似：当前位置到 **下一节点目标** 的距离小于阈值即视为到达；阈值建议与 `allowedDeviationXy`（若存在）或固定 **0.05 m** 可配置。
- 到达后：更新 `lastNodeId`、`lastNodeSequenceId`，弹出已完成的 `nodeState` / `edgeState`，执行节点动作（现有 `process_node_actions` 流程保留）。

### 4.4 建议填充/强化的状态字段

- `driving`：在沿边移动时为 `true`，在节点执行动作且无移动时为 `false`（可细化）。
- `distance_since_last_node`：从离开上一节点起沿路径累积弧长（米）。
- `velocity`：向量或标量（按你现有 `State` 结构体字段类型填写）。
- `agv_position.map_id`：与 `config.map_id` 或订单一致。

---

## 5. 模块划分、代码分层与集成点（刷新）

### 5.1 分层模型（自上而下）

```
┌─────────────────────────────────────────────────────────────┐
│  Composition / 进程入口 (main.rs)                             │
│  组装配置、加载地图资源、注入模拟器                             │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│  Application / 用例编排 (vehicle_simulator.rs)               │
│  订单生命周期、状态机、MQTT 载荷字段组装                      │
│  依赖抽象：地图查询、单步运动求解（不直接解析 XML）              │
└───────────────────────────┬─────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────────┐
│ Navigation    │   │ Map 基础设施   │   │ 现有几何工具       │
│ (new)         │   │ (new)         │   │ utils.rs          │
│ 运动步进、速度  │   │ 解析、模型、   │   │ NURBS / 直线      │
│ 合成、弧长状态 │   │ layout 变换   │   │ （保持，由导航层   │
│               │   │               │   │  调用）            │
└───────────────┘   └───────────────┘   └───────────────────┘
        │                   │
        └─────────┬─────────┘
                  ▼
┌─────────────────────────────────────────────────────────────┐
│  Domain / 协议与配置 (protocol/*, config.rs)                 │
│  仅数据结构，不依赖地图实现细节                                │
└─────────────────────────────────────────────────────────────┘
```

**分层原则简述**

| 层 | 职责 | 禁止 |
|----|------|------|
| **main** | 读配置、调用 `MapLoader::load`、构造 `Arc<dyn MapView>`（或具体类型）、传入 `VehicleSimulator::new` | 不包含订单或 MQTT 业务 |
| **vehicle_simulator** | 何时接单、何时 `update_state`、如何填 `State` / `Visualization` | 不解析 XML、不实现贝塞尔采样细节 |
| **navigation**（建议 `src/navigation/`） | 给定「当前边 + 订单 Edge + 可选地图几何」，输出下一步位姿与标量速度；维护弧长、`distance_since_last_node` | 不序列化 JSON、不连 broker |
| **map**（建议 `src/map/`） | OpenTCS XML → 内存模型；点/边查询；控制点 → 世界坐标折线 | 不感知 VDA5050 Order 结构 |
| **utils** | 纯函数：距离、NURBS 步进（现有） | 不读文件、不持状态 |

### 5.2 建议目录结构（实现阶段）

```
src/
  main.rs
  lib.rs                    // pub mod map; pub mod navigation;
  config.rs                 // 扩展：#[map] 段
  vehicle_simulator.rs      // 变薄：委托导航层
  mqtt_handler.rs           // 预期不变
  mqtt_utils.rs             // 不变
  utils.rs                  // 保持；可选仅导出给 navigation 使用
  map/
    mod.rs                  // re-export MapModel, load_from_path
    model.rs                // Point, Path, MapModel 数据结构
    opentcs_xml.rs          // 仅解析 XML → model（单责）
    layout.rs               // layoutScaleMm / flipY 等变换（单责）
  navigation/
    mod.rs
    traits.rs               // MapView, MotionStepper（见 §11）
    map_motion.rs           // 结合 MapModel + Order 边计算下一步
    speed_limits.rs         // min(订单 maxSpeed, 地图, config) 纯函数
```

### 5.3 启动与线程

- **启动时**：`main` 调用地图加载；失败则注入 **空实现**（`Option<Arc<dyn MapView>>` 为 `None` 或 `NoOpMap`），行为与 **当前无地图** 一致。
- **并发**：地图只读，`Arc` 共享；每车 `VehicleSimulator` 私有 **运动标量状态**（当前边、弧长、上一节点时间戳等），不放入全局可变静态变量。

### 5.4 多模拟器 / 多车（`robot_count` > 1）是否有影响？

**结论：按当前设计实现时，不应互相干扰；与现有 `main.rs` 多车启动方式兼容。**

| 维度 | 说明 |
|------|------|
| **地图数据** | 全进程 **一份只读** `Arc<MapModel>`（或 `Arc<dyn MapView>`），所有 `VehicleSimulator` **共享同一引用** 即可。解析只做一次，多车并发读无数据竞争。 |
| **运动状态** | 弧长、当前边、订单进度等必须在 **每个 `VehicleSimulator` 实例内部** 持有（如 `MotionContext`），**不得**放进 `MapModel` 或全局 `static`。设计已要求「每车私有」——满足则多车轨迹独立。 |
| **MQTT** | 现有逻辑按 `serial_number` 区分 topic（`spawn_vehicle_simulator` 中为每车追加序号），多车不会抢同一 topic。地图功能不改变 topic 规则。 |
| **异步任务** | 每车仍是独立的 `Arc<Mutex<VehicleSimulator>>` + 独立 subscribe/publish 任务；地图句柄用 `Arc::clone` 传入，**O(1)**，无额外拷贝大地图。 |
| **需注意的反模式** | 若在地图层缓存「当前车在哪个点」等 **可变仿真状态**，会破坏多车；应禁止。若未来做「地图热更新」，需用 `ArcSwap`/`RwLock` 等新 Arc 替换，并保证读路径仍 `Send + Sync`。 |

**实现检查清单（编码时）**：`MotionContext` 仅存在于 `VehicleSimulator`；`map` 模块无 `Mutex` 包裹的仿真字段；`main` 对每车 `VehicleSimulator::new(config_i, Some(map_arc.clone()))`。

---

## 6. SOLID 原则与本特性的对应关系

### S — Single Responsibility（单一职责）

- **`opentcs_xml.rs`**：只负责把 DOM/解析结果变成 `MapModel`，不负责运动。
- **`layout.rs` / `geometry`**：只负责坐标变换与折线离散，不负责 MQTT。
- **`map_motion.rs`**：只负责「一步仿真」的位姿与速度，不负责 `publish_state`。
- **`vehicle_simulator.rs`**：只负责订单/动作状态机与 **调用** 导航接口，不内联 XML 字符串处理。

### O — Open/Closed（开闭）

- 新增「贝塞尔离散更密」「映射表」时：扩展 `navigation` 或新增 `impl MapView`，**尽量不修改** `VehicleSimulator::accept_order` 的整体结构。
- 对 MC 行为保持开放：通过 **策略枚举**（如 `NodePositionSource::FromOrder | FromMap | Merge`）配置化，而非散落 `if`。

### L — Liskov Substitution（里氏替换）

- 若定义 `trait MapView { fn point(&self, id) -> Option<Point>; fn path(&self, id) -> Option<PathGeom>; }`，则 **`OpenTcsMap`** 与 **`EmptyMap`**（无地图回退）应对调用方可替换，且 **不破坏** `update_vehicle_position` 的契约（无地图时退化为现有直线/NURBS 逻辑）。

### I — Interface Segregation（接口隔离）

- 不把「解析 + 查询 + 运动 + 日志」塞进一个巨大 trait。
- 建议细分为：`MapView`（只读拓扑/几何查询）、可选 `IdMapping`（VDA id ↔ OpenTCS 名），导航层只依赖其需要的最小接口。

### D — Dependency Inversion（依赖倒置）

- **`VehicleSimulator`** 依赖 **`trait MotionStepper` 或 `trait MapView`**（由 `main` 注入具体类型），而不是依赖 `opentcs_xml` 模块。
- **单元测试** 中注入内存构造的 `MapModel` 或 mock，避免测试必须读真实 XML 文件。

---

## 7. 须修改 / 新增的文件与函数清单（待确认后编码）

以下按 **文件** 列出：主要 **符号**、**改动性质**、**原因**（与分层、SOLID 的关系）。

### 7.1 新增 `src/map/*`（无改动现有文件中的逻辑，仅新增模块）

| 符号（预期） | 职责 |
|--------------|------|
| `MapModel`, `Point`, `PathRecord` | 地图领域模型（`map/model.rs`） |
| `load_map_from_path(Path, &MapLayoutConfig) -> Result<MapModel, Error>` | 入口；失败由 main 处理（`map/mod.rs` 或 `opentcs_xml.rs`） |
| `parse_opentcs_model_xml(&str) -> MapModel` | 纯解析（`opentcs_xml.rs`） |
| `layout_transform(cp, &MapLayoutConfig) -> (f64, f64)` | layout→世界（`layout.rs`） |

**原因**：满足 **S**（解析与变换分离）、**D**（模拟器不依赖 XML 细节）。

### 7.2 新增 `src/navigation/*`

| 符号（预期） | 职责 |
|--------------|------|
| `trait MapView` | 点/路径几何查询（**I** 小接口） |
| `struct OpenTcsMapView { inner: Arc<MapModel> }` | 实现 `MapView` |
| `struct MotionContext { ... }` | 当前 sequence、边、是否 NURBS、弧长 s 等 |
| `fn step_motion(ctx, order_edge, agv, dt, cfg) -> MotionResult` | 单步结果：新坐标、theta、v、是否到达下一节点 |
| `fn resolve_speed_limit(edge, path_meta, settings) -> f32` | **speed_limits.rs**，纯函数 |

**原因**：把 **`calculate_new_position` 内的分支** 迁出 `VehicleSimulator`（**S**、**O**）。

### 7.3 `src/config.rs`

| 符号 | 改动 |
|------|------|
| `pub struct MapConfig { enabled, xml_path, layout_scale_mm, layout_flip_y, id_mapping_mode, ... }` | 新增 |
| `pub struct Config` | 增加 `pub map: MapConfig`（或 `Option<MapConfig>`） |
| `get_config()` | 反序列化新段；缺省使 `map.enabled = false` 与现行为一致 |

**原因**：配置集中（**S**），避免魔法路径散落在 `main`。

### 7.4 `src/main.rs`

| 符号 | 改动 |
|------|------|
| `main()` | 在 `get_config()` 后：若 `map.enabled`，则 `load_map` → `Arc<MapView>`；否则 `None` |
| `spawn_vehicle_simulator(...)` | 增加参数 `map: Option<Arc<dyn MapView + Send + Sync>>`（或泛型由具体类型擦除为 `Arc<OpenTcsMapView>`） |
| `VehicleSimulator::new(...)` | 传入地图句柄 |

**原因**：**组合根** 装配依赖（**D**）。

### 7.5 `src/vehicle_simulator.rs`（核心改动集中区）

| 符号 | 改动性质 | 原因 |
|------|----------|------|
| `VehicleSimulator` 结构体 | 增加字段：`map: Option<Arc<dyn MapView + Send + Sync>>`，`motion: MotionContext`（或等价私有状态） | 持有依赖而非全局（**D**） |
| `pub fn new(config, map)` | 签名扩展 | 注入 |
| `accept_order` / `process_order_nodes` | **可选**：在节点无 `node_position` 时，用 `MapView` 补全 `NodeState.node_position`（与 MC 一致策略可配置） | 满足 §3.2 |
| `calculate_new_position` | **重构为薄封装**：调用 `navigation::step_motion`；保留对 `utils::iterate_position*` 的委托（NURBS 路径不变） | **S**、**O** |
| `update_vehicle_position` | 使用 `step_motion` 返回值更新 `agv_position`；根据结果维护 `last_node_id` / 弹出 `node_states`；设置 **`driving`**、**`distance_since_last_node`**、**`velocity`**（当前 `State` 已有字段，多为未填） | 协议完整性与地图一致 |
| `process_order_edges` | 可选：校验 `start_node_id`/`end_node_id` 与地图 path 是否一致（日志或拒绝） | §9 可观测性 |
| `create_initial_state` | 若配置 `initial_point_name`，可从地图读初始坐标（可选 M2+） | 与 Java 对齐 |

**不计划修改**（除非协议需要）：`publish_state`、`publish_visualization`、`mqtt_handler` 调用方式。

### 7.6 `src/utils.rs`

| 符号 | 改动 |
|------|------|
| `iterate_position`, `iterate_position_with_trajectory`, NURBS 内部函数 | **默认保持不动**；由 `navigation` 调用 |
| （可选）抽取 `pub fn distance(x1,y1,x2,y2) -> f32` 若尚未统一 | 减少重复 |

**原因**：工具函数无状态（**S**）；避免把地图逻辑混入 **utils**（**S**）。

### 7.7 `src/lib.rs`

| 改动 |
|------|
| `pub mod map;` `pub mod navigation;` |

**原因**：集成测试与库导出。

### 7.8 `src/mqtt_handler.rs` / `mqtt_utils.rs` / `protocol/*`

| 预期 | 不变，除非 `Visualization` 需强制字段（一般已有 `agv_position`） |

### 7.9 `config.toml` 与 `Cargo.toml`

| 文件 | 改动 |
|------|------|
| `config.toml` | 增加 `[map]` 示例段（`enabled = false` 默认） |
| `Cargo.toml` | 增加 XML 解析依赖（如 `quick-xml` + `serde` 或手写解析）；按需 `thiserror` |

### 7.10 `tests/vehicle_simulator_tests.rs`

| 改动 |
|------|
| 构造 `VehicleSimulator::new` 时传入 `map: None`；新增 `navigation` / `map` 的单元测试文件 |

---

## 8. 调用链（数据流）简图

```
Order (MQTT) → mqtt_handler::handle_incoming_message
            → VehicleSimulator::process_order
            → accept_order / process_order_nodes (+ 可选地图补全 node_position)
            → update_state (tick)
                 → update_vehicle_position
                      → navigation::step_motion
                           → [有 trajectory] utils::iterate_position_with_trajectory
                           → [无 trajectory + 有地图路径] 折线/贝塞尔弧长步进
                           → [否则] utils::iterate_position
                      → 写回 state.agv_position, driving, velocity, distance_since_last_node
            → publish_state / publish_visualization
```

---

## 9. 错误处理与可观测性

- XML 解析失败：明确错误信息，回退无地图模式。
- 订单中 `nodeId` / `edgeId` 在地图中不存在：策略可选 — **拒绝订单** 或 **降级为订单内坐标直线**（建议可配置，默认降级并 `warn!`）。
- 日志：当前边名、弧长进度、选用速度 `v` 的分项值（便于与 Java 对比）。

---

## 10. 测试建议

- **单元测试**：已知两点一直线，固定 `v` 与 `dt`，断言若干步后坐标与终点距离。
- **黄金数据**：从 Java 模拟器导出同订单若干时刻的 `(x,y)`，Rust 侧对比容差。
- **XML fixture**：使用仓库内 `maps/youle-final-4.xml` 子集构造最小模型测试解析。

---

## 11. 交付节奏（建议分阶段）

| 阶段 | 内容 |
|------|------|
| **M1** | 解析 XML → `MapModel`；配置 `xml_path`；单位换算与世界坐标。 |
| **M2** | 订单边匹配地图 path；无 NURBS 时沿折线/直线积分；速度取 min。 |
| **M3** | `distance_since_last_node`、`driving`、`velocity` 等与协议字段对齐；日志与黄金对比。 |
| **M4** | BEZIER 离散、映射表、拓扑校验、加减速模型（可选）。 |

---

## 12. 待你确认的问题

1. **MC 下发的 `nodeId` / `edgeId`** 是否与 OpenTCS 中 **`point.name` / `path.name` 完全一致**？若否，倾向 **映射表** 还是 **命名规则**？
2. **`maxVelocity` 单位** 在你导出环境中是否确认为 **mm/s**？（部分项目为 mm/s 或内部单位。）
3. **`pathLayout` 控制点** 是否与 Java 侧使用同一套 **`layoutScaleMm` / `layoutFlipY`**？请提供 Java 中的默认值或配置文件片段，以便 Rust **逐点一致**。
4. 订单中 **带 NURBS `trajectory` 的边** 是否仍占大多数？若是，**M2 可优先保证 NURBS 路径不变**，地图仅补全无 trajectory 的边。

---

**请确认或修订以上设计（尤其 §5–§7、§12 与 SOLID 相关约定）。确认后按 §11 分阶段实现。**
