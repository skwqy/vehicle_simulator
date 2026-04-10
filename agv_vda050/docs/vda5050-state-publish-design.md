# VDA5050 state 报文发送 — 设计方案（待确认）

**版本**: 草案 v2  
**依据规范**: VDA5050 release 2.1.0（本地路径 `D:\source\yeefung\vda5050\VDA5050-release-2.1.0`，以其中 `VDA5050_EN.md` / `json_schemas/state.schema` 为准）；**周期与事件细则**以下文 **§3 检查清单**（中文实现说明 / 协议培训材料）为产品基线。  
**状态**: 仅设计归档，**确认后再实施编码**

---

## 1. 问题陈述

当前实现中，`state` 主题的发送行为与规范期望存在差距，主要包括：

1. **定时语义**：规范要求车辆在**固定周期**上报状态，并允许在**状态变化**时额外上报；现有逻辑虽用 `state_frequency` 与固定 `tick_time` 做节流，但周期计算方式与仿真步长耦合，且**未**在关键状态变化时显式补发。
2. **MQTT QoS**：规范 6.2 节要求 `state` 使用 **QoS 0（Best Effort）**；当前 `mqtt_publish` 对 JSON 发布统一使用 **QoS 1**，与规范不一致。
3. **与 factsheet 的协调**（若后续实现 factsheet）：规范在 factsheet 中定义 `minStateInterval`、`defaultStateInterval` 等，可与 §3 的 **30 秒** 及事件合并策略对照；若 factsheet 与 §3 冲突，以 **客户签字基线** 为准。

---

## 2. 规范要点摘录（2.1.0 英文主文档）

| 来源 | 要求 |
|------|------|
| §4 总则（约 L173） | 车辆应在 **regular interval（固定间隔）** 上报状态，**或**在 **status changes（状态变化）** 时上报。 |
| §6.2 MQTT QoS（约 L323–324） | 为降低开销，`order`、`instantActions`、`state`、`factsheet`、`visualization` 应使用 **QoS 0**；`connection` 使用 **QoS 1**。 |
| §6.10.6 state 载荷（L1011+） | `headerId` 每发送一条递增；`timestamp` 为 ISO 8601 UTC；字段集合以 `state.schema` 为准。 |
| §6.8.2 / instantActions 表（约 L893） | `stateRequest`：请求 AGV **再发送一条** state 报告（实现后应在收到时立即 publish state）。 |

（中文镜像见同目录 `VDA5050_ZH.md`，条文应对应一致。）

---

## 3. 与实现对齐的检查清单（30 秒周期 + 事件 + 合并）

以下条款作为模拟器 / 集成验收的**明确触发条件**（与 §2 总则一致，并细化数值与事件类型）。

### 3.1 周期上报（心跳）

| 编号 | 要求 |
|------|------|
| C-01 | AGV 通过 MQTT 向主控发布 `state` 时，在**无其它触发**的情况下，**至少每 30 秒**发布一次（即**相邻两条 state 的间隔不得超过约 30 秒**，除非另有事件提前发送）。 |

说明：英文主文档常用「regular interval」而不写死 30s；**30 秒**来自中文实现说明 / 培训材料中的可操作基线。若后续 factsheet 中 `defaultStateInterval` 不同，以实现 factsheet 为准并更新本表。

### 3.2 事件触发（须立即发送 state）

下列**任一**情况发生时，应在合理延迟内发送 `state`（可与 §3.3 合并为一条）：

| 编号 | 场景 | 说明 |
|------|------|------|
| E-01 | **订单** | 收到**新订单**，或对**已有订单的更新**。 |
| E-02 | **载荷** | **载货 / 载荷状态**发生变化（如取货、放货）。 |
| E-03 | **错误与警告** | 出现**新的**错误或警告条件。 |
| E-04 | **节点** | 车辆行驶**经过 / 越过**某一节点（与 `lastNodeId`、`nodeStates` 更新一致）。 |
| E-05 | **运行模式** | **操作模式**（`operatingMode`）发生变化。 |
| E-06 | **字段变化** | `driving` 发生变化。 |
| E-07 | **字段变化** | `nodeStates`、`edgeStates` 或 `actionStates` 中任一类发生有意义的变化。 |
| E-08 | **字段变化** | `maps`（若实现）发生变化。 |
| E-09 | **即时动作** | 收到 **`stateRequest`** instantAction 时，应再发一条 state（与 §2 一致）。 |

### 3.3 合并与流量控制

| 编号 | 要求 |
|------|------|
| M-01 | 在**尽量限制通信量**的前提下，若同一时刻多类事件同时相关（例如新订单通常同时更新 `nodeStates` 与 `edgeStates`；过节点可能同时带动多处字段），应**合并为单次** `state` 发布，而非为每个子变化各发一条。 |
| M-02 | 事件触发与 30 秒定时若在同一时间窗口内重叠，**只发一条**即可，但须满足「字段已反映最新状态」与「30 秒内至少有一条」的约束。 |
| M-03 | 若与 factsheet `minStateInterval` 联动，事件连发须不低于该最小间隔（产品可配置；未实现 factsheet 时可用本地默认）。 |

### 3.4 主题与负载其它说明（来自同一材料）

- AGV 状态集中在**单一 `state` 主题**，减轻主控与车载侧处理负担，并保证各块信息**同步**。

---

## 4. 现状摘要（代码，便于对照）

| 项目 | 当前行为 |
|------|----------|
| 发布循环 | `mqtt_handler::publish_vda_messages`：`tick_time = 50` ms 固定循环；每 tick 调用 `update_state()`；用计数器按 `state_frequency` 决定是否 `publish_state`。 |
| 配置 | `settings.state_frequency`（Hz，含义为「目标频率」）。 |
| 状态变化 | 未在订单接受、节点到达、动作完成等事件单独触发 `publish_state`（仅依赖下一次 tick 周期）。 |
| MQTT QoS | `mqtt_utils::mqtt_publish` 使用 `QOS_1` 发布所有 JSON。 |
| §3 检查清单 | **已实现（代码）**：`state_max_interval_secs` 心跳、`state_publish_pending` 事件、`state`/`visualization` QoS 0、`connection` QoS 1；合并依赖单 pending 标志。 |

---

## 5. 设计目标（对齐 §3）

1. **周期**：满足 **C-01**（默认最长间隔 30s；若配置更高频，可短于 30s，但不得长于 30s 无报文）。  
2. **事件**：实现 **E-01～E-09** 中当前模拟器已具备状态机的部分（订单、节点、载荷、错误、模式、`stateRequest` 等），并在触发时标记「待发送」或立即发送。  
3. **合并**：遵守 **M-01～M-03**。  
4. **QoS**：`state` 等为 **QoS 0**，`connection` 为 **QoS 1**（见 §2）。  
5. **仿真步长**：`update_state` 与 `publish_state` **解耦**（周期/事件驱动发布，不因 50ms tick 误判为「已定时发送」）。

---

## 6. 建议架构（编码阶段）

### 6.1 双通道驱动

- **定时器**：`interval(Duration::from_secs(30))`（或配置为 `min(T_config, 30)`，以不超过 30s 无 state 为准），到期则 `publish_state`。  
- **事件**：在 `VehicleSimulator` 内关键变更点设置 `state_pending = true` 或调用 `request_state_publish()`，由统一调度在**同一合并窗口**内发一条。

### 6.2 合并实现要点

- 单次订单处理结束、单节点到达：只打一次 dirty，下一循环或短延迟（如 0～50ms）合并发送。  
- 与 §3.3 **M-01** 一致：避免同一逻辑步骤内重复 publish。

### 6.3 MQTT 封装

- 按 topic 选择 QoS；`state` → 0，`connection` → 1。

### 6.4 配置项

| 配置项 | 建议 |
|--------|------|
| `state_max_interval_secs` 或等价 | 默认 `30`，对应 C-01。 |
| `state_frequency` / `minStateInterval` | 可与 factsheet 对齐；高频仅作为「允许更密」的上限，不替代 30s 上限约束。 |

---

## 7. 验收与测试建议（编码阶段）

- **C-01**：长时间无订单、无事件时，订阅 `state` 的**时间戳间隔** ≤ 30s（加时钟容差）。  
- **E-01～E-08**：各触发一次，验证在**一次事件后**收到带正确字段的 `state`（可与定时合并，但须在合理时延内）。  
- **M-01**：同一时刻注入订单+节点更新，统计 MQTT **条数**应为 1 条或符合合并策略。  
- **QoS**：抓包确认 `state` 为 **QoS 0**。  
- Schema：可选对照 `json_schemas/state.schema`。

---

## 8. 待确认项（产品 / 协议）

1. **30 秒**是否与目标客户的 factsheet `defaultStateInterval` 完全一致；若不一致，以谁为准。  
2. **事件触发**是否须受 `minStateInterval` 限制（防主控压力过大）。  
3. 是否在本阶段实现 **`stateRequest`**（E-09）及完整 **`maps`**（E-08）。

---

## 9. 变更影响范围（编码时参考）

- `src/mqtt_handler.rs`：发布循环、定时器、与 simulator 的事件通道。  
- `src/mqtt_utils.rs`：QoS 按 topic 区分。  
- `src/vehicle_simulator.rs`：关键状态变更处标记待发送、合并。  
- `config.toml` / `src/config.rs`：如新增 `state_max_interval_secs` 等。  
- `README`：说明 state 策略与 §3 检查清单对应关系。

---

*文档结束 — 确认 §8 后进入实现。*
