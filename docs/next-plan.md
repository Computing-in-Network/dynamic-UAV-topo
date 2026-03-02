# 下一步计划（阶段回顾后）

## P0（先做）

1. FDS 结果接入接口落地（已完成）
- 已完成：
  - `fire_adapter_fds.py` 已成为 Fire Mission 默认主路径
  - 输入契约已明确为文件轮询模式，支持 `normalized` 与 `fds_csv/fds_jsonl` profile
  - 坐标系转换与时间同步策略已在文档中固化
  - 输出保持兼容 `swarm_interfaces/msg/FireState` / `FireRegionState`

2. 火情从“热点点”升级到“扩展区域”（已完成到 P0 验收线）
- 已完成：
  - 火区半径/边界时序模型已接入 `FireRegionState`
  - 前端主链路已显示 `fire_regions`
- 后续增强：
  - 将圆形半径近似升级为真实边界多边形渲染

3. 稳定性与可运维性（已完成到 P0 验收线）
- 已完成：
  - `fire_mission_demo_*` 脚本增加统一日志与失败原因输出
  - 端口选择支持“自动找可用端口”模式
  - 默认 FDS 验收已纳入本地 smoke 基线

4. P0 收尾验收（当前进行中）
- 目标：将现有能力从“可演示”固化为“可重复验收”。
- 产物：
  - 补齐 FDS 适配直接测试与本地 smoke 基线
  - 同步 README / 计划 / 进度文档状态
  - 形成 P0 完整验收步骤

## P1（随后）

1. 任务规划算法升级
- 当前：按状态化评分重构（加入复访、去抖动、覆盖抑制和历史记忆），且仍保持 `MissionPlan` 兼容接口。
- 计划：加入覆盖率/复访周期/优先级衰减，减少抖动和重复覆盖。

2. 拓扑遮挡真实性升级
- 当前：`swarm_topology_analyzer_node` 已新增 `terrain_csv` 模式（场景 CSV 阻塞查询）并回退到 `altitude_gap`。
- 计划：接入地形/场景遮挡接口，保留现有近似模式作 fallback。

3. 压测指标体系完善
- 当前：在 `scripts/stress_100hz.sh` 增加端到端延迟与掉包率统计（p95 延迟、drop_ratio）。
- 计划：增加端到端延迟、掉包率、任务完成时延统计。

## P2（后续扩展）

1. 语义网络模拟节点（Node_Semantic_Net）
- 已完成：新增 `swarm_semantic_net_node`，支持基于 `links.weight` 的可配置丢包、链路降权/剔除、语义降频与延迟仿真。

2. 前端态势增强
- 火情时序回放、图层开关、任务解释（为什么分配给该 UAV）。

3. GPU/高阶拓扑方向
- 2026-02-24：已完成 Ricci/CUDA PoC 评估脚本（`scripts/ricci_cuda_poc.py`），对无向拓扑图进行 CPU/CUDA 执行时延与数值误差对比。
- 后续：结合后端真值场景，评估 Ricci Curvature 在路径规划中的收益/成本，逐步决策是否将 GPU 管线上链。

## 验收里程碑（建议）

1. 里程碑 A：真实 FDS 数据已驱动 `FireState`，并在前端显示扩展火区。（已完成到当前 P0 基线）
2. 里程碑 B：UAV 在 20 架机下持续跟踪火区，100Hz 压测稳定通过。
3. 里程碑 C：语义网络仿真接入并形成“火情-任务-通信-可视化”完整闭环。
