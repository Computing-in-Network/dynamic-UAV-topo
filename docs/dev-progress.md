# SwarmSim-100 开发进度

## 已完成（P1 基线已固化）

- 多实例进程管理（启动/停止）
- CPU affinity 分配与记录
- `SwarmState` 内部快照聚合
- 实例级运行指标快照（pid/core/launch/restart）
- 健康检查与自动重启（可配置 `max_restarts`）
- 命令模板变量渲染：`{id}` `{index}` `{mavlink_udp_port}` `{mavlink_tcp_port}` `{sim_udp_port}`
- 单元测试：亲和性、命令渲染、生命周期、自动重启
- `swarm_topology_analyzer_node` 初版：
  - 订阅 `/swarm/state_raw`，发布 `/swarm/state`
  - 生成 `links`（距离阈值、权重归一化）
  - 阶段性 `is_occluded` 判定（高度差近似）
  - 可配置遮挡模式：`occlusion_mode=none|altitude_gap`
  - 内置帧耗时统计日志：`avg_ms/max_ms/status`
- 可视化数据流统一为 `SwarmState.links`
- 新增 20 架机拓扑 profiling 脚本：`scripts/profile_topology_20.sh`
- 新增遮挡翻转验证脚本：`scripts/test_occlusion_flip.sh`
- 新增 100Hz 压力测试脚本：`scripts/stress_100hz.sh`（输出 `observed_hz`、`e2e_latency_ms`、`drop_ratio`、`mission_loop_ms`）
- 新增 Fire Mission MVP：
  - `scripts/fire_adapter_demo.py`（模拟 FDS 热点流）
  - `scripts/fire_adapter_fds.py`（FDS 文件适配器，当前默认主路径）
  - `scripts/fire_adapter_fds_core.py`（FDS 输入解析核心模块）
  - `scripts/mission_planner.py`（热点到 UAV 的贪心任务分配）
  - `swarm_uav_manager_node` 支持 `mission_follow_enabled` 按目标点推进
  - 一键脚本：`scripts/fire_mission_demo_start.sh` / `scripts/fire_mission_demo_check.sh`
  - 默认链路已切换为 FDS，`demo` 模式降级为显式回退
  - `fire_mission_demo_check.sh` 已在 FDS 模式下校验 `fire_regions`
- 新增 GPU/高阶拓扑 PoC：
  - `scripts/ricci_cuda_poc.py`（无向图边权共邻居评分 CPU/CUDA 对比脚本）
- P1 验收基线已固化：
  - `tests/test_mission_planner.py`
  - `scripts/test_occlusion_terrain_csv.sh`
  - `scripts/test_occlusion_terrain_fallback.sh`
  - `scripts/test_occlusion_terrain_invalid.sh`
  - `scripts/compare_occlusion_modes.sh`
  - `scripts/stress_100hz.sh`
  - `scripts/p1_acceptance.sh`

## 下一步（按 Spec 推进）

- 详细计划见：`docs/next-plan.md`
- 当前优先级：
  - 将火区显示从圆形近似升级为真实边界多边形
  - 进入 P2 前的发布整理与演示打磨
