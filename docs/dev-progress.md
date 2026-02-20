# SwarmSim-100 开发进度

## 已完成（Node_UAV_Manager + Topology Analyzer v0.3）

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

## 下一步（按 Spec 推进）

1. 将 `is_occluded` 从高度差近似切换到真实地形/场景遮挡检测接口。
2. 为 `Node_Topology_Analyzer` 增加 20 架机单帧耗时 profiling 脚本（目标 < 20ms）。
3. 新增遮挡翻转测试场景（固定两机轨迹，验证 `is_occluded` 翻转）。
