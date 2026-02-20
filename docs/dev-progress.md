# SwarmSim-100 开发进度

## 已完成（Node_UAV_Manager v0.2）

- 多实例进程管理（启动/停止）
- CPU affinity 分配与记录
- `SwarmState` 内部快照聚合
- 实例级运行指标快照（pid/core/launch/restart）
- 健康检查与自动重启（可配置 `max_restarts`）
- 命令模板变量渲染：`{id}` `{index}` `{mavlink_udp_port}` `{mavlink_tcp_port}` `{sim_udp_port}`
- 单元测试：亲和性、命令渲染、生命周期、自动重启

## 下一步（接入 PX4 SITL）

需要你提供：

1. 本机 PX4 SITL 启动命令模板（建议包含上述占位符）
2. PX4 可执行文件/脚本路径
3. ROS2 版本（Humble/Foxy）

收到后可直接实现：

- `Node_UAV_Manager` -> ROS2 发布 `SwarmState.msg`
- 与 `PoseStamped` 输入打通
- 20 架机 profiling 脚本
