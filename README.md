# dynamic-uav-topo

动态无人机集群仿真基线工程（ROS2 + PX4 SITL）。

当前阶段目标：

- 建立可运行的多实例 UAV 管理器。
- 打通 ROS2 消息发布链路（`/swarm/state_raw -> /swarm/state`）。
- 提供可视化最小闭环（浏览器实时查看 UAV 状态）。

## 已实现功能

- `Node_UAV_Manager` 核心能力：
  - 多实例进程启动/停止
  - CPU 亲和性分配
  - 健康检查与自动重启
  - 运行时快照与指标
- ROS2 消息接口：
  - `swarm_interfaces/msg/LinkState`
  - `swarm_interfaces/msg/UavState`
  - `swarm_interfaces/msg/SwarmState`
- ROS2 节点：
  - `swarm_uav_manager_node` 发布 `/swarm/state_raw`
  - `swarm_topology_analyzer_node` 订阅 raw 并发布 `/swarm/state`（含 `links`）
- PX4 SITL：
  - 多实例启动可用
- 可视化：
  - `scripts/ros2_visualization_server.py`
  - `web/cesium_deck.html`（Cesium + Deck.gl）
  - `web/index.html`（旧版轻量视图）

## 目录结构

- `src/` + `include/`：本地 C++ 管理器实现
- `ros2_ws/src/swarm_interfaces`：ROS2 消息包
- `ros2_ws/src/swarm_uav_manager`：ROS2 管理节点包
- `ros2_ws/src/swarm_topology_analyzer`：ROS2 拓扑分析节点包
- `scripts/`：构建、运行、验收脚本
- `web/`：前端可视化页面（轻量版）
- `docs/`：阶段文档

## 快速开始

### 1) 构建 ROS2 包

```bash
./scripts/ros2_build.sh
```

### 2) 运行 ROS2 管理节点（PX4 多实例）

```bash
./scripts/ros2_run_swarm_manager.sh 4
```

参数：

- 第1个：实例数（默认 `4`）
- 第2个：CPU 核数（默认 `nproc`）
- 第3个：状态发布频率 Hz（默认 `5`）
- 第4个：健康检查频率 Hz（默认 `1`）
- 第5个：输出 Topic（默认 `/swarm/state_raw`）

## 可视化演示

### 一键启动

```bash
./scripts/visual_demo_start.sh 2 2 8899
```

打开浏览器：`http://127.0.0.1:8899`

说明：

- 默认首页为 Cesium + Deck.gl 版本
- 旧版轻量页面：`http://127.0.0.1:8899/legacy`

### 一键停止

```bash
./scripts/visual_demo_stop.sh
```

### 一键验收（推荐）

```bash
./scripts/visual_demo_check.sh 8899 2 2
```

通过标准：

- 输出 `uav_count=2 link_count=...`
- 输出 `[visual_demo_check] PASS: 可视化 API 已返回 UAV 数据`

说明：

- 若 `8899` 被占用，验收脚本会自动选择附近可用端口并在 PASS 行中打印实际端口。

## 备注

- 拓扑分析当前为阶段算法（距离 + 高度差遮挡近似），已通过统一 `links` 接口输出。
