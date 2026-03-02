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

### 语义网络退化演示（非默认）

```bash
./scripts/semantic_net_demo.sh 4 4 8899 0.2 weight weight 0 5 1
```

参数说明：

- 默认退化链路：`packet_drop_rate=0.2`
- 默认 `drop_mode=weight`、`throttle_mode=weight`
- 默认可复现种子：`seed=1`

说明：

- 默认首页为 Cesium + Deck.gl 版本
- 旧版轻量页面：`http://127.0.0.1:8899/legacy`
- 若目标端口被占用，启动脚本会自动回退到附近可用端口并打印实际端口

### 一键停止

```bash
./scripts/visual_demo_stop.sh
./scripts/semantic_net_demo_stop.sh
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

## 20 架机拓扑 Profiling

```bash
./scripts/profile_topology_20.sh 20 20 100 8 20
```

参数：

- 第1个：实例数（默认 `20`）
- 第2个：CPU 核数（默认 `20`）
- 第3个：发布频率 Hz（默认 `100`）
- 第4个：采样时长秒（默认 `8`）
- 第5个：目标帧耗时 ms（默认 `20`）

输出示例：

- `[profile_topology_20] ... avg_ms=... max_ms=... status=PASS`
- `[profile_topology_20] PASS: 拓扑单帧耗时满足目标(<20ms)`

## 遮挡翻转验证

```bash
./scripts/test_occlusion_flip.sh
```

参数：

- 第1个：输入 Topic（默认 `/swarm/test_state_raw`）
- 第2个：输出 Topic（默认 `/swarm/test_state`）
- 第3个：遮挡高度差阈值米（默认 `35.0`）
- 第4个：单阶段超时秒（默认 `8`）

通过标准：

- 输出 `occluded_first=True occluded_second=False`
- 输出 `[test_occlusion_flip] PASS: is_occluded 已从 true 翻转为 false`

## 100Hz 压力测试

```bash
./scripts/stress_100hz.sh 20 20 100 10 20.0
```

参数：

- 第1个：实例数（默认 `20`）
- 第2个：CPU 核数（默认 `20`）
- 第3个：目标发布频率 Hz（默认 `100`）
- 第4个：采样时长秒（默认 `10`）
- 第5个：拓扑帧耗时目标 ms（默认 `20.0`）

输出包含：

- 拓扑 profile（`avg_ms/max_ms/status`）
- `/swarm/state` 实测频率（`observed_hz`）
- 系统 context switch 频率（`context_switch_hz`）

## Ricci/CUDA PoC 评估（#10）

```bash
./scripts/ricci_cuda_poc.py --nodes 600 --density 0.05 --seed 2026 --repeat 6 --warmup 1
```

可复现参数：

- `--nodes`: 节点数，默认 `600`
- `--density`: 无向图边密度（`[0,1]`）
- `--seed`: 随机数种子
- `--repeat`: 计时重复次数（去除 `--warmup` 预热）
- `--warmup`: 预热次数（不计入指标）
- `--disable-cuda`: 无需 CuPy 时可关闭 CUDA 通道

默认输出为 JSON，包括：

- `backend.cpu`：CPU 平均/中位数/95 分位时延
- `backend.cuda`：CUDA（若有）平均/中位数/95 分位时延及可用状态
- `graph`：边数和实际密度
- `accuracy.max_abs_diff`：CPU 与 CUDA 结果最大绝对差

更多说明见：`docs/ricci-cuda-poc.md`

## Fire Mission MVP（FDS 接入预演）

说明：当前先用 `fire_adapter_demo.py` 模拟 FDS 输出热点流，后续可替换为真实 FDS 解析器。

当前已提供首版 FDS 文件适配器：`scripts/fire_adapter_fds.py`（详见 `docs/fds-fire-adapter.md`）。
P0 当前已明确两类输入契约：

- `input_profile=normalized`：仓库内部规范化样例格式
- `input_profile=fds_csv`：更接近真实 FDS 导出表的热点单元格式

对应近真实样例见：`docs/examples/fds_cells_export_sample.csv`

已实现可视化叠加：

- 火情热点图层（`fire_hotspots`）：地图显示火点标记、地面影响圈、强度颜色。
- 火区面图层（`fire_regions`）：按热点聚类与扩散速度估算连续火区半径。
- 任务图层（`mission_targets`）：显示 UAV 到任务目标的虚线。
- 右上角状态摘要：`fire`、`max_fire`、`mission`、`basemap`。

一键启动：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899
```

使用 FDS 文件源启动：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv
```

一键停止：

```bash
./scripts/fire_mission_demo_stop.sh 8899
```

一键验收：

```bash
./scripts/fire_mission_demo_check.sh 8899 4 4
```

通过标准：

- 输出 `mission_msgs=... target_count=...`
- 输出 `tracked_uav=... moved_m=...` 且位移明显大于 0
- 输出 `[fire_mission_demo_check] PASS: 火场任务链路有效`

可视化地址与底图模式：

- 默认：`http://127.0.0.1:8899/cesium`
- 强制在线底图：`http://127.0.0.1:8899/cesium?basemap=osm`
- 强制网格底图：`http://127.0.0.1:8899/cesium?basemap=grid`
- 关闭火区面图层：`http://127.0.0.1:8899/cesium?fire_region=off`

双视角演示预留脚本（Gazebo 特写 + 前端全景）：

```bash
./scripts/dual_view_demo_start.sh 4 4 8899 mock
./scripts/dual_view_demo_stop.sh
```

双视角同步验收：

```bash
./scripts/dual_view_demo_check.sh 8899 2 2 mock 30.0 2500
```

布局建议与排障说明见：`docs/dual-view-demo.md`

## 备注

- 拓扑分析当前为阶段算法（距离 + 高度差遮挡近似），已通过统一 `links` 接口输出。

## 协作流程（Git Flow）

- 详细规范见：`docs/git-flow.md`
- Issue / Commit / PR 统一使用中文，并写清背景、改动、验证。

## 持续集成（CI）

### 触发

`develop` 分支的 push 与 PR 会自动触发 `Python Smoke CI`。

### 内容

- `python3 -m py_compile scripts/*.py tests/*.py`
- 所有 `tests/test_*.py` 逐个执行
- `scripts/ricci_cuda_poc.py --disable-cuda --nodes 12 --density 0.2 --repeat 2`

### 本地发布前检查

```bash
./scripts/release_smoke.sh
```

用于离线快速复核（无需 ROS/FDS/PX4/CUDA），结果中返回 `PASS: release smoke` 说明通过。

可选参数：

```bash
./scripts/release_smoke.sh --ros      # 强制执行 ROS2 构建 smoke
./scripts/release_smoke.sh --no-ros   # 强制跳过 ROS2 构建 smoke（默认）
./scripts/release_smoke.sh --help
```

建议本地启用中文 commit 模板：

```bash
git config commit.template .gitmessage-zh.txt
```
