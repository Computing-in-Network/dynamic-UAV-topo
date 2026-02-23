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

## Fire Mission MVP（FDS 接入预演）

说明：当前已支持两类输入：

- `fire_adapter_demo.py`：快速演示用模拟火情源。
- `fire_adapter_fds.py`：支持样例 JSON/CSV 与真实 FDS `*_devc.csv` 输入。

已实现可视化叠加：

- 火情热点图层（`fire_hotspots`）：地图显示火点标记、地面影响圈、强度颜色。
- 任务图层（`mission_targets`）：显示 UAV 到任务目标的虚线。
- 右上角状态摘要：`fire`、`max_fire`、`mission`、`basemap`。
- 火区扩展图层：根据热点强度与扩散速度显示动态火区面（`fire_area_km2` 估计值）。
- 图层开关：支持热点点/火区面/任务线显隐切换。
- 火情图例：侧栏显示高危/中危/低危阈值（当前阈值：`>=0.85`、`0.60-0.84`、`<0.60`）。
- FDS 原始输出接入：可通过 `fds_devc_csv` 模式将 `CHID_devc.csv` 转为 `FireState`，并同步发布 `/env/fds_raw`。
- 前端新增 FDS 原始层面板（`/env/fds_raw`）：显示传感器数量、max/min/avg、每个探针原始值与趋势（相对上一帧）。
- 可视化主次分层（降噪）：默认隐藏“热点点”，保留“任务线+火区面”作为主视图。
- 一键视图预设：`任务视图 / 火情视图 / 调试视图`，用于快速切换图层组合。

原始层 vs 决策层（前端观测口径）：

- 原始层（Raw）：FDS 直接输出（温度/烟浓度等）数值，不做任务语义解释。
- 决策层（Decision）：由规划器生成的可执行结果（热点强度、任务目标、执行状态）。
- 页面中“火圈/任务线”属于决策层；侧栏 “FDS 原始层” 数值块属于原始层。

一键启动：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899
```

可选第 4 个参数：`planner_mode`（`greedy|coverage`），例如：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 coverage
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

任务规划策略（Issue #6）：

- `planner_mode=greedy|coverage`（默认 `greedy`）
- `coverage` 模式新增参数：
- `coverage_revisit_sec`：复访周期约束（秒）
- `coverage_priority_decay`：近期已访问热点的优先级衰减
- `coverage_distance_weight`：距离惩罚权重
- `coverage_continuity_bonus`：连续跟踪同一热点的稳定性加分
- `coverage_retarget_threshold_m`：目标重定向阈值（米，`0` 为关闭保持机制）

离线对比评估脚本：

```bash
./scripts/eval_planner_coverage.py --ticks 120 --seed 42 --uavs 8 --hotspots 10
```

任务执行反馈闭环（Issue #7）：

- 新增话题：`/swarm/mission_status`
- 新增节点：`scripts/mission_status_tracker.py`
- 前端新增执行态势统计：`exec/done/fail`

消息与说明见：`docs/mission-status-loop.md`。

真实 FDS 最小闭环（安装后）：

- 最小场景：`data/fds_cases/min_fire/fds_min_fire.fds`
- 生成 `devc.csv`：`./scripts/fds_run_min_case.sh`
- 启动真实 FDS 输入链路：`./scripts/fire_mission_fds_start.sh 6 6 8913 coverage`
- 说明文档：`docs/fds-min-loop.md`

FDS 端到端压测（Issue #8）：

- 脚本：`./scripts/stress_fds_pipeline.sh 20 20 8913 20 20.0`
- 指标：`api_rtt`、`state_hz`、`plan/exec/done latency`、拓扑 profile
- 文档：`docs/fds-stress.md`

FDS 故障注入恢复（Issue #9）：

- 脚本：`./scripts/test_fire_adapter_recovery.sh 6 6 8913 coverage`
- 注入：断流、坏包、时钟漂移
- 文档：`docs/fds-fault-injection.md`

统一验收入口（Issue #23）：

- 汇报与演示话术：`docs/release-handover-2026-02-23.md`

- 一键脚本：`./scripts/run_full_validation.sh`
- 阶段报告：`docs/validation-report-2026-02-23.md`

可视化地址与底图模式：

- 默认：`http://127.0.0.1:8899/cesium`
- 强制在线底图：`http://127.0.0.1:8899/cesium?basemap=osm`
- 强制网格底图：`http://127.0.0.1:8899/cesium?basemap=grid`

真实 FDS CSV 接入示例：

```bash
./scripts/fire_adapter_fds_start.sh \
  data/fds_samples/fds_case_devc.sample.csv \
  fds_devc_csv \
  2.0 \
  /env/fire_state \
  offline \
  data/fds_samples/stream \
  .json \
  /tmp/fire_adapter_fds_checkpoint.json \
  data/fds_samples/fds_devc_mapping.sample.json
```

详细操作手册见：`docs/fds-integration.md`、`docs/fire-adapter-fds.md`。
统一场景规范见：`docs/fds-unified-scenario.md`（建议后续演示与回归都按该文档执行）。

## 备注

- 拓扑分析当前为阶段算法（距离 + 高度差遮挡近似），已通过统一 `links` 接口输出。

## 协作流程（Git Flow）

- 详细规范见：`docs/git-flow.md`
- Issue / Commit / PR 统一使用中文，并写清背景、改动、验证。

建议本地启用中文 commit 模板：

```bash
git config commit.template .gitmessage-zh.txt
```
