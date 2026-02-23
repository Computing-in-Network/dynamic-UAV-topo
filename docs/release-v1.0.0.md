# Release v1.0.0（首发版）发布说明

发布日期：2026-02-23  
分支基线：`develop`  
版本标签：`v1.0.0`

## 1. 版本定位

`v1.0.0` 是本项目首个“可演示、可回归、可扩展”的里程碑版本，完成了从 FDS 数据输入到 UAV 任务跟踪再到前端态势呈现的工程闭环。

核心目标：
- 跑通 `FDS -> Fire Adapter -> Mission Planner -> UAV 跟飞 -> Web 可视化` 全链路；
- 形成可复现的一键启动与验收方式；
- 建立后续算法迭代可依赖的统一场景基线（S1）。

## 2. 核心能力清单

### 2.1 FDS 接入与数据链路
- 支持 `*_devc.csv` 直接接入（`fds_devc_csv` 模式）。
- 兼容 FDS 常见 CSV 头部格式差异（如 `Time` 行不在首行）。
- 同步发布：
  - 决策层火情：`/env/fire_state`
  - 原始层数据：`/env/fds_raw`

### 2.2 任务规划与执行反馈
- 支持 `greedy` 与 `coverage` 两类规划策略。
- 增强多机跟飞能力：
  - 热点稀疏场景下允许多 UAV 协同跟随；
  - 加入负载惩罚与软容量约束，降低“单热点拥挤”。
- 执行反馈闭环：
  - `mission_status`（执行中/完成/失败、进度、原因）可实时回显。

### 2.3 可视化（Cesium + Deck.gl）
- 火情热点改为火焰 icon；
- 火点贴地渲染（地面火点簇）；
- 火情区域支持聚类并以平滑非圆边界展示；
- 图层主次分层：
  - 任务视图 / 火情视图 / 调试视图 一键切换；
- 状态栏指标：
  - `uav_count / link_count / fire / observed_fire / fire_area_km2 / mission / exec / done / fail / basemap / terrain`。

### 2.4 稳定性与运维
- 一键启动/停止脚本完善：
  - `fire_mission_demo_start/stop.sh`
  - `fire_mission_fds_start.sh`
- 统一验收脚本可执行并输出阶段性 PASS/FAIL 结论。
- 提供故障注入与恢复验证脚本（断流、坏包、时钟漂移场景）。

## 3. 统一场景基线（S1）

为保证结论可比，本版本固定采用 S1 场景口径：
- 文档：`docs/fds-unified-scenario.md`
- Case：`data/fds_cases/min_fire/fds_min_fire.fds`
- CSV：`data/fds_cases/min_fire/fds_min_fire_devc.csv`
- Mapping：`data/fds_cases/min_fire/fds_min_fire_devc_mapping.json`

后续版本新增场景请使用 `S2/S3...` 编号，避免覆盖 S1 基线。

## 4. 快速开始（推荐流程）

### 4.1 构建
```bash
./scripts/ros2_build.sh
```

### 4.2 生成最小 FDS 输出
```bash
./scripts/fds_run_min_case.sh
```

### 4.3 启动联调链路（6 架机）
```bash
./scripts/fire_mission_fds_start.sh 6 6 8914 coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  4.0 true 10 20.0
```

### 4.4 打开页面
- `http://127.0.0.1:8914/cesium?basemap=osm`

### 4.5 停止
```bash
./scripts/fire_mission_demo_stop.sh 8914
```

## 5. 验证与验收

### 5.1 统一验收
```bash
./scripts/run_full_validation.sh
```

结果摘要：
- 查看：`/tmp/full_validation/summary.txt`

### 5.2 关键观测点
- `/api/swarm_state` 中应有：
  - `uavs`
  - `fire_hotspots`
  - `mission_targets`
  - `mission_status`
  - `fds_raw`
- 页面应可看到：
  - 地面火点簇（火焰 icon）
  - 非圆形火情区域
  - UAV 跟飞任务线与执行状态变化

## 6. 已知限制（v1.0.0）

- UAV 运动仍为“工程化平滑跟踪模型”，非完整飞控/避障轨迹规划；
- 火区边界为几何估计与聚类边界，不等同于真实 CFD 物理边界；
- 真正高保真地形需要可用 Cesium Terrain 服务（无 token 时回退平地）；
- FDS 在个别环境可能受 MPI/OFI 网络限制，需要按文档设置环境变量。

## 7. 升级建议（v1.1.0 方向）

- 将火区边界升级为热力栅格 + 等值面（支持阈值切换）；
- 将 UAV 跟踪升级为轨迹规划器（转弯半径、速度约束、避障）；
- 引入多源观测融合（FDS + UAV 观测反馈反向修正火场状态）；
- 完善 release 测试矩阵（不同机型数量、不同热点密度、不同网络负载）。

## 8. 关联文档

- `README.md`
- `docs/fds-unified-scenario.md`
- `docs/fds-integration.md`
- `docs/fds-stress.md`
- `docs/fds-fault-injection.md`
- `docs/release-handover-2026-02-23.md`
