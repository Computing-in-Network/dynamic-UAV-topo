# FDS 统一场景规范（S1）

本文档用于统一后续 FDS 联调、演示、压测与回归的场景定义，避免“每次场景不同导致结果不可比”。

## 1. 适用范围

- 适用于本仓库所有 `FDS -> Fire Adapter -> Mission Planner -> 可视化` 链路测试。
- 默认场景编号：`S1-min-fire`。
- 后续新增场景必须新增编号（如 `S2-urban-wind`），不得覆盖 `S1`。

## 2. 运行前提条件（必须满足）

1. FDS 环境
- 已安装 FDS6，并可执行 `fds` 命令。
- 已执行：`source /home/zyren/FDS/FDS6/bin/FDS6VARS.sh`

2. ROS2 环境
- 已执行：`source /opt/ros/humble/setup.bash`
- 已执行：`source ros2_ws/install/setup.bash`

3. MPI/OFI 兼容配置（避免已知启动失败）
- `export FI_PROVIDER=tcp`
- `export I_MPI_FABRICS=shm:tcp`
- `export OMP_NUM_THREADS=1`

4. 端口与进程
- 启动前先执行 `./scripts/fire_mission_demo_stop.sh <port>`
- 保证目标端口未占用。

## 3. 场景定义（S1-min-fire）

1. 输入文件
- FDS case：`data/fds_cases/min_fire/fds_min_fire.fds`
- 结果 CSV：`data/fds_cases/min_fire/fds_min_fire_devc.csv`
- 列映射：`data/fds_cases/min_fire/fds_min_fire_devc_mapping.json`

2. 空间与语义
- 使用 mapping 文件中定义的热点位置作为统一语义锚点。
- WGS84 原点与局部坐标转换遵循 `docs/fds-data-contract.md`。

3. 时间与采样
- 统一按 `devc.csv` 的 `Time` 列（秒）驱动。
- Fire Adapter 发布频率默认 `4.0Hz`（演示）或按压测脚本参数覆盖。

## 4. 性能预算（统一口径）

1. FDS 生成预算（S1）
- 目标：最小 case 在开发机上可稳定生成 `devc.csv`。
- 命令：`./scripts/fds_run_min_case.sh`

2. 联动链路预算（S1）
- 拓扑分析：单帧目标 `< 20ms`
- 状态频率：`/swarm/state` 目标 `>= 5Hz`（演示配置）
- API RTT：`/api/swarm_state` 平均应维持低毫秒级

3. 压测口径
- 使用：`./scripts/stress_fds_pipeline.sh ...`
- 以脚本输出结论为准，不手工改判。

## 5. 可视化展示约束（统一演示口径）

1. 默认展示
- 使用“任务视图”作为默认视图（主信息优先）。
- 验证用圆形火情区域默认弱化，仅在调试视图观察。

2. 语义解释
- 原始层：`/env/fds_raw`（探针离散值）
- 决策层：`fire_hotspots/mission_targets/mission_status`

## 6. 标准运行步骤（S1）

1. 生成最小 FDS 输出
```bash
./scripts/fds_run_min_case.sh
```

2. 启动联动链路
```bash
./scripts/fire_mission_fds_start.sh 6 6 8913 coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  4.0 true 10 20.0
```

3. 访问页面
- `http://127.0.0.1:8913/cesium?basemap=osm`

4. 停止
```bash
./scripts/fire_mission_demo_stop.sh 8913
```

## 7. 验收清单（S1）

- `devc.csv` 可生成且列名匹配 mapping。
- `/api/swarm_state` 可返回 `uavs/fire_hotspots/mission_targets/fds_raw`。
- 前端可在预设视图间切换，主信息清晰可读。
- 压测与恢复脚本按统一口径可执行并产生日志。

## 8. 变更管理

- 修改 `S1` 参数需提交单独 PR，说明“变更原因 + 指标影响 + 回滚方案”。
- 大改场景请新增 `S2/S3...`，保留 `S1` 作为稳定基线。
