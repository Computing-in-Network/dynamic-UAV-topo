# ROS2 集成开发说明

## 目录

- `ros2_ws/src/swarm_interfaces`: 消息定义包
- `ros2_ws/src/swarm_uav_manager`: UAV Manager ROS2 节点包
- `ros2_ws/src/swarm_topology_analyzer`: 拓扑分析 ROS2 节点包

## 消息

- Topic: `/swarm/state_raw`（原始 UAV 状态）
- Topic: `/swarm/state`（拓扑分析后状态）
- Type: `swarm_interfaces/msg/SwarmState`

`UavState.msg` 字段：

- `id`
- `position[3]` (`lat, lon, alt`)
- `velocity[3]` (`vx, vy, vz`)
- `battery`
- `status` (`MISSION/ALARM/LANDED`)

`LinkState.msg` 字段：

- `source`
- `target`
- `weight`
- `is_occluded`

## 构建

```bash
./scripts/ros2_build.sh
```

## 运行（PX4 多实例）

```bash
./scripts/ros2_run_swarm_manager.sh 4
```

参数：

- 第1个：实例数（默认 `4`）
- 第2个：CPU 核数（默认 `nproc`）
- 第3个：状态发布频率 Hz（默认 `5`）
- 第4个：健康检查频率 Hz（默认 `1`）
- 第5个：输出 Topic（默认 `/swarm/state_raw`）

## 可视化演示（浏览器）

```bash
./scripts/visual_demo_start.sh 2 2 8899
```

打开：`http://127.0.0.1:8899`（Cesium + Deck.gl）

旧版页面：`http://127.0.0.1:8899/legacy`

停止：

```bash
./scripts/visual_demo_stop.sh
```

## 一键验收（推荐）

```bash
./scripts/visual_demo_check.sh 8899 2 2
```

预期输出包含：

- `uav_count=2 link_count=...`
- `[visual_demo_check] PASS: 可视化 API 已返回 UAV 数据`

说明：

- 若指定端口被占用，脚本会自动使用附近可用端口并在 PASS 行打印最终端口。

## 20 架机拓扑 Profiling

```bash
./scripts/profile_topology_20.sh 20 20 100 8 20
```

说明：

- 脚本会启动 `swarm_uav_manager_node`（输出 `/swarm/state_raw`）和 `swarm_topology_analyzer_node`（输出 `/swarm/state`）。
- `swarm_topology_analyzer_node` 会按窗口打印 `profile frames=... avg_ms=... max_ms=...`。
- 若最新窗口 `status=PASS`，脚本返回成功。

## 遮挡翻转验证

```bash
./scripts/test_occlusion_flip.sh
```

说明：

- 脚本会构造两阶段双机输入并发送到测试 Topic。
- 第一阶段使用大高度差（期望 `is_occluded=true`），第二阶段使用小高度差（期望 `is_occluded=false`）。
- 输出通过标准为 `occluded_first=True occluded_second=False`。

拓展：拓扑遮挡参数（`swarm_topology_analyzer_node`）

- `occlusion_mode`：支持 `none|altitude_gap|terrain_csv`，默认 `altitude_gap`。
- `occlusion_altitude_gap_m`：高度差阈值（`altitude_gap` 模式）。
- `occlusion_penalty`：遮挡发生时 link 权重削减比例。
- `occlusion_terrain_csv`：`lat,lon,alt` CSV 路径（`terrain_csv` 模式）。
- `occlusion_terrain_clearance_m`：连线到障碍面最小安全高度（`terrain_csv` 模式）。
- `occlusion_terrain_samples`：连线采样点数（`terrain_csv` 模式）。
- `terrain_csv` 加载失败时自动回退到 `altitude_gap`。

`terrain_csv` 输入契约：

- 纯文本 CSV，按 `lat,lon,alt` 三列组织。
- 首行可选表头，允许为 `lat,lon,alt`。
- 允许以 `#` 开头的注释行。
- 每行表示一个场景采样点，节点会在 UAV 连线采样点附近选取最近地形点进行阻塞判定。
- 若文件缺失、为空或无法解析出有效点，节点会记录告警并自动回退到 `altitude_gap`。

示例文件：

- `docs/examples/terrain_occlusion_sample.csv`

常用启动方式：

```bash
TOPO_OCCLUSION_MODE=terrain_csv \
TOPO_OCCLUSION_TERRAIN_CSV=./docs/examples/terrain_occlusion_sample.csv \
./scripts/visual_demo_start.sh 4 4 8899
```

同样的 `TOPO_OCCLUSION_*` 环境变量也可用于 `fire_mission_demo_start.sh` 与 `semantic_net_demo.sh`。

最小验收：

```bash
bash ./scripts/test_occlusion_terrain_csv.sh
```

通过标准：

- 输出 `occluded_low=True occluded_high=False`
- 输出 `[test_occlusion_terrain_csv] PASS: ...`

回退验收：

```bash
bash ./scripts/test_occlusion_terrain_fallback.sh
```

通过标准：

- 输出 `occluded_first=True occluded_second=False fallback_logged=True`
- 输出 `[test_occlusion_terrain_fallback] PASS: ...`

非法数据验收：

```bash
bash ./scripts/test_occlusion_terrain_invalid.sh
```

通过标准：

- 输出 `occluded_first=True occluded_second=False skip_logged=True fallback_logged=True`
- 输出 `[test_occlusion_terrain_invalid] PASS: ...`

模式对比验证：

```bash
bash ./scripts/compare_occlusion_modes.sh
```

通过标准：

- `scenario=clear_high` 下两种模式均为 `false`
- `scenario=terrain_ridge` 下 `altitude_gap=false` 且 `terrain_csv=true`
- `scenario=altitude_gap` 下 `altitude_gap=true` 且 `terrain_csv=false`
- 输出 `[compare_occlusion_modes] PASS: ...`

轻量性能验收：

```bash
bash ./scripts/profile_topology_terrain_csv.sh 20 20 100 8 20
```

通过标准：

- 输出 `status=PASS`
- 输出 `[profile_topology_terrain_csv] PASS: ...`

## 100Hz 压力测试

```bash
./scripts/stress_100hz.sh 20 20 100 10 20.0
```

说明：

- 脚本会启动 manager + topology，并持续采集 `/swarm/state` 消息数。
- 统计结果会输出：`observed_hz`、`context_switch_hz`、拓扑 `profile status`、`e2e_latency_ms(avg/p95)`、`drop_ratio`、`interval_ms(jitter/max)`。
- 当前通过标准：
  - 拓扑 `status=PASS`
  - `observed_hz >= 0.8 * target_hz`
  - `drop_ratio <= 0.20`

## Fire Mission MVP（模拟 FDS 输出）

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899
./scripts/fire_mission_demo_check.sh 8899 4 4
./scripts/fire_mission_demo_stop.sh
```

说明：

- `fire_adapter_demo.py` 按固定频率发布 `/env/fire_state`（热点移动轨迹）。
- `mission_planner.py` 订阅 `/env/fire_state + /swarm/state`，发布 `/swarm/mission_targets`。
- `swarm_uav_manager_node` 在 `mission_follow_enabled=true` 时会按目标点推进 UAV 位置。
- `mission_planner.py` 采用状态化评分策略，默认会在以下约束下生成一次性规划：
  - 最小火强度过滤：`min_intensity`
  - 距离惩罚：`distance_cost_scale`
  - 火情复访增强：`revisit_bonus`、`revisit_cycle_sec`
  - 任务保持稳定：`hold_bonus`、`hold_distance_m`、`hold_time_sec`
  - 切换惩罚：`reassign_penalty`
  - 覆盖散布约束：`coverage_min_distance_m`、`coverage_density_penalty`
- 关键参数可直接通过 ROS 参数注入：
  - `ros2 run ... mission_planner --ros-args -p min_intensity:=0.25 -p replan_hz:=1.0 -p revisit_bonus:=0.25 ...`
- 验收脚本会验证：
  - mission 目标已发布
  - 至少一架被分配任务的 UAV 在观测窗口内产生明显位移
  - mission 完成时延统计（`completion_s`）

### 任务规划稳态回归（P1）

可直接运行：

```bash
python3 tests/test_mission_planner.py
```

覆盖场景：

- 单热点：验证同一 UAV 持续锁定同一目标，不发生切换。
- 热点持续变化：验证冷却窗口抑制短周期改派。
- 多热点/多 UAV：验证相邻热点场景下的重复覆盖抑制。

输出指标：

- `switches`：任务切换次数，期望在单热点场景为 `0`。
- `repeat_assignments`：重复覆盖次数，期望在多 UAV 分散覆盖场景为 `0`。
- `revisit_gap_s`：复访/重规划间隔，用于确认测试输入时序固定。

判定通过：

- 所有断言通过，并输出 `test_mission_planner: PASS`。
- 若执行 `scripts/release_smoke.sh`，该测试会作为 `tests/test_*.py` 的一部分自动执行。

## 语义网络退化模拟

```bash
./scripts/semantic_net_demo.sh 4 4 8899 0.2 weight weight 0 0 1
```

参数说明：

- `PACKET_DROP_RATE`（默认 `0.2`）：`packet_drop_rate`
- `DROP_MODE`（默认 `weight`）：`none|fixed|weight`
- `THROTTLE_MODE`（默认 `weight`）：`none|weight`
- `DELAY_BASE_MS`（默认 `0`）：固定延迟基线（ms）
- `DELAY_SCALE_MS`（默认 `0`）：按权重衰减的最大额外延迟（ms）
- `SEED`（默认 `0`）：复现实验种子

`swarm_semantic_net_node` 主要参数：

- `input_topic`（默认 `/swarm/state`）
- `output_topic`（默认 `/swarm/state_semantic`）
- `packet_drop_rate`、`drop_mode`
- `weight_dropout_scale`、`min_link_weight`
- `jitter_std`
- `topic_rate_min_hz`、`topic_rate_max_hz`
- `delay_base_ms`、`delay_weight_scale_ms`、`delay_jitter_ms`

## 快速查看状态

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /swarm/state --once
ros2 topic echo /swarm/state_raw --once
```
