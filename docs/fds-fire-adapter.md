# FDS Fire Adapter（P0 契约版）

本文档说明如何使用 `scripts/fire_adapter_fds.py` 将 FDS 导出的离线结果接入 `/env/fire_state`。
当前 P0 阶段采用“文件轮询”作为正式输入边界：适配器轮询热点文件和可选区域文件，检测到文件更新时间变化后重新解析并按时间轴回放。

## 1. 输入边界（P0）

P0 阶段约束如下：

- 输入介质：本地文件（后续可扩展到流式输入）
- 热点主输入：1 个热点文件
- 可选区域输入：1 个区域边界文件
- 读取方式：按文件 `mtime/size` 轮询，文件更新后自动重载
- 输出 topic：
  - `/env/fire_state`
  - `/env/fire_region_state`（可选）

推荐目录结构：

```text
fds_exports/
  hotspots.csv
  regions.jsonl
```

也可直接传入仓库内的样例文件。

## 2. 输入格式与 Profile

支持两层概念：

- `input_format` / `region_input_format`：文件编码格式（`csv` / `jsonl`）
- `input_profile` / `region_input_profile`：字段契约（如何解释列名）

默认播放模式为 `timeline`，会按 `time_s` 连续回放热点序列。

### 2.1 `normalized`（默认，规范化输入）

这是仓库内部统一格式，适合样例、联调或预处理后的中间文件。

热点 CSV 推荐包含表头，字段如下：

- `time_s`：仿真时间（秒）
- `id`：热点标识
- `x_m`：FDS 局部坐标 X（米）
- `y_m`：FDS 局部坐标 Y（米）
- `z_m`：FDS 局部坐标 Z（米）
- `intensity`：强度（0~1）
- `spread_mps`：扩散速度（米/秒）

示例见：`docs/examples/fds_hotspots_sample.csv`

区域 JSONL（默认 `jsonl`）每行一个 region：

- `time_s`
- `id`
- `intensity`
- `spread_mps`
- `vertices`: `[[x_m, y_m, z_m], ...]` 至少 3 个点

示例见：`docs/examples/fds_regions_sample.jsonl`

### 2.2 `fds_csv`（更接近真实 FDS 导出表）

用于承接一类“由 FDS 原始结果导出的热点单元表”。当前版本支持以下常见字段别名：

- 时间：`time` / `time_s` / `sim_time_s`
- 单元标识：`cell_id`
  若不存在，则由 `grid_i/grid_j/grid_k` 自动生成 `cell_<i>_<j>_<k>`
- 坐标：`center_x_m` / `center_y_m` / `center_z_m`
- 强度：
  - 优先使用 `intensity` 或 `hrr_rel`
  - 若不存在，则尝试使用 `hrr_kw`（按 `3000kW -> 1.0` 归一化）
  - 若仍不存在，则回退到 `temp_c` 的经验映射
- 扩散速度：`front_speed_mps` / `spread_mps`

示例见：`docs/examples/fds_cells_export_sample.csv`

这不是直接要求 FDS 原始文件与本仓库完全同构，而是定义了一个“真实 FDS 导出到适配器前”的稳定契约，避免每次联调都手工改代码。

### 2.3 `fds_jsonl`（区域边界契约）

用于承接上游从 FDS 结果提取出的火区边界。每行一个 region，支持以下字段：

- 时间：`time` / `time_s` / `sim_time_s`
- 区域标识：`region_id` / `id`
- 强度：`intensity` / `hrr_rel`
- 扩散速度：`front_speed_mps` / `spread_mps`
- 边界点：
  - `points`: `[{x,y,z}, ...]`
  - 或 `vertices`: `[[x,y,z], ...]`

## 3. 坐标转换

适配器将 FDS 局部平面坐标转换为 WGS84：

- `lat = origin_lat + north_m / 111000`
- `lon = origin_lon + east_m / (111000 * cos(origin_lat))`
- `alt = origin_alt_m + z_m`

默认：

- `x_m -> east_m`
- `y_m -> north_m`

如你的 FDS 轴定义相反，可设置 `xy_swap:=true`。

## 4. 时间同步策略

- `time_mode:=source_offset`（默认）  
  首次样本建立“源时间 -> ROS 时间”偏移，后续按源时间增量推演。
- `time_mode:=ros_now`  
  忽略源时间，使用当前 ROS 时间戳。

新增参数：

- `playback_mode:=timeline|latest`：时间轴回放或仅最新帧。
- `replay_speed:=1.0`：回放倍速。
- `loop_timeline:=true|false`：是否循环回放。
- `source_time_scale:=1.0`：源时间缩放（如输入毫秒可设 `0.001`）。
- `input_profile:=normalized|fds_csv`：热点字段解释方式。
- `region_input_profile:=normalized|fds_jsonl`：区域字段解释方式。
- `enforce_monotonic_stamp:=true|false`：是否强制时间戳单调递增。
- `publish_regions:=true|false`：是否发布真实区域 topic。
- `region_output_topic:=/env/fire_region_state`：区域输出 topic。
- `region_input_path`：区域输入文件路径。
- `region_input_format:=jsonl|csv`：区域输入格式。

## 5. 坐标映射增强

除 `xy_swap` 外，新增以下参数用于对齐 FDS 场景坐标：

- `xy_yaw_deg`：水平面旋转角（度）
- `x_scale/y_scale/z_scale`：各轴缩放
- `x_offset_m/y_offset_m/z_offset_m`：各轴偏移（米）

## 6. 启动示例

仅启动适配器：

```bash
python3 scripts/fire_adapter_fds.py --ros-args \
  -p output_topic:=/env/fire_state \
  -p region_output_topic:=/env/fire_region_state \
  -p input_path:=docs/examples/fds_hotspots_sample.csv \
  -p region_input_path:=docs/examples/fds_regions_sample.jsonl \
  -p input_format:=csv \
  -p input_profile:=normalized \
  -p region_input_format:=jsonl \
  -p region_input_profile:=normalized \
  -p origin_lat:=39.9042 \
  -p origin_lon:=116.4074 \
  -p time_mode:=source_offset \
  -p playback_mode:=timeline \
  -p replay_speed:=1.0
```

使用更接近真实导出的 FDS 表格：

```bash
python3 scripts/fire_adapter_fds.py --ros-args \
  -p output_topic:=/env/fire_state \
  -p input_path:=docs/examples/fds_cells_export_sample.csv \
  -p input_format:=csv \
  -p input_profile:=fds_csv \
  -p time_mode:=source_offset \
  -p playback_mode:=timeline
```

在 Fire Mission Demo 中，当前默认主路径就是 FDS 输入：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899
```

可选补充参数：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv source_offset 1.5
```

## 7. 开发验证建议

最小本地验证：

```bash
python3 tests/test_fire_adapter_fds_core.py
python3 -m py_compile scripts/fire_adapter_fds.py scripts/fire_adapter_fds_core.py
```

联调验证：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv
./scripts/fire_mission_demo_check.sh 8899 4 4 fds docs/examples/fds_hotspots_sample.csv csv
```

常见失败原因：

- 输入文件路径错误：适配器会持续告警 `input file not found`
- 列名不匹配：请检查 `input_profile` 是否与文件字段一致
- 区域点数不足 3：该 region 会被忽略
- 时间单位不一致：可通过 `source_time_scale` 修正

## 8. 回退方案

若 FDS 输入异常，可切回原始模拟火情：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 demo
```
