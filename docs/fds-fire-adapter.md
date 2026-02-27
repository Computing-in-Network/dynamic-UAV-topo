# FDS Fire Adapter（首版）

本文档说明如何使用 `scripts/fire_adapter_fds.py` 将 FDS 导出的离线结果接入 `/env/fire_state`。

## 1. 输入格式

首版支持两种输入：

- `csv`（默认）
- `jsonl`

默认按“文件轮询”读取，文件被更新后会重新解析并发布最新时刻的火点。
默认播放模式为 `timeline`，会按 `time_s` 连续回放热点序列。

### CSV 列定义

推荐包含表头，字段如下：

- `time_s`：仿真时间（秒）
- `id`：热点标识
- `x_m`：FDS 局部坐标 X（米）
- `y_m`：FDS 局部坐标 Y（米）
- `z_m`：FDS 局部坐标 Z（米）
- `intensity`：强度（0~1）
- `spread_mps`：扩散速度（米/秒）

示例见：`docs/examples/fds_hotspots_sample.csv`

## 2. 坐标转换

适配器将 FDS 局部平面坐标转换为 WGS84：

- `lat = origin_lat + north_m / 111000`
- `lon = origin_lon + east_m / (111000 * cos(origin_lat))`
- `alt = origin_alt_m + z_m`

默认：

- `x_m -> east_m`
- `y_m -> north_m`

如你的 FDS 轴定义相反，可设置 `xy_swap:=true`。

## 3. 时间同步策略

- `time_mode:=source_offset`（默认）  
  首次样本建立“源时间 -> ROS 时间”偏移，后续按源时间增量推演。
- `time_mode:=ros_now`  
  忽略源时间，使用当前 ROS 时间戳。

新增参数：

- `playback_mode:=timeline|latest`：时间轴回放或仅最新帧。
- `replay_speed:=1.0`：回放倍速。
- `loop_timeline:=true|false`：是否循环回放。
- `source_time_scale:=1.0`：源时间缩放（如输入毫秒可设 `0.001`）。
- `enforce_monotonic_stamp:=true|false`：是否强制时间戳单调递增。

## 4. 坐标映射增强

除 `xy_swap` 外，新增以下参数用于对齐 FDS 场景坐标：

- `xy_yaw_deg`：水平面旋转角（度）
- `x_scale/y_scale/z_scale`：各轴缩放
- `x_offset_m/y_offset_m/z_offset_m`：各轴偏移（米）

## 5. 启动示例

仅启动适配器：

```bash
python3 scripts/fire_adapter_fds.py --ros-args \
  -p output_topic:=/env/fire_state \
  -p input_path:=docs/examples/fds_hotspots_sample.csv \
  -p input_format:=csv \
  -p origin_lat:=39.9042 \
  -p origin_lon:=116.4074 \
  -p time_mode:=source_offset \
  -p playback_mode:=timeline \
  -p replay_speed:=1.0
```

在 Fire Mission Demo 中启用 FDS 输入：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv
```

可选补充参数：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv source_offset 1.5
```

## 6. 回退方案

若 FDS 输入异常，可切回原始模拟火情：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 demo
```
