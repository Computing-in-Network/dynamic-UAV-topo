# FDS Fire Adapter（首版）

本文档说明如何使用 `scripts/fire_adapter_fds.py` 将 FDS 导出的离线结果接入 `/env/fire_state`。

## 1. 输入格式

首版支持两种输入：

- `csv`（默认）
- `jsonl`

默认按“文件轮询”读取，文件被更新后会重新解析并发布最新时刻的火点。

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

## 4. 启动示例

仅启动适配器：

```bash
python3 scripts/fire_adapter_fds.py --ros-args \
  -p output_topic:=/env/fire_state \
  -p input_path:=docs/examples/fds_hotspots_sample.csv \
  -p input_format:=csv \
  -p origin_lat:=39.9042 \
  -p origin_lon:=116.4074 \
  -p time_mode:=source_offset
```

在 Fire Mission Demo 中启用 FDS 输入：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 fds docs/examples/fds_hotspots_sample.csv csv
```

## 5. 回退方案

若 FDS 输入异常，可切回原始模拟火情：

```bash
./scripts/fire_mission_demo_start.sh 4 4 8899 demo
```
