# Fire Adapter（离线 FDS 输入）使用说明

## 功能

- 从离线 FDS 样例文件读取火情（JSON/CSV）。
- 转换为 `swarm_interfaces/msg/FireState` 并发布到 ROS2。

## 运行

```bash
./scripts/fire_adapter_fds_start.sh \
  data/fds_samples/fds_fire_snapshot.sample.json \
  auto \
  1.0 \
  /env/fire_state
```

参数：

1. 输入文件路径（默认 JSON 样例）
2. 输入格式 `auto|json|csv`（默认 `auto`）
3. 发布频率 Hz（默认 `1.0`）
4. 输出 topic（默认 `/env/fire_state`）

## 验证

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /env/fire_state --once
```

预期：

- `hotspots` 数组存在，且至少包含 `id/position/intensity/spread_mps`。

## 常见故障

- `input file not found`：检查输入路径。
- `unsupported input_format`：仅支持 `auto|json|csv`。
