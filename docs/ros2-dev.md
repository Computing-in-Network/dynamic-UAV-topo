# ROS2 集成开发说明

## 目录

- `ros2_ws/src/swarm_interfaces`: 消息定义包
- `ros2_ws/src/swarm_uav_manager`: UAV Manager ROS2 节点包

## 消息

- Topic: `/swarm/state`
- Type: `swarm_interfaces/msg/SwarmState`

`UavState.msg` 字段：

- `id`
- `position[3]` (`lat, lon, alt`)
- `velocity[3]` (`vx, vy, vz`)
- `battery`
- `status` (`MISSION/ALARM/LANDED`)

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

## 快速查看状态

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /swarm/state --once
```
