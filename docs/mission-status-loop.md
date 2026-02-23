# 任务执行反馈闭环（Issue #7）

## 新增能力

- 新增话题：`/swarm/mission_status`
- 新增节点：`scripts/mission_status_tracker.py`
- 状态机：`IDLE/ASSIGNED/EXECUTING/COMPLETED/FAILED`
- 失败原因：`timeout`、`uav_missing`

## 消息定义

- `swarm_interfaces/msg/MissionStatusItem.msg`
- `swarm_interfaces/msg/MissionStatus.msg`

字段含义：

- `uav_id`：无人机 ID
- `target_ref`：目标引用（从 mission reason 中提取）
- `state`：任务状态
- `progress`：0~1 进度估计
- `distance_m`：到目标距离（米）
- `reason`：状态原因

## 运行与验证

```bash
./scripts/ros2_build.sh
./scripts/fire_mission_demo_start.sh 6 6 8913 coverage
```

查看状态话题：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /swarm/mission_status --once
```

一键验收：

```bash
./scripts/fire_mission_demo_check.sh 8913 6 6
```

验收通过时会输出：

- `mission_status_msgs=...`
- `done=... fail=...`

## 前端展示

`/api/swarm_state` 新增 `mission_status` 字段，Cesium 侧栏新增：

- 执行中数量 `exec`
- 已完成数量 `done`
- 失败数量 `fail`
- UAV 表格新增 Mission 状态列
