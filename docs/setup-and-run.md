# Setup & Run (Ubuntu 22.04)

## 已完成安装

- ROS2 Humble
- rosdep 初始化
- PX4-Autopilot 源码与依赖
- PX4 SITL 冒烟构建通过

## 新开终端后的准备

```bash
source /opt/ros/humble/setup.bash
```

## 启动 4 架 PX4 SITL（由 Node_UAV_Manager 管理）

```bash
./scripts/run_px4_swarm.sh 4
```

参数说明：

- 第一个参数：实例数量（默认 `4`）
- 第二个参数：可用 CPU 核数（默认 `nproc`）
- 第三个参数：运行时长秒数（默认 `-1` 表示持续运行直到 `Ctrl+C`）

## 单独验证 PX4 SITL

```bash
cd third_party/PX4-Autopilot
CCACHE_DISABLE=1 make px4_sitl -j$(nproc)
```
