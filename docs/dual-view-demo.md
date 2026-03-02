# 双视角并列演示（Gazebo 特写 + 前端全景）

本文档用于说明双视角演示模式的布局、启动方式、同步检查与排障。

## 1. 布局规范

建议采用固定并列布局：

- 左侧 40%：Gazebo 特写窗口（局部姿态、机动细节）。
- 右侧 60%：前端全景窗口（火区、链路、任务、全体 UAV）。

此布局为建议值，可根据显示器调整，但建议保持“左特写、右全景”一致。

## 2. 一键启动/停止

启动（mock 特写源）：

```bash
./scripts/dual_view_demo_start.sh 4 4 8899 mock
```

启动（真实 Gazebo 特写进程）：

```bash
GAZEBO_CMD='gz sim your_world.sdf' ./scripts/dual_view_demo_start.sh 4 4 8899 gazebo
```

停止：

```bash
./scripts/dual_view_demo_stop.sh
```

## 3. 同步性验收

执行双视角同步检查（默认阈值：距离 <= 30m，时间差 <= 2500ms）：

```bash
./scripts/dual_view_demo_check.sh 8899 2 2 mock 30.0 2500
```

输出示例：

- `samples=8 max_dist_m=... max_dt_ms=... uid=...`
- `[dual_view_demo_check][INFO] PASS: 双视角同步检查通过 ...`

## 4. 对齐策略

- 统一 UAV ID：特写与全景均按 `uav_id` 关联同一对象。
- 统一时间戳：同步检查按毫秒级时间戳比较。
- 轨迹偏差：以经纬高三维距离计算误差阈值。

## 5. 常见问题排查

1. 启动失败（`E_START_FAIL`）  
查看 `/tmp/dual_view_start.log` 与 visual/fire 相关日志。

2. 同步失败（`E_SYNC`）  
先检查特写源是否返回 `ok=true`，再检查 ID 是否匹配、时间戳是否同源。

3. 端口冲突  
脚本会自动回退到附近端口；实际端口记录在 `.dual_view_demo_pids`。

4. 无 Gazebo 环境  
先用 `mock` 模式完成链路与同步脚本验收，后续切换 `gazebo` 模式。
