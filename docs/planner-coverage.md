# 任务规划覆盖优先策略（Issue #6）

## 目标

在保留 `greedy` 基线策略的同时，新增 `coverage` 模式，降低重复覆盖并引入复访周期约束。

## 策略说明

- `greedy`：按热点强度排序，每个热点分配最近未占用 UAV。
- `coverage`：对每个 `UAV-热点` 对计算综合分数并做逐步匹配：
- 热点优先级：`intensity` + `revisit_score` + `unseen_bonus`
- 距离惩罚：按 `coverage_distance_norm_m` 归一化后加权扣分
- 连续性奖励：同一 UAV 连续追踪同一热点时加分，减少轨迹抖动

## 新增参数

- `planner_mode`: `greedy|coverage`
- `coverage_revisit_sec`: 目标复访周期（秒）
- `coverage_priority_decay`: 近期已访问热点的优先级衰减（0~1）
- `coverage_distance_weight`: 距离惩罚权重
- `coverage_intensity_weight`: 强度项权重
- `coverage_revisit_weight`: 复访项权重
- `coverage_unseen_bonus`: 首次覆盖加分
- `coverage_continuity_bonus`: 连续跟踪加分
- `coverage_distance_norm_m`: 距离归一化尺度（米）
- `coverage_retarget_threshold_m`: 抖动控制阈值（米），`0` 表示关闭“保持原目标”机制

## 运行

```bash
./scripts/fire_mission_demo_start.sh 6 6 8913 coverage
```

对照基线：

```bash
./scripts/fire_mission_demo_start.sh 6 6 8913 greedy
```

## 离线评估

```bash
./scripts/eval_planner_coverage.py --ticks 120 --seed 42 --uavs 8 --hotspots 10
```

输出包含：

- `coverage_rate`：每个周期被分配到的热点占比
- `window_cov`：最近 12 tick 的窗口覆盖率（越高表示轮巡更充分）
- `revisit_gap_p95`：复访间隔的 p95（tick）
- `jitter_avg_m`：UAV 每步平均机动距离（越小通常越稳）
