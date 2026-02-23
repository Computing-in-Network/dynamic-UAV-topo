# FDS 场景端到端压测（Issue #8）

## 目标

在 FDS 输入场景下，验证：

- 火情输入 (`/env/fire_state`)
- 任务规划 (`/swarm/mission_targets`)
- 任务状态 (`/swarm/mission_status`)
- 拓扑分析 (`/swarm/state`)
- 可视化 API (`/api/swarm_state`)

的链路稳定性与关键指标。

## 脚本

- `scripts/stress_fds_pipeline.sh`

参数：

1. `instance_count`（默认 20）
2. `total_cores`（默认 20）
3. `vis_port`（默认 8913）
4. `duration_seconds`（默认 20）
5. `profile_target_ms`（默认 20.0）
6. `input_csv`（默认最小 case 输出）
7. `mapping_json`（默认最小 case mapping）
8. `fire_publish_hz`（默认 4.0）
9. `planner_mode`（默认 coverage）
10. `auto_run_fds`（默认 1，先自动生成最小 devc.csv）

## 运行

```bash
./scripts/stress_fds_pipeline.sh 20 20 8913 20 20.0
```

## 输出指标

- 拓扑 profile 行（来自 `swarm_topology_analyzer`）
- `api_rtt_avg_ms` / `api_rtt_p95_ms`
- `state_hz`
- `plan_latency_p50_ms` / `plan_latency_p95_ms`
- `exec_latency_p50_ms` / `exec_latency_p95_ms`
- `done_latency_p50_ms` / `done_latency_p95_ms`（若有完成）

指标明细文件：`/tmp/fds_stress_metrics.json`

## 通过条件（当前版本）

- 拓扑 profile 为 `status=PASS`
- `state_hz >= 3.5`
- `plan_latency_count >= 1`

## 常见问题

- 若 FDS 在当前环境报 OFI/MPI 错误，先在 shell 中设置：
  - `export FI_PROVIDER=tcp`
  - `export I_MPI_FABRICS=shm:tcp`
  - `export OMP_NUM_THREADS=1`
- 若端口占用，换 `vis_port` 重试。
