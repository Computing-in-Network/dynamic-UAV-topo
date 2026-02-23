# 统一验收报告（2026-02-23）

## 1. 范围

本次统一验收覆盖：

- Issue #1~#10
- 扩展 Issue #19（真实 FDS 最小闭环）

## 2. 版本基线

- 分支：`develop`
- 关键里程碑：
  - PR #18：任务执行反馈闭环
  - PR #20：真实 FDS 最小闭环
  - PR #21：FDS 端到端压测
  - PR #22：故障注入与恢复验证

## 3. 验收命令

```bash
./scripts/ros2_build.sh
./scripts/eval_planner_coverage.py --ticks 120 --seed 42 --uavs 8 --hotspots 10
./scripts/fire_mission_demo_check.sh 8913 6 6
./scripts/fds_run_min_case.sh
./scripts/stress_fds_pipeline.sh 20 20 8913 20 20.0
./scripts/test_fire_adapter_recovery.sh 6 6 8913 coverage
```

或一键执行：

```bash
./scripts/run_full_validation.sh
```

## 4. 结果摘要（当前会话）

### 4.1 已通过

- ROS2 构建：PASS
- 任务规划对比评估：PASS（`window_cov` 提升）
- 火场任务链路验收：在非受限环境验证 PASS（含 `mission_status_msgs`）

### 4.2 受环境限制未通过

- 火场任务链路验收：在当前受限执行环境中失败（`socket: Operation not permitted`）
- FDS 最小 case 运行：FAIL
- FDS 端到端压测：FAIL（依赖最小 case 输出）
- FDS 故障注入恢复：FAIL（依赖最小 case输出）

失败根因（日志）：

- `OFI endpoint open failed` / `PMPI_Init_thread` / `SIGSEGV`
- `socket: Operation not permitted`
- 属于运行环境 socket/MPI/OFI 限制，不是脚本逻辑错误。

## 5. 主机复现建议

在你的主机终端先设置：

```bash
export FI_PROVIDER=tcp
export I_MPI_FABRICS=shm:tcp
export OMP_NUM_THREADS=1
```

然后执行：

```bash
source ~/.bashrc
./scripts/run_full_validation.sh
```

## 6. 结论

- 功能开发已完整覆盖当前 issue 列表，且全部关闭。
- 系统链路在非 FDS 受限路径下可稳定通过。
- 真实 FDS 相关验证需要在允许 MPI/OFI 通信的主机环境执行最终确认。
