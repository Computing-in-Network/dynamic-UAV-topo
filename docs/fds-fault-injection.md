# Fire Adapter 故障注入与恢复验证（Issue #9）

## 目标

验证在真实 FDS 输入链路下，以下故障不会导致系统卡死，并且可恢复：

1. 断流（Fire Adapter 进程中断）
2. 坏包（CSV 列损坏/字段不匹配）
3. 时钟漂移（时间戳跳变、回拨）

## 脚本

- `scripts/test_fire_adapter_recovery.sh`

参数：

1. `instance_count`（默认 6）
2. `total_cores`（默认 6）
3. `vis_port`（默认 8913）
4. `planner_mode`（默认 coverage）
5. `good_csv`（默认 `data/fds_cases/min_fire/fds_min_fire_devc.csv`）
6. `mapping_json`（默认 `data/fds_cases/min_fire/fds_min_fire_devc_mapping.json`）
7. `publish_hz`（默认 2.0）

## 运行

```bash
./scripts/test_fire_adapter_recovery.sh 6 6 8913 coverage
```

## 验收标准

- 三类故障注入后，`/api/swarm_state` 仍可访问，`uavs` 与 `mission_status` 不为空。
- 每次故障恢复后，`fire_hotspots + mission_targets + mission_status` 重新进入就绪状态。
- 脚本输出 `PASS`。

## 输出

- 启动日志：`/tmp/fire_mission_fds_start_recovery.log`
- 恢复适配器日志：`/tmp/fire_adapter_fds_recovery.log`
- FDS 生成日志（若自动生成）：`/tmp/fds_run_recovery.log`

## 备注

- `scripts/fire_mission_demo_stop.sh` 已增加 `fire_adapter_fds.py` 兜底清理，避免注入测试后残留进程。
