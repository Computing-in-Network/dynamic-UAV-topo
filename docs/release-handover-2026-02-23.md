# 最终交付清单与演示话术（2026-02-23）

## 一、最终交付清单

- UAV集群状态链路：`/swarm/state_raw -> /swarm/state`
- 火情输入链路：`FDS devc.csv -> /env/fire_state`
- 任务规划链路：`/swarm/mission_targets`
- 任务执行闭环：`/swarm/mission_status`
- 可视化链路：`/api/swarm_state -> web/cesium_deck.html`

关键脚本：

- `scripts/fds_run_min_case.sh`
- `scripts/fire_mission_fds_start.sh`
- `scripts/stress_fds_pipeline.sh`
- `scripts/test_fire_adapter_recovery.sh`
- `scripts/run_full_validation.sh`

关键文档：

- `docs/fds-integration.md`
- `docs/fds-min-loop.md`
- `docs/fds-stress.md`
- `docs/fds-fault-injection.md`
- `docs/validation-report-2026-02-23.md`

## 二、演示前检查（3分钟）

```bash
cd ~/dynamic-uav-topo
source ~/.bashrc
./scripts/ros2_build.sh
```

## 三、现场演示流程（8~12分钟）

1. 生成真实火情输入：

```bash
./scripts/fds_run_min_case.sh
```

2. 启动整链路：

```bash
./scripts/fire_mission_fds_start.sh 20 20 8924 coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  4.0 true 20 20.0
```

打开：`http://127.0.0.1:8924/cesium?basemap=osm`

3. 核心数据证明：

```bash
curl --noproxy '*' -s http://127.0.0.1:8924/api/swarm_state | \
python3 -c "import sys,json;d=json.load(sys.stdin);print('uavs',len(d.get('uavs',[])),'fire',len(d.get('fire_hotspots',[])),'mission',len(d.get('mission_targets',[])),'status',len(d.get('mission_status',[])))"
```

4. 端到端压测：

```bash
./scripts/stress_fds_pipeline.sh 20 20 8923 20 20.0 \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  4.0 coverage 0
```

5. 故障恢复：

```bash
./scripts/test_fire_adapter_recovery.sh 6 6 8922 coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  2.0
```

6. 统一验收：

```bash
./scripts/run_full_validation.sh
cat /tmp/full_validation/summary.txt
```

## 四、结语模板

“当前版本已完成从真实 FDS 输入到任务执行反馈的工程闭环，且通过压测与故障恢复验证，具备可复现、可回归、可演示的项目交付能力。”
