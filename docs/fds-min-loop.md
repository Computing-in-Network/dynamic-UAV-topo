# 真实 FDS 最小闭环实操

## 目标

跑通：`FDS -> *_devc.csv -> fire_adapter_fds -> mission planner -> 前端`。

## 1. 生成最小 FDS 输出

```bash
./scripts/fds_run_min_case.sh
```

成功后会看到：

- `data/fds_cases/min_fire/fds_min_fire_devc.csv`
- 日志：`/tmp/fds_min_fire.log`

## 2. 启动真实 FDS 输入链路

```bash
./scripts/fire_mission_fds_start.sh 6 6 8913 coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  2.0
```

## 3. 验证

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /env/fire_state --once
ros2 topic echo /swarm/mission_status --once
curl --noproxy '*' -s http://127.0.0.1:8913/api/swarm_state | head -n 40
```

浏览器：

- `http://127.0.0.1:8913/cesium?basemap=osm`

## 4. 停止

```bash
./scripts/fire_mission_demo_stop.sh 8913
```

## 常见问题

- `fds command not found`：先 `source /home/zyren/FDS/FDS6/bin/FDS6VARS.sh`
- `missing output *_devc.csv`：看 `/tmp/fds_min_fire.log`
- 页面有 UAV 无火情：检查 `fire_adapter_fds_real.log` 和 mapping 列名是否匹配
- 若日志里出现 `OFI endpoint open failed`：执行前先
  `export FI_PROVIDER=tcp; export I_MPI_FABRICS=shm:tcp; export OMP_NUM_THREADS=1`
