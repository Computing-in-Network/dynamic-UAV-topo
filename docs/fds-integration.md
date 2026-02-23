# FDS 集成操作手册（中文）

## 1. 当前接入能力

- 已支持 FDS `*_devc.csv` -> `FireState` 的离线解析通路。
- 前端展示的是项目态势图层（火点/火区），不是 Smokeview 原生渲染窗口。

## 2. 本机检查是否安装 FDS

```bash
which fds || which fds6
which smokeview || which smv
```

若命令无输出，表示本机尚未安装可执行文件。

## 3. FDS 输出接入步骤

1. 用 FDS 跑出场景，确保生成 `CHID_devc.csv`（或同结构 CSV）。
2. 按传感器列名编写映射文件（可参考 `data/fds_samples/fds_devc_mapping.sample.json`）。
3. 启动适配器：

```bash
./scripts/fire_adapter_fds_start.sh \
  /path/to/CHID_devc.csv \
  fds_devc_csv \
  2.0 \
  /env/fire_state \
  offline \
  data/fds_samples/stream \
  .json \
  /tmp/fire_adapter_fds_checkpoint.json \
  /path/to/fds_devc_mapping.json
```

4. 启动任务链路和可视化，检查 `/api/swarm_state` 中 `fire_hotspots` 是否有数据。

## 4. 验证命令

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /env/fire_state --once
curl --noproxy '*' -s http://127.0.0.1:8913/api/swarm_state | head -n 20
```

## 5. 常见问题

- 页面有 UAV 但无火情：检查 `fire_adapter_fds.py` 是否在运行，及映射列名是否与 CSV 一致。
- 有火情但无底图：与底图服务连通性相关，火情链路本身可独立验证。
- 端口释放失败：优先使用 `./scripts/fire_mission_demo_stop.sh <port>` 并检查残留 PID。
