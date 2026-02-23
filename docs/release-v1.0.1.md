# Release v1.0.1（小版本修复）发布说明

发布日期：2026-02-23  
分支基线：`develop`  
版本标签：`v1.0.1`

## 1. 版本定位

`v1.0.1` 为 `v1.0.0` 的稳定性修复版本，重点解决 Docker 常驻部署场景下“底图偶发蓝底不可恢复”的问题，并补齐可复现的部署文档。

## 2. 本次修复内容

### 2.1 底图自动恢复机制
- 文件：`web/cesium_deck.html`
- 变更：新增 OSM 连通性探测逻辑。
- 行为：当底图因网络抖动从 `OSM` 降级到 `GRID` 后，前端会周期性探测 `/tiles/osm/0/0/0.png`，探测成功自动切回 `OSM`。

### 2.2 Docker 常驻运行支持
- 文件：`docker/Dockerfile`
- 文件：`docker/run_fds_demo.sh`
- 文件：`.dockerignore`
- 变更：新增最小运行镜像与容器入口脚本，支持 `--restart unless-stopped` 的长期运行方式。

### 2.3 部署文档更新
- 文件：`README.md`
- 变更：新增 Docker 常驻运行章节，明确以下关键约束：
  - 代理服务器：`192.168.0.5:7890`
  - 容器内外使用同路径挂载：`/home/zyren/dynamic-uav-topo`（兼容 ROS2 symlink-install 绝对路径）

## 3. 修复价值

- 解决“网络恢复后仍长期蓝底”的可用性问题；
- 降低运维反复手动重启页面/服务的成本；
- 形成标准化 Docker 启动方式，便于后续演示与回归。

## 4. 升级与验证

### 4.1 启动命令

```bash
docker run -d --name dynamic-uav-topo-v1 --restart unless-stopped --network host \
  -e REPO_DIR=/home/zyren/dynamic-uav-topo \
  -e VIS_PORT=8928 \
  -e INSTANCE_COUNT=6 \
  -e TOTAL_CORES=6 \
  -e HTTP_PROXY=http://192.168.0.5:7890 \
  -e HTTPS_PROXY=http://192.168.0.5:7890 \
  -e NO_PROXY=127.0.0.1,localhost \
  -v /home/zyren/dynamic-uav-topo:/home/zyren/dynamic-uav-topo \
  dynamic-uav-topo:v1.0.1
```

### 4.2 验证项

- 打开：`http://127.0.0.1:8928/cesium`
- 接口：`/api/swarm_state` 返回 `uavs/fire_hotspots/mission_targets`
- 状态栏：`basemap=OSM`

## 5. 兼容性说明

- 业务接口与话题定义未变；
- 本版本为兼容性修复，可直接从 `v1.0.0` 升级。
