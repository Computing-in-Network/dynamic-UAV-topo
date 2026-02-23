# FDS 输出到 ROS2 的数据契约（Issue #1）

## 1. 目标与范围

本文档定义 FDS（Fire Dynamics Simulator）结果如何映射到本项目的 ROS2 火情接口。

- 输入侧：FDS 后处理结果（离线文件或增量输出）
- 输出侧：`swarm_interfaces/msg/FireState`
- 当前阶段：先定义“热点级别契约”，后续可扩展火区面数据

## 2. 统一时间基准

- 时间基准：`Unix Epoch`（UTC，毫秒）
- 推荐字段：
  - `timestamp_ms`: 结果采样时间（整数毫秒）
  - `sim_time_s`: FDS 仿真时间（秒，浮点）
- ROS2 映射：
  - `FireState.stamp` 由 `timestamp_ms` 转换得到

## 3. 坐标系与单位

### 3.1 FDS 局部坐标

- 记作 `(x_fds, y_fds, z_fds)`，单位：米
- 原点：由场景配置定义（例如火场网格原点）

### 3.2 地理坐标（WGS84）

- 输出位置：`(lat, lon, alt)`  
- 单位：
  - `lat/lon`: 度
  - `alt`: 米（海拔或相对高度，需在数据头声明）

### 3.3 近似转换公式（局部小范围）

给定场景原点 `lat0, lon0, alt0`，将 FDS 局部坐标转换到 WGS84：

```text
lat = lat0 + (y_fds / 111000.0)
lon = lon0 + (x_fds / (111000.0 * cos(lat0 * π / 180)))
alt = alt0 + z_fds
```

说明：

- 该公式适用于小范围区域（如单厂区/单园区）。
- 大范围场景建议后续切换到 ENU/ECEF 严格转换。

## 4. 热点对象字段定义

热点对象映射到 `swarm_interfaces/msg/Hotspot`：

- `id: string`  
  热点唯一标识，建议稳定且可追踪（如 `hotspot_a`）。

- `position: float64[3]`  
  `[lat, lon, alt]`。

- `intensity: float32`  
  归一化火情强度，范围建议 `[0, 1]`。
  推荐由温度、热释放率等指标归一化得到。

- `spread_mps: float32`  
  该热点等效扩散速度（米/秒），用于前端和任务规划预测。

## 5. FireState 输出契约

`swarm_interfaces/msg/FireState`：

- `stamp: builtin_interfaces/Time`
- `hotspots: Hotspot[]`

输出要求：

1. 同一时刻热点数组应是完整快照（snapshot），而非增量 patch。
2. 同一 `id` 在连续时刻应保持身份稳定。
3. `intensity` 与 `spread_mps` 必须有单位语义并在样例中说明来源。

## 6. 输入样例结构（JSON）

推荐最小 JSON 结构：

```json
{
  "schema_version": "1.0",
  "origin_wgs84": {
    "lat0": 39.9042,
    "lon0": 116.4074,
    "alt0_m": 80.0
  },
  "timestamp_ms": 1771598000000,
  "sim_time_s": 120.0,
  "hotspots_local_m": [
    {
      "id": "hotspot_a",
      "x_m": 35.0,
      "y_m": 110.0,
      "z_m": 15.0,
      "intensity_01": 0.92,
      "spread_mps": 0.8
    }
  ]
}
```

## 7. 质量门槛（验收）

1. 字段完整性：必须满足 `id/position/intensity/spread_mps`。
2. 坐标可解释：样例中必须包含 `origin_wgs84`。
3. 时间一致性：`timestamp_ms` 和 `sim_time_s` 均可用。
4. 可解析性：样例文件可由脚本直接读取并发布到 `/env/fire_state`。

## 8. 后续扩展（非本 Issue）

- 火区面（Polygon / Raster）契约
- 风场与地形影响因子
- 多源融合（FDS + 传感器实测）
