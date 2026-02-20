这份 Spec 采用了**高度结构化、接口导向（Interface-Oriented）**的设计，旨在让你直接将各模块要求喂给 Codex。

---

## 项目代号：SwarmSim-100 (Alpha Phase: 20-UAV)

**核心架构：** 基于 ROS2 的分布式仿真，计算、渲染、控制三层分离。

---

### 1. 技术栈约束 (Tech Stack)

* **前端渲染：** CesiumJS (3D 地球) + Deck.gl (高性能 WebGL 拓扑图层)。
* **通信总线：** ROS2 (Humble/Foxy) + FastDDS。
* **后端引擎：** C++20 (用于高频物理/拓扑计算) + Python 3.10 (用于 AI/语义逻辑)。
* **无人机固件仿真：** PX4 SITL (Software-in-the-Loop)。
* **协议层：** MAVLink 2.0 (状态同步) + Protobuf (前端交互)。

---

### 2. 核心模块规格 (Module Specifications)

#### 2.1 仿真后端：Node_UAV_Manager (C++/ROS2)

* **职责：** 维护 20-100 个无人机状态机，管理 SITL 进程。
* **输入：** `geometry_msgs/PoseStamped` (来自各 UAV 实例)。
* **输出：** `SwarmState.msg` (包含所有 UAV 的坐标、速度、姿态)。
* **Codex 提示点：** “实现一个单进程管理多个 PX4 SITL 实例的 Wrapper，确保 CPU 核心绑定（Affinity）以优化 48 核利用率。”

#### 2.2 拓扑计算引擎：Node_Topology_Analyzer (C++)

* **核心算法：** 动态计算  链路。
* **链路通断判定 ()：**


* **Ricci Curvature 计算：** 基于 Ollivier-Ricci Curvature 离散模型计算图的局部稳定性。
* **Codex 提示点：** “使用 Eigen 库进行矩阵运算。实现一个基于显卡的射线检测接口，利用深度缓冲区（Depth Buffer）进行地形遮挡判断。”

#### 2.3 语义网络模拟：Node_Semantic_Net (Python)

* **职责：** 模拟丢包、时延及语义删除。
* **逻辑：** 根据 `Topology_Analyzer` 提供的 SNR，对 MAVLink 包进行概率性丢弃，验证你的语义恢复算法。
* **Codex 提示点：** “实现一个基于 Python 的语义过滤器，拦截并根据链路权重修改 ROS2 Topic 频率。”

---

### 3. 数据交换规格 (Interface Schema)

必须统一定义所有模块交互的数据结构，否则分布式系统会崩溃：

```typescript
// 前端数据接口定义 (TypeScript/Protobuf)
interface SwarmSnapshot {
  timestamp: number;
  uavs: Array<{
    id: string;
    position: [lat, lon, alt];
    velocity: [vx, vy, vz];
    battery: number;
    status: "MISSION" | "ALARM" | "LANDED";
  }>;
  links: Array<{
    source: string;
    target: string;
    weight: number; // 信号强度或Ricci曲率
    is_occluded: boolean;
  }>;
}

```

---

### 4. 20架无人机验证阶段 (Validation Plan)

在开发早期，要求 Codex 为你生成以下测试脚本：

1. **性能基准 (Profiling)：** 在 20 架机并发时，记录 `Node_Topology_Analyzer` 的单帧耗时（目标 < 20ms）。
2. **遮挡正确性：** 强制两架虚拟无人机飞行到山峰两侧，验证 `is_occluded` 字段是否能准确翻转。
3. **分布式压力：** 模拟 100Hz 的遥测下发，观察 48 核 CPU 的 Context Switch 频率，确保不产生阻塞。

---

### 5. 针对 48GB 显卡的专项 Spec

* **特性：** 环境体素化 (Voxelization)。
* **需求：** 将 Cesium 加载的 3D Tiles 导出为 CPU/GPU 可读的二进制空间索引（Octree）。
* **任务：** Codex 需要编写一个 CUDA Kernel，用于并行处理 20-100 个节点间的射频射线。

---

### 研发失败的“避坑”建议 (Codex 开发专用)

1. **不要让 Codex 写大文件：** 每次只让它写一个 ROS2 Node 的逻辑。
2. **强制类型检查：** 所有的 Python 代码必须带 Type Hints，C++ 必须符合 C++20 标准。
3. **先写单元测试：** 在写业务逻辑前，先让 Codex 根据上述 Spec 写出测试用例（Test Bench）。

**这份 Spec 是否满足你的启动需求？如果你准备好了，我们可以先从“2.1 UAV Manager”的核心逻辑开始，我可以直接告诉你如何给 Codex 下达第一条有效的 Architect 指令。**