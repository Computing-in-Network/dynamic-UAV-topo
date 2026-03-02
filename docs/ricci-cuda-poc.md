# Ricci/CUDA PoC 说明

## 目标

为 Issue #10 提供最小可运行评估：在固定随机拓扑条件下，比较 CPU 与 CuPy CUDA 实现的
`common-neighbor proxy`（用于 Ricci Curvature 估算场景的代替指标）耗时与一致性，
为后续决策 `是否需要上 CUDA` 提供量化依据。

## 运行方式

```bash
./scripts/ricci_cuda_poc.py --nodes 600 --density 0.05 --seed 2026 --repeat 6 --warmup 1
```

## 关键输出字段

- `input`: 输入参数快照（便于复现实验）
- `graph.edges`: 生成的无向边数
- `graph.actual_density`: 边数对应的实际密度
- `backend.cpu.avg_ms`: CPU 平均耗时（单位毫秒）
- `backend.cpu.p50_ms`: CPU 中位数耗时
- `backend.cpu.p95_ms`: CPU 95% 分位耗时
- `backend.cuda.available`: CUDA 通道是否可用
- `backend.cuda.error`: CUDA 不可用/运行异常原因
- `accuracy.max_abs_diff`: CPU 与 CUDA 输出最大绝对差
- `accuracy.speedup_cpu_vs_cuda`: CPU/CUDA 时延比（CUDA 可用时给出）

## 快速验收

在无 CUDA 的环境，执行以下命令应始终成功返回：

```bash
./scripts/ricci_cuda_poc.py --disable-cuda
```

预期：
- `backend.cuda.available` 为 `false`
- 仍有完整 CPU 指标
- 进程退出码为 `0`

在有 CUDA 环境下，建议记录：
- `backend.cuda.avg_ms` 与 `backend.cpu.avg_ms`
- `accuracy.max_abs_diff`（应接近 0）
- `accuracy.speedup_cpu_vs_cuda`（>1 表示 CUDA 更快）
