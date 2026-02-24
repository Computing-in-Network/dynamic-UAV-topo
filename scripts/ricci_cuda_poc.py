#!/usr/bin/env python3
"""Ricci/CUDA PoC（最小可运行原型）.

该脚本用于评估将拓扑评分（基于共同邻居近似）从 CPU
迁移到 CUDA 的可行性：生成随机无向图、批量执行并记录时延分布，
再可选对比 CuPy 输出的一致性。
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    import cupy as cp
except Exception:  # pragma: no cover - 可选依赖
    cp = None


@dataclass
class BackendMetrics:
    name: str
    avg_ms: float
    p50_ms: float
    p95_ms: float
    min_ms: float
    max_ms: float
    repeats: int
    available: bool
    timings_ms: list[float]
    error: str | None = None


def build_graph(nodes: int, density: float, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    max_edges = nodes * (nodes - 1) // 2
    target_edges = int(max_edges * density)

    tri_rows, tri_cols = np.triu_indices(nodes, k=1)
    mask = np.zeros(max_edges, dtype=np.uint8)

    if target_edges > 0:
        picks = rng.choice(max_edges, size=target_edges, replace=False)
        mask[picks] = 1

    mat = np.zeros((nodes, nodes), dtype=np.float32)
    mat[tri_rows[mask == 1], tri_cols[mask == 1]] = 1.0
    return mat + mat.T


def curvature_proxy_cpu(adj: np.ndarray) -> np.ndarray:
    deg = adj.sum(axis=1)
    common = adj @ adj
    n = adj.shape[0]
    out = np.zeros((n, n), dtype=np.float32)
    rows, cols = np.where(np.triu(adj > 0, k=1))

    for i, j in zip(rows.tolist(), cols.tolist()):
        denom = float(min(deg[i], deg[j]))
        out[i, j] = out[j, i] = float(common[i, j] / denom) if denom > 0.0 else 0.0
    return out


def curvature_proxy_cuda(adj: np.ndarray) -> tuple[np.ndarray, float]:
    if cp is None:
        raise RuntimeError("缺少 CuPy 依赖")

    cupy = cp.asarray(adj)
    cp.cuda.Stream.null.synchronize()
    t0 = time.perf_counter()

    deg = cp.sum(cupy, axis=1)
    common = cupy @ cupy
    triu = cp.triu(cupy, k=1)
    idx = cp.where(triu > 0)
    i_idx = idx[0].astype(cp.int32)
    j_idx = idx[1].astype(cp.int32)
    denom = cp.maximum(cp.minimum(deg[i_idx], deg[j_idx]), 1.0)
    val = common[i_idx, j_idx] / denom

    out = cp.zeros_like(cupy)
    out[i_idx, j_idx] = val
    out[j_idx, i_idx] = val

    cp.cuda.Stream.null.synchronize()
    cost_ms = (time.perf_counter() - t0) * 1000.0
    return cp.asnumpy(out), cost_ms


def _run_cpu_bench(adj: np.ndarray, repeat: int, warmup: int) -> tuple[np.ndarray, list[float]]:
    timings_ms: list[float] = []
    output = None
    for _idx in range(repeat + warmup):
        t0 = time.perf_counter()
        output = curvature_proxy_cpu(adj)
        timings_ms.append((time.perf_counter() - t0) * 1000.0)
    return output, timings_ms[warmup:]


def _run_cuda_bench(
    adj: np.ndarray, repeat: int, warmup: int
) -> tuple[np.ndarray | None, BackendMetrics]:
    if cp is None:
        return None, BackendMetrics(
            name="cuda",
            avg_ms=0.0,
            p50_ms=0.0,
            p95_ms=0.0,
            min_ms=0.0,
            max_ms=0.0,
            repeats=0,
            available=False,
            timings_ms=[],
            error="not_installed",
        )

    timings_ms: list[float] = []
    out: np.ndarray | None = None
    for i in range(repeat + warmup):
        try:
            out, t_ms = curvature_proxy_cuda(adj)
        except Exception as e:  # pragma: no cover - 与 GPU 环境相关
            return None, BackendMetrics(
                name="cuda",
                avg_ms=0.0,
                p50_ms=0.0,
                p95_ms=0.0,
                min_ms=0.0,
                max_ms=0.0,
                repeats=0,
                available=False,
                timings_ms=[],
                error=f"{type(e).__name__}:{e}",
            )
        timings_ms.append(t_ms)
    return out, _build_metrics("cuda", timings_ms[warmup:], available=True)


def _build_metrics(name: str, timings: list[float], available: bool) -> BackendMetrics:
    if not timings:
        return BackendMetrics(
            name=name,
            avg_ms=0.0,
            p50_ms=0.0,
            p95_ms=0.0,
            min_ms=0.0,
            max_ms=0.0,
            repeats=0,
            available=available,
            timings_ms=[],
            error=None if available else "disabled",
        )

    values = sorted(timings)
    n = len(values)
    p95_index = int(0.95 * (n - 1))
    return BackendMetrics(
        name=name,
        avg_ms=statistics.mean(timings),
        p50_ms=values[n // 2],
        p95_ms=values[p95_index],
        min_ms=min(values),
        max_ms=max(timings),
        repeats=len(timings),
        available=available,
        timings_ms=timings,
        error=None,
    )


def _to_json_serializable(obj: Any) -> Any:
    if isinstance(obj, BackendMetrics):
        return {
            "name": obj.name,
            "avg_ms": round(obj.avg_ms, 6),
            "p50_ms": round(obj.p50_ms, 6),
            "p95_ms": round(obj.p95_ms, 6),
            "min_ms": round(obj.min_ms, 6),
            "max_ms": round(obj.max_ms, 6),
            "repeats": obj.repeats,
            "available": obj.available,
            "timings_ms": [round(v, 6) for v in obj.timings_ms],
            "error": obj.error,
        }
    return obj


def _calc_density(edges: int, nodes: int) -> float:
    max_edges = nodes * (nodes - 1) / 2.0
    if max_edges == 0:
        return 0.0
    return edges / max_edges


def main() -> int:
    parser = argparse.ArgumentParser(description="Ricci/CUDA PoC 可复现实验")
    parser.add_argument("--nodes", type=int, default=600)
    parser.add_argument("--density", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--repeat", type=int, default=6)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--disable-cuda", action="store_true")
    args = parser.parse_args()

    if args.nodes < 2:
        raise SystemExit("nodes 必须 >= 2")
    if not (0.0 <= args.density <= 1.0):
        raise SystemExit("density 必须在 [0, 1] 之间")
    if args.repeat < 1:
        raise SystemExit("repeat 必须 >= 1")
    if args.warmup < 0:
        raise SystemExit("warmup 必须 >= 0")

    adj = build_graph(args.nodes, args.density, args.seed)
    edge_count = int(adj.sum() / 2)

    cpu_out, cpu_timings = _run_cpu_bench(adj, repeat=args.repeat, warmup=args.warmup)
    cpu_metrics = _build_metrics("cpu", cpu_timings, available=True)

    cuda_out: np.ndarray | None = None
    cuda_metrics = BackendMetrics(
        name="cuda",
        avg_ms=0.0,
        p50_ms=0.0,
        p95_ms=0.0,
        min_ms=0.0,
        max_ms=0.0,
        repeats=0,
        available=False,
        timings_ms=[],
        error="disabled_by_flag",
    )

    if not args.disable_cuda:
        cuda_out, cuda_metrics = _run_cuda_bench(adj, repeat=args.repeat, warmup=args.warmup)

    max_abs_diff = None
    if cuda_out is not None and cuda_out.size and cpu_out is not None:
        max_abs_diff = float(np.max(np.abs(cpu_out - cuda_out)))

    result = {
        "input": {
            "nodes": args.nodes,
            "density": args.density,
            "seed": args.seed,
            "repeat": args.repeat,
            "warmup": args.warmup,
        },
        "graph": {
            "edges": edge_count,
            "undirected": True,
            "actual_density": _calc_density(edge_count, args.nodes),
        },
        "backend": {
            "cpu": cpu_metrics,
            "cuda": cuda_metrics,
        },
        "accuracy": {
            "max_abs_diff": max_abs_diff,
        },
    }

    if cuda_metrics.available and cpu_metrics.available and cuda_metrics.avg_ms > 0.0:
        result["accuracy"]["speedup_cpu_vs_cuda"] = cpu_metrics.avg_ms / cuda_metrics.avg_ms

    print(json.dumps(result, ensure_ascii=False, indent=2, default=_to_json_serializable))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
