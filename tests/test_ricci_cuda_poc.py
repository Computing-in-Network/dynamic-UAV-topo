#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from ricci_cuda_poc import build_graph, curvature_proxy_cpu, _build_metrics  # noqa: E402


def assert_true(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def test_build_graph_reproducible() -> None:
    g1 = build_graph(12, 0.2, 42)
    g2 = build_graph(12, 0.2, 42)
    assert_true(np.array_equal(g1, g2), "相同种子应复现一致的拓扑")
    assert_true(g1.shape == (12, 12), "图矩阵形状应为 NxN")
    assert_true(np.allclose(g1, g1.T), "拓扑矩阵应对称")
    assert_true(np.all(np.diag(g1) == 0), "无自环")


def test_build_graph_zero_density() -> None:
    g = build_graph(16, 0.0, 7)
    assert_true(int(g.sum()) == 0, "密度 0 时应没有边")


def test_curvature_cpu_no_divide() -> None:
    g = np.zeros((5, 5), dtype=np.float32)
    out = curvature_proxy_cpu(g)
    assert_true(np.all(out == 0.0), "全空图 curvature 应为 0")


def test_build_metrics_empty() -> None:
    m = _build_metrics("cpu", [], available=True)
    assert_true(m.repeats == 0, "空统计的 repeats 应为 0")
    assert_true(m.avg_ms == 0.0, "空统计平均耗时应为 0")
    assert_true(m.p50_ms == 0.0, "空统计中位数应为 0")


def main() -> None:
    test_build_graph_reproducible()
    test_build_graph_zero_density()
    test_curvature_cpu_no_divide()
    test_build_metrics_empty()
    print("test_ricci_cuda_poc: PASS")


if __name__ == "__main__":
    main()
