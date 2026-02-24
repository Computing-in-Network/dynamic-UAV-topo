#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from dual_view_sync import evaluate_sync  # noqa: E402


def assert_true(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def test_pass_case() -> None:
    pairs = [
        (1000, [39.90, 116.40, 100.0], 1050, [39.90001, 116.40001, 101.0]),
        (1500, [39.9002, 116.4003, 100.0], 1520, [39.90021, 116.40029, 100.8]),
    ]
    r = evaluate_sync(pairs, max_dist_m=40.0, max_time_diff_ms=1000)
    assert_true(r["ok"], "应通过同步检查")
    assert_true(r["samples"] == 2, "样本计数应正确")


def test_fail_case() -> None:
    pairs = [
        (1000, [39.90, 116.40, 100.0], 5000, [39.91, 116.42, 180.0]),
    ]
    r = evaluate_sync(pairs, max_dist_m=30.0, max_time_diff_ms=1000)
    assert_true(not r["ok"], "应触发阈值失败")


def test_empty_case() -> None:
    r = evaluate_sync([], max_dist_m=30.0, max_time_diff_ms=1000)
    assert_true(not r["ok"], "空样本不应通过")
    assert_true(r["reason"] == "no_samples", "空样本原因应为 no_samples")


def main() -> None:
    test_pass_case()
    test_fail_case()
    test_empty_case()
    print("test_dual_view_sync: PASS")


if __name__ == "__main__":
    main()
