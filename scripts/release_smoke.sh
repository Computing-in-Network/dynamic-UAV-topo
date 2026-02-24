#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

log() { echo "[release_smoke] $*"; }
fail() { log "FAIL: $*"; exit 1; }

echo "[release_smoke] start"

if ! python3 - <<'PY'
import importlib.util
import sys
if importlib.util.find_spec("numpy") is None:
    print("missing_numpy")
    sys.exit(1)
PY
then
  fail "缺少 numpy 依赖，请先执行: pip install numpy"
fi

for py in scripts/*.py tests/*.py; do
  log "compile ${py}"
  python3 -m py_compile "${py}"
done

for f in tests/test_*.py; do
  log "run ${f}"
  python3 "${f}"
done

log "run ricci cuda poc (no-cuda mode)"
if ! python3 - <<'PY'
import json
import subprocess
import sys

cmd = [
    "python3",
    "scripts/ricci_cuda_poc.py",
    "--disable-cuda",
    "--nodes", "12",
    "--density", "0.2",
    "--seed", "2026",
    "--repeat", "2",
]
res = subprocess.run(cmd, check=False, capture_output=True, text=True)
if res.returncode != 0:
    print("ricci_cuda_poc_exit_nonzero")
    sys.exit(2)

try:
    obj = json.loads(res.stdout.strip())
except json.JSONDecodeError:
    print("ricci_cuda_poc_output_invalid")
    sys.exit(3)

backend = obj.get("backend", {}).get("cuda", {})
if not isinstance(backend, dict) or "available" not in backend:
    print("ricci_cuda_poc_no_backend")
    sys.exit(4)
PY
then
  fail "ricci_cuda_poc 回归检查未通过"
fi

log "PASS: release smoke"
