#!/usr/bin/env bash

set -u

log_info() {
  local tag="$1"
  shift
  echo "[${tag}][INFO] $*"
}

log_warn() {
  local tag="$1"
  shift
  echo "[${tag}][WARN] $*" >&2
}

log_error() {
  local tag="$1"
  local code="$2"
  shift 2
  echo "[${tag}][ERROR][${code}] $*" >&2
}

pick_available_port() {
  local base_port="$1"
  local span="${2:-50}"
  python3 - <<PY
import socket
base = int("${base_port}")
span = int("${span}")
for p in range(base, base + span):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", p))
        print(p)
        break
    except OSError:
        pass
    finally:
        s.close()
else:
    print(-1)
PY
}

kill_pid_graceful() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  kill "${pid}" >/dev/null 2>&1 || true
  for _ in $(seq 1 15); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  kill -9 "${pid}" >/dev/null 2>&1 || true
}

kill_pattern() {
  local pattern="$1"
  pkill -f "${pattern}" >/dev/null 2>&1 || true
  sleep 0.2
  pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
}

kill_port_listener() {
  local port="$1"
  if [[ -z "${port}" ]]; then
    return 0
  fi
  local pids=""

  if command -v lsof >/dev/null 2>&1; then
    pids="$(lsof -tiTCP:${port} -sTCP:LISTEN 2>/dev/null | sort -u || true)"
  fi

  if [[ -z "${pids}" ]] && command -v fuser >/dev/null 2>&1; then
    pids="$(fuser -n tcp "${port}" 2>/dev/null | tr ' ' '\n' | sed '/^$/d' | sort -u || true)"
  fi

  if [[ -z "${pids}" ]]; then
    pids="$(ss -ltnp 2>/dev/null | sed -n "s/.*:${port}[[:space:]].*pid=\([0-9]\+\).*/\1/p" | sort -u || true)"
  fi

  if [[ -n "${pids}" ]]; then
    for p in ${pids}; do
      kill_pid_graceful "${p}"
    done
  fi
}

