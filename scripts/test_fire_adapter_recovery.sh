#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE_COUNT="${1:-6}"
TOTAL_CORES="${2:-6}"
VIS_PORT="${3:-8913}"
PLANNER_MODE="${4:-coverage}"
GOOD_CSV="${5:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc.csv}"
MAPPING_JSON="${6:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc_mapping.json}"
PUBLISH_HZ="${7:-2.0}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

TMP_BAD_CSV="/tmp/fds_bad_packet_devc.csv"
TMP_DRIFT_CSV="/tmp/fds_drift_devc.csv"
NEW_FIRE_PID=""

cleanup() {
  if [[ -n "${NEW_FIRE_PID}" ]]; then
    kill "${NEW_FIRE_PID}" >/dev/null 2>&1 || true
  fi
  "${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" "${VIS_PORT}" >/dev/null 2>&1 || true
  rm -f "${TMP_BAD_CSV}" "${TMP_DRIFT_CSV}"
}
trap cleanup EXIT

fetch_counts() {
  python3 - <<PY
import json,urllib.request
port=int('${VIS_PORT}')
url=f'http://127.0.0.1:{port}/api/swarm_state'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    d=json.loads(opener.open(url,timeout=1.5).read().decode('utf-8'))
except Exception:
    print('-1 -1 -1 -1 -1 -1 -1')
    raise SystemExit(0)
ms=d.get('mission_status',[]) or []
exec_n=sum(1 for x in ms if int(x.get('state',0))==2)
done_n=sum(1 for x in ms if int(x.get('state',0))==3)
fail_n=sum(1 for x in ms if int(x.get('state',0))==4)
print(len(d.get('uavs',[]) or []),len(d.get('fire_hotspots',[]) or []),len(d.get('mission_targets',[]) or []),len(ms),exec_n,done_n,fail_n)
PY
}

wait_ready() {
  python3 - <<PY
import json,time,urllib.request
port=int('${VIS_PORT}')
url=f'http://127.0.0.1:{port}/api/swarm_state'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
end=time.time()+20
while time.time()<end:
    try:
        d=json.loads(opener.open(url,timeout=1.5).read().decode('utf-8'))
        if len(d.get('uavs',[]))>0 and len(d.get('fire_hotspots',[]))>0 and len(d.get('mission_targets',[]))>0 and len(d.get('mission_status',[]))>0:
            print('ready')
            raise SystemExit(0)
    except Exception:
        pass
    time.sleep(0.5)
print('timeout')
raise SystemExit(1)
PY
}

restart_fire_adapter() {
  local input_csv="$1"
  if [[ -n "${NEW_FIRE_PID}" ]]; then
    kill "${NEW_FIRE_PID}" >/dev/null 2>&1 || true
  fi
  python3 "${ROOT_DIR}/scripts/fire_adapter_fds.py" --ros-args \
    -p input_path:="${input_csv}" \
    -p input_format:=fds_devc_csv \
    -p publish_hz:="${PUBLISH_HZ}" \
    -p output_topic:=/env/fire_state \
    -p mode:=offline \
    -p fds_devc_mapping:="${MAPPING_JSON}" \
    > /tmp/fire_adapter_fds_recovery.log 2>&1 &
  NEW_FIRE_PID=$!
}

if [[ ! -f "${GOOD_CSV}" ]]; then
  echo "[test_fire_adapter_recovery] GOOD_CSV 不存在，尝试生成..."
  if ! "${ROOT_DIR}/scripts/fds_run_min_case.sh" >/tmp/fds_run_recovery.log 2>&1; then
    echo "[test_fire_adapter_recovery] FAIL: 生成 FDS CSV 失败，见 /tmp/fds_run_recovery.log"
    exit 1
  fi
fi

if [[ ! -f "${GOOD_CSV}" || ! -f "${MAPPING_JSON}" ]]; then
  echo "[test_fire_adapter_recovery] FAIL: 输入文件缺失"
  exit 1
fi

"${ROOT_DIR}/scripts/fire_mission_fds_start.sh" \
  "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${VIS_PORT}" "${PLANNER_MODE}" \
  "${GOOD_CSV}" "${MAPPING_JSON}" "${PUBLISH_HZ}" \
  >/tmp/fire_mission_fds_start_recovery.log 2>&1

if ! wait_ready >/dev/null; then
  echo "[test_fire_adapter_recovery] FAIL: 初始链路未就绪"
  exit 2
fi

read -r MANAGER_PID TOPO_PID FIRE_PID PLANNER_PID MISSION_STATUS_PID VIS_PID < "${ROOT_DIR}/.fire_demo_pids" || true

# 1) 断流故障：杀掉 fire adapter
kill "${FIRE_PID}" >/dev/null 2>&1 || true
sleep 3
read -r u1 f1 m1 s1 e1 d1 x1 <<<"$(fetch_counts)"
if (( u1 <= 0 )) || (( s1 < 0 )); then
  echo "[test_fire_adapter_recovery] FAIL: 断流后系统不可用"
  exit 3
fi

# 恢复：重启正确 adapter
restart_fire_adapter "${GOOD_CSV}"
if ! wait_ready >/dev/null; then
  echo "[test_fire_adapter_recovery] FAIL: 断流恢复失败"
  exit 4
fi

# 2) 坏包故障：构造列名错误 CSV
python3 - <<PY
from pathlib import Path
src=Path('${GOOD_CSV}').read_text(encoding='utf-8').splitlines()
if len(src)<3:
    raise SystemExit(1)
src[0]='"Time","BROKEN_A","BROKEN_B"'
Path('${TMP_BAD_CSV}').write_text('\n'.join(src)+'\n',encoding='utf-8')
PY

restart_fire_adapter "${TMP_BAD_CSV}"
sleep 2
if kill -0 "${NEW_FIRE_PID}" >/dev/null 2>&1; then
  echo "[test_fire_adapter_recovery] FAIL: 坏包注入后 fire adapter 未失败退出"
  exit 5
fi
read -r u2 f2 m2 s2 e2 d2 x2 <<<"$(fetch_counts)"
if (( u2 <= 0 )) || (( s2 < 0 )); then
  echo "[test_fire_adapter_recovery] FAIL: 坏包阶段系统不可用"
  exit 6
fi

# 坏包恢复
restart_fire_adapter "${GOOD_CSV}"
if ! wait_ready >/dev/null; then
  echo "[test_fire_adapter_recovery] FAIL: 坏包恢复失败"
  exit 7
fi

# 3) 时钟漂移注入：构造时间戳跳变/回拨 CSV
python3 - <<PY
import csv
from pathlib import Path
rows=list(csv.reader(open('${GOOD_CSV}','r',encoding='utf-8')))
if len(rows)<4:
    raise SystemExit(1)
out=[]
for i,r in enumerate(rows):
    if i<2:
        out.append(r)
        continue
    t=float(r[0])
    if i%3==0:
        t=t+120.0
    elif i%5==0:
        t=max(0.0,t-60.0)
    r2=list(r)
    r2[0]=f"{t:.3f}"
    out.append(r2)
with open('${TMP_DRIFT_CSV}','w',encoding='utf-8',newline='') as f:
    csv.writer(f).writerows(out)
PY

restart_fire_adapter "${TMP_DRIFT_CSV}"
sleep 3
read -r u3 f3 m3 s3 e3 d3 x3 <<<"$(fetch_counts)"
if (( u3 <= 0 )) || (( s3 <= 0 )); then
  echo "[test_fire_adapter_recovery] FAIL: 时钟漂移阶段系统不可用"
  exit 8
fi

# 最终恢复到标准输入
restart_fire_adapter "${GOOD_CSV}"
if ! wait_ready >/dev/null; then
  echo "[test_fire_adapter_recovery] FAIL: 时钟漂移恢复失败"
  exit 9
fi

read -r u4 f4 m4 s4 e4 d4 x4 <<<"$(fetch_counts)"
echo "[test_fire_adapter_recovery] after_recovery uavs=${u4} fire=${f4} mission=${m4} status=${s4} exec=${e4} done=${d4} fail=${x4}"
echo "[test_fire_adapter_recovery] PASS: 断流/坏包/时钟漂移注入与恢复验证通过"
