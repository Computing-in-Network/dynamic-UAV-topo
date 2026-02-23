#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_PATH="${1:-${ROOT_DIR}/data/fds_samples/fds_fire_snapshot.sample.json}"
INPUT_FORMAT="${2:-auto}"
PUBLISH_HZ="${3:-1.0}"
OUTPUT_TOPIC="${4:-/env/fire_state}"
MODE="${5:-offline}"
STREAM_DIR="${6:-${ROOT_DIR}/data/fds_samples/stream}"
STREAM_GLOB="${7:-*.json}"
CHECKPOINT_FILE="${8:-/tmp/fire_adapter_fds_checkpoint.json}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

exec python3 "${ROOT_DIR}/scripts/fire_adapter_fds.py" --ros-args \
  -p input_path:="${INPUT_PATH}" \
  -p input_format:="${INPUT_FORMAT}" \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p output_topic:="${OUTPUT_TOPIC}" \
  -p mode:="${MODE}" \
  -p stream_dir:="${STREAM_DIR}" \
  -p stream_glob:="${STREAM_GLOB}" \
  -p checkpoint_file:="${CHECKPOINT_FILE}"
