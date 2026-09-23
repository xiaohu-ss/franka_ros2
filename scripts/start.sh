#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="/home/tmr-user/ros2_ws"
LOG_DIR="${WORKSPACE_DIR}/logs"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ROSBRIDGE_LOG="${LOG_DIR}/rosbridge_${TIMESTAMP}.log"
TMR_LAUNCH_LOG="${LOG_DIR}/tmr_launch_${TIMESTAMP}.log"
ROSBRIDGE_PID_FILE="${LOG_DIR}/rosbridge.pid"
TMR_LAUNCH_PID_FILE="${LOG_DIR}/tmr_launch.pid"

ROSBRIDGE_PID=""
TMR_LAUNCH_PID=""

cleanup() {
  set +e

  if [[ -n "${ROSBRIDGE_PID}" ]] && kill -0 "${ROSBRIDGE_PID}" 2>/dev/null; then
    kill "${ROSBRIDGE_PID}"
  fi

  if [[ -n "${TMR_LAUNCH_PID}" ]] && kill -0 "${TMR_LAUNCH_PID}" 2>/dev/null; then
    kill "${TMR_LAUNCH_PID}"
  fi

  wait "${ROSBRIDGE_PID}" 2>/dev/null
  wait "${TMR_LAUNCH_PID}" 2>/dev/null

  rm -f "${ROSBRIDGE_PID_FILE}" "${TMR_LAUNCH_PID_FILE}"
}

handle_signal() {
  cleanup
  exit 143
}

trap cleanup EXIT
trap handle_signal INT TERM

mkdir -p "${LOG_DIR}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup file: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE_SETUP}" >&2
  echo "Run colcon build in ${WORKSPACE_DIR} before starting this service." >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u

cd "${WORKSPACE_DIR}"

echo "Starting rosbridge_server. Log: ${ROSBRIDGE_LOG}"
ros2 run rosbridge_server rosbridge_websocket \
  --ros-args \
  -p address:=0.0.0.0 \
  -p port:=9090 \
  -p call_services_in_new_thread:=true \
  >"${ROSBRIDGE_LOG}" 2>&1 &
ROSBRIDGE_PID="$!"
echo "${ROSBRIDGE_PID}" >"${ROSBRIDGE_PID_FILE}"

echo "Starting TMR Nav2 bringup. Log: ${TMR_LAUNCH_LOG}"
ros2 launch franka_bringup tmrv0_2.launch.py \
  robot_config_file:=tmr.config.yaml \
  controller_name:=swerve_drive_controller \
  use_nav2:=true \
  >"${TMR_LAUNCH_LOG}" 2>&1 &
TMR_LAUNCH_PID="$!"
echo "${TMR_LAUNCH_PID}" >"${TMR_LAUNCH_PID_FILE}"

echo "rosbridge_server pid: ${ROSBRIDGE_PID}"
echo "tmr launch pid: ${TMR_LAUNCH_PID}"

set +e
wait -n "${ROSBRIDGE_PID}" "${TMR_LAUNCH_PID}"
EXIT_CODE="$?"
set -e

echo "A managed process exited; stopping remaining processes." >&2
exit "${EXIT_CODE}"
