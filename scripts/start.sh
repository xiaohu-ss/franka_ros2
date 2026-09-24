#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="/home/tmr-user/ros2_ws"
LOG_DIR="${WORKSPACE_DIR}/logs"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
FRANKA_ROBOT_IP="172.16.16.10"
SPINE_IP="tmr"
FRANKA_WAIT_TIMEOUT=120

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ROSBRIDGE_LOG="${LOG_DIR}/rosbridge_${TIMESTAMP}.log"
TMR_LAUNCH_LOG="${LOG_DIR}/tmr_launch_${TIMESTAMP}.log"
SPINE_LAUNCH_LOG="${LOG_DIR}/spine_launch_${TIMESTAMP}.log"
ROSBRIDGE_PID_FILE="${LOG_DIR}/rosbridge.pid"
TMR_LAUNCH_PID_FILE="${LOG_DIR}/tmr_launch.pid"
SPINE_LAUNCH_PID_FILE="${LOG_DIR}/spine_launch.pid"

ROSBRIDGE_PID=""
TMR_LAUNCH_PID=""
SPINE_LAUNCH_PID=""

stop_process_group() {
  local pid="$1"

  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -- "-${pid}" 2>/dev/null
  fi
}

stop_stale_managed_processes() {
  pkill -TERM -f 'rosbridge_server/rosbridge_websocket' 2>/dev/null || true
  pkill -TERM -f 'rosbridge_server rosbridge_websocket' 2>/dev/null || true
  pkill -TERM -f 'franka_spine_server spine.launch.py' 2>/dev/null || true
  pkill -TERM -f 'spine_action_server_node.py' 2>/dev/null || true
  pkill -TERM -f 'franka_bringup tmrv0_2.launch.py' 2>/dev/null || true
  sleep 1
}

cleanup() {
  set +e

  stop_process_group "${ROSBRIDGE_PID}"
  stop_process_group "${TMR_LAUNCH_PID}"
  stop_process_group "${SPINE_LAUNCH_PID}"

  wait "${ROSBRIDGE_PID}" 2>/dev/null
  wait "${TMR_LAUNCH_PID}" 2>/dev/null
  wait "${SPINE_LAUNCH_PID}" 2>/dev/null

  rm -f "${ROSBRIDGE_PID_FILE}" "${TMR_LAUNCH_PID_FILE}" "${SPINE_LAUNCH_PID_FILE}"
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

echo "Waiting for Franka robot at ${FRANKA_ROBOT_IP}..."
FRANKA_WAIT_DEADLINE=$((SECONDS + FRANKA_WAIT_TIMEOUT))
until ping -c1 -W1 "${FRANKA_ROBOT_IP}" >/dev/null 2>&1; do
  if (( SECONDS >= FRANKA_WAIT_DEADLINE )); then
    echo "Timed out waiting for Franka robot at ${FRANKA_ROBOT_IP}" >&2
    exit 1
  fi
  sleep 2
done
echo "Franka robot is reachable at ${FRANKA_ROBOT_IP}."

echo "Stopping stale managed rosbridge/spine/TMR processes, if any."
stop_stale_managed_processes

echo "Starting rosbridge_server. Log: ${ROSBRIDGE_LOG}"
setsid ros2 run rosbridge_server rosbridge_websocket \
  --ros-args \
  -p address:=0.0.0.0 \
  -p port:=9090 \
  -p call_services_in_new_thread:=true \
  >"${ROSBRIDGE_LOG}" 2>&1 &
ROSBRIDGE_PID="$!"
echo "${ROSBRIDGE_PID}" >"${ROSBRIDGE_PID_FILE}"

echo "Starting Spine bringup. Log: ${SPINE_LAUNCH_LOG}"
setsid ros2 launch franka_spine_server spine.launch.py \
  spine_ip:="${SPINE_IP}" \
  >"${SPINE_LAUNCH_LOG}" 2>&1 &
SPINE_LAUNCH_PID="$!"
echo "${SPINE_LAUNCH_PID}" >"${SPINE_LAUNCH_PID_FILE}"

echo "Starting TMR Nav2 bringup. Log: ${TMR_LAUNCH_LOG}"
setsid ros2 launch franka_bringup tmrv0_2.launch.py \
  robot_config_file:=tmr.config.yaml \
  controller_name:=swerve_drive_controller \
  use_nav2:=true \
  >"${TMR_LAUNCH_LOG}" 2>&1 &
TMR_LAUNCH_PID="$!"
echo "${TMR_LAUNCH_PID}" >"${TMR_LAUNCH_PID_FILE}"

echo "rosbridge_server pid: ${ROSBRIDGE_PID}"
echo "tmr launch pid: ${TMR_LAUNCH_PID}"
echo "spine launch pid: ${SPINE_LAUNCH_PID}"

set +e
wait -n "${ROSBRIDGE_PID}" "${TMR_LAUNCH_PID}" "${SPINE_LAUNCH_PID}"
EXIT_CODE="$?"
set -e

echo "A managed process exited; stopping remaining processes." >&2
exit "${EXIT_CODE}"
