#!/usr/bin/env bash
# immediately start the service with sudo systemctl start franka-tmr.service
set -euo pipefail

SERVICE_NAME="franka-tmr.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
WORKSPACE_DIR="/home/tmr-user/ros2_ws"
REPO_DIR="${WORKSPACE_DIR}/src/franka_ros2"
START_SCRIPT="${REPO_DIR}/scripts/start.sh"

if [[ ! -f "${START_SCRIPT}" ]]; then
  echo "Missing start script: ${START_SCRIPT}" >&2
  exit 1
fi

chmod +x "${START_SCRIPT}"

sudo tee "${SERVICE_PATH}" >/dev/null <<EOF
[Unit]
Description=Franka TMR rosbridge and Nav2 bringup
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=tmr-user
WorkingDirectory=${WORKSPACE_DIR}
ExecStart=${START_SCRIPT}
Restart=on-failure
RestartSec=5
KillMode=control-group
TimeoutStopSec=30
LimitRTPRIO=99
LimitMEMLOCK=infinity
LimitNICE=-20
AmbientCapabilities=CAP_SYS_NICE
CapabilityBoundingSet=CAP_SYS_NICE

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"

echo "Installed and enabled ${SERVICE_NAME}."
echo "It will start automatically on next boot."
echo "To start it now, run: sudo systemctl start ${SERVICE_NAME}"
echo "To inspect logs, run: journalctl -u ${SERVICE_NAME} -f"
