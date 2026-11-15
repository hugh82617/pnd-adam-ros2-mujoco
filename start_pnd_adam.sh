#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$ROOT/pnd_mujoco/simulate_python"
LOG_DIR="${PND_LOG_DIR:-/tmp}"
PIDS=()
export ROOT SIM_DIR

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "Workspace is not built. Run: bash scripts/setup_environment.sh" >&2
  exit 1
fi
if [ ! -f "$SIM_DIR/pnd_mujoco.py" ]; then
  echo "MuJoCo entry point was not found: $SIM_DIR/pnd_mujoco.py" >&2
  exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
  echo "setsid is required (Ubuntu package: util-linux)." >&2
  exit 1
fi
mkdir -p "$LOG_DIR" || exit 1

# ROS setup scripts may reference unset variables. Load them before starting
# MuJoCo as well, since the simulator imports rclpy even in DDS mode.
set +u
source /opt/ros/humble/setup.bash || exit 1
source "$ROOT/install/setup.bash" || exit 1
set -u

cleanup() {
  echo ""
  echo "Stopping PND Adam stack..."
  for pid in "${PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  sleep 1
  wait 2>/dev/null || true
  echo "Stopped."
}

trap cleanup EXIT INT TERM

setsid bash -c 'cd "$SIM_DIR" && exec python3 pnd_mujoco.py' \
  > "$LOG_DIR/pnd_mujoco.log" 2>&1 &
PIDS+=($!)

setsid bash -c '
  source /opt/ros/humble/setup.bash
  cd "$ROOT"
  source install/setup.bash
  exec ros2 launch pnd_ros2_control pnd_control.launch.py
' > "$LOG_DIR/pnd_ros2_control.log" 2>&1 &
PIDS+=($!)

setsid bash -c '
  source /opt/ros/humble/setup.bash
  cd "$ROOT"
  source install/setup.bash
  exec ros2 launch pnd_adam_moveit_config moveit_bringup.launch.py \
    start_robot_state_publisher:=false
' > "$LOG_DIR/pnd_moveit.log" 2>&1 &
PIDS+=($!)

sleep 2

echo "PND Adam stack started."
echo "Logs:"
echo "  $LOG_DIR/pnd_mujoco.log"
echo "  $LOG_DIR/pnd_ros2_control.log"
echo "  $LOG_DIR/pnd_moveit.log"
echo "Press Ctrl-C to stop all processes."

tail -f \
  "$LOG_DIR/pnd_mujoco.log" \
  "$LOG_DIR/pnd_ros2_control.log" \
  "$LOG_DIR/pnd_moveit.log"
