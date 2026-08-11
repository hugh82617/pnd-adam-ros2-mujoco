#!/usr/bin/env bash
set -u

ROOT="$HOME/pnd_adam_ros2_publish"
SIM_DIR="$HOME/pnd_mujoco/simulate_python"
LOG_DIR="${PND_LOG_DIR:-/tmp}"
PIDS=()
export ROOT SIM_DIR

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
