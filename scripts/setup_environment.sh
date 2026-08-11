#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash"
  echo "Install ROS 2 Humble first, then rerun this script."
  exit 1
fi

echo "[1/4] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-yaml \
  libyaml-cpp-dev \
  libspdlog-dev \
  libboost-all-dev \
  libglfw3-dev \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-planners-chomp \
  ros-humble-pilz-industrial-motion-planner \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

echo "[2/4] Installing Python dependencies..."
python3 -m pip install --user -r requirements.txt
python3 -m pip install --user -e ./pnd_sdk_python

echo "[3/4] Building ROS2 workspace..."
source /opt/ros/humble/setup.bash
colcon build --packages-select pnd_adam_description pnd_ros2_control pnd_adam_moveit_config

echo "[4/4] Done."
echo "Run: bash start_pnd_adam.sh"
