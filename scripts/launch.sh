#!/bin/bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
bash scripts/display.sh
exec ros2 launch ab_robot demo.launch.py "$@"
