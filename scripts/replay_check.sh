#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play artifacts/final-video/rosbag --clock --topics \
    /tf /tf_static /odom /scan /depth /depth/points /gps/fix \
    /plan /ground_truth /global_costmap/costmap /route_status \
    > artifacts/replay.log 2>&1 &
play_pid=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p use_sim_time:=true \
    -p robot_description:="$(xacro src/ab_robot/urdf/robot.urdf.xacro)" \
    -r /tf:=/replay_unused_tf -r /tf_static:=/replay_unused_tf_static \
    > artifacts/replay-rsp.log 2>&1 &
rsp_pid=$!
trap 'kill -TERM "$play_pid" "$rsp_pid" 2>/dev/null || true; wait || true' EXIT
python3 scripts/graph_evidence.py artifacts/replay-graph.json
