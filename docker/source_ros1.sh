# How to call this script:
# source source_ros1.sh

echo "Sourcing ROS1 (Noetic)"
source /opt/ros/noetic/setup.bash
echo "Sourcing local ROS1 workspace"
source ../ros1_ws/devel/setup.bash

rosparam load bridge.yaml
