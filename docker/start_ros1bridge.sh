# How to call this script:
# source source_bridge.sh

# This script is used when you want to run or compile the ros1_bridge.

# Adjust this value if your roscore is located elsewhere
echo "Sourcing ROS1 (Noetic)"
source /opt/ros/noetic/setup.bash
rosparam load bridge.yaml
echo "Sourcing ROS2 (Foxy)"
source /opt/ros/foxy/setup.bash
echo "Sourcing bridge_ws"
source ../bridge_ws/install/setup.bash
ros2 run ros1_bridge parameter_bridge
