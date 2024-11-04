#!/bin/bash
set -e

#build ros1 msg
source /opt/ros/noetic/setup.bash
cd /home/custom_msg_ros1_bridge/ros1_ws && catkin_make

#build ro2 msg
source /opt/ros/foxy/setup.bash
cd /home/custom_msg_ros1_bridge/ros2_ws 
colcon build
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#build ros1_bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash0
source /home/custom_msg_ros1_bridge/ros1_ws/devel/setup.bash
source /home/custom_msg_ros1_bridge/ros2_ws/install/setup.bash

cd /home/custom_msg_ros1_bridge/bridge_ws 
colcon build --symlink-install --cmake-force-configure


# parameter bridge
# source /opt/ros/noetic/setup.bash

tail -f /dev/null


