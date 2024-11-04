#!/bin/bash

cd ../ros2_ws
colcon build --packages-select my_ros2_pkg
cd ../docker
./source_ros2.sh
ros2 run my_ros2_pkg my_publisher