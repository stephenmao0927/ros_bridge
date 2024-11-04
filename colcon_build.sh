#!/bin/bash

CONTAINER_NAME="ros1_bridge"      
TARGET_FOLDER="/home/custom_msg_ros1_bridge/bridge_ws"    
BUILD_COMMAND="colcon build"                     

docker exec -it "$CONTAINER_NAME" bash -c "source /opt/ros/foxy/setup.bash && source /opt/ros/noetic/setup.bash && cd $TARGET_FOLDER && $BUILD_COMMAND"