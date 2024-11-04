# Custom interface bridging between ROS 1 & 2

This repository gives an example how to bridge custom interfaces between ROS 1 & 2 using the ros1_bridge package (https://github.com/ros2/ros1_bridge).
At the time of writing this, it's possible to bridge topics, services & `tf2_msgs`, but not actions.


```
### on host RUN

# in terminal 1
cd custom_msg_ros1_bridge/docker
bash build.sh
docker compose up

# in terminal 2
roscore

# in terminal 3
source source_ros1.sh
rosparam load bridge.yaml
source source_bridge.sh
ros2 run ros1_bridge parameter_bridge
```

```
### Echo the topic in ros1
source source_ros1.sh
rostopic echo /topic_name
```

