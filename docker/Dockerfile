FROM ubuntu:20.04
ENV DEBIAN_FRONTEND noninteractive

# Choose ROS & ROS2 distributions
ENV ROS_DISTRO noetic
ENV ROS2_DISTRO foxy

# Setting locale
RUN echo "-----Setting up locale-----"
RUN apt-get update && apt-get install -y locales && \
    dpkg-reconfigure locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8
ENV LC_ALL   en_US.UTF-8
ENV LANG     en_US.UTF-8
ENV LANGUAGE en_US.UTF-8


# Setting apt source list
RUN echo "-----UPDATE & Install apt sources-----"
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release git vim wget unzip net-tools

# Install Python
RUN echo "-----Install Python-----"
RUN apt install python3-pip -y && \
    apt install libgl1-mesa-dev -y && \
    pip3 install opencv-python && \
    pip3 install mysql-connector-python 

# Install ROS1
RUN echo "-----Install ROS-----"
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get update && apt-get install -y ros-$ROS_DISTRO-desktop 
    

# Install ROS2
RUN echo "-----Install ROS2-----"
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y ros-$ROS2_DISTRO-desktop ros-foxy-rmw-cyclonedds-cpp

# Install zenoh
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null && \
    apt-get update -y && \
    apt-get install -y zenoh-bridge-ros2dds || true

RUN apt update
RUN apt install python3-argcomplete
RUN apt update

# Install ROS plugins
RUN echo "-----Install ROS Plugins-----"
RUN apt-get install -y python3-catkin-tools python3-colcon-common-extensions python3-rosdep python3-argcomplete

WORKDIR /home

RUN mkdir -p /custom_msg_ros1_bridge/ros1_ws/src/
# Setting ros1_pkg
RUN echo "-----Execution of ros1 environment construction-----"
RUN cd /custom_msg_ros1_bridge/ros1_ws 
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash;"
# Setting ros2_pkg
RUN echo "-----Execution of ros2 environment construction-----"
RUN mkdir -p /custom_msg_ros1_bridge/ros2_ws/src/
RUN cd ../custom_msg_ros1_bridge/ros2_ws
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash;"


RUN mkdir -p /custom_msg_ros1_bridge/bridge_ws/src/
RUN mkdir -p /custom_msg_ros1_bridge/docker


# .basrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
#RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
#RUN echo "export CYCLONEDDS_URI=file:///home/custom_msg_ros1_bridge/docker/cyclonedds.xml" >> ~/.bashrc

# Copy the cyclonedds.xml configuration file
# COPY docker/cyclonedds.xml /etc/cyclonedds.xml
COPY docker/cyclonedds.xml /home/custom_msg_ros1_bridge/docker/cyclonedds.xml
# Set environment variables
ENV CYCLONEDDS_URI=file:////home/custom_msg_ros1_bridge/docker/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# RUN echo "export CYCLONEDDS_URI=file:///etc/cyclonedds.xml" >> ~/.bashrc

RUN echo "source /home/custom_msg_ros1_bridge/bridge_ws/install/setup.bash" >> ~/.bashrc

#RUN echo "export FASTRTPS_DEFAULT_PROFILES_FILE=robot_1.xml" >> ~/.bashrc

# Setting ros1_bridgecyclonedds.xml
#     git clone https://github.com/ros2/ros1_bridge.git
# RUN cd /ros1_bridge_ws && \
#     /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; source /opt/ros/$ROS2_DISTRO/setup.bash; colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

# Starting directory
RUN echo "-----Setup is complete!-----"
RUN echo "-----The starting directory is /custom_msg_ros1_bridge.-----"
WORKDIR /home/custom_msg_ros1_bridge/docker

# COPY docker/bridge.yaml /

COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]