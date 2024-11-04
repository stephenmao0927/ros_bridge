# Readme

This repository provides a modified version of [ros1_bridge package](https://github.com/ros2/ros1_bridge), which bridges custom interfaces between ROS 1 & 2. It allows users to get a extremely smooth way to enable the module.

## 1 To run

The whole module is dockerized. Download docker image by:

```bash
docker pull stephenmao0927/ros1_bridge:v1
```

Run a  script to enable the module:

```bash
./run_bridge.sh
```

It will compose a container which will automatically be deleted if you type `ctrl+C` in terminal to stop the module. If you use it for the first time, it will build `ros bridge ` module and the process will take about 5 minutes.

## 2 To Customize

Find `bridge.yaml` in `docker` folder. Modify the topics you want to transfer between ROS1 and ROS2. Examples are provided in the file.

To set IP address to get the module working properly, find `cyclonedds.xml` and input the IP address:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain>
        <General>
            !!!!!!!!!!!!!!!!! PUT YOUR HOST IP ADDRESS HERE !!!!!!!!!!!!!!!!!!
            <DontRoute>true</DontRoute>
        </General>       
    </Domain>
</CycloneDDS> 
```

## 3 To be noted

1. The current version of script `run_bridge.sh` is designed for **docker-compose**. If you are using **docker compose**, you will need to make changes.

2. We have a test script which publish a ros2 message. If the bridge works fine, the topic is supposed to be seen by using `rostopic` commands with ROS 1. To test, enter the docker and run:

   ```bash
   ros2 run my_ros2_pkg my_publisher
   ```

   
