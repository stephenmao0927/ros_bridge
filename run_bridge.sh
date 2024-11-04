#!/bin/bash

docker-compose -f docker/docker-compose.yml up -d

FOLDER="../bridge_ws/log"

if [ ! -d "$FOLDER" ]; then
    echo "Folder $FOLDER does not exist. Running the setup script..."
    sh colcon_build.sh
    wait $!  # 等待最近一个后台进程结束
else 
    echo "Folder $FOLDER already exists. Skipping setup script."
fi

sh new_terminal.sh  # 这里也可以考虑是否需要等待该脚本执行完成

cleanup() {
    echo "Caught Ctrl+C! Cleaning up..."
    docker-compose -f docker/docker-compose.yml down
    exit 1
}

trap cleanup INT

while true; do
    echo "ROS bridge operating ..."
    sleep 1
done

echo "Normal shutdown"
docker-compose -f docker/docker-compose.yml down
