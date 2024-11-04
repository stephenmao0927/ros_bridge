#!/bin/bash

sleep 1

x-terminal-emulator -e bash -c "./in_container.sh" &

echo "Container $CONTAINER_NAME has been accessed and script $SCRIPT_TO_RUN is running in a new terminal."
