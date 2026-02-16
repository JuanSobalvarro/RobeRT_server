#!/bin/bash

# on error exit immediately
set -e

echo "Compiling the server..."

colcon build --symlink-install --packages-select robert_server

echo "Running the server..."

# check if the build was successful
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# check if the executable exists
if [ ! -f install/robert_server/lib/robert_server/robert_server ]; then
    echo "Executable not found. Exiting."
    exit 1
fi

source install/setup.bash
ros2 run robert_server robert_server
