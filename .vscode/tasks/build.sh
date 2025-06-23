#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
# Check if the build was successful
if [ $? -eq 0 ]; then
        source install/setup.bash
        echo "Build successful. Environment sourced."
else
        echo "Build failed. Please check the output for errors."
        exit 1
fi