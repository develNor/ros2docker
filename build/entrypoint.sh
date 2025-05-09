#!/bin/bash
set -e
set -x

echo "**** Entrypoint script starting ****"

if [ -d "/shared/ros2ws/src" ]; then
    if [[ "${CHECK_ROS2WS_DEPENDENCIES:-0}" == "1" ]]; then
        echo "Checking for missing dependencies..."
        rosdep update --rosdistro ${ROS_DISTRO}
        output=$(rosdep install --from-paths /shared/ros2ws/src --ignore-src --simulate 2>&1)
        if [ -z "$output" ]; then
            echo "All dependencies are satisfied."
        else
            echo "Dependencies are missing. Add these installations to the Dockerfile:"
            echo "$output"
            exit 1
        fi
    else
        echo "Skipping dependency check."
    fi
    if [[ "${BUILD_ROS2WS:-0}" == "1" ]]; then
        echo "Building ROS 2 workspace..."
        pushd /shared/ros2ws
        colcon build
        popd
    else
        echo "Skipping building ROS 2 workspace."
    fi
    exec "$@"
else
    echo "ERROR: No ROS 2 workspace found at /shared/ros2ws/src"
    exit 1
fi