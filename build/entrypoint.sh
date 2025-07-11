#!/bin/bash
set -e
set -x

echo "**** Entrypoint script starting ****"

source /opt/ros/jazzy/setup.bash

if [ -d "/ws/ros2src" ]; then
    echo "ROS 2 workspace found at /ws/ros2src"

    sudo mkdir -p /ros2ws
    sudo chown "$CONTAINER_USERNAME":"$CONTAINER_USERNAME" /ros2ws

    ln -s /ws/ros2src /ros2ws/src

    if [[ "${CHECK_ROS2WS_DEPENDENCIES:-0}" == "1" ]]; then
        echo "Checking for missing dependencies..."
        rosdep update --rosdistro ${ROS_DISTRO}
        output=$(rosdep install --from-paths /ros2ws/src --ignore-src --simulate 2>&1)
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
        pushd /ros2ws
        colcon build
        popd
    else
        echo "Skipping building ROS 2 workspace."
    fi
else
    echo "No ROS 2 workspace found at /ws/ros2src. Skipping"
fi

exec "$@"