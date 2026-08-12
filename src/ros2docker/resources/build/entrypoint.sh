#!/bin/bash
set -e

if [[ "${ROS2DOCKER_TRACE:-0}" == "1" ]]; then
    set -x
fi

echo "**** Entrypoint script starting ****"

# shellcheck source=/dev/null  # generated at image build time; not present at lint time.
source "${CUSTOM_WS}/install/setup.bash"

source_ros2_workspace() {
    local setup_file="$1"
    if [[ ! -f "$setup_file" ]]; then
        return 0
    fi

    # shellcheck source=/dev/null  # runtime overlay path is not constant.
    source "$setup_file"
    echo "Sourced ROS 2 workspace overlay: $setup_file"
}

if [ -d "/ws/ros2src" ]; then
    echo "ROS 2 workspace found at /ws/ros2src"

    sudo mkdir -p /ros2ws
    sudo chown "$CONTAINER_USERNAME":"$CONTAINER_USERNAME" /ros2ws

    if [[ -L /ros2ws/src ]]; then
        ln -sfn /ws/ros2src /ros2ws/src
    elif [[ -e /ros2ws/src ]]; then
        echo "/ros2ws/src already exists and is not a symlink."
        exit 1
    else
        ln -s /ws/ros2src /ros2ws/src
    fi

    if [[ "${CHECK_ROS2WS_DEPENDENCIES:-0}" == "1" ]]; then
        echo "Checking for missing dependencies..."
        # sudo /opt/ros_venv/bin/pip install --upgrade rosdep # not sure if needed
        rosdep update --rosdistro "${ROS_DISTRO}"
        if rosdep install --from-paths /ros2ws/src --ignore-src --simulate; then
            echo "All dependencies are satisfied."
        else
            echo "Dependencies are missing. See rosdep output above."
            exit 1
        fi
    else
        echo "Skipping dependency check."
    fi

    if [[ "${BUILD_ROS2WS:-0}" == "1" ]]; then
        echo "Building ROS 2 workspace..."
        pushd /ros2ws >/dev/null
        colcon_args=()
        if [[ "${ROS2WS_SYMLINK_INSTALL:-1}" == "1" ]]; then
            colcon_args+=(--symlink-install)
        fi
        if [[ "${ROS2WS_SUPPRESS_UNUSED_CMAKE_WARNINGS:-1}" == "1" ]]; then
            colcon_args+=(--cmake-args --no-warn-unused-cli)
        fi
        # `python3 -m colcon`, not `colcon`: the executable on PATH is the apt
        # one from the ROS base image, whose own shebang pins it to
        # /usr/bin/python3 no matter where PATH points. setuptools then writes
        # that interpreter into every generated console script, and `ros2 run`
        # execs the script — so a node could not import anything from
        # PIP_PACKAGES, which is installed into the venv. Running colcon as a
        # module uses the venv python (first on PATH), which finds the
        # apt-installed colcon through --system-site-packages and stamps the
        # venv interpreter into the scripts it generates.
        python3 -m colcon build "${colcon_args[@]}"
        popd >/dev/null
    else
        echo "Skipping building ROS 2 workspace."
    fi

    source_ros2_workspace /ros2ws/install/setup.bash
else
    echo "No ROS 2 workspace found at /ws/ros2src. Skipping"
fi

exec "$@"
