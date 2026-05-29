# Use the official image for ROS2 Lyrical under Ubuntu 26.04
ARG BASE_IMAGE=osrf/ros:lyrical-desktop-full-resolute

FROM ${BASE_IMAGE}

##############################################################################
# 1) Base Environment
##############################################################################
ARG TZ=Europe/Berlin
ENV TZ=${TZ}
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

##############################################################################
# 2) Install Base Apt Dependencies
##############################################################################
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    python3-venv \
    gosu \
    ssh \
    curl \
    sudo \
    vim \
    git \
    wget \
    unzip \
    moreutils \
    ca-certificates \
    build-essential \
    libyaml-cpp-dev \
    x11-apps \
    iperf3 \
    # ros-$ROS_DISTRO-domain-bridge \ not available in Lyrical yet. include soon!
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-foxglove-msgs \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-ffmpeg-image-transport \
    ros-$ROS_DISTRO-foxglove-compressed-video-transport \
    ros-$ROS_DISTRO-gps-msgs \
    ros-$ROS_DISTRO-ament-copyright \
    ros-$ROS_DISTRO-ament-flake8 \
    ros-$ROS_DISTRO-ament-pep257 \
 && rm -rf /var/lib/apt/lists/*

##############################################################################
# 2b) Custom overlay workspace – packages baked into the image
#     (third-party source-only packages + project-specific bake_packages)
#     Workspace chain: /opt/ros/$ROS_DISTRO → /opt/custom_ws/install
##############################################################################
ENV CUSTOM_WS=/opt/custom_ws

# novatel_oem7_msgs: no binary deb for Kilted yet, build from source
RUN mkdir -p ${CUSTOM_WS}/src \
    && git clone --depth 1 --branch kilted \
       https://github.com/novatel/novatel_oem7_driver.git \
       ${CUSTOM_WS}/src/novatel_oem7_driver

# bake_packages: project-specific ROS packages staged by build.py
# from paths listed in config "bake_ros_packages"
COPY bake_packages/ ${CUSTOM_WS}/src/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${CUSTOM_WS} \
    && colcon build --merge-install --packages-select novatel_oem7_msgs \
    && rm -rf ${CUSTOM_WS}/src/novatel_oem7_driver \
    && colcon build --merge-install \
    && rm -rf ${CUSTOM_WS}/src ${CUSTOM_WS}/build ${CUSTOM_WS}/log

# install zenoh and zenoh-plugin-ros2dds
ARG ZENOH_VERSION=1.9.0
ARG ZENOH_ROS2DDS_VERSION=${ZENOH_VERSION}
ENV ZENOH_PLATFORM=x86_64-unknown-linux-gnu
# Zenoh router
# - see: https://download.eclipse.org/zenoh/zenoh/latest/
RUN mkdir -p /opt/zenoh \
    && wget -qO /tmp/zenoh.zip \
         "https://download.eclipse.org/zenoh/zenoh/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-${ZENOH_PLATFORM}-standalone.zip" \
    && unzip -q /tmp/zenoh.zip -d /opt/zenoh \
    && rm /tmp/zenoh.zip
# ROS2 bridge plugin
# - see: https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/latest/
RUN mkdir -p /opt/zenoh/lib \
    && wget -qO /tmp/zenoh-plugin-ros2dds.zip \
         "https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/${ZENOH_ROS2DDS_VERSION}/zenoh-plugin-ros2dds-${ZENOH_ROS2DDS_VERSION}-${ZENOH_PLATFORM}-standalone.zip" \
    && unzip -q /tmp/zenoh-plugin-ros2dds.zip -d /opt/zenoh \
    && rm /tmp/zenoh-plugin-ros2dds.zip
ENV PATH="/opt/zenoh:${PATH}"

# mcap CLI for fast filtering of rosbags
ARG MCAP_CLI_VERSION=0.0.62
RUN curl -L https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv${MCAP_CLI_VERSION}/mcap-linux-amd64 -o /usr/local/bin/mcap \
    && chmod +x /usr/local/bin/mcap

# ##############################################################################
# # 3) Setup shared directory
# ##############################################################################
# RUN groupadd --system shared && usermod -aG shared root
# RUN mkdir -p /ros2ws && \
#     chown root:shared /ros2ws && \
#     chmod g+rwXs /ros2ws
# RUN echo 'umask 002' >> /etc/profile.d/umask.sh
# RUN setfacl -d -m g::rwx /ros2ws

##############################################################################
# 5) Setup Virtual Python Environment
##############################################################################
ENV PYTHON_VENV_PATH=/opt/ros_venv
RUN python3 -m venv $PYTHON_VENV_PATH --system-site-packages
ENV PATH="$PYTHON_VENV_PATH/bin:$PATH"
# set PYTHONPATH to include the venv site-packages. Hacky but might be required.
# RUN VENV_SITE="$("$PYTHON_VENV_PATH/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')" \
#  && ln -sfn "$VENV_SITE" "$PYTHON_VENV_PATH/site-packages"
# ENV PYTHONPATH="${PYTHON_VENV_PATH}/site-packages"

##############################################################################
# 6) Install Base Pip Dependencies
##############################################################################
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir \
        PyYAML \
        catmux \
        rosbags

##############################################################################
# 7) Hotfix: Replace deprecated rosdep version shipped with ROS 2 Jazzy image
##############################################################################
# The base image includes an outdated rosdep installed via APT,
# which uses the deprecated 'pkg_resources' API and causes a DeprecationWarning.
# To avoid this warning, we remove the APT version and install the latest rosdep via pip.
RUN apt-get remove -y python3-rosdep && \
    pip install --no-cache-dir --upgrade rosdep

##############################################################################
# 8) Install user-specified APT dependencies
##############################################################################
ARG APT_PACKAGES=""
RUN if [ -n "${APT_PACKAGES}" ]; then \
        apt-get update && \
        apt-get install -y ${APT_PACKAGES} && \
        rm -rf /var/lib/apt/lists/*; \
    fi

##############################################################################
# 9) Install user-specified Python packages
##############################################################################
ARG PIP_PACKAGES=""
RUN if [ -n "${PIP_PACKAGES}" ]; then \
        pip install --no-cache-dir ${PIP_PACKAGES}; \
    fi

##############################################################################
# 10) Environment settings
##############################################################################
RUN echo "source ${PYTHON_VENV_PATH}/bin/activate" >> /root/.bashrc
RUN echo "source ${CUSTOM_WS}/install/setup.bash" >> /root/.bashrc
RUN echo '[ -f /ros2ws/install/setup.bash ] && source /ros2ws/install/setup.bash' >> /root/.bashrc
RUN echo 'alias catmux="tmux -L catmux"' >> /root/.bashrc
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

##############################################################################
# 11) EntryPoint
##############################################################################
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

##############################################################################
# 12) Create Non-Root User
##############################################################################
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV CONTAINER_USERNAME=containeruser
RUN userdel -r $(id -un 1000) \
    && groupadd --gid "$USER_GID" "$CONTAINER_USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -m "$CONTAINER_USERNAME" \
    && echo "$CONTAINER_USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$CONTAINER_USERNAME \
    && chmod 0440 /etc/sudoers.d/$CONTAINER_USERNAME \
    && cp /root/.bashrc "/home/$CONTAINER_USERNAME/.bashrc" \
    && chown "$USER_UID:$USER_GID" "/home/$CONTAINER_USERNAME/.bashrc"
    # && usermod -aG shared "$CONTAINER_USERNAME" \
