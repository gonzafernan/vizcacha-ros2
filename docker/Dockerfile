ARG ROS_DISTRO=humble

#################################
# Base Image for Vizcacha Robot #
#################################

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base
SHELL ["/bin/bash", "-c"]

# Install system packages
RUN apt-get update
RUN apt-get install -y \
    git \
    clang-tidy \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    qtwayland5
RUN apt-get install -y \
    ros-$ROS_DISTRO-urdf-tutorial \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Intel Realsense setup
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
    tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
    https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update
RUN apt-get install -y \
    librealsense2 \
    librealsense2-dev \
    librealsense2-utils \
    librealsense2-gl
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Create Colcon workspace with external dependencies
RUN mkdir -p /ws/src
# WORKDIR /ws/src
# COPY ros2.repos .
# RUN vcs import < ros2.repos

# Build the base Colcon workspace, installing dependencies first
WORKDIR /ws

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src \
    --skip-keys="librealsense2" --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install"

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM base AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=$UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
# RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid $UID --gid $GID --create-home $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

# Give user access to serial devices
RUN usermod -aG dialout $USERNAME

USER ${USERNAME}

COPY setup.sh /
RUN echo "source /setup.sh" >> /home/${USERNAME}/.bashrc
