ARG ROS_DISTRO=humble

#################################
# Base Image for Vizcacha Robot #
#################################

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base
SHELL ["/bin/bash", "-c"]

# Create Colcon workspace with external dependencies
RUN mkdir -p /ws/src
# WORKDIR /ws/src
# COPY ros2.repos .
# RUN vcs import < ros2.repos

# Build the base Colcon workspace, installing dependencies first
WORKDIR /ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update && apt-get upgrade -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

# Install useful tools
RUN apt install -y qtwayland5

RUN apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui \
    && apt install -y ros-$ROS_DISTRO-xacro \
    && apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs \
    && apt install -y ros-$ROS_DISTRO-plotjuggler-ros

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
