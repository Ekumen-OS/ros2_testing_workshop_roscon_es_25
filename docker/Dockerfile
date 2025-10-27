# Base image: ROS 2 Jazzy Core
FROM ros:jazzy-ros-core

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Copy requirement files and install dependencies (ignore comments and empty lines)
COPY docker/apt_packages.txt .
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(grep -vE '^\s*#' apt_packages.txt | grep -vE '^\s*$') && \
    rm -rf /var/lib/apt/lists/*
RUN rm apt_packages.txt

# Some dependencies need to be installed with pip instead of apt
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && \
    python3 -m pip install --no-cache-dir --break-system-packages colcon-lint && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep as root. Then regular users can rosdep update
RUN rosdep init

# Take ownership of the existing non-root user (ID 1000)
ARG USERNAME=ros2
ARG USER_UID=1000
ARG USER_GID=1000

# The Ubuntu 24 base image has a default user 'ubuntu' at UID/GID 1000.
# We modify this user to match the host user's name.
RUN apt-get update && apt-get install -y sudo \
    && groupmod --gid $USER_GID --new-name $USERNAME ubuntu \
    && usermod --uid $USER_UID --gid $USER_GID --login $USERNAME --home /home/$USERNAME --move-home ubuntu \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Worspace and work directory
ENV WS=/home/${USERNAME}/ws
RUN mkdir -p ${WS} && chown -R ${USER_UID}:${USER_GID} ${WS}
WORKDIR ${WS}

# Copy only package manifests first so this layer is cacheable even if source changes
# The .dockerignore file excludes all source code so this layer stays small
# and only rebuilds when manifests change.
COPY --chown=${USERNAME}:${USERNAME} modules/ ${WS}/src/

# Set the new user
USER $USERNAME

# Install dependencies via rosdep
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS 2 and colcon autocompletion on container startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
