# Base image: ROS 2 Jazzy desktop full
FROM osrf/ros:jazzy-desktop-full

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Copy requirement files and install dependencies (ignore comments and empty lines)
COPY docker/requirements.txt .
RUN apt-get update && \
    apt-get install --no-install-recommends -y $(grep -vE '^\s*#' requirements.txt | grep -vE '^\s*$') && \
    rm -rf /var/lib/apt/lists/*
RUN rm requirements.txt

# Some dependencies need to be installed with pip instead of apt
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && \
    python3 -m pip install --no-cache-dir --break-system-packages colcon-lint && \
    rm -rf /var/lib/apt/lists/*

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

# Create workspace and set ownership
RUN mkdir -p /home/${USERNAME}/ws && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}/ws

# Set the new user and work directory
USER $USERNAME
WORKDIR /home/$USERNAME/ws
SHELL ["/bin/bash", "-c"]

# Copy the packages and install the dependencies
COPY --chown=$USERNAME:$USERNAME modules/ /home/${USERNAME}/ws/src/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS 2 and colcon autocompletion on container startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Default entrypoint
ENTRYPOINT ["/bin/bash"]
