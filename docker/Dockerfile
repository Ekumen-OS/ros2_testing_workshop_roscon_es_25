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
# RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && rm -rf /var/lib/apt/lists/*
# RUN python3 -m pip install --no-cache-dir --break-system-packages ament-lint

# TODO Deal with the annoying user that Ubuntu 24 already has on uid 1000
# Create a non-root user, removing the default 'ubuntu' user
# RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu
# ARG USERNAME=ros2
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# RUN apt-get update && apt-get install -y sudo \
#     && groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
#     && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME

# --- Take ownership of the existing non-root user (ID 1000) ---
ARG USERNAME=ros2
ARG USER_UID=1000
ARG USER_GID=1000

# The ROS base image has a user 'ros' at UID/GID 1000.
# We modify this user to match the host user's name.
RUN apt-get update && apt-get install -y sudo \
    && groupmod --gid $USER_GID --new-name $USERNAME ubuntu \
    && usermod --uid $USER_UID --gid $USER_GID --login $USERNAME --home /home/$USERNAME --move-home ubuntu \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the new user and work directory
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS 2 on container startup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Default entrypoint
ENTRYPOINT ["/bin/bash"]
