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

ARG USER=developer
ARG GROUP=roscon

RUN deluser ubuntu

RUN addgroup --gid 1001 $GROUP \
    && adduser --uid 1001 --ingroup $GROUP --home /home/$USER --shell /bin/bash --disabled-password --gecos "" $USER \
    && adduser $USER sudo \
    && echo "$USER ALL=NOPASSWD: ALL" > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER

COPY docker/fixuid_config.yml /etc/fixuid/config.yml
RUN /bin/bash -c '\
     ARCH=`uname -m` && if [ "$ARCH" == "aarch64" ]; then FIXUID_ARCH="arm64"; else FIXUID_ARCH="amd64"; fi \
  && curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-$FIXUID_ARCH.tar.gz | tar -C /usr/local/bin -xzf - \
  && chmod 4755 /usr/local/bin/fixuid \
  && cd /etc/fixuid \
  && sed -i "s/_USER_/$USER/" config.yml \
  && sed -i "s/_GROUP_/$GROUP/" config.yml'

# Worspace and work directory
ENV WS=/home/${USER}/ws
RUN mkdir -p ${WS} && chown -R ${USER}:${GROUP} ${WS}
WORKDIR ${WS}

# Copy only package manifests first so this layer is cacheable even if source changes
# The .dockerignore file excludes all source code so this layer stays small
# and only rebuilds when manifests change.
COPY --chown=${USER}:${GROUP} modules/ ${WS}/src/

# Set the new user
USER $USER:$GROUP

# Install dependencies via rosdep
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update && \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

# Source ROS 2 and colcon autocompletion on container startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh"]
CMD ["bash"]
