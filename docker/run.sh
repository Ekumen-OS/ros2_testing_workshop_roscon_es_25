#!/usr/bin/env bash
set -e

# Configuration
USERNAME=developer
IMAGE_NAME="ekumenlabs/ros2-testing-workshop-roscon-es-25"
ROS_DISTRO="jazzy"
TAG="${ROS_DISTRO}"
CONTAINER_NAME="ros2-testing-worshop-roscon-es-25-container"
SCRIPT_FOLDER_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
REPOSITORY_FOLDER_PATH="${SCRIPT_FOLDER_PATH}/.."
WORKSPACE_ROOT_CONTAINER="/home/${USERNAME}/ws"

# Flags
BUILD_FIRST=false

# Helper function
function print_usage() {
  echo "Usage: $0 [OPTIONS] [COMMAND]"
  echo "Runs the Docker container for the ROS 2 workshop."
  echo
  echo "Options:"
  echo "  --build                   Build the Docker image before running."
  echo "  -n, --name      <name>    Set the image name (default: ${IMAGE_NAME})"
  echo "  -t, --tag       <tag>     Set the image tag (default: ${TAG})"
  echo "  -c, --container <name>    Set the container name (default: ${CONTAINER_NAME})"
  echo "  -h, --help                Show this help message"
  echo
  echo "Any additional arguments are passed to the container's entrypoint."
}

# Parse args
ARGS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --build)
      BUILD_FIRST=true
      shift
      ;;
    -n|--name)
      IMAGE_NAME="$2"
      shift; shift
      ;;
    -t|--tag)
      TAG="$2"
      shift; shift
      ;;
    -c|--container)
      CONTAINER_NAME="$2"
      shift; shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      # Forward unknown args to docker run
      ARGS+=("$1")
      shift
      ;;
  esac
done

# Check for Docker permissions instead of hardcoding 'sudo'
if ! docker info > /dev/null 2>&1; then
  echo -e "\n\e[31mError:\e[0m Docker is not running or you don't have permission to use it."
  echo "Please ensure the Docker daemon is running."
  echo "To fix permission issues, you can either:"
  echo "  1. Add your user to the 'docker' group: \e[33msudo usermod -aG docker ${USER}\e[0m (requires logout/login)"
  echo "  2. Run this script with sudo: \e[33msudo $0\e[0m"
  exit 1
fi

# Check for NVIDIA Container Toolkit for GPU support
NVIDIA_FLAGS=""
if command -v dpkg >/dev/null 2>&1 && dpkg -l | grep -q nvidia-container-toolkit; then
  NVIDIA_FLAGS="--gpus all"
else
  # if dpkg isn't present (non-debian host) or toolkit not installed, we leave NVIDIA_FLAGS empty
  echo -e "\n\e[33mWarning:\e[0m 'nvidia-container-toolkit' not detected (or dpkg unavailable). GPU support may be disabled."
fi

# Build if requested
if [ "$BUILD_FIRST" = true ]; then
  echo "Building image first as requested..."
  "${SCRIPT_FOLDER_PATH}/build.sh" -n "${IMAGE_NAME}" -t "${TAG}"
fi

# Check if a container with the same name already exists
if [ "$(docker ps -a -q -f name=${CONTAINER_NAME})" ]; then
   echo -e "\n\e[31mError:\e[0m Container '\e[1m${CONTAINER_NAME}\e[0m' already exists."
   echo "You can either remove it with 'docker rm ${CONTAINER_NAME}'"
   echo "or run this script with a different name: '-c <new_name>'"
   exit 1
fi

# Ensure local build artifact folders (build/, install/, log/) exist
# and are writable by the current user.
mkdir -p "${REPOSITORY_FOLDER_PATH}"/{build,install,log}
# If they were previously created by Docker as root, adjust ownership.
for dir in build install log; do
  folder="${REPOSITORY_FOLDER_PATH}/${dir}"
  # Get current owner UID
  owner_uid=$(stat -c "%u" "$folder" 2>/dev/null || echo "")
  if [ "$owner_uid" = "0" ]; then
    echo "Fixing ownership of $dir (was root-owned)..."
    sudo chown -R "$(id -u):$(id -g)" "$folder"
  fi
done

# Allow GUI applications
xhost +

# Run the container
docker run -it \
  --user=$(id -u):$(id -g) \
  --net=host \
  ${NVIDIA_FLAGS} \
  --name "${CONTAINER_NAME}" \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-${USER} \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --device /dev/dri:/dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${REPOSITORY_FOLDER_PATH}/modules:${WORKSPACE_ROOT_CONTAINER}/src/ \
  -v ${REPOSITORY_FOLDER_PATH}/build:${WORKSPACE_ROOT_CONTAINER}/build/ \
  -v ${REPOSITORY_FOLDER_PATH}/install:${WORKSPACE_ROOT_CONTAINER}/install/ \
  -v ${REPOSITORY_FOLDER_PATH}/log:${WORKSPACE_ROOT_CONTAINER}/log/ \
  -v ${HOME}/.ssh:/home/${USERNAME}/.ssh \
  -w ${WORKSPACE_ROOT_CONTAINER} \
  "${IMAGE_NAME}:${TAG}" \
  "${ARGS[@]}"

# Disallow GUI applications after container exits
xhost -

# Function to be able to overwrite the image on exit
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image called '$IMAGE_NAME' with the current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting docker image..."
      sudo docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT
