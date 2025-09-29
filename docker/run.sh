#!/usr/bin/env bash
set -e

# Configuration
USERNAME=$(whoami)
IMAGE_NAME="ros2-testing-workshop-roscon-es-25"
ROS_DISTRO="jazzy"
TAG="${ROS_DISTRO}"
CONTAINER_NAME="ros2-testing-worshop-roscon-es-25-container"
SCRIPT_FOLDER_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
REPOSITORY_FOLDER_PATH="${SCRIPT_FOLDER_PATH}/.."
# TODO Review this (whether to allow dynamic user name or fixed one)
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
  echo "  -n, --name      <name>    Set the image name (default: ${DEFAULT_IMAGE_NAME})"
  echo "  -t, --tag       <tag>     Set the image tag (default: ROS distro name)"
  echo "  -c, --container <name>    Set the container name (default: ${DEFAULT_CONTAINER_NAME})"
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
      ARGS+=("$1") # Forward unknown args to docker run
      shift
      ;;
  esac
done

# Check Docker permissions
if ! docker info > /dev/null 2>&1; then
  echo -e "\n\e[31mError:\e[0m Docker is not running or you don't have permission."
  echo "Please start Docker or run this script with 'sudo'."
  exit 1
fi

# Check for NVIDIA Container Toolkit for GPU support
if ! dpkg -l | grep -q nvidia-container-toolkit; then
  echo -e "\n\e[33mWarning:\e[0m 'nvidia-container-toolkit' not found."
  echo "NVIDIA GPU support will be disabled. To enable it, run:"
  echo "sudo apt-get install nvidia-container-toolkit"
  NVIDIA_FLAGS=""
else
  NVIDIA_FLAGS="--gpus all"
fi

# Build if requested
if [ "$BUILD_FIRST" = true ]; then
  echo "Building image first as requested..."
  "${SCRIPT_FOLDER_PATH}/build.sh" -n "${IMAGE_NAME}" -t "${TAG}"
fi

# Check if name container is already taken
# Check if a container with the same name already exists
if [ "$(docker ps -a -q -f name=${CONTAINER_NAME})" ]; then
   echo -e "\n\e[31mError:\e[0m Container '\e[1m${CONTAINER_NAME}\e[0m' already exists."
   echo "You can either remove it with 'docker rm ${CONTAINER_NAME}'"
   echo "or run this script with a different name: '-c <new_name>'"
   exit 1
fi

# Allow GUI applications
xhost +

# TODO: Check why graphical tools (Rviz) are not working correctly (OpenGL not using the GPU)
# Run the container
docker run -it \
  --net=host \
  ${NVIDIA_FLAGS} \
  --name "${CONTAINER_NAME}" \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-${USER} \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "${REPOSITORY_FOLDER_PATH}/ws:${WORKSPACE_ROOT_CONTAINER}" \
  -v "${HOME}/.ssh:/home/${USERNAME}/.ssh" \
  -w "${WORKSPACE_ROOT_CONTAINER}" \
  "${IMAGE_NAME}:${TAG}" \
  "${ARGS[@]}"

# Disallow GUI applications after container exits
xhost -

# Function to be able to overwrite the image on exit
# TODO (jesus): Check why this is not working sometimes after changes
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
