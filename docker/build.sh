#!/usr/bin/env bash
set -e

# Default values for configuration
IMAGE_NAME="ros2-testing-workshop-roscon-es-25"
ROS_DISTRO="jazzy"
PLATFORM="linux/amd64"
TAG="${ROS_DISTRO}"
SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
CONTEXT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile

# Helper Function
function print_usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "Builds the Docker image for the ROS 2 workshop."
  echo
  echo "Options:"
  echo "  -n, --name      <name>    Set the image name (default: ${IMAGE_NAME})"
  echo "  -t, --tag       <tag>     Set the image tag (default: ${TAG})"
  echo "  -p, --platform  <platform> Set the target platform (default: ${PLATFORM})"
  echo "  -h, --help                Show this help message"
}

# Argument Parsing
while [[ $# -gt 0 ]]; do
  case $1 in
    -n|--name)
      IMAGE_NAME="$2"
      shift; shift
      ;;
    -t|--tag)
      TAG="$2"
      shift; shift
      ;;
    -p|--platform)
      PLATFORM="$2"
      shift; shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
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

echo "Building Docker image: ${IMAGE_NAME}:${TAG}"
docker build \
  --platform "${PLATFORM}" \
  --build-arg USERNAME="$(whoami)" \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  -f "${DOCKERFILE_PATH}" \
  -t "${IMAGE_NAME}:${TAG}" \
  "${CONTEXT_FOLDER_PATH}"
