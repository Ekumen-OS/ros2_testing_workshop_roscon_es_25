#!/usr/bin/env bash
set -e

# Configuration
CONTAINER_NAME="ros2-testing-workshop-roscon-es-25-container"

# Helper function
function print_usage() {
  echo "Usage: $0 [OPTIONS] [COMMAND]"
  echo "Joins a running container for the ROS 2 workshop."
  echo
  echo "Options:"
  echo "  -c, --container <name>    Container name to join (default: ${CONTAINER_NAME})"
  echo "  -h, --help                Show this help message"
  echo
  echo "Any additional arguments are passed to the container's entrypoint."
}

# Parse args
ARGS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    -c|--container)
      CONTAINER_NAME="$2"
      shift; shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      # Forward unknown args to docker exec
      ARGS+=("$1")
      shift
      ;;
  esac
done

# Default command if none provided
if [[ ${#ARGS[@]} -eq 0 ]]; then
  ARGS=("bash")
fi

# Allow GUI applications
xhost +

# Join a running container
docker exec -it "$CONTAINER_NAME" "${ARGS[@]}"

# Disallow GUI applications after container exits
xhost -
