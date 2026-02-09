#!/bin/bash

# Build the Docker image for the FYP robot arm project
# Run from the workspace root: ./docker/build_image.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "  Building FYP Robot Arm Docker Image"
echo "Workspace root: $WORKSPACE_ROOT"
echo ""

DOCKER_BUILDKIT=1 docker build \
  --network host \
  --build-arg ros_distro=humble \
  -t fyp_robot_arm:latest \
  -f docker/dockerfile "$@" "$WORKSPACE_ROOT"


echo "  Image built successfully"
echo "  Image: fyp_robot_arm:latest"