#!/bin/bash

# Start the Docker container for the FYP robot arm project
# Run from anywhere: ./docker/start_container.sh

set -e

IMAGE_NAME="fyp_robot_arm"
IMAGE_TAG="latest"
CONTAINER_NAME="fyp_robot_arm"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "  Starting FYP Robot Arm Container"

# ---- Check if image exists ----
if ! docker image inspect ${IMAGE_NAME}:${IMAGE_TAG} &>/dev/null; then
    echo "ERROR: Docker image '${IMAGE_NAME}:${IMAGE_TAG}' not found."
    echo "Please build it first:  ./docker/build_image.sh"
    exit 1
fi

# ---- Remove existing container with same name (if stopped) ----
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container '${CONTAINER_NAME}'..."
    docker rm -f ${CONTAINER_NAME}
fi

# ---- ROS_DOMAIN_ID ----
if [ -z "${ROS_DOMAIN_ID}" ]; then
    echo "ROS_DOMAIN_ID is not set. Defaulting to 1."
    export ROS_DOMAIN_ID=1
else
    echo "ROS_DOMAIN_ID is set to ${ROS_DOMAIN_ID}"
fi

# ---- Allow X11 display access ----
xhost +local:docker 2>/dev/null || true

# ---- GPU support detection ----
GPU_FLAGS=""
if command -v nvidia-smi &>/dev/null; then
    echo "NVIDIA GPU detected — enabling GPU passthrough."
    GPU_FLAGS="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "No NVIDIA GPU detected — using software rendering."
fi

# ---- Create persistent directories ----
mkdir -p ~/.vscode-server-cache

echo ""
echo "Workspace mounted from: ${WORKSPACE_ROOT}"
echo ""

docker run -it \
    --network=host \
    --privileged \
    ${GPU_FLAGS} \
    --device=/dev/dri:/dev/dri \
    --restart unless-stopped \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${HOME}/.vscode-server-cache:/home/user/.vscode-server" \
    -v "${WORKSPACE_ROOT}:/home/user/ros2_ws" \
    -e "DISPLAY=${DISPLAY}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -e "QT_X11_NO_MITSHM=1" \
    --ipc=host \
    -w /home/user/ros2_ws/ \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    /bin/bash

echo ""
echo "Container stopped."
