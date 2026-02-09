#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="helios2-ros2"
IMAGE_TAG="${IMAGE_TAG:-latest}"
CONTAINER_NAME="${CONTAINER_NAME:-helios2-ros2-dev}"

CONTAINER_WS="${CONTAINER_WS:-/home/user/helios2-ros2}"

# Allow X11 for GUI apps
xhost +si:localuser:root >/dev/null 2>&1 || true

# Ensure runtime dir exists (some Qt/GUI stacks care)
mkdir -p /tmp/runtime-root

# Stop/remove old container if it exists (works even with --rm)
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  --privileged \
  --gpus all \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${PROJECT_ROOT}:${CONTAINER_WS}" \
  -v /lib/modules:/lib/modules:ro \
  -v /usr/src:/usr/src:ro \
  -v /var/lib/dkms:/var/lib/dkms \
  -w "${CONTAINER_WS}" \
  "${IMAGE_NAME}:${IMAGE_TAG}" \
  bash -lc '
    set -e
    install_broadcom_driver
    exec bash
  '

