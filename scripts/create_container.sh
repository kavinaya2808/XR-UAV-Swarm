#!/usr/bin/env bash
# Create the long-lived dev container (run on the host, once).
# Mounts this repo's ros2_ws into the container, uses host networking so
# rosbridge (9090) is reachable from Unity, and enables GPU + X11 if available.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-xr-uav-swarm:jazzy}"
NAME="${CONTAINER:-xr-swarm}"

if docker container inspect "$NAME" >/dev/null 2>&1; then
  echo "Container '$NAME' already exists (remove it with: docker rm -f $NAME)"
  exit 0
fi

GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
  GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

X11_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
  xhost +local:docker >/dev/null 2>&1 || true
  X11_ARGS=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi

docker create -it --name "$NAME" \
  --net=host --ipc=host \
  "${GPU_ARGS[@]}" "${X11_ARGS[@]}" \
  -v "$REPO/ros2_ws:/root/ros2_ws" \
  "$IMAGE" bash >/dev/null

echo "Created container '$NAME' from $IMAGE (workspace: $REPO/ros2_ws)"
