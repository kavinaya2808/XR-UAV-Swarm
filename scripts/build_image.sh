#!/usr/bin/env bash
# Build the ROS 2 backend Docker image (run on the host, from anywhere).
# Only needed once, or after changing docker/Dockerfile.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-xr-uav-swarm:jazzy}"
docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO/docker" "$@"
echo "Built $IMAGE"
