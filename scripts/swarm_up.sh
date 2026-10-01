#!/usr/bin/env bash
# One command to run the backend (on the host):
#   starts the container (creates it if missing), builds the workspace if
#   needed, and launches Crazyswarm2 sim + rosbridge + swarm_commander.
# Extra args go to ros2 launch, e.g.  ./scripts/swarm_up.sh rviz:=True
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${CONTAINER:-xr-swarm}"

docker container inspect "$NAME" >/dev/null 2>&1 || "$REPO/scripts/create_container.sh"
xhost +local:docker >/dev/null 2>&1 || true
docker start "$NAME" >/dev/null

echo "Unity RosConnector: ws://$(hostname -I | awk '{print $1}'):9090"
exec docker exec -it "$NAME" bash -ic '
  cd /root/ros2_ws
  if [ ! -f install/setup.bash ]; then
    echo "[swarm_up] first run: building workspace..."
    colcon build --symlink-install && source install/setup.bash
  fi
  ros2 launch swarm_control swarm_bringup.launch.py '"$*"
