#!/usr/bin/env bash
# Open another terminal inside the running backend container.
set -euo pipefail
NAME="${CONTAINER:-xr-swarm}"
docker start "$NAME" >/dev/null
exec docker exec -it "$NAME" bash
