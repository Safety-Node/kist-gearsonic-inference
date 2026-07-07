#!/bin/bash
# Launch (or re-attach to) a persistent named container.
# Reuse across sessions so builds, caches, and background processes survive
# until you explicitly `docker rm kist-inference`.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=kist-inference

if [ "$(docker ps -q -f name=^${CONTAINER}$)" ]; then
    # Already running → attach a new shell to it
    docker exec -it "${CONTAINER}" /bin/bash
elif [ "$(docker ps -aq -f name=^${CONTAINER}$)" ]; then
    # Exists but stopped → start it back up
    docker start -ai "${CONTAINER}"
else
    # First run → create it
    docker run -it \
        --name "${CONTAINER}" \
        --gpus all \
        --network host \
        --cap-add=SYS_NICE \
        -v "${REPO_DIR}:/workspace/kist-gearsonic-inference" \
        -v /opt/apps/roboticsservice:/opt/apps/roboticsservice:ro \
        -w /workspace/kist-gearsonic-inference \
        kist-inference-dev
fi
