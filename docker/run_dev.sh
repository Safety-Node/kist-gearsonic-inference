#!/bin/bash
# Launch the dev container with GPU access, source mounted, and host networking
# (needed for DDS UDP multicast to reach the robot on the LAN).

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run --rm -it \
    --gpus all \
    --network host \
    -v "${REPO_DIR}:/workspace/kist-gearsonic-inference" \
    -v /opt/apps/roboticsservice:/opt/apps/roboticsservice:ro \
    -w /workspace/kist-gearsonic-inference \
    kist-inference-dev
