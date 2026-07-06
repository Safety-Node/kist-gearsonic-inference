#!/bin/bash
# Build the dev container image.

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    -t kist-inference-dev \
    -f "${REPO_DIR}/docker/Dockerfile.dev" \
    "${REPO_DIR}"
