# x86_64 workstation container.
# TensorRT 10.7 + CUDA 12.6 to match the Jetson (JetPack 6 / TRT 10.7) target.
FROM nvcr.io/nvidia/tensorrt:24.12-py3

ENV DEBIAN_FRONTEND=noninteractive

# ─── system deps ──────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libyaml-cpp-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/kist-gearsonic-inference

CMD ["/bin/bash"]
