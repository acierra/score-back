# Build-environment image: contains the toolchain and dependencies only.
# The source tree is bind-mounted at runtime and compiled inside the container:
#
#   docker build -t cloud-point-rpc-dev .
#   docker run -d --name cprpc-dev --network=host -v "$(pwd)":/app cloud-point-rpc-dev
#   docker exec -it cprpc-dev meson setup build-docker
#   docker exec -it cprpc-dev meson compile -C build-docker
#   docker exec -it cprpc-dev ./build-docker/src/cloud_point_rpc_cli config.yml
#
# Use a dedicated build directory (build-docker) so host and container
# builds never share Meson state.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# - build-essential, meson, ninja-build, cmake, pkg-config: toolchain
# - git: Meson subprojects (asio, json, yaml-cpp, glog, jsonrpccxx, gtest)
# - libopencv-dev: OpenCV 4 incl. contrib modules (ximgproc for the WLS filter)
# - libunwind-dev: glog stack traces
# - python3, python3-openpyxl: scripts/scared_overview.py (--xlsx needs openpyxl)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    meson \
    ninja-build \
    cmake \
    pkg-config \
    git \
    ca-certificates \
    libunwind-dev \
    libopencv-dev \
    python3 \
    python3-openpyxl \
    && rm -rf /var/lib/apt/lists/*

# The mounted checkout is usually owned by a different uid than root inside
# the container; git refuses to touch it otherwise (Meson subprojects need git).
RUN git config --global --add safe.directory '*'

WORKDIR /app

# Keep the container alive; build and run via `docker exec`.
CMD ["tail", "-F", "/dev/null"]
