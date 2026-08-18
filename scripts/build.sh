#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

source /opt/ros/humble/setup.bash
export PATH="/opt/cmake/bin:/usr/lib/nvidia-cuda-toolkit/bin:$PATH"

colcon build --symlink-install --parallel-workers 1 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -Dkiss_matcher_DIR="$WORKSPACE_ROOT/install/kiss_matcher_ros/lib/cmake/kiss_matcher"
