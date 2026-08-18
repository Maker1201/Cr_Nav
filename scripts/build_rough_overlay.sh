#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# Build the rough-terrain / dual-Orbbec overlay without touching the root-owned
# default build/, install/, and log/ directories.
set +u
source install/setup.bash
set -u
colcon --log-base "$WORKSPACE_ROOT/log_rough" build   --symlink-install   --build-base "$WORKSPACE_ROOT/build_rough"   --install-base "$WORKSPACE_ROOT/install_rough"   --packages-select get_urdf me_nav2_bringup terrain_perception

echo "Overlay ready: $WORKSPACE_ROOT/install_rough/setup.bash"
echo "Use ./scripts/nav2_rough_sim.sh or ./scripts/mapping_rough_sim.sh"
