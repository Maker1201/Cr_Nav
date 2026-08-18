#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

MAP_FILE="$WORKSPACE_ROOT/src/me_nav2_bringup/pcd/fast_lio_map.pcd"
ros2 service call /map_save std_srvs/srv/Trigger

if [ -f "$MAP_FILE" ]; then
  echo "Saved PCD: $MAP_FILE"
else
  echo "Map service returned, but expected PCD was not found: $MAP_FILE" >&2
  echo "Check the running fast_lio map_file_path parameter or restart mapping_sim.sh." >&2
fi
