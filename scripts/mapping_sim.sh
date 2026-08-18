#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 仿真建图启动脚本
# 在桌面环境中使用 gnome-terminal 多窗口启动；在容器中自动退化为后台进程 + 日志文件。
# 容器模式下每个任务使用独立进程组，Ctrl-C 会清理其 ROS/Tk 子进程。

LOG_DIR="$WORKSPACE_ROOT/log/mapping_sim_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$WORKSPACE_ROOT/log/.mapping_sim.pgids"
PGIDS=()
USE_TERMINAL=0
STOP_PATTERNS=(
  "/gui_teleop/lib/gui_teleop/gui_teleop_node"
)

stop_recorded_pgids() {
  if [ ! -f "$PID_FILE" ]; then
    return
  fi
  while IFS= read -r pgid; do
    [ -n "$pgid" ] || continue
    kill -TERM "-$pgid" 2>/dev/null || true
  done < "$PID_FILE"
  sleep 1
  while IFS= read -r pgid; do
    [ -n "$pgid" ] || continue
    kill -KILL "-$pgid" 2>/dev/null || true
  done < "$PID_FILE"
  rm -f "$PID_FILE"
}

stop_workspace_gazebo() {
  local world_path="$WORKSPACE_ROOT/install/get_urdf/share/get_urdf/worlds/3d.world"
  pkill -TERM -f "$world_path" 2>/dev/null || true
  sleep 0.5
  pkill -KILL -f "$world_path" 2>/dev/null || true
}

stop_orphans() {
  for pattern in "${STOP_PATTERNS[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 0.5
  for pattern in "${STOP_PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
}

if [ "${1:-}" = "--stop" ]; then
  stop_recorded_pgids
  stop_workspace_gazebo
  stop_orphans
  exit 0
fi
if command -v gnome-terminal >/dev/null 2>&1; then
  USE_TERMINAL=1
else
  mkdir -p "$LOG_DIR"
  : > "$PID_FILE"
  echo "gnome-terminal not found; running processes in background."
  echo "Logs: $LOG_DIR"
fi

cleanup() {
  trap - INT TERM EXIT
  if [ "${#PGIDS[@]}" -gt 0 ]; then
    echo "Stopping mapping_sim processes..."
    for pgid in "${PGIDS[@]}"; do
      kill -TERM "-$pgid" 2>/dev/null || true
    done
    sleep 1
    for pgid in "${PGIDS[@]}"; do
      kill -KILL "-$pgid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    rm -f "$PID_FILE"
  fi
  stop_workspace_gazebo
  stop_orphans
}
trap cleanup INT TERM HUP EXIT

run_task() {
  local title="$1"
  local cmd="$2"

  if [ "$USE_TERMINAL" -eq 1 ]; then
    gnome-terminal --title="$title" -- bash -lc "cd '$WORKSPACE_ROOT'; source install/setup.bash; $cmd; exec bash"
  else
    local logfile
    logfile="$LOG_DIR/$(echo "$title" | tr ' /' '__').log"
    setsid bash -lc "cd '$WORKSPACE_ROOT'; source install/setup.bash; $cmd" >"$logfile" 2>&1 &
    local pgid=$!
    PGIDS+=("$pgid")
    echo "$pgid" >> "$PID_FILE"
    echo "[$pgid] $title -> $logfile"
  fi
}

# 可能会导致 /cmd_vel 话题被占用：GUI 控制小车
run_task "GUI控制" "ros2 run gui_teleop gui_teleop_node"
run_task "FAST-LIO 里程计" "ros2 launch fast_lio mapping.launch.py use_sim_time:=true rviz:=false"
run_task "lio_interface" "ros2 launch lio_interface lio_interface_launch.py"
run_task "Gazebo 仿真" "killall -9 gzserver gzclient 2>/dev/null || true; ros2 launch get_urdf get_urdf_launch.py rviz:=false"
run_task "sensor_scan_generation" "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"
run_task "3d点云转2d" "ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"
run_task "slam_toolbox 建图" "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WORKSPACE_ROOT/src/me_nav2_bringup/config/slam_toolbox_params.yaml"
run_task "Nav2 导航" "ros2 launch me_nav2_bringup my_nav2_launch.py"

if [ "$USE_TERMINAL" -eq 0 ]; then
  echo "All processes started. Press Ctrl-C to stop them."
  wait || true
fi
