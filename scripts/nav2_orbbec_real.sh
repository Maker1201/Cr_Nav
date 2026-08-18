#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  echo "Missing install/setup.bash. Run: ./scripts/build.sh" >&2
  exit 1
fi

# Orbbec Gemini 336L 地形识别实机导航启动脚本

# point_lio
# gnome-terminal --title="Livox Point-LIO 驱动" -- bash -c "
# set +u; source install/setup.bash; set -u;
# ros2 launch livox_ros_driver2 point_lio_msg_MID360_launch.py"

# gnome-terminal --title="Point-LIO 里程计" -- bash -c "
# set +u; source install/setup.bash; set -u;
# ros2 launch point_lio point_lio.launch.py \
#   point_lio_cfg_dir:=/home/pio/Nav2_3D_ws/src/localization/point_lio/config/mid360_real.yaml"

# gnome-terminal --title="Point-LIO lio_interface" -- bash -c "
# set +u; source install/setup.bash; set -u;
# ros2 launch lio_interface pointlio_lio_interface_launch.py"


# fast_lio
gnome-terminal --title="Livox Fast-LIO 驱动" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py"

gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch fast_lio mapping.launch.py use_sim_time:=false"

gnome-terminal --title="Fast-LIO lio_interface" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=false"

# ---------




gnome-terminal --title="Orbbec Gemini 336L 双相机" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch me_nav2_bringup orbbec_gemini_336l_dual_launch.py"

gnome-terminal --title="Orbbec 相机TF" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch me_nav2_bringup orbbec_static_tf_launch.py"

gnome-terminal --title="机器人描述" -- bash -c "
killall -9 gzserver gzclient;
set +u; source install/setup.bash; set -u;
ros2 launch gld_robot_description gld_robot_description_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=false"

gnome-terminal --title="3d点云转2d" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py use_sim_time:=false"

gnome-terminal --title="Orbbec地形识别" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch terrain_perception terrain_perception_launch.py use_sim_time:=false"

# gnome-terminal --title="small_gicp 重定位" -- bash -c "
# set +u; source install/setup.bash; set -u;
# ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"

gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"

gnome-terminal --title="Nav2 导航" -- bash -c "
set +u; source install/setup.bash; set -u;
ros2 launch me_nav2_bringup my_nav2_launch.py"
