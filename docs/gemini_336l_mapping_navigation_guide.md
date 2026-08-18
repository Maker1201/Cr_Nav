# Gemini 336L 建图导航与实机部署操作指南

本文档说明如何在当前 `Lidar_nav2_ws` 工程中使用前后两个 Orbbec Gemini 336L 进行粗糙地形识别、建图和导航验证。内容分为两部分：Gazebo 虚拟环境测试、实际部署测试。

> Docker 内工程路径：`/root/workspace/Lidar_nav2_ws`  
> 宿主机工程路径：`/home/robot/ros2_humble_ws/Lidar_nav2_ws`

## 1. 功能关系说明

当前 Gemini 336L 相关功能分成三层：

1. 相机数据层
   - Gazebo 仿真中：由机器人 URDF 中的前后深度相机插件发布点云。
   - 实机中：由 Orbbec 官方 ROS2 驱动发布前后相机点云。

2. 地形识别层
   - 包：`terrain_perception`
   - 输入：
     - `/orbbec_front/depth/points`
     - `/orbbec_rear/depth/points`
   - 输出：
     - `/terrain_scan`：给 Nav2 costmap 使用的类 LaserScan 障碍话题。
     - `/terrain_obstacles`：识别出的地形障碍点云，主要用于 RViz 检查。

3. 导航避障层
   - `me_nav2_bringup/config/nav2_params.yaml` 中已经把 `/terrain_scan` 加入 local costmap observation source。
   - Nav2 会把地形识别结果作为局部障碍参与避障。

## 2. Gazebo 虚拟环境测试

### 2.1 编译粗糙地形和 Gemini 相关 overlay

进入 Docker：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/build.sh
```

编译成功后应生成：

```bash
install/setup.bash
```

如果提示找不到 `install/setup.bash`，说明还没有完成工作空间编译。

### 2.2 修改粗糙地形参数

粗糙地形配置文件：

```bash
/root/workspace/Lidar_nav2_ws/src/get_urdf/config/rough_terrain.yaml
```

常用修改项：

```yaml
rough_terrain:
  ramp_height_m: 0.22          # 红色坡高度
  ramp_slope_length_m: 1.70    # 上坡/下坡长度，越大坡越缓
  ramp_platform_length_m: 3.00 # 坡顶平台长度

  gravel_area:
    low_count: 22              # 低碎石数量
    mid_count: 10              # 中碎石数量
    high_count: 5              # 高碎石数量
    min_stone_distance_m: 0.32 # 碎石最小间距

  pit_area:
    pit_depth_m: 0.05          # 坑深
    pit_count: 7               # 坑数量
    min_pit_center_distance_m: 0.90 # 坑中心最小距离
```

修改后需要重启 Gazebo 才会重新生成 `/tmp/lidar_nav2_rough_terrain.world`。

### 2.3 启动 Gazebo 粗糙地形建图

用于新建地图：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/mapping_rough_sim.sh
```

这个脚本会启动：

- Gazebo 粗糙地形场景
- FAST-LIO 仿真里程计
- lio_interface
- sensor_scan_generation
- 3D 点云转 2D scan
- terrain_perception 地形识别
- slam_toolbox 建图
- Nav2
- GUI 控制小车

建图时建议在 RViz 中检查：

```bash
ros2 topic list | grep -E "orbbec|terrain|map|scan"
ros2 topic hz /orbbec_front/depth/points
ros2 topic hz /orbbec_rear/depth/points
ros2 topic hz /terrain_scan
ros2 topic hz /terrain_obstacles
```

正常情况下应能看到：

```text
/orbbec_front/depth/points
/orbbec_rear/depth/points
/terrain_scan
/terrain_obstacles
/map
```

### 2.4 保存地图

建图完成后，在另一个终端执行：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/save_map.sh
```

如果你的 `save_map.sh` 支持传参，可以按项目原来的方式指定地图名；如果不支持，就使用脚本默认保存路径。

保存后确认地图文件存在，例如：

```bash
find /root/workspace/Lidar_nav2_ws -name "*.yaml" -o -name "*.pgm"
```

### 2.5 启动 Gazebo 粗糙地形导航

用于加载已有地图进行导航验证：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/nav2_rough_sim.sh
```

如果需要 GUI 手动控制小车，可加：

```bash
./scripts/nav2_rough_sim.sh --teleop
```

停止仿真：

```bash
./scripts/nav2_rough_sim.sh --stop
pkill -f gzserver
pkill -f gzclient
```

### 2.6 RViz 中如何确认 Gemini 地形避障有效

建议在 RViz 中观察以下内容：

- RobotModel：确认前后相机 TF 是否存在。
- PointCloud2：
  - `/orbbec_front/depth/points`
  - `/orbbec_rear/depth/points`
- PointCloud2：`/terrain_obstacles`
- LaserScan：`/terrain_scan`
- Map / Local Costmap：确认地形障碍是否进入局部代价地图。

如果 `/orbbec_front/depth/points` 有数据，但 `/terrain_scan` 没数据，优先检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_footprint orbbec_front_depth_optical_frame
ros2 run tf2_ros tf2_echo base_footprint orbbec_rear_depth_optical_frame
```

如果 TF 报错，说明相机 frame 没有连到 `base_footprint`。

### 2.7 调整地形识别阈值

配置文件：

```bash
/root/workspace/Lidar_nav2_ws/src/terrain_perception/config/terrain_perception.yaml
```

常用参数：

```yaml
max_drop_depth_m: 0.05   # 超过这个深度的坑认为危险
max_step_height_m: 0.04  # 超过这个高度的凸起认为危险
max_slope_deg: 10.0      # 超过这个坡度认为危险
min_obstacle_points: 4   # 障碍点数量阈值，增大可减少误检

front_roi_x_min_m: 0.05
front_roi_x_max_m: 1.40
rear_roi_x_min_m: -1.40
rear_roi_x_max_m: -0.05
roi_y_abs_m: 0.55
roi_z_min_m: -0.35
roi_z_max_m: 0.85
```

调参建议：

- 误检太多：增大 `min_obstacle_points`，或增大 `max_step_height_m/max_drop_depth_m`。
- 漏检坑洼：减小 `max_drop_depth_m`。
- 漏检凸起碎石：减小 `max_step_height_m`。
- 检测范围太窄：增大 `roi_y_abs_m` 或前后 ROI 范围。
- 检测太远导致噪声多：减小 `front_roi_x_max_m`、增大 `rear_roi_x_min_m` 的绝对范围控制。

修改后重启 `terrain_perception` 所在脚本。

## 3. 实际部署测试

### 3.1 实机部署前检查

确认硬件：

- 前 Gemini 336L 已安装并固定。
- 后 Gemini 336L 已安装并固定。
- 两个相机通过 USB 直连或可靠 USB hub 接入工控机。
- Livox/MID360、底盘、电源、急停、网络都正常。

建议初始安装姿态：

- 前相机：安装在车体前部，朝前，向下俯视约 30 度。
- 后相机：安装在车体后部，朝后，向下俯视约 30 度。
- 高度建议先用 `0.35m` 左右，后续按实机结构调整。

当前默认配置在：

```bash
/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/orbbec_cameras.yaml
```

默认前相机：

```yaml
front:
  xyz: [0.32, 0.0, 0.35]
  rpy: [0.0, 0.52, 0.0]
```

默认后相机：

```yaml
rear:
  xyz: [-0.32, 0.0, 0.35]
  rpy: [0.0, 0.52, 3.14159]
```

含义：

- `xyz`：相机相对 `base_footprint` 的安装位置。
  - x 正方向是车头。
  - y 正方向是车左。
  - z 正方向是上方。
- `rpy`：相机相对 `base_footprint` 的姿态。
  - roll：绕 x 轴旋转。
  - pitch：俯仰角，当前 `0.52rad` 约等于 30 度。
  - yaw：航向角，后相机 `3.14159rad` 表示朝后。

### 3.2 安装/编译 Orbbec 驱动

当前工程中驱动包位置：

```bash
/root/workspace/Lidar_nav2_ws/src/OrbbecSDK_ROS2
```

先安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-camera-info-manager
```

然后编译：

```bash
cd /root/workspace/Lidar_nav2_ws
colcon build --packages-select orbbec_camera_msgs orbbec_description orbbec_camera
```

如果你继续使用 overlay 编译方式，也可以：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/build.sh
```

编译完成后 source：

```bash
source install/setup.bash
```

### 3.3 识别两个 Gemini 336L

先只插一个相机，查看设备：

```bash
cd /root/workspace/Lidar_nav2_ws
source install/setup.bash
ros2 run orbbec_camera list_devices_node
```

记录它的 `serial_number`。然后换另一个相机重复一次。

也可以查看 USB 拓扑：

```bash
lsusb
lsusb -t
```

`usb_port` 是 Linux USB 拓扑路径，例如：

```text
2-1.3
2-1.4
```

注意：`usb_port` 不是 `/dev/ttyUSB0`，也不是普通文件路径。换 USB 口、换 hub、换线后可能变化。实机稳定部署时更推荐用 `serial_number`。

### 3.4 修改前后相机配置

编辑：

```bash
/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/orbbec_cameras.yaml
```

把前后相机的 `serial_number` 填进去：

```yaml
orbbec_cameras:
  front:
    camera_name: orbbec_front
    serial_number: "前相机序列号"
    usb_port: "2-1.3"

  rear:
    camera_name: orbbec_rear
    serial_number: "后相机序列号"
    usb_port: "2-1.4"
```

如果你暂时不知道序列号，可以先用 `usb_port` 测试；如果前后数据反了，交换 `front/rear` 的 `serial_number` 或 `usb_port`。

验证前后是否正确：

```bash
ros2 topic hz /orbbec_front/depth/points
ros2 topic hz /orbbec_rear/depth/points
```

然后用手遮住前相机，看 `/orbbec_front/depth/points` 或 RViz 中前相机点云是否变化；遮住后相机，看 `/orbbec_rear/depth/points` 是否变化。

### 3.5 修改相机安装位姿 TF

仍然修改：

```bash
/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/orbbec_cameras.yaml
```

重点是：

```yaml
front:
  xyz: [0.32, 0.0, 0.35]
  rpy: [0.0, 0.52, 0.0]

rear:
  xyz: [-0.32, 0.0, 0.35]
  rpy: [0.0, 0.52, 3.14159]
```

调整方法：

- 前相机实际更靠前：增大 front 的 x。
- 前相机实际更靠后：减小 front 的 x。
- 后相机实际更靠后：减小 rear 的 x。
- 相机更高：增大 z。
- 相机更低：减小 z。
- 相机俯视角不够：调整 pitch。
- 后相机不是正后方：调整 rear 的 yaw。

验证 TF：

```bash
ros2 launch me_nav2_bringup orbbec_static_tf_launch.py
ros2 run tf2_ros tf2_echo base_footprint orbbec_front_depth_optical_frame
ros2 run tf2_ros tf2_echo base_footprint orbbec_rear_depth_optical_frame
```

如果 TF 不连通，`terrain_perception` 就无法把相机点云转换到 `base_footprint`，也就不会正常发布 `/terrain_scan`。

### 3.6 实机建图流程

启动实机建图：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/mapping_orbbec_real.sh
```

该脚本会启动：

- Livox Fast-LIO 驱动
- FAST-LIO 里程计
- lio_interface
- Orbbec Gemini 336L 双相机驱动
- Orbbec 静态 TF
- 机器人描述
- sensor_scan_generation
- 3D 点云转 2D
- terrain_perception
- slam_toolbox 建图

注意：当前脚本中 `slam_params_file` 如果还是旧路径，例如：

```bash
/home/pio/Nav2_3D_ws/src/me_nav2_bringup/config/slam_toolbox_params.yaml
```

实机部署时应改成当前工程路径，注意路径中不能有空格：

```bash
/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/slam_toolbox_params.yaml
```

建图时检查：

```bash
ros2 topic list | grep -E "livox|cloud|orbbec|terrain|scan|map"
ros2 topic hz /orbbec_front/depth/points
ros2 topic hz /orbbec_rear/depth/points
ros2 topic hz /terrain_scan
ros2 topic hz /map
```

保存地图：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/save_map.sh
```

### 3.7 实机导航流程

启动实机导航：

```bash
cd /root/workspace/Lidar_nav2_ws
./scripts/nav2_orbbec_real.sh
```

该脚本会启动：

- Livox Fast-LIO 驱动
- FAST-LIO 里程计
- lio_interface
- Orbbec Gemini 336L 双相机驱动
- Orbbec 静态 TF
- 机器人描述
- sensor_scan_generation
- 3D 点云转 2D
- terrain_perception
- KISS + GICP 重定位
- Nav2 导航

导航前确认：

```bash
ros2 topic hz /terrain_scan
ros2 topic echo /local_costmap/costmap --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint orbbec_front_depth_optical_frame
ros2 run tf2_ros tf2_echo base_footprint orbbec_rear_depth_optical_frame
```

如果 `/terrain_scan` 没有数据，按顺序检查：

1. `/orbbec_front/depth/points`、`/orbbec_rear/depth/points` 是否有数据。
2. 相机 point cloud 的 `header.frame_id` 是否能 TF 到 `base_footprint`。
3. `terrain_perception.yaml` 中 ROI 是否覆盖实际地面区域。
4. `max_drop_depth_m`、`max_step_height_m` 是否设置太宽松导致不判定为障碍。

### 3.8 实机初次测试建议

第一次实机测试不要直接高速导航，建议按以下顺序：

1. 架空或低速检查底盘控制和急停。
2. 单独启动 Orbbec 双相机，确认前后话题不反。
3. 单独启动 Orbbec TF，确认 RViz 中相机位置方向正确。
4. 单独启动 `terrain_perception`，用纸箱、木板、浅坑模拟障碍，看 `/terrain_obstacles` 是否出现。
5. 低速手动控制小车靠近碎石/坡/坑，观察 `/terrain_scan` 和 local costmap。
6. 只在确认局部避障有效后，再启动 Nav2 自动导航。

### 3.9 常见问题

#### 话题没有 `/orbbec_front/depth/points`

检查 Orbbec 驱动是否启动，检查 `serial_number` 或 `usb_port` 是否写错：

```bash
ros2 topic list | grep orbbec
ros2 run orbbec_camera list_devices_node
lsusb -t
```

#### 前后相机反了

交换 `orbbec_cameras.yaml` 中 front/rear 的 `serial_number`，或交换 `usb_port`。

#### RViz 中点云方向不对

修改 `orbbec_cameras.yaml` 里的 `rpy`。后相机通常需要 yaw 接近 `3.14159`。

#### `/terrain_scan` 有数据但 Nav2 不避障

检查 `nav2_params.yaml` 的 local costmap 是否包含 `terrain_scan` observation source，并确认 Nav2 已重新启动。

#### `/terrain_scan` 没数据但相机点云正常

大概率是 TF 或 ROI 问题：

```bash
ros2 run tf2_ros tf2_echo base_footprint orbbec_front_depth_optical_frame
ros2 run tf2_ros tf2_echo base_footprint orbbec_rear_depth_optical_frame
```

然后调大 ROI：

```yaml
roi_y_abs_m: 0.70
front_roi_x_max_m: 1.80
rear_roi_x_min_m: -1.80
```

#### 误检太多

调大：

```yaml
min_obstacle_points: 8
max_step_height_m: 0.06
max_drop_depth_m: 0.07
```

#### 漏检太多

调小：

```yaml
max_step_height_m: 0.03
max_drop_depth_m: 0.04
```

## 4. 实机部署必改清单

实机真正部署前，至少确认以下文件：

1. `/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/orbbec_cameras.yaml`
   - 填写前后相机 `serial_number`。
   - 或确认 `usb_port` 与实际 USB 拓扑一致。
   - 修改 `xyz/rpy` 为实际安装位姿。

2. `/root/workspace/Lidar_nav2_ws/src/terrain_perception/config/terrain_perception.yaml`
   - 按实车越障能力修改 `max_drop_depth_m`、`max_step_height_m`、`max_slope_deg`。
   - 按相机视野修改 ROI。

3. `/root/workspace/Lidar_nav2_ws/scripts/mapping_orbbec_real.sh`
   - 确认 `slam_params_file` 路径是当前工程路径。

4. `/root/workspace/Lidar_nav2_ws/scripts/nav2_orbbec_real.sh`
   - 确认重定位、地图、Nav2 参数使用的是当前工程路径和实际地图。

5. `/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/config/nav2_params.yaml`
   - 确认 local costmap 中包含 `/terrain_scan`。
   - 修改机器人尺寸、膨胀半径、障碍层参数以匹配实车。

## 5. 推荐验证顺序

Gazebo：

```bash
./scripts/build.sh
./scripts/mapping_rough_sim.sh
# 建图完成后保存地图
./scripts/save_map.sh
./scripts/nav2_rough_sim.sh
```

实机：

```bash
# 1. 编译
./scripts/build.sh

# 2. 检查相机
source install/setup.bash
ros2 run orbbec_camera list_devices_node
lsusb -t

# 3. 修改 orbbec_cameras.yaml

# 4. 实机建图
./scripts/mapping_orbbec_real.sh

# 5. 保存地图
./scripts/save_map.sh

# 6. 实机导航
./scripts/nav2_orbbec_real.sh
```
