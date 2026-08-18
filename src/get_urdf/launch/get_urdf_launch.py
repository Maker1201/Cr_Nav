import math
import os
import random
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_rough_terrain_config(config_path):
    defaults = {
        'enabled': True,
        'terrain_seed': 336,
        'ramp_height_m': 0.22,
        'ramp_slope_length_m': 1.60,
        'ramp_platform_length_m': 1.45,
        'ramp_width_m': 2.15,
        'ramp_areas': {
            'top_outer': {'center': [-1.2, 8.85], 'yaw': 0.0, 'width': 2.25},
            'top_inner': {'center': [-0.55, 6.35], 'yaw': 0.0, 'width': 2.20},
            'right_side': {'center': [11.15, 1.2], 'yaw': 1.5708, 'width': 2.25},
            'left_bottom': {'center': [-12.0, -7.4], 'yaw': 1.5708, 'width': 2.05},
        },
        'gravel_area': {
            'x_min_m': -4.6,
            'x_max_m': 4.8,
            'y_min_m': -3.6,
            'y_max_m': 4.2,
            'avoid_spawn_radius_m': 0.75,
            'low_count': 22,
            'mid_count': 10,
            'high_count': 5,
            'low_height_range_m': [0.015, 0.070],
            'mid_height_range_m': [0.090, 0.160],
            'high_height_range_m': [0.220, 0.360],
            'radius_range_m': [0.035, 0.100],
            'min_stone_distance_m': 0.32,
        },
        'pit_area': {
            'center': [4.2, -7.75],
            'yaw': 0.0,
            'length_m': 8.2,
            'width_m': 2.25,
            'platform_height_m': 0.12,
            'pit_depth_m': 0.05,
            'pit_count': 7,
            'pit_length_range_m': [0.45, 1.20],
            'pit_width_range_m': [0.30, 0.80],
        },
    }

    if not config_path or not os.path.exists(config_path):
        return defaults

    try:
        import yaml
        with open(config_path, 'r') as stream:
            loaded = yaml.safe_load(stream) or {}
        loaded = loaded.get('rough_terrain', loaded)
        return _deep_update(defaults, loaded)
    except Exception:
        return defaults


def _deep_update(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def _box_link(name, x, y, z, roll, pitch, yaw, sx, sy, sz, material):
    return f"""
      <link name='{name}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
        <collision name='{name}_collision'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name='{name}_visual'>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>"""


def _cylinder_link(name, x, y, z, radius, length, material):
    return f"""
      <link name='{name}'>
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <collision name='{name}_collision'>
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
        </collision>
        <visual name='{name}_visual'>
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>"""


def _rough_terrain_model(values):
    rng = random.Random(int(values.get('terrain_seed', 336)))
    parts = []

    def as_range(items, default_min, default_max):
        if isinstance(items, (list, tuple)) and len(items) >= 2:
            return float(items[0]), float(items[1])
        return default_min, default_max

    def rotate_offset(cx, cy, local_x, local_y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return cx + c * local_x - s * local_y, cy + s * local_x + c * local_y

    def add_side_walls(prefix, cx, cy, yaw, length, width, center_z, wall_width, wall_height, material):
        for side_name, local_y in [('left', width / 2.0 + wall_width / 2.0), ('right', -width / 2.0 - wall_width / 2.0)]:
            wx, wy = rotate_offset(cx, cy, 0.0, local_y, yaw)
            parts.append(_box_link(f'{prefix}_{side_name}_side_wall', wx, wy, center_z, 0, 0, yaw, length, wall_width, wall_height, material))

    # Red areas: complete ramp = up slope + flat platform + down slope, with solid side curbs.
    ramp_height = max(float(values.get('ramp_height_m', 0.22)), 0.04)
    slope_len = max(float(values.get('ramp_slope_length_m', 1.70)), 0.40)
    platform_len = max(float(values.get('ramp_platform_length_m', 1.45)), 0.30)
    default_width = max(float(values.get('ramp_width_m', 2.15)), 0.50)
    ramp_wall_w = max(float(values.get('ramp_side_wall_width_m', 0.10)), 0.03)
    ramp_wall_h = max(float(values.get('ramp_side_wall_height_m', 0.18)), 0.04)
    ramp_angle = math.atan2(ramp_height, slope_len)
    deck_t = 0.04

    for name, area in values.get('ramp_areas', {}).items():
        cx, cy = [float(v) for v in area.get('center', [0.0, 0.0])]
        yaw = float(area.get('yaw', 0.0))
        width = max(float(area.get('width', default_width)), 0.50)
        up_x, up_y = rotate_offset(cx, cy, -(platform_len / 2.0 + slope_len / 2.0), 0.0, yaw)
        down_x, down_y = rotate_offset(cx, cy, platform_len / 2.0 + slope_len / 2.0, 0.0, yaw)
        parts.append(_box_link(f'ramp_{name}_up_slope', up_x, up_y, ramp_height / 2.0, 0, -ramp_angle, yaw, slope_len, width, deck_t, 'Gazebo/Red'))
        parts.append(_box_link(f'ramp_{name}_platform', cx, cy, ramp_height, 0, 0, yaw, platform_len, width, deck_t, 'Gazebo/Red'))
        parts.append(_box_link(f'ramp_{name}_down_slope', down_x, down_y, ramp_height / 2.0, 0, ramp_angle, yaw, slope_len, width, deck_t, 'Gazebo/Red'))
        add_side_walls(f'ramp_{name}_up', up_x, up_y, yaw, slope_len, width, ramp_height / 2.0, ramp_wall_w, ramp_wall_h, 'Gazebo/Red')
        add_side_walls(f'ramp_{name}_platform', cx, cy, yaw, platform_len, width, ramp_height + ramp_wall_h / 2.0, ramp_wall_w, ramp_wall_h, 'Gazebo/Red')
        add_side_walls(f'ramp_{name}_down', down_x, down_y, yaw, slope_len, width, ramp_height / 2.0, ramp_wall_w, ramp_wall_h, 'Gazebo/Red')

    # Green area: less dense gravel only, with passable/mid/high classes.
    gravel = values.get('gravel_area', {})
    xmin = float(gravel.get('x_min_m', -3.2))
    xmax = float(gravel.get('x_max_m', 3.5))
    ymin = float(gravel.get('y_min_m', -2.7))
    ymax = float(gravel.get('y_max_m', 3.2))
    avoid_radius = float(gravel.get('avoid_spawn_radius_m', 0.80))
    radius_min, radius_max = as_range(gravel.get('radius_range_m'), 0.035, 0.100)
    min_stone_dist = max(float(gravel.get('min_stone_distance_m', 0.32)), 0.0)
    placed_stones = []

    def sample_gravel_point():
        for _ in range(180):
            x = rng.uniform(xmin, xmax)
            y = rng.uniform(ymin, ymax)
            if math.hypot(x, y) <= avoid_radius:
                continue
            if all(math.hypot(x - px, y - py) >= min_stone_dist for px, py in placed_stones):
                placed_stones.append((x, y))
                return x, y
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        placed_stones.append((x, y))
        return x, y

    stone_specs = [
        ('low', int(gravel.get('low_count', 22)), as_range(gravel.get('low_height_range_m'), 0.015, 0.070), 'Gazebo/Green'),
        ('mid', int(gravel.get('mid_count', 10)), as_range(gravel.get('mid_height_range_m'), 0.090, 0.160), 'Gazebo/Orange'),
        ('high', int(gravel.get('high_count', 5)), as_range(gravel.get('high_height_range_m'), 0.220, 0.360), 'Gazebo/Black'),
    ]
    for label, count, height_range, material in stone_specs:
        for idx in range(max(count, 0)):
            x, y = sample_gravel_point()
            h = rng.uniform(height_range[0], height_range[1])
            radius = rng.uniform(radius_min, radius_max)
            parts.append(_cylinder_link(f'gravel_{label}_{idx + 1}', x, y, h / 2.0, radius, h, material))

    # Yellow area: complete course = up slope + pothole platform + down slope.
    pit = values.get('pit_area', {})
    pcx, pcy = [float(v) for v in pit.get('center', [4.2, -7.75])]
    pyaw = float(pit.get('yaw', 0.0))
    platform_len = max(float(pit.get('platform_length_m', pit.get('length_m', 5.8))), 1.0)
    pwid = max(float(pit.get('width_m', 2.25)), 0.8)
    pheight = max(float(pit.get('platform_height_m', 0.12)), 0.04)
    slope_len_yellow = max(float(pit.get('slope_length_m', 1.45)), 0.4)
    pdepth = float(pit.get('pit_depth_m', 0.05))
    pit_count = max(int(pit.get('pit_count', 5)), 0)
    min_pit_dist = max(float(pit.get('min_pit_center_distance_m', 0.90)), 0.0)
    pit_len_min, pit_len_max = as_range(pit.get('pit_length_range_m'), 0.45, 0.85)
    pit_w_min, pit_w_max = as_range(pit.get('pit_width_range_m'), 0.30, 0.60)
    side_wall_w = max(float(pit.get('side_wall_width_m', 0.08)), 0.03)
    side_wall_h = max(float(pit.get('side_wall_height_m', 0.16)), 0.04)
    pothole_wall_t = max(float(pit.get('pothole_wall_thickness_m', 0.045)), 0.02)
    yellow_slope_angle = math.atan2(pheight, slope_len_yellow)

    up_cx, up_cy = rotate_offset(pcx, pcy, -(platform_len / 2.0 + slope_len_yellow / 2.0), 0.0, pyaw)
    down_cx, down_cy = rotate_offset(pcx, pcy, platform_len / 2.0 + slope_len_yellow / 2.0, 0.0, pyaw)
    parts.append(_box_link('pit_course_up_slope', up_cx, up_cy, pheight / 2.0, 0, -yellow_slope_angle, pyaw, slope_len_yellow, pwid, deck_t, 'Gazebo/Yellow'))
    parts.append(_box_link('pit_course_down_slope', down_cx, down_cy, pheight / 2.0, 0, yellow_slope_angle, pyaw, slope_len_yellow, pwid, deck_t, 'Gazebo/Yellow'))
    add_side_walls('pit_course_up_slope', up_cx, up_cy, pyaw, slope_len_yellow, pwid, pheight / 2.0, side_wall_w, side_wall_h, 'Gazebo/Yellow')
    add_side_walls('pit_course_down_slope', down_cx, down_cy, pyaw, slope_len_yellow, pwid, pheight / 2.0, side_wall_w, side_wall_h, 'Gazebo/Yellow')

    pit_rects = []
    for idx in range(pit_count):
        for _ in range(160):
            lx = rng.uniform(-platform_len * 0.40, platform_len * 0.40)
            ly = rng.uniform(-pwid * 0.28, pwid * 0.28)
            l = rng.uniform(pit_len_min, pit_len_max)
            w = rng.uniform(pit_w_min, pit_w_max)
            center_ok = all(math.hypot(lx - ox, ly - oy) >= min_pit_dist for ox, oy, _ol, _ow in pit_rects)
            edge_ok = all(abs(lx - ox) > (l + ol) * 0.55 or abs(ly - oy) > (w + ow) * 0.55 for ox, oy, ol, ow in pit_rects)
            if center_ok and edge_ok:
                pit_rects.append((lx, ly, l, w))
                break

    tile = 0.35
    x_steps = max(int(math.ceil(platform_len / tile)), 1)
    y_steps = max(int(math.ceil(pwid / tile)), 1)
    for ix in range(x_steps):
        lx = -platform_len / 2.0 + (ix + 0.5) * platform_len / x_steps
        sx = platform_len / x_steps
        for iy in range(y_steps):
            ly = -pwid / 2.0 + (iy + 0.5) * pwid / y_steps
            sy = pwid / y_steps
            if any(abs(lx - px) < pl / 2.0 and abs(ly - py) < pw / 2.0 for px, py, pl, pw in pit_rects):
                continue
            x, y = rotate_offset(pcx, pcy, lx, ly, pyaw)
            parts.append(_box_link(f'pit_course_platform_tile_{ix + 1}_{iy + 1}', x, y, pheight - deck_t / 2.0, 0, 0, pyaw, sx, sy, deck_t, 'Gazebo/Yellow'))

    add_side_walls('pit_course_platform', pcx, pcy, pyaw, platform_len, pwid, pheight + side_wall_h / 2.0, side_wall_w, side_wall_h, 'Gazebo/Yellow')

    for idx, (lx, ly, l, w) in enumerate(pit_rects):
        x, y = rotate_offset(pcx, pcy, lx, ly, pyaw)
        bottom_z = max(pheight - pdepth, 0.006)
        parts.append(_box_link(f'pothole_{idx + 1}_bottom', x, y, bottom_z - 0.006, 0, 0, pyaw, l, w, 0.012, 'Gazebo/DarkGrey'))
        wall_z = bottom_z + pdepth / 2.0
        parts.append(_box_link(f'pothole_{idx + 1}_front_wall', x + math.cos(pyaw) * l / 2.0, y + math.sin(pyaw) * l / 2.0, wall_z, 0, 0, pyaw, pothole_wall_t, w + 2.0 * pothole_wall_t, pdepth, 'Gazebo/Yellow'))
        parts.append(_box_link(f'pothole_{idx + 1}_rear_wall', x - math.cos(pyaw) * l / 2.0, y - math.sin(pyaw) * l / 2.0, wall_z, 0, 0, pyaw, pothole_wall_t, w + 2.0 * pothole_wall_t, pdepth, 'Gazebo/Yellow'))
        parts.append(_box_link(f'pothole_{idx + 1}_left_wall', x - math.sin(pyaw) * w / 2.0, y + math.cos(pyaw) * w / 2.0, wall_z, 0, 0, pyaw, l, pothole_wall_t, pdepth, 'Gazebo/Yellow'))
        parts.append(_box_link(f'pothole_{idx + 1}_right_wall', x + math.sin(pyaw) * w / 2.0, y - math.cos(pyaw) * w / 2.0, wall_z, 0, 0, pyaw, l, pothole_wall_t, pdepth, 'Gazebo/Yellow'))

    return f"""
    <model name='rough_terrain_field'>
      <static>true</static>{''.join(parts)}
    </model>
"""


def _generate_rough_world(pkg_share_path, config_path):
    values = _parse_rough_terrain_config(config_path)
    base_world_path = os.path.join(pkg_share_path, 'worlds', '3d.world')
    output_path = os.path.join(tempfile.gettempdir(), 'lidar_nav2_rough_terrain.world')
    with open(base_world_path, 'r') as stream:
        world = stream.read()
    insert_at = world.rfind('  </world>')
    if insert_at < 0:
        raise RuntimeError(f'Invalid Gazebo world file: missing </world> in {base_world_path}')
    # Keep every model and state entry from 3d.world byte-for-byte; only append the
    # generated rough terrain model before the world closes.
    world = world[:insert_at] + _rough_terrain_model(values) + world[insert_at:]
    with open(output_path, 'w') as stream:
        stream.write(world)
    return output_path


def _launch_setup(context, *args, **kwargs):
    rviz = LaunchConfiguration('rviz')
    pkg_share_path = get_package_share_directory('get_urdf')
    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'nav2_new.rviz')

    world_arg = LaunchConfiguration('world').perform(context)
    terrain_config_path = LaunchConfiguration('terrain_config').perform(context)
    if os.path.isabs(world_arg):
        world_file_path = world_arg
    else:
        world_file_path = os.path.join(pkg_share_path, 'worlds', world_arg)

    if os.path.basename(world_file_path) == 'rough_terrain.world':
        world_file_path = _generate_rough_world(pkg_share_path, terrain_config_path)

    urdf_file_path = os.path.join(pkg_share_path, 'model', 'simple_car.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return [
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file_path],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-entity', 'simple_car', '-topic', 'robot_description']),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(rviz)),
    ]


def generate_launch_description():
    pkg_share_path = get_package_share_directory('get_urdf')
    default_config_path = os.path.join(pkg_share_path, 'config', 'rough_terrain.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start the get_urdf RViz window'),
        DeclareLaunchArgument(
            'world', default_value='3d.world',
            description='World file name under get_urdf/worlds, or an absolute world path'),
        DeclareLaunchArgument(
            'terrain_config', default_value=default_config_path,
            description='Rough-terrain YAML used when world:=rough_terrain.world'),
        OpaqueFunction(function=_launch_setup),
    ])
