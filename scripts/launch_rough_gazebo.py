#!/usr/bin/env python3
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# This launch file is intentionally workspace-relative. It works both on the
# host path and inside Docker, as long as it is run from this workspace layout.
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
GET_URDF_SRC = WORKSPACE_ROOT / 'src' / 'get_urdf'
sys.path.insert(0, str(GET_URDF_SRC / 'launch'))
from get_urdf_launch import _generate_rough_world  # noqa: E402


def generate_launch_description():
    urdf_path = GET_URDF_SRC / 'model' / 'simple_car.urdf'
    terrain_config = GET_URDF_SRC / 'config' / 'rough_terrain.yaml'
    world_path = _generate_rough_world(str(GET_URDF_SRC), str(terrain_config))

    print(f'[rough_gazebo] workspace: {WORKSPACE_ROOT}', flush=True)
    print(f'[rough_gazebo] world: {world_path}', flush=True)
    print(f'[rough_gazebo] urdf: {urdf_path}', flush=True)

    with open(urdf_path, 'r') as stream:
        robot_desc = stream.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
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
    ])
