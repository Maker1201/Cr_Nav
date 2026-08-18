import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _parse_orbbec_config(config_path):
    cameras = {}
    current = None
    if not os.path.exists(config_path):
        return cameras
    with open(config_path, 'r') as stream:
        for raw_line in stream:
            clean = raw_line.split('#', 1)[0].rstrip()
            if not clean.strip():
                continue
            stripped = clean.strip()
            if stripped in ('orbbec_cameras:',):
                continue
            if clean.startswith('  ') and not clean.startswith('    ') and stripped.endswith(':'):
                current = stripped[:-1]
                cameras[current] = {}
                continue
            if current and clean.startswith('    ') and ':' in stripped:
                key, value = [item.strip() for item in stripped.split(':', 1)]
                value = value.strip('"\'')
                if value.startswith('[') and value.endswith(']'):
                    value = [float(item.strip()) for item in value[1:-1].split(',') if item.strip()]
                cameras[current][key] = value
    return cameras


def _static_tf_node(camera, parent, child, xyz, rpy):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{camera}_{child}_static_tf'.replace('/', '_'),
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        output='screen')


def _setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('camera_config').perform(context)
    cameras = _parse_orbbec_config(config_path)
    nodes = []
    for key in ('front', 'rear'):
        camera = cameras.get(key, {})
        if not camera:
            continue
        parent = camera.get('parent_frame', 'base_footprint')
        link = camera.get('link_frame', f'orbbec_{key}_link')
        depth = camera.get('depth_frame', f'orbbec_{key}_depth_optical_frame')
        xyz = camera.get('xyz', [0.0, 0.0, 0.0])
        rpy = camera.get('rpy', [0.0, 0.0, 0.0])
        nodes.append(_static_tf_node(key, parent, link, xyz, rpy))
        nodes.append(_static_tf_node(key, link, depth, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]))
    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('me_nav2_bringup'),
        'config',
        'orbbec_cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=default_config),
        OpaqueFunction(function=_setup),
    ])
