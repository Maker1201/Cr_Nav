import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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


def _camera_include(camera):
    orbbec_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'gemini_330_series.launch.py')
    args = {
        'camera_name': camera.get('camera_name', 'orbbec'),
        'enable_point_cloud': 'true',
        'enable_colored_point_cloud': 'false',
        'depth_registration': 'false',
    }
    if camera.get('serial_number'):
        args['serial_number'] = camera['serial_number']
    if camera.get('usb_port'):
        args['usb_port'] = camera['usb_port']
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch),
        launch_arguments=args.items())


def _setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('camera_config').perform(context)
    cameras = _parse_orbbec_config(config_path)
    actions = []
    for key in ('front', 'rear'):
        camera = cameras.get(key, {})
        if camera:
            actions.append(_camera_include(camera))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('me_nav2_bringup'),
        'config',
        'orbbec_cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=default_config),
        OpaqueFunction(function=_setup),
    ])
