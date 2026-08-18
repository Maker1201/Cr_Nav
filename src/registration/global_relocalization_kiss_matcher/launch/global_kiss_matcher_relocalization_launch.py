from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="global_relocalization_kiss_matcher",
        executable="global_kiss_matcher_relocalization_exec",
        namespace="",
        output="screen",
        emulate_tty=True,
        remappings=remappings,
        parameters=[
            {
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "voxel_resolution": 0.25,
                "use_global_initialization": True,
                "use_kiss_recovery": True,
                "gicp_max_consecutive_failures": 2,
                "recovery_min_points": 1000,
                "recovery_cooldown_sec": 2.0,
                "verify_kiss_with_gicp": True,
                "loop.num_inliers_threshold": 3,
                "loop.overlap_threshold": 5.0,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "lidar_frame": "livox_frame",
                "robot_base_frame": "base_footprint",
                "prior_pcd_file": "/root/workspace/Lidar_nav2_ws/src/me_nav2_bringup/pcd/fast_lio_map.pcd",
                "input_cloud_topic": "/registered_scan",
            }
        ],
    )

    return LaunchDescription([node])
