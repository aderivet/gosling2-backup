import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    laser_filter_param_file = PathJoinSubstitution([
        f1tenth_launch_pkg_prefix,
        "config", "filters", "laser_filter.yaml"
    ])

    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace='lidar',
            parameters=[
                laser_filter_param_file,
                {'use_sim_time': False}
            ],
            remappings=[
                # ('scan_filtered', 'scan_filtered'),
                ('scan', '/lidar/scan')
            ]
        )
    ])
