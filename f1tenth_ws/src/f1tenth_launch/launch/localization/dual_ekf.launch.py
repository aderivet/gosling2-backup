"""
Todo: include ekf_launch files instead of setting nodes
Todo: add condition to select nodes
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    ekf_odom_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_odom.yaml")
    ekf_map_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_map.yaml")

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    odom_params_file = LaunchConfiguration('odom_params_file')
    map_params_file = LaunchConfiguration('map_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf_odom = LaunchConfiguration('use_ekf_odom')  # ekf_node, ukf_node
    use_ekf_map = LaunchConfiguration('use_ekf_map')  # ekf_node, ukf_node
    odom_frequency = LaunchConfiguration('odom_frequency')
    map_frequency = LaunchConfiguration('map_frequency')
    odom_node_name = LaunchConfiguration('odom_node_name')  # ekf_filter_node, ukf_filter_node
    map_node_name = LaunchConfiguration('map_node_name')  # ekf_filter_node, ukf_filter_node
    log_level = LaunchConfiguration('log_level')

    # declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')

    declare_odom_params_file_cmd = DeclareLaunchArgument(
            'odom_params_file',
            default_value=ekf_odom_config_file,
            description='Full path to the Kalman filter YAML file to use.')

    declare_map_params_file_cmd = DeclareLaunchArgument(
            'map_params_file',
            default_value=ekf_map_config_file,
            description='Full path to the Kalman filter YAML file to use.')

    declare_kf_odom_type = DeclareLaunchArgument(
            'use_ekf_odom', default_value='True',
            description='whether to use ekf. If false uses ukf instead')

    declare_kf_map_type = DeclareLaunchArgument(
            'use_ekf_map', default_value='True',
            description='whether to use ekf. If false uses ukf instead')

    declare_odom_frequency_la = DeclareLaunchArgument(
            'odom_frequency', default_value='100.0',
            description='Sensor fusion frequency')

    declare_map_frequency_la = DeclareLaunchArgument(
            'map_frequency', default_value='30.0',
            description='Sensor fusion frequency')

    declare_odom_node_name = DeclareLaunchArgument(
            'odom_node_name', default_value='ekf_odom_node',
            description='ekf_node, ukf_node')

    declare_map_node_name = DeclareLaunchArgument(
            'map_node_name', default_value='ekf_map_node',
            description='ekf_node, ukf_node')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    # Specify actions/nodes
    kf_bringup_group = GroupAction([
        PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),

        # Local/odom
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [f1tenth_launch_pkg_prefix, 'launch/localization', 'ekf_odom.launch.py']
                )),
                condition=IfCondition(use_ekf_odom),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': odom_params_file,
                    'use_ekf': use_ekf_odom,
                    'frequency': odom_frequency,
                    'node_name': odom_node_name,
                    'use_namespace': use_namespace,
                    'namespace': namespace,
                }.items()
        ),

        # global/map
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [f1tenth_launch_pkg_prefix, 'launch/localization', 'ekf_map.launch.py']
                )),
                condition=IfCondition(use_ekf_map),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': map_params_file,
                    'use_ekf': use_ekf_map,
                    'frequency': map_frequency,
                    'node_name': map_node_name,
                    'use_namespace': use_namespace,
                    'namespace': namespace,
                }.items()
        )
    ])

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_odom_params_file_cmd)
    ld.add_action(declare_map_params_file_cmd)
    ld.add_action(declare_kf_odom_type)
    ld.add_action(declare_kf_map_type)
    ld.add_action(declare_odom_frequency_la)
    ld.add_action(declare_map_frequency_la)
    ld.add_action(declare_odom_node_name)
    ld.add_action(declare_map_node_name)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(kf_bringup_group)

    return ld
