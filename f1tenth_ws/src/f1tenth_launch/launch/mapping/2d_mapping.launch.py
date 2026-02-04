"""
Todo: add boolean flag to launch the vehicle_interface, sensors, etc. so that online mapping can be launched alone
Todo: launch joystick in a separate package, i.e vehicle_bringup
Todo: pass arguments to mapper
Todo: load from ROSBAG
Todo: refer to https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/slam_launch.py
Todo: add argument to select online/offline mapping
Todo: either launch odom ekf here or include this file in a higher level bringup file
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Setup default directories
    default_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rviz_cfg_path = os.path.join(f1tenth_launch_dir, 'rviz', 'slam_toolbox.rviz')
    # rviz_cfg_path = os.path.join(f1tenth_launch_dir, 'rviz', 'slam_toolbox.rviz')

    offline_mapping_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")

    # Setup launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    autostart = LaunchConfiguration('autostart', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    offline_mapping = LaunchConfiguration('offline_mapping', default='True')
    offline_mapping_param_file = LaunchConfiguration('offline_mapping_param_file',
                                                     default=offline_mapping_param_file_path)
    online_mapping_param_file = LaunchConfiguration('online_mapping_param_file',
                                                    default=online_mapping_param_file_path)
    map_file_path = LaunchConfiguration('map_file_path', default=default_map_file_path)
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='True')
    map_topic = LaunchConfiguration('map_topic', default='map')
    with_rviz = LaunchConfiguration('with_rviz', default='False')
    rviz_cfg_path_param = LaunchConfiguration('rviz_cfg_path_param', default=rviz_cfg_path)
    save_map_timeout = LaunchConfiguration('save_map_timeout', default=2000.0)
    free_thresh_default = LaunchConfiguration('free_thresh_default', default=0.25)
    occupied_thresh_default = LaunchConfiguration('occupied_thresh_default', default=0.65)
    lifecycle_nodes = ['map_saver']

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the navigation stack')
    declare_offline_mapping = DeclareLaunchArgument(
            "offline_mapping",
            default_value=offline_mapping,
            description="Offline/simulation maping.",
    )
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation or ROSBAG clock if true')
    offline_mapping_param_file_la = DeclareLaunchArgument(
            "offline_mapping_param_file",
            default_value=offline_mapping_param_file,
            description="Path to config file for mapping nodes",
    )
    online_mapping_param_file_la = DeclareLaunchArgument(
            "online_mapping_param_file",
            default_value=online_mapping_param_file,
            description="Path to config file for mapping nodes",
    )

    map_topic_la = DeclareLaunchArgument('map_topic', default_value=map_topic,
                                         description="The default map topic.")

    map_file_path_la = DeclareLaunchArgument('map_file_path', default_value=map_file_path,
                                             description="location to store the map periodically.")

    with_rviz_param = DeclareLaunchArgument(
            'with_rviz',
            default_value=with_rviz,
            description='Launch rviz in addition to other nodes'
    )

    rviz_cfg_path_param_la = DeclareLaunchArgument(
            'rviz_cfg_path_param',
            default_value=rviz_cfg_path_param,
            description='Launch RVIZ2 with the specified config file'
    )

    offline_slam_launch = Node(
            condition=IfCondition([offline_mapping]),
            parameters=[
                offline_mapping_param_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
    )

    online_slam_launch = Node(
            condition=IfCondition(PythonExpression(['not ', offline_mapping])),
            parameters=[
                online_mapping_param_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(with_rviz),
            arguments=['-d', rviz_cfg_path_param]
    )

    # Nodes launching commands
    # the map saver node doesn't work, might need to setup yaml path for map_server. Use rosservices instead, e.g:
    # ros2 run nav2_map_server map_saver_cli -f /f1tenth/data/maps/raslab
    # ros2 service call /slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph "{filename: '/f1tenth/data/maps/raslab'}"
    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[
                {'map_topic': map_topic},
                {'map_url': map_file_path},
                {'save_map_timeout': save_map_timeout},
                {'free_thresh_default': free_thresh_default},
                {'occupied_thresh_default': occupied_thresh_default},
                {'map_subscribe_transient_local': map_subscribe_transient_local}])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        use_sim_time_la,
        declare_offline_mapping,
        with_rviz_param,
        offline_mapping_param_file_la,
        online_mapping_param_file_la,
        map_topic_la,
        map_file_path_la,
        rviz_cfg_path_param_la,
        offline_slam_launch,
        online_slam_launch,
        rviz2,
        start_map_saver_server_cmd,
        start_lifecycle_manager_cmd
    ])
