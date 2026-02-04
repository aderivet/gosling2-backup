from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    # Create the launch configuration variables
    lidar_config = LaunchConfiguration('lidar_config')
    launch_filter = LaunchConfiguration('launch_filter')

    # Launch arguments
    lidar_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/sensors',
            'ydlidar_X4.yaml')

    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config_file,
                                     description='Path to the YDLIDAR parameters file to use.')

    declare_launch_filter_cmd = DeclareLaunchArgument(
            'launch_filter',
            default_value='True',
            description='Whether to launch the LIDAR filter')

    # Create Launch Description
    ld = LaunchDescription([lidar_la, declare_launch_filter_cmd])

    # Setup nodes
    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                               executable='ydlidar_ros2_driver_node',
                               name='ydlidar_ros2_driver_node',
                               output='screen',
                               emulate_tty=True,
                               parameters=[lidar_config],
                               namespace='lidar')

    laserscan_filter = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'laser_filter.launch.py']
            )),
            condition=IfCondition([launch_filter]),
    )

    ld.add_action(lidar_node)
    ld.add_action(laserscan_filter)

    return ld

