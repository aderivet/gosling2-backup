from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    mux_config_file = os.path.join(
            get_package_share_directory('f1tenth_stack'),
            'config',
            'mux.yaml'
    )

    mux_config = LaunchConfiguration('mux_config')

    mux_la = DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config_file,
            description='Descriptions for ackermann mux configs')

    ld = LaunchDescription([mux_la])

    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[mux_config],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', '/vehicle/ackermann_cmd')]
    )

    # add nodes to the launch description
    ld.add_action(ackermann_mux_node)

    return ld
