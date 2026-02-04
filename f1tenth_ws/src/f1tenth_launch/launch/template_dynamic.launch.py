"""
todo: move to a ROS best practices github repository

A nice tutorial: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get package directories
    template_dir = get_package_share_directory('template_package')  # or FindPackageShare('template_package')

    # Get launch directories
    template_launch_dir = os.path.join(template_dir, 'launch')  # or PathJoinSubstitution([template_dir, 'launch'])

    # Setup default directories
    template_config_file_path = os.path.join(template_dir, 'config', 'template.yaml')

    # Setup launch configuration variables
    template_variable = LaunchConfiguration('template_variable', default=True)
    template_config_file = LaunchConfiguration('template_config_file', default=template_config_file_path)

    # Declare launch arguments
    template_launch_arg = DeclareLaunchArgument('template_variable', default_value=template_variable,
                                                description="Template description")
    template_config_file_arg = DeclareLaunchArgument('template_config_file', default_value=template_config_file,
                                                     description="Path to a config file")

    # Add launch arguments to a list
    launch_args = [
        template_launch_arg,
        template_config_file_arg
    ]

    # To convert a launch_configuration variable to a string to use with Python functions
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    ros_distro_string = ros_distro.perform(context)

    '''
    Launch nodes. 
    To check conditions, use IfCondition(template_variable) or LaunchConfigurationEquals('template_variable', 'True'),
    UnlessCondition(template_variable) or LaunchConfigurationNotEquals('template_variable', 'True')
    '''
    # Run nodes
    template_node_launch = Node(
                        condition=IfCondition(template_variable),
                        package='template_package_name',
                        executable='template_executable_name',
                        name='template_node_name',
                        output='screen',
                        parameters=[
                            {'template_parameter': True},
                            {'template_parameter2': True},
                            template_config_file
                        ],
                        remappings=[
                            ('from_topic1', 'to_topic1'),  # input topic: /vehicle/sensors/imu/raw
                            ('from_topic2', 'to_topic2'),  # output topic: /vehicle/sensors/imu/data
                        ]
                )

    # Include launch files
    template_python_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([template_launch_dir, 'template.launch.py'])
            ),
            condition=IfCondition(template_variable),
            launch_arguments={
                "template_argument": template_variable
            }.items()
    )

    # XMLLaunchDescriptionSource is for files of the form "*.launch.xml"
    template_xml_launch_include = IncludeLaunchDescription(
                        XMLLaunchDescriptionSource(  # or FrontendLaunchDescriptionSource
                                launch_file_path=PathJoinSubstitution([
                                    FindPackageShare("template_package"), "launch", "template.launch.xml",
                                ]),
                        ),
                        launch_arguments={
                            "arg1": template_variable,
                            "arg2": "True",
                        }.items()
                )

    #  FrontendLaunchDescriptionSource is for files of the form "*.xml"
    template_xml_launch_include2 = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('template_package'), 'launch', 'template_launch.xml'
            ]),
        ),
        launch_arguments={
            'arg1': 'True',
            'arg2': 'arg2',
        }.items()
    )

    # to delay the execution of a node/launch
    delay_launch = TimerAction(period=10.0, actions=[IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('template_package'), 'launch', 'template_launch.xml'
            ]),
        ),
        launch_arguments={
            'arg1': 'True',
            'arg2': 'arg2',
        }.items()
    )
    ])

    # Add launch arguments and nodes to the launch description
    ld = launch_args + [template_node_launch,
                        template_python_launch_include,
                        template_xml_launch_include,
                        template_xml_launch_include2]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
