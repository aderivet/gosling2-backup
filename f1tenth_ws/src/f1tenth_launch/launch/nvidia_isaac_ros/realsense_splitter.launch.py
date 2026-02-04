import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, \
    SetParametersFromFile, \
    SetParameter
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get package directories
    # realsense_config_dir = get_package_share_directory('nvblox_examples_bringup')  # todo: switch to my realsense config
    realsense_config_dir = get_package_share_directory('f1tenth_launch')

    # Setup default directories
    # realsense_config.yaml, realsense.yaml
    realsense_config_file_path = os.path.join(realsense_config_dir, 'config', 'sensors', 'realsense_config.yaml')

    # Setup launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    launch_realsense_driver = LaunchConfiguration('launch_realsense_driver', default=False)
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container',
                                                               default=False)
    component_container_name = LaunchConfiguration('component_container_name', default='realsense_container')
    intra_process_comms = LaunchConfiguration("intra_process_comms", default=True)
    realsense_config_file = LaunchConfiguration('realsense_config_file', default=realsense_config_file_path)
    # '/camera/camera/depth/image_rect_raw' or '/camera/camera/aligned_depth_to_color/image_raw'
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/camera/depth/image_rect_raw')
    input_qos = LaunchConfiguration('input_qos', default='SENSOR_DATA')
    output_qos = LaunchConfiguration('output_qos', default='SENSOR_DATA')

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    launch_realsense_driver_launch_arg = DeclareLaunchArgument('launch_realsense_driver',
                                                               default_value=launch_realsense_driver,
                                                               description="Whether or not to "
                                                                           "launch the Realsense Camera.")

    attach_to_shared_component_container_arg = DeclareLaunchArgument('attach_to_shared_component_container',
                                                                     default_value=attach_to_shared_component_container,
                                                                     description="Whether or not to join a container")

    component_container_name_arg = DeclareLaunchArgument('component_container_name',
                                                         default_value=component_container_name,
                                                         description="The name of the optional container to join")

    intra_process_comms_arg = DeclareLaunchArgument('intra_process_comms',
                                                    default_value=intra_process_comms,
                                                    description="Whether to launch "
                                                                "components with intra process communication.")

    realsense_config_file_arg = DeclareLaunchArgument('realsense_config_file', default_value=realsense_config_file,
                                                      description="Path to a config file")

    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic,
                                            description="The depth topic to remap.")

    input_qos_arg = DeclareLaunchArgument('input_qos', default_value=input_qos,
                                          description="The QoS of the realsense topic published by the driver.")

    output_qos_arg = DeclareLaunchArgument('output_qos', default_value=output_qos,
                                           description="The output QoS of the topics published by the "
                                                       "realsense splitter node.")

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        launch_realsense_driver_launch_arg,
        attach_to_shared_component_container_arg,
        component_container_name_arg,
        intra_process_comms_arg,
        realsense_config_file_arg,
        depth_topic_arg,
        input_qos_arg,
        output_qos_arg
    ]

    # Run nodes
    '''Create a container if not joining one for the realsense splitter'''
    realsense_splitter_container = Node(
            name=component_container_name,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            condition=UnlessCondition(attach_to_shared_component_container)
    )

    realsense_splitter_component = ComposableNode(
            namespace="camera",
            name='realsense_splitter_node',
            package='realsense_splitter',
            plugin='nvblox::RealsenseSplitterNode',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_qos': input_qos,
                'output_qos': output_qos,
            }],
            remappings=[
                ('input/infra_1', 'camera/infra1/image_rect_raw'),
                ('input/infra_1_metadata', 'camera/infra1/metadata'),
                ('input/infra_2', 'camera/infra2/image_rect_raw'),
                ('input/infra_2_metadata', 'camera/infra2/metadata'),
                ('input/depth', 'camera/depth/image_rect_raw'),
                ('input/depth_metadata', 'camera/depth/metadata'),
                ('input/pointcloud', 'camera/depth/color/points'),
                ('input/pointcloud_metadata', 'camera/depth/metadata'),
            ]
    )

    realsense_driver_component = LoadComposableNodes(
            condition=IfCondition(launch_realsense_driver),
            target_container=component_container_name,
            composable_node_descriptions=[
                # Realsense Driver Node Factory
                ComposableNode(
                        namespace="camera",
                        package='realsense2_camera',
                        plugin='realsense2_camera::RealSenseNodeFactory',
                        parameters=[realsense_config_file],  # todo: pass the input QoS
                        extra_arguments=[{'use_intra_process_comms': intra_process_comms}]
                )
            ]
    )

    load_composable_nodes = LoadComposableNodes(
            target_container=component_container_name,
            composable_node_descriptions=[
                # RealSense splitter node
                realsense_splitter_component
            ]
    )

    # Add launch arguments and nodes to the launch description
    ld = launch_args + [realsense_splitter_container,
                        realsense_driver_component,
                        load_composable_nodes]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
