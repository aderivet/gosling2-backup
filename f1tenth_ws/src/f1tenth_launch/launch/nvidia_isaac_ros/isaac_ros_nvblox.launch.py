"""
Do not run this live even though it runs on the GPU as it still uses a lot of CPU resources,
e.g 100% on Jetson Orin Nano.
Todo: (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py)
    * add people segmentation
    * add dynamic object removal
"""

import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, \
    SetParametersFromFile, SetParameter
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
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nvblox_bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Get launch directories
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_dir, 'launch', 'nvidia_isaac_ros')

    # Setup default directories
    base_config_dir = os.path.join(nvblox_bringup_dir, 'config', 'nvblox')
    specialization_dir = os.path.join(base_config_dir, 'specializations')

    # Config files
    base_config = os.path.join(base_config_dir, 'nvblox_base.yaml')
    dynamics_config = os.path.join(specialization_dir, 'nvblox_dynamics.yaml')
    segmentation_config = os.path.join(specialization_dir, 'nvblox_segmentation.yaml')
    realsense_config = os.path.join(specialization_dir, 'nvblox_realsense.yaml')

    # Setup launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    global_frame = LaunchConfiguration('global_frame', default='odom')
    launch_realsense_driver = LaunchConfiguration('launch_realsense_driver', default=False)
    launch_realsense_splitter = LaunchConfiguration('launch_realsense_splitter', default=False)
    depth_topic = LaunchConfiguration('depth_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    input_qos = LaunchConfiguration('input_qos', default='SENSOR_DATA')
    remove_dynamic_objects = LaunchConfiguration('remove_dynamic_objects', default=False)
    remove_people = LaunchConfiguration('remove_people', default=False)
    launch_visual_slam = LaunchConfiguration('launch_visual_slam', default=False)
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container',
                                                               default=False)
    component_container_name = LaunchConfiguration('component_container_name', default='nvblox_container')

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo or ROSBAG) clock if true. '
                        'Also used to set mapping mode to online (False) or offline (True) ')
    global_frame_arg = DeclareLaunchArgument('global_frame',
                                             default_value=global_frame,
                                             description="The name of the TF frame in which the map is built. "
                                                         "Set to odom since nvblox cannot create a map frame. ")
    launch_realsense_driver_arg = DeclareLaunchArgument('launch_realsense_driver',
                                                        default_value=launch_realsense_driver,
                                                        description="Whether to start the camera.")
    launch_realsense_splitter_arg = DeclareLaunchArgument('launch_realsense_splitter',
                                                          default_value=launch_realsense_splitter,
                                                          description="Whether to launch the "
                                                                      "realsense splitter component.")

    depth_topic_arg = DeclareLaunchArgument(
                'depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw',
                description='Raw unaligned depth topic to subscribe to. E.g '
                            '"/camera/camera/aligned_depth_to_color/image_raw", '
                            '"/camera/camera/depth/image_rect_raw", '
                            '"/camera/depth_registered/image_rect", '
                            '"/camera/realsense_splitter_node/output/depth", '  # if using realsense splitter
                            '"/camera/realigned_depth_to_color/image_raw"')

    depth_info_topic_arg = DeclareLaunchArgument(
            'depth_info_topic', default_value='/camera/camera/aligned_depth_to_color/camera_info',
            description='Raw unaligned depth topic to subscribe to. E.g '
                        '"/camera/camera/aligned_depth_to_color/camera_info", '
                        '"/camera/camera/depth/camera_info"')

    left_image_topic_la = DeclareLaunchArgument(
            'left_image_topic', default_value='/camera/camera/infra1/image_rect_raw',
            description='/camera/camera/infra1/image_rect_raw or /camera/realsense_splitter_node/output/infra_1')

    right_image_topic_la = DeclareLaunchArgument(
            'right_image_topic', default_value='/camera/camera/infra2/image_rect_raw',
            description='/camera/camera/infra2/image_rect_raw or /camera/realsense_splitter_node/output/infra_2')

    input_qos_arg = DeclareLaunchArgument('input_qos', default_value=input_qos,
                                          description="The QoS of the image and IMU topics. "
                                                      "Options: SENSOR_DATA, DEFAULT, SYSTEM_DEFAULT")

    remove_dynamic_objects_arg = DeclareLaunchArgument('remove_dynamic_objects',
                                                       default_value=remove_dynamic_objects,
                                                       description="Remove moving objects from the map.")

    remove_people_arg = DeclareLaunchArgument('remove_people',
                                              default_value=remove_people,
                                              description="Remove people from the map.")

    launch_visual_slam_arg = DeclareLaunchArgument('launch_visual_slam',
                                                   default_value=launch_visual_slam,
                                                   description="Whether to launch the visual slam node.")

    attach_to_shared_component_container_arg = DeclareLaunchArgument('attach_to_shared_component_container',
                                                                     default_value=attach_to_shared_component_container,
                                                                     description="Whether or not to join a container")

    component_container_name_arg = DeclareLaunchArgument('component_container_name',
                                                         default_value=component_container_name,
                                                         description="The name of the optional container to join")

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        global_frame_arg,
        launch_realsense_driver_arg,
        launch_realsense_splitter_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        left_image_topic_la,
        right_image_topic_la,
        input_qos_arg,
        remove_dynamic_objects_arg,
        remove_people_arg,
        launch_visual_slam_arg,
        attach_to_shared_component_container_arg,
        component_container_name_arg
    ]

    # Run nodes
    '''Create a container if not joining one for the realsense splitter'''
    nvblox_container = Node(
            name=component_container_name,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            condition=UnlessCondition(attach_to_shared_component_container)
    )

    # todo: use a launch context to check dynamic object and people flags
    load_composable_nodes = LoadComposableNodes(
            target_container=component_container_name,
            composable_node_descriptions=[
                # nvblox static node
                ComposableNode(
                        # condition=UnlessCondition(remove_people),
                        name='nvblox_node',
                        package='nvblox_ros',
                        plugin='nvblox::NvbloxNode'),
                # # Nvblox human removal node
                # ComposableNode(
                #         condition=IfCondition(remove_people),
                #         name='nvblox_human_node',
                #         package='nvblox_ros',
                #         plugin='nvblox::NvbloxHumanNode')
            ]
    )

    # Include launch files
    realsense_splitter_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nvidia_isaac_launch_dir, 'realsense_splitter.launch.py'])
            ),
            condition=IfCondition(launch_realsense_splitter),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'launch_realsense_driver': launch_realsense_driver,
                'attach_to_shared_component_container': 'True',
                'component_container_name': component_container_name,
                'input_qos': input_qos,
            }.items()
    )

    visual_slam_launch_include = TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                PathJoinSubstitution(
                                        [nvidia_isaac_launch_dir, 'isaac_ros_visual_slam_realsense.launch.py'])
                        ),
                        condition=IfCondition(launch_visual_slam),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'two_d_mode': 'True',
                            'base_frame': 'base_link',
                            'publish_map_to_odom_tf': 'False',
                            'publish_odom_to_baselink_tf': 'True',
                            'launch_realsense_driver': 'False',
                            'left_image_topic': left_image_topic,
                            'right_image_topic': right_image_topic,
                            'image_qos': input_qos,  # 'DEFAULT'
                            'imu_qos': input_qos,  # 'DEFAULT'
                            'attach_to_shared_component_container': 'True',
                            'component_container_name': component_container_name,
                        }.items()
                )
            ]
    )

    nvblox_group_action = GroupAction(
            actions=[
                # Set parameters with specializations
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParametersFromFile(base_config),
                SetParametersFromFile(dynamics_config, condition=IfCondition(remove_dynamic_objects)),
                SetParametersFromFile(segmentation_config, condition=IfCondition(remove_people)),
                SetParametersFromFile(realsense_config),
                SetParameter(name='global_frame', value=global_frame),
                SetParameter(name='input_qos', value=input_qos),
                SetParameter(name='maximum_sensor_message_queue_length', value=10),
                SetParameter(name='map_clearing_frame_id', value='base_link'),
                SetParameter(name='slice_visualization_attachment_frame_id', value='base_link'),

                # Remappings for realsense data
                SetRemap(src=['camera_0/depth/image'],
                         dst=[depth_topic]),
                SetRemap(src=['camera_0/depth/camera_info'],
                         dst=[depth_info_topic]),
                SetRemap(src=['camera_0/color/image'],
                         dst=['/camera/camera/color/image_raw']),
                SetRemap(src=['camera_0/color/camera_info'],
                         dst=['/camera/camera/color/camera_info']),

                # Include the node container
                load_composable_nodes
            ]
    )

    # Add launch arguments and nodes to the launch description
    ld = launch_args + [nvblox_container,
                        realsense_splitter_launch_include,
                        visual_slam_launch_include,
                        nvblox_group_action]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
