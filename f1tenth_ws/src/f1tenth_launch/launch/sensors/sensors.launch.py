#!/usr/bin/env python3
"""
Todo: launch realsense URDF/Xacro with robot state publisher using _d435i.urdf.xacro or test_d435i_camera.urdf.xacro (https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  | https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/launch/view_model.launch.py)
Todo: switch to XML launch to simplify.

Steps:
    * include lidar launch file and pass launch filter argument bool to lidar launch file
    * include camera_depth launch file and pass arguments for camera_depth filter,  IMU and IMU filter
    * (optional) include depth_image_to_laserscan launch file and pass argument to filter laserscan
    * (optional) publish lidar odometry
    * (optional) publish visual odometry
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    # Parameter files
    lidar_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'ydlidar_X4.yaml')

    depth_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_config.yaml')
    realsense_imu_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_imu_config.yaml')

    # depth_launch_path = PathJoinSubstitution(
    #         [FindPackageShare('f1tenth_launch'), 'launch', 'depth_image.launch.py']
    # )

    depth_sensor_name = 'realsense'
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    approx_sync = LaunchConfiguration('approx_sync')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles')
    reset_realsense = LaunchConfiguration('reset_realsense')
    publish_realsense_pointcloud = LaunchConfiguration('publish_realsense_pointcloud')
    align_realsense_depth = LaunchConfiguration('align_realsense_depth')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled')
    realsense_emitter_on_off = LaunchConfiguration('realsense_emitter_on_off')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=True)
    camera_launch_delay = LaunchConfiguration('camera_launch_delay', default=6.0)
    laserscan_launch_delay = LaunchConfiguration('laserscan_launch_delay', default=2.0)

    # Launch Arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    approx_sync_la = DeclareLaunchArgument(
            'approx_sync', default_value='True',
            description='Synchronize topics')
    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config,
                                     description='Path to the YDLIDAR parameters file to use.')
    depth_la = DeclareLaunchArgument('depth_config',
                                     default_value=depth_config,
                                     description='Path to the Realsense parameters file to use.')
    realsense_imu_la = DeclareLaunchArgument('realsense_imu_config',
                                             default_value=realsense_imu_config,
                                             description='Path to the Realsense IMU parameters file to use.')
    stereo_to_pointcloud_la = DeclareLaunchArgument('stereo_to_pointcloud',
                                                    default_value='False',
                                                    description='Whether to publish a PointCloud2 message from stereo '
                                                                'images.')
    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value='False',
                                                        description='Whether to publish a PointCloud2 message from a '
                                                                    'depth image.')
    detect_ground_and_obstacles_la = DeclareLaunchArgument('detect_ground_and_obstacles',
                                                           default_value='False',
                                                           description='Whether to use RTABmaps obstacle detector.')

    reset_realsense_la = DeclareLaunchArgument(
            'reset_realsense',
            default_value='False',
            description='Whether to reset the realsense device.')

    publish_realsense_pointcloud_la = DeclareLaunchArgument('publish_realsense_pointcloud',
                                                            default_value='True',
                                                            description='Whether to publish PointClouds using '
                                                                        'librealsense SDK. Could be disabled when '
                                                                        'recording ROSBags or mapping. ')

    align_realsense_depth_la = DeclareLaunchArgument('align_realsense_depth',
                                           default_value='True',
                                           description='Whether to align the depth to other frames')

    realsense_emitter_enabled_la = DeclareLaunchArgument(
            'realsense_emitter_enabled',
            default_value='0',
            description='Whether to enable the IR emitters to improve depth and pointcloud quality. '
                        'Unfortunately, this renders the stereo IR cameras unusable for mapping, '
                        'VSLAM, VIO odometry, etc. '
                        'Disable when mapping or running VIO enable if accurate pointclouds are essential.')

    realsense_emitter_on_off_la = DeclareLaunchArgument(
            'realsense_emitter_on_off',
            default_value='False',
            description='Whether to alternate enabling/disabling the emitters. '
                        'This can be used to simultaneously '
                        'get accurate depth maps and pointclouds (when in the on state, i.e enabled) and '
                        'have usable IR images (when in the off state)')

    launch_realsense_splitter_node_la = DeclareLaunchArgument(
            'launch_realsense_splitter_node', default_value=launch_realsense_splitter_node,
            description='Whether to launch the realsense splitter node.')

    camera_launch_delay_la = DeclareLaunchArgument(
            'camera_launch_delay', default_value=camera_launch_delay,
            description='Delay in seconds before launching the camera nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')

    laserscan_launch_delay_la = DeclareLaunchArgument(
            'laserscan_launch_delay', default_value=laserscan_launch_delay,
            description='Delay in seconds before launching the laserscan nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')

    # Create Launch Description
    ld = LaunchDescription([use_sim_time_la, approx_sync_la,
                            lidar_la, depth_la,
                            stereo_to_pointcloud_la, depthimage_to_pointcloud_la, detect_ground_and_obstacles_la,
                            reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
                            realsense_emitter_enabled_la,
                            realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
                            camera_launch_delay_la, laserscan_launch_delay_la])

    # Nodes
    lidar_node = TimerAction(
            period=laserscan_launch_delay,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(PathJoinSubstitution(
                                [f1tenth_launch_dir, 'launch/sensors', 'ydlidar.launch.py']
                        ))
                )
            ]
    )

    # depth_image_node = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(depth_launch_path),
    #         condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
    #         launch_arguments={'sensor': depth_sensor_name}.items()
    #     )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'realsense_d435i.launch.py']
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'reset_realsense': reset_realsense,
                'enable_pointcloud': publish_realsense_pointcloud,
                "align_depth": align_realsense_depth,
                'emitter_enabled': realsense_emitter_enabled,
                'emitter_on_off': realsense_emitter_on_off,
                'launch_realsense_splitter_node': launch_realsense_splitter_node,
            }.items()
    )

    depth_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'scan_time': 0.0333},  # 1 / (desired_frequency, i.e 30.0 Hz)
                        {'use_sim_time': use_sim_time},
                        {'output_frame': "camera_link"},  # camera_link, sensor_kit_link
                        # {'scan_height': 1},  # number of pixel rows to use. The minimum is selected
                        {'range_min': 0.01},  # 0.45
                        {'range_max': 10.0},
                        # {
                        #     'qos_overrides./parameter_events.publisher.depth': 5,
                        #     'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                        #     # 'qos_overrides./parameter_events.publisher.history': 'keep_last',
                        #     # 'qos_overrides./parameter_events.publisher.durability': 'volatile'
                        # }  # Sensor Data QoS. For now has no effect
                        ],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),  # /camera/camera/aligned_depth_to_color/image_raw
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan/from_depth_image')
            ],
            # arguments=['depth:=/camera/camera/depth/image_rect_raw',
            #            'depth_camera_info:=/camera/camera/depth/camera_info',
            #            'scan:=/scan/from_depth_image']
    )

    # depth and/or stereo to Pointcloud
    stereo_and_depth_image_processing_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'stereo_and_depth_image_processing.launch.py']
            )),
            # condition=IfCondition([imu_only]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'approx_sync': approx_sync,
                'queue_size': '1',  # default: 10
                'depthimage_to_pointcloud': depthimage_to_pointcloud,
                'stereo_to_pointcloud': stereo_to_pointcloud,
                'left_image_topic': '/camera/camera/infra1/image_rect_raw',
                'right_image_topic': '/camera/camera/infra2/image_rect_raw',
                'rgb_image_topic': '/camera/camera/color/image_raw',
                'depth_image_topic': '/camera/camera/aligned_depth_to_color/image_raw', # '/camera/camera/depth/image_rect_raw',
                'color_pointcloud': 'True',
                'use_image_proc': 'False',
                'use_rtabmap': 'True',
                'detect_ground_and_obstacles': detect_ground_and_obstacles,
                'register_depth': 'False',
                'rtabmap_depth_decimation': '2',  # 1 means no decimation,
                'rtabmap_voxel_size': '0.1',  # 0.0 means no filtering
            }.items()
    )

    # ######### RTabMap depth to pointcloud to depth. Note: use a voxelized pointcloud
    rtabmap_obstacle_and_floor_detection_node = Node(
            name='rtabmap_obstacle_and_floor_detection_node',
            condition=IfCondition(detect_ground_and_obstacles),
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[
                {'frame_id': 'base_link'},  # 'camera_link', 'base_link', 'sensor_kit_link', base_footprint
                {'queue_size': '1'},
                {'approx_sync': approx_sync},
                {'use_sim_time': use_sim_time},
                {'min_cluster_size': 20},  # Minimum size of the segmented clusters to keep. Default=20
                {'max_obstacles_height': 5.0},  # Maximum height of obstacles. Default=0.0
            ],
            remappings=[
                ('cloud', '/camera/camera/depth/color/points'),
                ('obstacles', '/camera/camera/obstacles_from_cloud'),
                ('ground', '/camera/camera/ground_from_cloud'),
                ('proj_obstacles', '/camera/camera/projected_obstacles')
            ]
    )

    image_processing_group = TimerAction(
            period=camera_launch_delay,
            actions=[
                GroupAction([
                    realsense_node,
                    # realsense_imu_node,
                    # depth_to_laserscan_node,
                    stereo_and_depth_image_processing_node,
                    rtabmap_obstacle_and_floor_detection_node
                ])
            ]
    )

    # Add nodes to launch description
    ld.add_action(lidar_node)

    # ld.add_action(realsense_node)
    # # ld.add_action(realsense_imu_node)
    # # ld.add_action(depth_to_laserscan_node)  # disabled because it is not used. Costmaps can just use pointclouds instead. Also, causes significant performance drop and CPU overhead.
    #
    # ld.add_action(stereo_and_depth_image_processing_node)
    # ld.add_action(rtabmap_obstacle_and_floor_detection_node)

    ld.add_action(image_processing_group)
    return ld
