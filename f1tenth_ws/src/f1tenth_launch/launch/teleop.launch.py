"""
Launches: vehicle driver, joystick driver, mux, and sensors
Todo: setup loglevel
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable, LogInfo, TimerAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    # Get launch directories
    vehicle_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'vehicle')
    sensor_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'sensors')
    localization_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'localization')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')

    # Declare launch configuration variables
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace',
                                            default='')  # used to distinguish between multiple F1/10s
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default="False")
    use_gpu = LaunchConfiguration('use_gpu', default=True)
    launch_joystick = LaunchConfiguration('launch_joystick', default=True)
    launch_sensors = LaunchConfiguration('launch_sensors', default=True)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=True)
    launch_tfs = LaunchConfiguration('launch_tfs', default=True)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    launch_visualization = LaunchConfiguration('launch_visualization', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)

    deadman_buttons = LaunchConfiguration('deadman_buttons', default="[4, 9]")
    steering_button = LaunchConfiguration('steering_button', default=2)
    max_speed = LaunchConfiguration('max_speed', default=5.0)
    max_steering = LaunchConfiguration('max_steering', default=0.34)
    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_steering_rate', default=3.2)
    vesc_poll_rate = LaunchConfiguration('vesc_poll_rate', default=200.0)
    launch_ackermann_to_vesc_node = LaunchConfiguration('launch_ackermann_to_vesc_node', default='True')
    launch_vesc_to_odom_node = LaunchConfiguration('launch_vesc_to_odom_node', default='True')
    launch_throttle_interpolator_node = LaunchConfiguration('launch_throttle_interpolator_node', default='False')

    approx_sync = LaunchConfiguration('approx_sync', default='True')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud', default='False')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud', default='False')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles', default='False')

    reset_realsense = LaunchConfiguration('reset_realsense', default='False')
    publish_realsense_pointcloud = LaunchConfiguration('publish_realsense_pointcloud', default='False')
    align_realsense_depth = LaunchConfiguration('align_realsense_depth', default='True')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled', default='0')
    realsense_emitter_on_off = LaunchConfiguration('realsense_emitter_on_off', default='False')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)

    camera_launch_delay = LaunchConfiguration('camera_launch_delay', default='6.0')
    laserscan_launch_delay = LaunchConfiguration('laserscan_launch_delay', default='2.0')

    # Declare launch arguments
    declare_f1tenth_namespace_cmd = DeclareLaunchArgument(
            'f1tenth_namespace',
            default_value=f1tenth_namespace,
            description='Top-level namespace to distinguish between each F1/10 (robot).')

    declare_use_f1tenth_namespace_cmd = DeclareLaunchArgument(
            'use_f1tenth_namespace',
            default_value=use_f1tenth_namespace,
            description='Whether to apply a namespace to the entire stack. '
                        'If launch argument for f1tenth_namespace is empty and this is true, '
                        'uses the $VEHICLE_NAME environment variable. '
                        'Defaults to the username if $VEHICLE_NAME doesn\'t exist and the namespace argument is empty.')

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

    launch_joystick_arg = DeclareLaunchArgument('launch_joystick', default_value=launch_joystick,
                                                description="Launch the joystick driver, "
                                                            "teleop (speed and steering), and mux nodes.")

    launch_sensors_arg = DeclareLaunchArgument('launch_sensors', default_value=launch_sensors,
                                               description="Launch the sensors")

    launch_vehicle_arg = DeclareLaunchArgument('launch_vehicle', default_value=launch_vehicle,
                                               description="Launch the vehicle")

    launch_tfs_arg = DeclareLaunchArgument('launch_tfs', default_value=launch_tfs,
                                           description="Launch the tfs")

    launch_localization_arg = DeclareLaunchArgument('launch_localization',
                                                    default_value=launch_localization,
                                                    description="Launch the localization components.")
    launch_local_localization_arg = DeclareLaunchArgument('launch_local_localization',
                                                          default_value=launch_local_localization,
                                                          description="Launch the local localization component.")
    launch_global_localization_arg = DeclareLaunchArgument('launch_global_localization',
                                                           default_value=launch_global_localization,
                                                           description="Launch the global localization component.")
    launch_visualization_arg = DeclareLaunchArgument('launch_visualization',
                                                     default_value=launch_visualization,
                                                     description="Launch RViz.")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    rviz_config_arg = DeclareLaunchArgument('rviz_config_file',
                                            default_value=rviz_config_file,
                                            description="The path to the rviz configuration file.")

    deadman_buttons_la = DeclareLaunchArgument(
            'deadman_buttons',
            default_value=deadman_buttons,
            description='Buttons used to arm the vehicle actuators.')

    steering_button_la = DeclareLaunchArgument(
            'steering_button',
            default_value=steering_button,
            description='Button used to control the steering angle. 2 for DualShock/DualSense, 3 for Logitech F710')

    max_speed_la = DeclareLaunchArgument(
            'max_speed',
            default_value=max_speed,
            description='The maximum speed in m/s.')

    max_steering_la = DeclareLaunchArgument(
            'max_steering',
            default_value=max_steering,
            description='The maximum steering angle in rads.')

    max_acceleration_la = DeclareLaunchArgument(
            'max_acceleration',
            default_value=max_acceleration,
            description='The maximum acceleration in m/s^2.')

    max_steering_rate_la = DeclareLaunchArgument(
            'max_steering_rate',
            default_value=max_steering_rate,
            description='The maximum steering rate in rads/s.')

    vesc_poll_rate_la = DeclareLaunchArgument(
            'vesc_poll_rate',
            default_value=vesc_poll_rate,
            description='The frequency at which to send/receive messages from/to the VESC.')

    declare_launch_ackermann_to_vesc_node = DeclareLaunchArgument(
            'launch_ackermann_to_vesc_node',
            default_value=launch_ackermann_to_vesc_node,
            description='Send ackermann commands to the VESC.')
    declare_launch_vesc_to_odom_node = DeclareLaunchArgument(
            'launch_vesc_to_odom_node',
            default_value=launch_vesc_to_odom_node,
            description='Publish odometry messages from the VESC.')
    declare_launch_throttle_interpolator_node = DeclareLaunchArgument(
            'launch_throttle_interpolator_node',
            default_value=launch_throttle_interpolator_node,
            description='Interpolate commands before sending to the VESC. '
                        'Set to False if using MPC (or increase limits), True otherwise')

    approx_sync_la = DeclareLaunchArgument(
            'approx_sync', default_value='True',
            description='Synchronize topics')

    stereo_to_pointcloud_la = DeclareLaunchArgument('stereo_to_pointcloud',
                                                    default_value=stereo_to_pointcloud,
                                                    description='Whether to publish a PointCloud2 message from stereo '
                                                                'images.')

    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value=depthimage_to_pointcloud,
                                                        description='Whether to publish a PointCloud2 message from a '
                                                                    'depth image.')
    detect_ground_and_obstacles_la = DeclareLaunchArgument('detect_ground_and_obstacles',
                                                           default_value=detect_ground_and_obstacles,
                                                           description='Whether to use RTABmaps obstacle detector.')

    reset_realsense_la = DeclareLaunchArgument(
            'reset_realsense',
            default_value='False',
            description='Whether to reset the realsense device.')

    publish_realsense_pointcloud_la = DeclareLaunchArgument('publish_realsense_pointcloud',
                                                            default_value=publish_realsense_pointcloud,
                                                            description='Whether to publish PointClouds using '
                                                                        'librealsense SDK. Could be disabled when '
                                                                        'recording ROSBags or mapping. ')
    align_realsense_depth_la = DeclareLaunchArgument('align_realsense_depth',
                                           default_value=align_realsense_depth,
                                           description='Whether to align the depth to other frames')

    realsense_emitter_enabled_la = DeclareLaunchArgument(
            'realsense_emitter_enabled',
            default_value=realsense_emitter_enabled,
            description='Whether to enable the IR emitters to improve depth and pointcloud quality. '
                        'Unfortunately, this renders the stereo IR cameras unusable for mapping, '
                        'VSLAM, VIO odometry, etc. '
                        'Disable when mapping or running VIO enable if accurate pointclouds are essential.')

    realsense_emitter_on_off_la = DeclareLaunchArgument(
            'realsense_emitter_on_off',
            default_value=realsense_emitter_on_off,
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

    launch_args = [
        declare_f1tenth_namespace_cmd,
        declare_use_f1tenth_namespace_cmd,
        use_gpu_la,
        launch_joystick_arg,
        launch_sensors_arg,
        launch_vehicle_arg,
        launch_tfs_arg,
        launch_localization_arg,
        launch_local_localization_arg,
        launch_global_localization_arg,
        launch_visualization_arg,
        declare_use_sim_time_cmd,
        rviz_config_arg,
        deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la,
        max_acceleration_la, max_steering_rate_la, vesc_poll_rate_la,
        declare_launch_ackermann_to_vesc_node, declare_launch_vesc_to_odom_node,
        declare_launch_throttle_interpolator_node,
        approx_sync_la, stereo_to_pointcloud_la, depthimage_to_pointcloud_la,
        detect_ground_and_obstacles_la, reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
        realsense_emitter_enabled_la, realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
        camera_launch_delay_la, laserscan_launch_delay_la
    ]

    # Launch nodes
    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)

    joystick_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'joystick.launch.py'])
            ),
            condition=IfCondition(launch_joystick),
            launch_arguments={
                "deadman_buttons": deadman_buttons,
                "steering_button": steering_button,
                "max_speed": max_speed,
                "max_steering": max_steering
            }.items()
    )

    ackermann_mux_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'ackermann_mux.launch.py'])
            ),
            condition=IfCondition(launch_joystick)
    )

    sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sensor_include_dir, 'sensors.launch.py'])
            ),
            condition=IfCondition(launch_sensors),
            launch_arguments={
                "approx_sync": approx_sync,
                "stereo_to_pointcloud": stereo_to_pointcloud,
                "depthimage_to_pointcloud": depthimage_to_pointcloud,
                "detect_ground_and_obstacles": detect_ground_and_obstacles,
                'reset_realsense': reset_realsense,
                "publish_realsense_pointcloud": publish_realsense_pointcloud,
                "align_realsense_depth": align_realsense_depth,
                "realsense_emitter_enabled": realsense_emitter_enabled,
                "realsense_emitter_on_off": realsense_emitter_on_off,
                "launch_realsense_splitter_node": launch_realsense_splitter_node,
                "camera_launch_delay": camera_launch_delay,
                "laserscan_launch_delay": laserscan_launch_delay
            }.items()
    )

    vehicle_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'vehicle.launch.py'])
            ),
            condition=IfCondition(launch_vehicle),
            launch_arguments={
                "launch_imu_filter": 'False',  # disabled since not all the vehicle controllers have IMUs.
                "launch_ackermann_to_vesc_node": launch_ackermann_to_vesc_node,
                "launch_vesc_to_odom_node": launch_vesc_to_odom_node,
                "launch_throttle_interpolator_node": launch_throttle_interpolator_node,
                "max_acceleration": max_acceleration,
                "max_steering_rate": max_steering_rate,
                "vesc_poll_rate": vesc_poll_rate,
            }.items()
    )

    tf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'static_transformations.launch.py'])
            ),
            condition=LaunchConfigurationEquals('launch_tfs', 'True')
    )

    localization_launch = TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                PathJoinSubstitution([localization_include_dir, 'localization.launch.py'])
                        ),
                        condition=IfCondition(launch_localization),
                        launch_arguments={
                            "launch_sensor_fusion": 'True',
                            "launch_ekf_odom": launch_local_localization,
                            "launch_ekf_map": launch_global_localization,
                            "launch_slam_toolbox_localizer": 'False',
                            "launch_rtabmap_localizer": 'False',
                            'launch_pointcloud_odometry': 'False',
                            'launch_rgbd_odometry': 'False',
                            'launch_stereo_odometry': 'True',
                            'launch_laserscan_odometry': 'True',
                            'launch_amcl': launch_global_localization,
                            # "map_file": map_file,
                            "use_sim_time": use_sim_time,
                            "use_gpu": use_gpu,
                        }.items()
                )
            ]
    )

    visualization_launch = None

    # determine what namespace to use
    if use_f1tenth_namespace_string.lower() == 'true':
        if not f1tenth_namespace_string.lower().strip():
            # if launch argument is empty, use the VEHICLE_NAME environment variable.
            # Defaults to the username if $VEHICLE_NAME doesn't exist and the namespace argument is empty.
            f1tenth_namespace = EnvironmentVariable(
                    'VEHICLE_NAME',
                    default_value=EnvironmentVariable('USER')
            )

    nodes_to_launch = GroupAction(
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_f1tenth_namespace),
                        namespace=f1tenth_namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                # nodes
                joystick_launch,
                ackermann_mux_launch,
                sensors_launch,
                vehicle_launch,
                tf_launch,
                localization_launch
            ]
    )  # append F1/10 namespace to all nodes

    # return the launch description
    ld = launch_args + [nodes_to_launch]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
