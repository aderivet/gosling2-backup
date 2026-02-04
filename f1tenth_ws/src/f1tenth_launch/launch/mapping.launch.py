"""
Launches (optionally): 2D mapping (online or offline), 3D mapping (online or offline).
    Can also choose GPU/CPU mode.

Mapping notes:
    * RTABMap's (CPU + 3D) 2D map is more accurate and consistent than SLAM toolbox's implementation.
    * For now: using Depth + Color with RTABMap works better than stereo. Todo: test with good localization
    * Both GPU (Isaac Nvblox) and RTABMap work perfectly and they both use Color + Depth.
    * Use RTABMap for future localization support

Todo:
    * Write instructions for saving 2D maps
    * Write instructions for saving RTABMaps
    * Write instructions for saving Nvblox maps
    * Write instructions for saving isaac ros vlsam maps
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction, \
    SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get launch directories
    f1tenth_launch_bringup_dir = os.path.join(f1tenth_launch_dir, 'launch')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    vehicle_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'vehicle')
    sensor_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'sensors')
    localization_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'localization')
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_bringup_dir, 'nvidia_isaac_ros')

    # Setup default directories.
    map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    offline_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")
    default_2d_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'rtabmap.db')

    # Setup launch configuration variables
    # todo: refactor namespaces for multiple F1/10s
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace',
                                            default='')  # used to distinguish between multiple F1/10s
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace', default=False)
    namespace = LaunchConfiguration('namespace', '')  # todo: remove from here and nested launch files
    use_namespace = LaunchConfiguration('use_namespace', default=False)  # todo: remove from here and nested launch files
    map_file = LaunchConfiguration('map_file', default=map_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_gpu = LaunchConfiguration('use_gpu', default=False)
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    launch_joystick = LaunchConfiguration('launch_joystick', default=False)
    launch_sensors = LaunchConfiguration('launch_sensors', default=False)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=False)
    launch_tfs = LaunchConfiguration('launch_tfs', default=False)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    launch_visualization = LaunchConfiguration('launch_visualization', default='False')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    launch_2d_mapping = LaunchConfiguration('launch_2d_mapping', default=False)
    launch_3d_mapping = LaunchConfiguration('launch_3d_mapping', default=True)
    life_long_mapping = LaunchConfiguration('life_long_mapping', default=False)
    offline_mapping_2d_param_file = LaunchConfiguration('offline_mapping_2d_param_file',
                                                        default=offline_mapping_2d_param_file_path)
    online_mapping_2d_param_file = LaunchConfiguration('online_mapping_2d_param_file',
                                                       default=online_mapping_2d_param_file_path)
    map_2d_file = LaunchConfiguration('default_2d_map_file', default=default_2d_map_file_path)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)

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

    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf', default=True)

    # Declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value=autostart,
            description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value=use_composition,
            description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value=use_respawn,
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

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
    rviz_config_arg = DeclareLaunchArgument('rviz_config_file',
                                            default_value=rviz_config_file,
                                            description="The path to the rviz configuration file.")

    launch_2d_mapping_arg = DeclareLaunchArgument('launch_2d_mapping',
                                                  default_value=launch_2d_mapping,
                                                  description="Enable 2D mapping.")
    launch_3d_mapping_arg = DeclareLaunchArgument('launch_3d_mapping',
                                                  default_value=launch_3d_mapping,
                                                  description="Enable 3D mapping.")

    life_long_mapping_arg = DeclareLaunchArgument('life_long_mapping',
                                                  default_value=life_long_mapping,
                                                  description="If set to True, the map will not be deleted but will "
                                                              "instead add to a preexisting map. Setting to false "
                                                              "deletes the old map and starts again.")

    offline_mapping_2d_param_file_la = DeclareLaunchArgument('offline_mapping_2d_param_file',
                                                             default_value=offline_mapping_2d_param_file,
                                                             description="Path to the config file for the "
                                                                         "offline 2D mapping node.")

    online_mapping_2d_param_file_la = DeclareLaunchArgument('online_mapping_2d_param_file',
                                                            default_value=online_mapping_2d_param_file,
                                                            description="Path to the config file for the "
                                                                        "online 2D mapping node.")
    map_2d_file_la = DeclareLaunchArgument('map_2d_file',
                                           default_value=map_2d_file,
                                           description="Path to the save the 2D map.")

    rtabmap_database_file_la = DeclareLaunchArgument('rtabmap_database_file',
                                                     default_value=rtabmap_database_file,
                                                     description="Path to the config file for the 3D mapping node.")

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
            'approx_sync', default_value=approx_sync,
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

    publish_map_to_odom_tf_la = DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value=publish_map_to_odom_tf,
            description='Whether to publish the map to the odometry frame transformation.'
    )

    # Add launch arguments to a list
    launch_args = [
        stdout_linebuf_envvar,
        declare_f1tenth_namespace_cmd,
        declare_use_f1tenth_namespace_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        use_gpu_la,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        launch_joystick_arg,
        launch_sensors_arg,
        launch_vehicle_arg,
        launch_tfs_arg,
        launch_localization_arg,
        launch_local_localization_arg,
        launch_global_localization_arg,
        launch_visualization_arg,
        rviz_config_arg,
        launch_2d_mapping_arg,
        launch_3d_mapping_arg,
        life_long_mapping_arg,
        offline_mapping_2d_param_file_la,
        online_mapping_2d_param_file_la,
        map_2d_file_la,
        rtabmap_database_file_la,
        deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la,
        max_acceleration_la, max_steering_rate_la, vesc_poll_rate_la,
        declare_launch_ackermann_to_vesc_node, declare_launch_vesc_to_odom_node,
        declare_launch_throttle_interpolator_node,
        approx_sync_la, stereo_to_pointcloud_la, depthimage_to_pointcloud_la,
        detect_ground_and_obstacles_la, reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
        realsense_emitter_enabled_la, realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
        camera_launch_delay_la, laserscan_launch_delay_la,
        publish_map_to_odom_tf_la
    ]

    ''' Launch Nodes '''
    # use OpaqueFunction to correctly get the runtime LaunchConfiguration.
    #  E.g ZED ROS2 wrapper |
    #  https://robotics.stackexchange.com/a/103368 |
    #  https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ |
    #  https://robotics.stackexchange.com/a/104402
    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)

    use_sim_time_string = use_sim_time.perform(context)
    life_long_mapping_string = life_long_mapping.perform(context)
    realsense_splitter_enabled_string = launch_realsense_splitter_node.perform(context)

    teleop_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'teleop.launch.py'])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_gpu": use_gpu,
                "launch_joystick": launch_joystick,
                "launch_sensors": launch_sensors,
                "launch_vehicle": launch_vehicle,
                "launch_tfs": launch_tfs,
                "launch_localization": launch_localization,
                "launch_local_localization": launch_local_localization,
                "launch_global_localization": launch_global_localization,
                "launch_visualization": 'False',
                "rviz_config_file": rviz_config_file,
                "deadman_buttons": deadman_buttons,
                "steering_button": steering_button,
                "max_speed": max_speed,
                "max_steering": max_steering,
                "launch_ackermann_to_vesc_node": launch_ackermann_to_vesc_node,
                "launch_vesc_to_odom_node": launch_vesc_to_odom_node,
                "launch_throttle_interpolator_node": launch_throttle_interpolator_node,
                "max_acceleration": max_acceleration,
                "max_steering_rate": max_steering_rate,
                "vesc_poll_rate": vesc_poll_rate,
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

    # todo: setup lifelong mapping
    #  (https://github.com/SteveMacenski/slam_toolbox/blob/humble/launch/lifelong_launch.py).
    #  Use the online config but change the node
    mapping_2d_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '2d_mapping.launch.py'])
            ),
            condition=IfCondition(launch_2d_mapping),
            launch_arguments={
                "offline_mapping": use_sim_time,
                "use_sim_time": use_sim_time,
                "offline_mapping_param_file": offline_mapping_2d_param_file,
                "online_mapping_param_file": online_mapping_2d_param_file,
                "map_file_path": map_2d_file,
                "map_topic": 'map',
                "with_rviz": 'True',  # launch_visualization
                # "rviz_cfg_path_param": rviz_config_path,
            }.items()
    )

    mapping_3d_queue_size = '10000'  # offline mapping
    if use_sim_time_string.lower() == 'false':
        # online_mapping
        mapping_3d_queue_size = '50'  # online mapping

    delete_old_map = '-d '
    if life_long_mapping_string.lower() == 'true':
        delete_old_map = ' '

    left_image_topic = '/camera/camera/infra1/image_rect_raw'
    right_image_topic = '/camera/camera/infra2/image_rect_raw'
    depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'  # /camera/camera/depth/image_rect_raw
    depth_info_topic = '/camera/camera/aligned_depth_to_color/camera_info'  # /camera/camera/depth/camera_info
    if realsense_splitter_enabled_string.lower() == 'true':
        left_image_topic = '/camera/realsense_splitter_node/output/infra_1'
        right_image_topic = '/camera/realsense_splitter_node/output/infra_2'
        # todo: might need to launch RTABMaps depth realignment to color node if using realsense splitter
        depth_topic = '/camera/realsense_splitter_node/output/depth'
        depth_info_topic = '/camera/camera/depth/camera_info'

    # See (https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h#L161)
    mapping_3d_cpu_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '3d_mapping.launch.py'])
            ),
            condition=UnlessCondition(use_gpu),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_stereo": 'False',  # False=use_depth + color, True=use_stereo
                "localization": 'False',
                "queue_size": mapping_3d_queue_size,
                "publish_map_tf": publish_map_to_odom_tf,
                "wait_imu_to_init": 'True',
                "imu_topic": '/camera/camera/imu/filtered',
                "left_image_topic": left_image_topic,
                "right_image_topic": right_image_topic,
                "depth_topic": depth_topic,
                "odom_topic": '/odometry/local',
                "approx_sync": 'True',
                "rtabmap_viz_view": 'False',  # launch_visualization
                "rviz_view": 'False',  # launch_visualization
                "database_path": rtabmap_database_file,
                "rtabmap_args": f'{delete_old_map}'
                                '--RGBD/LoopClosureReextractFeatures true '
                                '--Rtabmap/CreateIntermediateNodes true '
                                # always update the map not only when the robot moves. todo
                                # '--map_always_update False'
                                '--Vis/MinInliers 15 '  # default=20, 15 [tested]
                                '--Vis/EstimationType 0 '  # 0=more accurate, 1=faster
                                '--Vis/MaxDepth 0 '
                                '--RGBD/LinearUpdate 0.001 '
                                '--RGBD/AngularUpdate 0.001 '
                                '--GFTT/QualityLevel 0.00001 '
                                '--Stereo/MinDisparity 0.0 '
                                '--Stereo/MaxDisparity 64.0 '  # default=128.0, 64 [tested]
                                '--Stereo/OpticalFlow false '  # default=false [tested]
                                # '--Vis/RoiRatios 0,0,0,.2 '
                                # "--Kp/RoiRatios 0,0,0,.2 "
                                '--Vis/BundleAdjustment 1 '
                                '--Vis/CorNNDR 0.6 '
                                '--Vis/CorGuessWinSize 20 '
                                '--Vis/PnPFlags 0 '
                                '--Vis/CorType 0 '  # 0=Features Matching, 1=Optical Flow
                                '--Reg/Force3DoF true '
                                '--RGBD/NeighborLinkRefining true '  # when using laserscan
                                '--RGBD/ProximityBySpace true '  # when using laserscan
                                '--Reg/Strategy 2 '  # when using laserscan. 0=Vis, 1=Icp, 2=VisIcp. Tested with 1 and 2.
                                '--Icp/VoxelSize 0.05 '  
                                '--Icp/MaxCorrespondenceDistance 0.1 '
                                # set DetectionRate to 0 to use image rate. Default=1
                                '--Rtabmap/DetectionRate 30 '
                                '--RGBD/CreateOccupancyGrid false '  # tested with false. todo: test
                                # Grid/Sensor: 0=laser scan, 1=depth image(s), 2=both
                                '--Grid/Sensor 0 '  # tested with 0. todo: test
                                '--Grid/RangeMax 12.0 '  # 0=inf
                                # enable ray-tracing to clear out cells and mark as free 
                                # space in occupancy grid map
                                # '--Grid/RayTracing False '  # todo: test
                                ##'--Grid/FromDepth False '  # generate 2D map from depth. Warning: do not use as this wrongly transfers its value as a boolean to Grid/Sensor. Use Grid/Sensor instead
                                # '--Optimizer/Strategy 2 '  # 2=gtsam (might be better for localization)
                                '--Optimizer/Slam2D true '
                                # A 3D occupancy grid is required if you want an OctoMap (3D ray tracing).
                                # Set to false if you want only a 2D map, the cloud will be projected on xy plane.
                                # A 2D map can be still generated if checked, but it requires more memory and time to generate it.
                                # Ignored if laser scan is 2D and Grid/Sensor is 0.
                                # '--Grid/3D true '
                                '--Optimizer/GravitySigma 0',
            }.items()
    )

    mapping_3d_gpu_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nvidia_isaac_launch_dir, 'isaac_ros_nvblox.launch.py'])
            ),
            condition=IfCondition(use_gpu),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "global_frame": 'odom',  # only set to map if another node is publishing the map frame
                "launch_realsense_driver": 'False',
                "launch_realsense_splitter": 'False',
                # "publish_map_tf": 'True',
                "depth_topic": depth_topic,
                "depth_info_topic": depth_info_topic,
                "left_image_topic": left_image_topic,
                "right_image_topic": right_image_topic,
                "input_qos": 'SENSOR_DATA',
                "remove_dynamic_objects": 'False',
                "remove_people": 'False',
                "launch_visual_slam": 'False',
                "attach_to_shared_component_container": 'False',
                "component_container_name": 'nvblox_container',
            }.items()
    )

    mapping_3d_group = GroupAction(
            condition=IfCondition(launch_3d_mapping),
            actions=[
                # set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),

                # nodes
                mapping_3d_cpu_node,
                mapping_3d_gpu_node
            ]
    )

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
                teleop_launch,
                mapping_2d_node,
                mapping_3d_group
            ]
    )  # append F1/10 namespace to all nodes

    ld = launch_args + [
                nodes_to_launch
            ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
    