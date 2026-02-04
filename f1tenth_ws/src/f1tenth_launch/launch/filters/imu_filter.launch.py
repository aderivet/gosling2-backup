"""
Todo: setup imu corrector node name
Todo: add bias estimation using AMCL and Odom. Any Odom and PoseWithCovarianceStamped message will suffice, e.g. NDT, VSLAM (https://github.com/autowarefoundation/autoware.universe/blob/main/sensing/imu_corrector/launch/gyro_bias_estimator.launch.xml | https://github.com/autowarefoundation/autoware.universe/tree/main/sensing/imu_corrector)
"""

import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    imu_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/filters/imu_filter.yaml")
    imu_corrector_param_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, "config/filters/imu_corrector.yaml")

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_corrector_params_file = LaunchConfiguration('imu_corrector_params_file')

    # The parent frame to be used in publish_tf of the Madgwick or Complementary filter.
    # Should be set to the frame_id of the raw imu message (e.g imu_link) or base_link
    imu_frame = LaunchConfiguration('imu_frame')
    imu_frame_la = DeclareLaunchArgument(
            'imu_frame',
            default_value='',
            description='Frame ID for the IMU message of the Madgwick or Complementary filter.')

    imu_corrector_frame = LaunchConfiguration('imu_corrector_frame')
    imu_corrector_frame_la = DeclareLaunchArgument(
            'imu_corrector_frame',
            default_value='camera_imu_optical_frame',
            description='Frame ID for the IMU message of the IMU corrector/bias remover. E.g base_link, sensor_kit_link')

    # Whether to publish a TF transform that represents the orientation of the IMU,
    # using the frame specified in fixed_frame as the parent frame and the frame
    # given in the input imu message as the child frame. Should be used for
    # debugging and visualization purposes only.
    publish_tf = False

    # If set to true, publish transforms from imu_frame to fixed frame instead of the other way around.
    # Should be used for debugging and visualization purposes only.
    reverse_tf = False

    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    imu_corrector_output_topic = LaunchConfiguration('imu_corrector_output_topic')
    remove_gravity_vector = LaunchConfiguration('remove_gravity_vector')
    imu_gyro_stddev = LaunchConfiguration('imu_gyro_stddev')
    imu_accel_stddev = LaunchConfiguration('imu_accel_stddev')
    imu_orientation_stddev = LaunchConfiguration('imu_orientation_stddev')
    node_name = LaunchConfiguration('node_name')
    imu_corrector_node_name = LaunchConfiguration('imu_corrector_node_name')
    use_madgwick_filter = LaunchConfiguration('use_madgwick_filter')
    remove_imu_bias = LaunchConfiguration('remove_imu_bias')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')

    declare_imu_corrector_params_file_cmd = DeclareLaunchArgument(
            'imu_corrector_params_file',
            default_value=imu_corrector_param_file_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    input_topic_la = DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='Raw IMU message')

    imu_corrector_output_topic_la = DeclareLaunchArgument(
            'imu_corrector_output_topic',
            default_value='/imu/bias_removed',
            description='Output topic for the IMU corrector node')

    output_topic_la = DeclareLaunchArgument(
            'output_topic',
            default_value='',
            description='Raw IMU message')

    remove_gravity_vector_la = DeclareLaunchArgument(
            'remove_gravity_vector',
            default_value='False',
            description='Whether or not to remove the gravity vector. Only valid for the Madgwick Filter')

    imu_gyro_stddev_la = DeclareLaunchArgument(
            'imu_gyro_stddev',
            default_value='0.1',
            description='Standard deviation of the gyroscope noise. This will be squared to get the (co)variance.')

    imu_accel_stddev_la = DeclareLaunchArgument(
            'imu_accel_stddev',
            default_value='0.1',
            description='Standard deviation of the accelerometer noise. This will be squared to get the (co)variance.')

    imu_orientation_stddev_la = DeclareLaunchArgument(
            'imu_orientation_stddev',
            default_value='0.1',
            description='Standard deviation of the orientation noise. This will be squared to get the (co)variance.')

    node_name_la = DeclareLaunchArgument(
            'node_name',
            default_value='imu_filter',
            description='Whether or not to remove the gravity vector.')
    imu_corrector_node_name_la = DeclareLaunchArgument(
            'imu_corrector_node_name',
            default_value='imu_bias_removal_node',
            description='Name for the IMU bias corrector/removal node.')
    use_madgwick_filter_la = DeclareLaunchArgument(
            'use_madgwick_filter',
            default_value='True',
            description='Whether or not to use the Madgwick Filter. Uses the complementary filter if False.')
    remove_imu_bias_la = DeclareLaunchArgument(
            'remove_imu_bias',
            default_value='True',
            description='Whether or not to remove constant additive noise from the measurement.')

    ld = LaunchDescription([declare_namespace_cmd, declare_use_namespace_cmd, declare_use_sim_time_cmd,
                            declare_imu_corrector_params_file_cmd,
                            imu_frame_la, imu_corrector_frame_la, input_topic_la, output_topic_la,
                            imu_corrector_output_topic_la, imu_corrector_node_name_la,
                            remove_gravity_vector_la,
                            imu_gyro_stddev_la, imu_accel_stddev_la, imu_orientation_stddev_la,
                            node_name_la, use_madgwick_filter_la, remove_imu_bias_la])

    imu_filter_with_correction_node = GroupAction(
            condition=IfCondition(remove_imu_bias),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='base_link', value=imu_corrector_frame),
                SetParameter(name='angular_velocity_stddev_xx', value=imu_gyro_stddev),
                SetParameter(name='angular_velocity_stddev_yy', value=imu_gyro_stddev),
                SetParameter(name='angular_velocity_stddev_zz', value=imu_gyro_stddev),
                SetParameter(name='acceleration_stddev', value=imu_accel_stddev),
                SetParameter(name='orientation_stddev', value=imu_orientation_stddev),

                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst=imu_corrector_output_topic),
                SetRemap(src='imu/data', dst=output_topic),

                # IncludeLaunchDescription(
                #         XMLLaunchDescriptionSource(  # or FrontendLaunchDescriptionSource
                #                 launch_file_path=PathJoinSubstitution([
                #                     FindPackageShare("imu_corrector"), "launch", "imu_corrector.launch.xml",
                #                 ]),
                #         ),
                #         launch_arguments={
                #             "input_topic": input_topic,
                #             "output_topic": imu_corrector_output_topic,
                #             "param_file": imu_corrector_params_file,
                #         }.items()
                # ),

                Node(
                        package='imu_corrector',
                        executable='imu_corrector_node',
                        name=imu_corrector_node_name,  # f"{node_name_string}_corrector_node",
                        output={'both': 'log'},
                        parameters=[imu_corrector_params_file],
                        remappings=[
                            ('input', input_topic),  # input topic: /vehicle/sensors/imu/raw
                            ('output', imu_corrector_output_topic),  # output topic: /vehicle/sensors/imu/data
                        ]
                ),

                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                )
            ]
    )

    imu_filter_without_correction_node = GroupAction(
            condition=UnlessCondition(remove_imu_bias),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='orientation_stddev', value=imu_orientation_stddev),

                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst=input_topic),
                SetRemap(src='imu/data', dst=output_topic),
                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                )
            ]
    )

    ld.add_action(imu_filter_with_correction_node)
    ld.add_action(imu_filter_without_correction_node)

    return ld
