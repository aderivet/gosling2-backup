# Copyright 2023 Amadeusz Szymko
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('f1tenth_autoware_launch_py')
    nav2_pkg_prefix = FindPackageShare('nav2_bringup')

    localization_param_file = PathJoinSubstitution(
        [pkg_prefix, 'config/localization/amcl.param.yaml'])
    
    ekf_param_file = PathJoinSubstitution(
        [pkg_prefix, 'config/localization/ekf_localizer.param.yaml'])

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                nav2_pkg_prefix, 'launch', 'localization_launch.py'
            ]),
        ),
        launch_arguments={
            'params_file': localization_param_file,
            'map': LaunchConfiguration('map_yaml')
        }.items()
    )

    gyro_odometer_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('gyro_odometer'), 'launch', 'gyro_odometer.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_vehicle_twist_with_covariance_topic': '/sensing/vehicle_velocity_converter/twist_with_covariance',
            'input_imu_topic': '/sensing/imu_corrector/imu',
            'output_twist_with_covariance_topic': 'twist_estimator/twist_with_covariance',
            'output_twist_with_covariance_raw_topic': 'twist_estimator/twist_with_covariance_raw',
            'output_twist_topic': 'twist_estimator/twist',
            'output_twist_raw_topic': 'twist_estimator/twist_raw'
        }.items()
    )

    twist2odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('twist2odom'), 'launch', 'twist2odom.launch.py'
            ]),
        ),
        launch_arguments={
            'publish_tf': 'true',
            'in_twist': 'twist_estimator/twist_with_covariance',
            'out_odom': 'odometry',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom'
        }.items()
    )
    
    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('ekf_localizer'), 'launch', 'ekf_localizer.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_initial_pose_name': '/initialpose',
            'input_pose_with_cov_name': '/amcl_pose',
            'input_twist_with_cov_name': 'twist_estimator/twist_with_covariance',
            'output_odom_name': 'pose_twist_fusion_filter/kinematic_state',
            'output_pose_name': 'pose_twist_fusion_filter/pose',
            'output_pose_with_covariance_name': 'pose_twist_fusion_filter/pose_with_covariance',
            'output_biased_pose_name': 'pose_twist_fusion_filter/biased_pose',
            'output_biased_pose_with_covariance_name': 'pose_twist_fusion_filter/biased_pose_with_covariance',
            'output_twist_name': 'pose_twist_fusion_filter/twist',
            'output_twist_with_covariance_name': 'pose_twist_fusion_filter/twist_with_covariance',
            'param_file': ekf_param_file
        }.items()
    )

    stop_filter_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('stop_filter'), 'launch', 'stop_filter.launch.xml'
            ]),
        ),
        launch_arguments={
            'use_twist_with_covariance': 'true',
            'input_odom_name': 'pose_twist_fusion_filter/kinematic_state',
            'input_twist_with_covariance_name': 'pose_twist_fusion_filter/twist_with_covariance',
            'output_odom_name': 'kinematic_state',
        }.items()
    )

    twist2accel_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('twist2accel'), 'launch', 'twist2accel.launch.xml'
            ]),
        ),
        launch_arguments={
            'use_odom': 'true',
            'in_odom': 'kinematic_state',
            'in_twist': 'twist_estimator/twist_with_covariance',
            'out_accel': 'acceleration',
        }.items()
    )
    
    # required by Autoware API, temporary workaround
    init_state = Node(
        package='f1tenth_autoware_launch_py',
        executable='init_state.sh',
        name='init_state',
        namespace='localization',
        arguments=['--ros-args', '--disable-stdout-logs']
    )
    
    localization_trigger = Node(
        package='f1tenth_autoware_launch_py',
        executable='localization_trigger.sh',
        name='localization_trigger',
        namespace='localization',
        arguments=['--ros-args', '--disable-stdout-logs']
    )
    
    delayed_localization_trigger = TimerAction(
        period=5.0,
        actions=[localization_trigger]
    )
        
    group = GroupAction(
        [
            PushRosNamespace('localization'),
            gyro_odometer_launch,
            twist2odom_launch,
            ekf_launch,
            stop_filter_launch,
            twist2accel_launch,
        ]
    )

    return [
        nav2_localization_launch,
        group,
        init_state,
        # delayed_localization_trigger
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('map_yaml', 'imola.yaml')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
