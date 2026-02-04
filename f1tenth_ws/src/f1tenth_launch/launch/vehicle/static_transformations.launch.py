#!/usr/bin/env python3
"""
Todo: switch to new style

Todo: switch to URDF
Base_link here refers to the rear_axle

                    base_link (rear_axle)
                            |
          __________________|__________________
         |                                    |
    base_footprint(ground)             sensor_kit (aluminum plate)
                                              |

"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    urdf_path = os.path.join(get_package_share_directory('f1tenth_launch'), 'urdf/f1tenth.urdf.xacro')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true')

    base_link_to_sensor_kit_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_sensor_kit',
            arguments=['0.0084836', '0.0', '0.0454914', '0.0', '0.0', '0.0', 'base_link', 'sensor_kit_link']
    )  # rear axle to aluminum plate back edge (centered)

    lidar_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_sensor_kit_to_laser',
            arguments=['0.0757164', '0.0', '0.0886086', '0.0', '0.0', '0.0', 'sensor_kit_link', 'lidar']
    )  # aluminum plate to YDLidar

    # todo: pass base_link to camera_bottom_screw as an argument
    # set use_nominal_extrinsics:=True to use ideal dimensions instead of the calibrated dimensions.
    # Useful for simulations, e.g Gazebo
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='realsense_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', str(urdf_path), ' ',
                                                             'use_nominal_extrinsics:=False']), value_type=str)
            }],
            output='screen'
    )  # aluminum plate to Realsense

    vesc_imu_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_sensor_kit_to_vesc_imu',
            arguments=['0.1326164', '0.0', '-0.01218', '-1.57079633', '0.0', '3.14159265', 'sensor_kit_link', 'imu_link']
    )  # aluminum plate to VESC IMU (/sensors/imu/raw)

    base_footprint_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_sensor_kit_to_basefootprint',
            arguments=['0.0', '0.0', '-0.033', '0.0', '0.0', '0.0', 'base_link', 'base_footprint']
    )  # rear axle to ground

    base_link_to_rear_axle_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_rear_axle',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'rear_axle']  # wheelbase
    )  # base_link == rear_axle

    rear_axle_to_front_axle_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_front_axle',
            arguments=['0.256', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'front_axle']  # wheelbase
    )  # rear axle to front axle

    rear_axle_to_right_rear_wheel_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_right_rear_wheel',
            arguments=['0.0', '-0.1016', '0.0', '0.0', '0.0', '0.0', 'base_link', 'right_rear_wheel']
    )  # rear axle to right rear wheel

    rear_axle_to_left_rear_wheel_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_left_rear_wheel',
            arguments=['0.0', '0.1016', '0.0', '0.0', '0.0', '0.0', 'base_link', 'left_rear_wheel']
    )  # rear axle to left rear wheel

    rear_axle_to_right_front_wheel_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_right_front_wheel',
            arguments=['0.256', '-0.1016', '0.0', '0.0', '0.0', '0.0', 'base_link', 'right_front_wheel']
    )  # rear axle to right front wheel

    rear_axle_to_left_front_wheel_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_left_front_wheel',
            arguments=['0.256', '0.1016', '0.0', '0.0', '0.0', '0.0', 'base_link', 'left_front_wheel']
    )  # rear axle to left front wheel

    static_tf_group = GroupAction(
            actions=[
                SetParameter(name='use_sim_time', value=use_sim_time),
                base_link_to_sensor_kit_tf_node,
                lidar_static_tf_node,
                # camera_static_tf_node,
                robot_state_publisher_node,
                vesc_imu_static_tf_node,
                base_footprint_tf_node,
                base_link_to_rear_axle_tf_node,
                rear_axle_to_front_axle_tf_node,
                rear_axle_to_right_rear_wheel_tf_node,
                rear_axle_to_left_rear_wheel_tf_node,
                rear_axle_to_right_front_wheel_tf_node,
                rear_axle_to_left_front_wheel_tf_node
            ]
    )

    ld = LaunchDescription([
        use_sim_time_la,
        static_tf_group
    ])

    return ld
