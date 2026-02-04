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

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    vehicle_info_param_path = LaunchConfiguration('vehicle_param_file').perform(context)

    with open(vehicle_info_param_path, 'r') as f:
        vehicle_info_param = yaml.safe_load(f)['/**']['ros__parameters']

    with open(LaunchConfiguration('freespace_planner_param_path').perform(context), 'r') as f:
        freespace_planner_param = yaml.safe_load(f)['/**']['ros__parameters']
        
    with open(LaunchConfiguration('trajectory_loader_param_path').perform(context), 'r') as f:
        trajectory_loader_param = yaml.safe_load(f)['/**']['ros__parameters']

    mission_planner_launch = GroupAction([
        PushRosNamespace('planning/mission_planning'),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                launch_file_path=PathJoinSubstitution([
                    FindPackageShare('mission_planner'), 'launch', 'mission_planner.launch.xml'
                ])
            )
        )])

    goal_pose_visualizer_launch = GroupAction([
        PushRosNamespace('planning/mission_planning'),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                launch_file_path=PathJoinSubstitution([
                    FindPackageShare('mission_planner'), 'launch', 'goal_pose_visualizer.launch.xml'
                ])
            )
        )])

    container = ComposableNodeContainer(
        name='planning_container',
        namespace='planning',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=[]
    )

    freespace_planner = ComposableNode(
        package='freespace_planner',
        plugin='freespace_planner::FreespacePlannerNode',
        name='racing_planner',
        namespace='planning',
        remappings=[
            ('~/input/route', 'mission_planning/route'),
            ('~/input/occupancy_grid', '/map'),
            ('~/input/scenario', 'scenario_planning/scenario'),
            ('~/input/odometry', '/localization/kinematic_state'),
            ('~/output/trajectory', 'racing_planner/trajectory'),
            ('is_completed', 'racing_planner/is_completed'),
        ],
        parameters=[
            freespace_planner_param,
            vehicle_info_param,
        ],
        extra_arguments=[
            {'use_intra_process_comms': LaunchConfiguration('use_intra_process')}
        ],
    )

    trajectory_loader = ComposableNode(
        package='trajectory_loader',
        plugin='trajectory_loader::TrajectoryLoaderNode',
        name='racing_planner',
        namespace='planning',
        remappings=[
            ('~/input/odometry', '/localization/kinematic_state'),
            ('~/output/trajectory', 'racing_planner/trajectory'),
        ],
        parameters=[
            trajectory_loader_param,
            {
                'csv_path': LaunchConfiguration('csv_path')
            },
        ],
        extra_arguments=[
            {'use_intra_process_comms': LaunchConfiguration('use_intra_process')}
        ],
    )

    freespace_planner_load = LoadComposableNodes(
        condition=UnlessCondition(LaunchConfiguration('use_trajectory_loader')),
        composable_node_descriptions=[freespace_planner],
        target_container='/planning/planning_container'
    )

    trajectory_loader_load = LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration('use_trajectory_loader')),
        composable_node_descriptions=[trajectory_loader],
        target_container='/planning/planning_container'
    )

    # required by freespace planner, temporary workaround
    scenario = Node(
        package='f1tenth_autoware_launch_py',
        executable='scenario.sh',
        name='scenario',
        namespace='planning',
        arguments=['--ros-args', '--disable-stdout-logs']
    )

    return [
        mission_planner_launch,
        goal_pose_visualizer_launch,
        container,
        freespace_planner_load,
        trajectory_loader_load,
        scenario
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('vehicle_param_file')
    add_launch_arg('freespace_planner_param_path')
    add_launch_arg('trajectory_loader_param_path')
    add_launch_arg('csv_path')
    add_launch_arg('use_trajectory_loader', 'false')
    add_launch_arg('use_intra_process', 'false')
    add_launch_arg('use_multithread', 'false')

    set_container_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_multithread')),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_multithread')),
    )

    return LaunchDescription([
        *declared_arguments,
        set_container_executable,
        set_container_mt_executable,
        OpaqueFunction(function=launch_setup)
    ])
