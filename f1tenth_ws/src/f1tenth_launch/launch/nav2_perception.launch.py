"""

Launches costmap.
Autoware calls this perception. https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/launch/f1tenth_launch/launch/f1tenth_perception.launch.py?ref_type=heads
"""

# Copyright 2020-2022, The Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    costmap_2d_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/costmap_2d.yaml')
    costmap_2d_param = DeclareLaunchArgument(
        'costmap_2d_param_file',
        default_value=costmap_2d_param_file,
        description='Path to config file for nav2_costmap_2d node'
    )

    lifecycle_nodes = ['costmap/costmap']

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                name='nav2_costmap_2d',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[LaunchConfiguration("costmap_2d_param_file")]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(costmap_2d_param)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
