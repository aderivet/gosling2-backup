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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    default_ad_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("default_ad_api"), "launch", "default_ad_api.launch.py"
            ]),
        ),
        condition=IfCondition(LaunchConfiguration("launch_default_ad_api"))
    )

    ad_api_adaptors_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("ad_api_adaptors"), "launch", "rviz_adaptors.launch.xml"
            ]),
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz_adaptors"))
    )

    container = ComposableNodeContainer(
            name='container',
            namespace='autoware_api/external/rtc_controller',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='autoware_iv_external_api_adaptor',
                    plugin='external_api::RTCController',
                    name='external_api_adaptor'
                    )
            ]
    )

    rosbridge_server_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"
            ]),
        ),
        launch_arguments={
            "respawn": LaunchConfiguration("rosbridge_respawn"),
            "max_message_size": LaunchConfiguration("rosbridge_max_message_size"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("rosbridge_enabled"))
    )

    return [
        default_ad_api_launch,
        ad_api_adaptors_launch,
        container,
        rosbridge_server_launch
    ]


def generate_launch_description():
    declared_arguments = []
    
    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg("launch_default_ad_api", "true")
    add_launch_arg("launch_rviz_adaptors", "true")
    add_launch_arg("rosbridge_enabled", "false")
    add_launch_arg("rosbridge_respawn", "true")
    add_launch_arg("rosbridge_max_message_size", "10000000")

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
