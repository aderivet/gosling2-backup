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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    vehicle_launch_pkg = LaunchConfiguration('vehicle_model').perform(context) + '_launch'
    config_dir = PathJoinSubstitution([FindPackageShare(LaunchConfiguration('sensor_model').perform(context) + '_description'), 'config']).perform(context)
    vehicle_model_file = PathJoinSubstitution([FindPackageShare('tier4_vehicle_launch'), 'urdf', 'vehicle.xacro']).perform(context)
    # amcl_model_file = PathJoinSubstitution([FindPackageShare('f1tenth_sensor_kit_description'), 'urdf', 'amcl.xacro']).perform(context)
    
    vehicle_model = LaunchConfiguration('vehicle_model').perform(context)
    sensor_model = LaunchConfiguration('sensor_model').perform(context)
    
    vehicle_description_node = Node(
        name='robot_state_publisher',
        namespace='',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
            'robot_description': ParameterValue(Command([
                f'xacro {vehicle_model_file} vehicle_model:={vehicle_model} sensor_model:={sensor_model} config_dir:={config_dir}'
                ]), value_type=str)
            }
        ]
    )
    
    # two TF trees are needed - laser link has to be separated from the rest of the vehicle
    # amcl_description_node = Node(
    #     name='amcl_state_publisher',
    #     namespace='',
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[
    #         {
    #         'robot_description': ParameterValue(Command([
    #             f'xacro {amcl_model_file}'
    #             ]), value_type=str)
    #         }
    #     ]
    # )
    
    vehicle_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(vehicle_launch_pkg), 'launch', 'vehicle_interface.launch.py'
            ]),
        ),
        launch_arguments={
            }.items(),
        condition=IfCondition(LaunchConfiguration('launch_vehicle_interface'))
    )

    return [
        vehicle_description_node,
        # amcl_description_node,
        vehicle_interface_launch
    ]


def generate_launch_description():
    declared_arguments = []
    
    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg('vehicle_model', 'true')
    add_launch_arg('sensor_model', 'f1tenth_sensor_kit')
    add_launch_arg('launch_vehicle_interface', 'true')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
