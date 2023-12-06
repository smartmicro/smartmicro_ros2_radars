# Copyright 2021 Apex.AI, Inc.
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

import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'umrr_ros2_driver'


def generate_launch_description():
    """Generate the launch description."""
    radar_sensor_params = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'param/radar.sensor.template.yaml')
    
    radar_adapter_params = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'param/radar.adapter.template.yaml')

    rviz_cfg_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config/rviz/radar.rviz')

    radar_node = Node(
        package=PACKAGE_NAME,
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        parameters=[radar_sensor_params, radar_adapter_params]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)]
    )

    return LaunchDescription([
        radar_node,
        rviz2
    ])
