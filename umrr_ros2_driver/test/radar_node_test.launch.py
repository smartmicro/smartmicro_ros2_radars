# Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany
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
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import sensor_msgs.msg as sensor_msgs
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = 'umrr_ros2_driver'


@pytest.mark.launch_test
def generate_test_description():

    radar_node = Node(
        package=PACKAGE_NAME,
        executable='smartmicro_radar_node_exe',
        name='smartmicro_radar_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(PACKAGE_NAME),
                         'param/radar.template.yaml'),
        ],
    )

    return (
        launch.LaunchDescription([
            radar_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'radar_node': radar_node,
        }
    )


class TestSmartNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        print("Initialize")
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        print("setUp")
        self.test_node = rclpy.create_node('test_node')

    def tearDown(self):
        print("Destroy node")
        self.test_node.destroy_node()

    def test_smart_node_publishes(self):
        print("Inisde the main node test ......")
        # Expect the smartnode to publish strings on '/umrr/targets_'
        data_rx_s1 = []
        data_rx_s2 = []

        def data_rx_s1_callback(msg):
            data_rx_s1.append(msg)

        def data_rx_s2_callback(msg):
            data_rx_s2.append(msg)

        sub_s1 = self.test_node.create_subscription(
            sensor_msgs.PointCloud2,
            '/umrr/targets_0',
            data_rx_s1_callback,
            10
        )
        sub_s2 = self.test_node.create_subscription(
            sensor_msgs.PointCloud2,
            '/umrr/targets_1',
            data_rx_s2_callback,
            10
        )
        try:
            # Wait until the publisher publishes
            end_time = time.time() + 20

            while time.time() < end_time:
                rclpy.spin_once(self.test_node, timeout_sec=0.1)
                if len(data_rx_s1) > 1 and len(data_rx_s2) > 1:
                    print(f"Data received at {time.time()}")

            self.assertGreater(len(data_rx_s1), 1)
            self.assertGreater(len(data_rx_s2), 1)

        finally:
            self.test_node.destroy_subscription(sub_s1)
            self.test_node.destroy_subscription(sub_s2)
