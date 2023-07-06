// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <umrr_ros2_driver/smartmicro_radar_node.hpp>

using smartmicro::drivers::radar::SmartmicroRadarNode;

/// @test Test that if we publish one message, it generates a state estimate which is sent out.
TEST(SmartmicroRadarNodeTest, Create)
{
  rclcpp::init(0, nullptr);
  ASSERT_TRUE(rclcpp::ok());
  rclcpp::NodeOptions node_options{};
  EXPECT_THROW(std::make_shared<SmartmicroRadarNode>(node_options), std::runtime_error);
  node_options.append_parameter_override("adapters.adapter_0.hw_type", "eth");
  node_options.append_parameter_override("adapters.adapter_0.hw_dev_id", 2);
  node_options.append_parameter_override("adapters.adapter_0.port", 55555);
  node_options.append_parameter_override("adapters.adapter_0.hw_iface_name", "eth0");
  node_options.append_parameter_override("sensors.sensor_0.id", 10);
  node_options.append_parameter_override("sensors.sensor_0.ip", "172.22.10.102");
  node_options.append_parameter_override("sensors.sensor_0.model", "umrr96");
  node_options.append_parameter_override("sensors.sensor_0.inst_type", "port_based");
  node_options.append_parameter_override("sensors.sensor_0.data_type", "port_based");
  node_options.append_parameter_override("sensors.sensor_0.uifname", "umrr96_t153_automotive");
  node_options.append_parameter_override("sensors.sensor_0.uifmajorv", 1);
  node_options.append_parameter_override("sensors.sensor_0.uifminorv", 2);
  node_options.append_parameter_override("sensors.sensor_0.uifpatchv", 2);
  auto node = std::make_shared<SmartmicroRadarNode>(node_options);
  ASSERT_TRUE(node != nullptr);
  rclcpp::Rate sleepRate(std::chrono::seconds(5));
  sleepRate.sleep();
  rclcpp::shutdown();
}
