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

class SmartmicroRadarNodeTest : public ::testing::Test {
protected:
    void TearDown() override {
      smartmicro::drivers::radar::check_signal = true;
      rclcpp::Rate sleepRate(std::chrono::seconds(5));
      sleepRate.sleep();
      std::cout << "SHUTDOWN PROCEDURE" << std::endl;
      rclcpp::shutdown();
    }
};

/// @test Test that if we publish one message, it generates a state estimate which is sent out.
TEST_F(SmartmicroRadarNodeTest, Create)
{
  rclcpp::init(0, nullptr);
  ASSERT_TRUE(rclcpp::ok());
  rclcpp::NodeOptions node_options{};
  EXPECT_THROW(std::make_shared<SmartmicroRadarNode>(node_options), std::runtime_error);
  node_options.append_parameter_override("master_client_id", 11);
  node_options.append_parameter_override("hw_dev_id", 2);
  node_options.append_parameter_override("hw_port", 55555);
  node_options.append_parameter_override("hw_iface_name", "eth0");
  node_options.append_parameter_override("sensors.sensor_0.id", 22);
  node_options.append_parameter_override("sensors.sensor_0.ip", "172.22.10.102");
  node_options.append_parameter_override("sensors.sensor_0.model", "umrr96");
  auto node = std::make_shared<SmartmicroRadarNode>(node_options);
  EXPECT_TRUE(node != nullptr);
  smartmicro::drivers::radar::check_signal = true;
  rclcpp::Rate sleepRate(std::chrono::seconds(10));
  sleepRate.sleep();
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
