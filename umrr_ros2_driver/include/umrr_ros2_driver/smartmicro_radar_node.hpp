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
// The initial version of the code was developed by Apex.AI and
// was thereafter adapted and extended by smartmicro.

#ifndef UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
#define UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_

#include "umrr_ros2_msgs/srv/set_ip.hpp"
#include "umrr_ros2_msgs/srv/set_mode.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>
#include <umrr11_t132_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr11_t132_automotive_v1_1_1/comtargetlistport/ComTargetListPort.h>
#include <umrr96_t153_automotive_v1_2_1/DataStreamServiceIface.h>
#include <umrr96_t153_automotive_v1_2_1/comtargetlistport/ComTargetListPort.h>
#include <umrr9f_t169_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/ComTargetListPort.h>

#include <array>
#include <memory>
#include <string>

namespace smartmicro {
namespace drivers {
namespace radar {

namespace detail {

constexpr auto kMaxSensorCount = 8UL;

struct SensorConfig {
  std::uint32_t id{};
  std::string ip{};
  std::uint32_t port{};
  std::string frame_id{};
  std::uint32_t history_size{};
  std::string model{};
};
} // namespace detail

///
/// @brief      The class for the Smartmicro radar node.
///
class UMRR_ROS2_DRIVER_PUBLIC SmartmicroRadarNode : public ::rclcpp::Node {
public:
  ///
  /// ROS 2 parameter constructor.
  ///
  /// @param[in]  node_options  Node options for this node.
  ///
  explicit SmartmicroRadarNode(const rclcpp::NodeOptions &node_options);

protected:
  ///
  /// @brief      A timer to handle the initialization.
  ///
  void my_timer_callback() { timer->cancel(); }

  ///
  /// @brief      Callback triggered on shutdown call.
  ///
  void shutdown_call();

private:
  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr11 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  target_list_port  The target list port
  ///
  void targetlist_callback_umrr11(
      const std::uint32_t sensor_idx,
      const std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_1::
                                comtargetlistport::ComTargetListPort>
          &targetlist_port_umrr11);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr96 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  target_list_port  The target list port
  ///
  void targetlist_callback_umrr96(
      const std::uint32_t sensor_idx,
      const std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_1::
                                comtargetlistport::ComTargetListPort>
          &targetlist_port_umrr96);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  target_list_port  The target list port
  ///
  void targetlist_callback_umrr9f(
      const std::uint32_t sensor_idx,
      const std::shared_ptr<com::master::umrr9f_t169_automotive_v1_1_1::
                                comtargetlistport::ComTargetListPort>
          &targetlist_port_umrr9f);

  ///
  /// @brief      Read parameters and update the json config files required by
  /// Smart Access C++ API.
  ///
  void update_config_files_from_params();

  ///
  /// @brief      Callaback for getting the instruction response.
  ///
  void
  sensor_response(const com::types::ClientId client_id,
                  const std::shared_ptr<com::master::ResponseBatch> &response,
                  const std::string instruction_name);

  ///
  /// @brief      Callback for changing IP address.
  ///
  void sensor_response_ip(
      const com::types::ClientId client_id,
      const std::shared_ptr<com::master::ResponseBatch> &response);

  ///
  /// @brief      Send instructions to the sensor.
  ///
  void radar_mode(
      const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
      std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> response);

  ///
  /// @brief      Configure the sensor ip address.
  ///
  void
  ip_address(const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request,
             std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> response);

  rclcpp::Service<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SetIp>::SharedPtr ip_addr_srv_;
  std::shared_ptr<com::master::CommunicationServicesIface> m_services{};
  std::array<detail::SensorConfig, detail::kMaxSensorCount> m_sensors{};
  std::shared_ptr<
      com::master::umrr11_t132_automotive_v1_1_1::DataStreamServiceIface>
      data_umrr11{};
  std::shared_ptr<
      com::master::umrr96_t153_automotive_v1_2_1::DataStreamServiceIface>
      data_umrr96{};
  std::shared_ptr<
      com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface>
      data_umrr9f{};
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,
             detail::kMaxSensorCount>
      m_publishers{};
  std::size_t m_number_of_sensors{};
  rclcpp::TimerBase::SharedPtr timer;
  com::types::ClientId client_id;
  std::uint64_t response_type{};
};

} // namespace radar
} // namespace drivers
} // namespace smartmicro

#endif // UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
