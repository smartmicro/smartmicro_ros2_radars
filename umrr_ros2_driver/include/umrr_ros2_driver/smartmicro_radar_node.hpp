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


#include "umrr_ros2_msgs/srv/set_param.hpp"
#include "umrr_ros2_msgs/srv/set_ip.hpp"
#include "umrr_ros2_msgs/msg/port_header.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <umrr11_t132_automotive_v1_1_1/comtargetlistport/ComTargetListPort.h>
#include <umrr96_t153_automotive_v1_2_1/comtargetlistport/ComTargetListPort.h>
#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>

#include <array>
#include <memory>
#include <string>

namespace smartmicro
{
namespace drivers
{
namespace radar
{

namespace detail
{

constexpr auto kMaxSensorCount = 10UL;

struct SensorConfig
{
  std::uint32_t id{};
  std::string ip{};
  std::uint32_t port{};
  std::string iface_name{};
  std::string frame_id{};
  std::uint32_t history_size{};
};
}  // namespace detail

///
/// @brief      The class for the Smartmicro radar node.
///
class UMRR_ROS2_DRIVER_PUBLIC SmartmicroRadarNode : public ::rclcpp::Node
{
public:
  ///
  /// ROS 2 parameter constructor.
  ///
  /// @param[in]  node_options  Node options for this node.
  ///
  explicit SmartmicroRadarNode(const rclcpp::NodeOptions & node_options);

private:
  ///
  /// @brief      A callback that is called when a new target list port for umrr11 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  target_list_port  The target list port
  ///
  void targetlist_callback_umrr11(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_1::comtargetlistport::ComTargetListPort> & target_list_port);

  ///
  /// @brief      A callback that is called when a new target list port for umrr96 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  target_list_port  The target list port
  ///
  void targetlist_callback_umrr96(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_1::comtargetlistport::ComTargetListPort> & target_list_port);

  ///
  /// @brief      Read parameters and update the json config files required by Smart Access C++ API.
  ///
  void update_config_files_from_params();

   ///
  /// @brief      Callaback for getting the instruction response.
  ///
  void sensor_response(const ClientId,const std::shared_ptr<com::master::ResponseBatch> & response);
  
  ///
  /// @brief      Callaback for changing IP address.
  ///
  void sensor_response_ip(const ClientId,const std::shared_ptr<com::master::ResponseBatch> & response_ip_change);
  
  ///
  /// @brief      Send instructions to the sensor.
  ///
  void change_radar_mode(const std::shared_ptr<umrr_ros2_msgs::srv::SetParam::Request> request,
                      std::shared_ptr<umrr_ros2_msgs::srv::SetParam::Response> response);

  ///
  /// @brief      Configure the sensor.
  ///
  void change_ip_address(const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request_ip,
                     std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> response_ip);

  ///
  /// @brief      Configure the sensor.
  ///
  void my_timer_callback()
  {
    my_timer->cancel();
    RCLCPP_INFO(this->get_logger(), "Timer has been cancelled!");
  }
  
  rclcpp::Service<umrr_ros2_msgs::srv::SetParam>::SharedPtr command_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SetIp>::SharedPtr ip_addr_srv_;
  std::shared_ptr<com::master::CommunicationServicesIface> m_services{};
  std::array<detail::SensorConfig, detail::kMaxSensorCount> m_sensors{};
  std::array<
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,
    detail::kMaxSensorCount> m_publishers{};
  std::size_t m_number_of_sensors{};
  rclcpp::TimerBase::SharedPtr my_timer;
  ClientId client_id;
};

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

#endif  // UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
