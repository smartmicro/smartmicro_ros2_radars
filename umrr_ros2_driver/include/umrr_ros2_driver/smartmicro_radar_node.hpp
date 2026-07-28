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

#include <CommunicationServicesIface.h>
#include <InstructionServiceIface.h>
#include <umrr11_t132_automotive_v1_1_2/DataStreamServiceIface.h>
#include <umrr96_t153_automotive_v1_2_2/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_0_3/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_2_2/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_4_1/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_5_0/DataStreamServiceIface.h>
#include <umrr9d_t152_automotive_v1_7_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_2_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v2_4_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v3_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v3_2_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_0_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_1_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v1_3_0/DataStreamServiceIface.h>
#include <umrr9f_t169_mse_v2_0_0/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_0_1/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_2_1/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_4_0/DataStreamServiceIface.h>
#include <umrra4_automotive_v1_6_0/DataStreamServiceIface.h>
#include <umrra4_mse_v1_0_0/DataStreamServiceIface.h>
#include <umrra4_mse_v2_1_0/DataStreamServiceIface.h>
#include <umrra4_mse_v3_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v1_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v2_0_0/DataStreamServiceIface.h>
#include <umrra1_t166_b_automotive_v2_0_1/DataStreamServiceIface.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <umrr_ros2_msgs/msg/can_object_header.hpp>
#include <umrr_ros2_msgs/msg/can_target_header.hpp>
#include <umrr_ros2_msgs/msg/port_fault_report.hpp>
#include <umrr_ros2_msgs/msg/port_fault_report_header.hpp>
#include <umrr_ros2_msgs/msg/port_fault_reports_msg.hpp>
#include <umrr_ros2_msgs/msg/port_object_header.hpp>
#include <umrr_ros2_msgs/msg/port_target_header.hpp>
#include <umrr_ros2_msgs/srv/firmware_download.hpp>
#include <umrr_ros2_msgs/srv/get_mode.hpp>
#include <umrr_ros2_msgs/srv/get_status.hpp>
#include <umrr_ros2_msgs/srv/send_command.hpp>
#include <umrr_ros2_msgs/srv/set_ip.hpp>
#include <umrr_ros2_msgs/srv/set_mode.hpp>

#include <umrr_ros2_driver/update_service.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace smartmicro
{
namespace drivers
{
namespace radar
{
namespace detail
{
constexpr auto kMaxSensorCount = 10UL;
constexpr auto kMaxHwCount = 6UL;

struct SensorConfig
{
  std::uint32_t id{};
  std::uint32_t dev_id{};
  std::string frame_id{};
  std::uint32_t history_size{};
  std::string ip{};
  std::string link_type{};
  std::string model{};
  std::uint32_t port{};
  std::string inst_type{};
  std::string data_type{};
  std::string uifname{};
  std::uint32_t uifmajorv{};
  std::uint32_t uifminorv{};
  std::uint32_t uifpatchv{};
  std::string pub_type{};
};

struct HWConfig
{
  std::uint32_t hw_dev_id{};
  std::string hw_iface_name{};
  std::string hw_type{};
  std::uint32_t baudrate{};
  std::uint32_t port{};
};
}  // namespace detail

///
/// @brief      The class for the Smartmicro radar node.
///
class UMRR_ROS2_DRIVER_PUBLIC SmartmicroRadarNode : public ::rclcpp::Node
{
public:
  ///
  /// @brief      Constructs the Smartmicro radar node.
  ///
  /// @param[in]  node_options  Node options for this node.
  ///
  explicit SmartmicroRadarNode(const rclcpp::NodeOptions & node_options);

protected:
  ///
  /// @brief      Requests node shutdown.
  ///
  void on_shutdown_callback();

private:
  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrra4_v1_0_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrra4_mse_v1_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrra4_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v1_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrra4_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v1_0_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrra4_mse_v1_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrra4_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrra4_v2_1_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrra4_mse_v2_1_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrra4_mse_v2_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v2_1_0::comobjectlist::ComObjectList> &
    objectlist_port_umrra4_mse_v2_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v2_1_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrra4_mse_v2_1_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrra4_mse_v2_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v2_1_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_mse_v2_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrra4_v3_0_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrra4_mse_v3_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrra4_mse_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v3_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrra4_mse_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v3_0_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrra4_mse_v3_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrra4_mse_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v3_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_mse_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new fault report for
  /// umrra4_v3_0_0 T169 MSE arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  fault_report_umrra4_mse_v3_0_0  The fault list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void faultreport_callback_umrra4_mse_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v3_0_0::faultreports::FaultReports> &
    fault_report_umrra4_mse_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrr9f_v1_0_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrr9f_mse_v1_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrr9f_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v1_0_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_mse_v1_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrr9f_v1_1_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrr9f_mse_v1_1_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrr9f_mse_v1_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v1_1_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_mse_v1_1_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_mse_v1_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrr9f_v1_3_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrr9f_mse_v1_3_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrr9f_mse_v1_3_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_3_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v1_3_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_mse_v1_3_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_mse_v1_3_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_3_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new object list port for
  /// umrr9f_v2_0_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_port_umrr9f_mse_v2_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void objectlist_callback_umrr9f_mse_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v2_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v2_0_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_mse_v2_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_mse_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v2_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new fault report for
  /// umrr9f_v2_0_0 T169 MSE arrives.
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  fault_report_umrr9f_mse_v2_0_0  The fault list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void faultreport_callback_umrr9f_mse_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v2_0_0::faultreports::FaultReports> &
    fault_report_umrr9f_mse_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr11 T132 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr11  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///

  void targetlist_callback_umrr11(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr11,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr96 T153 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr96  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr96(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr96,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v1_1_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v1_1_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v1_1_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::ComTargetListPort> &
    targetlist_port_umrr9f_v1_1_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v2_0_0 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v2_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::ComTargetListPort> &
    targetlist_port_umrr9f_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v2_1_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v2_1_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v2_1_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_1_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v2_2_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v2_2_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v2_2_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_2_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v2_4_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v2_4_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v2_4_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_4_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v3_0_0 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v3_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9f_v3_2_0 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9f_v3_2_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9f_v3_2_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v3_2_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v3_2_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new fault report for
  /// umrr9f_v3_2_0 T169 arrives.
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  fault_report_umrr9f_v3_2_0  The fault report port.
  /// @param[in]  client_id The client_id of the sensor
  ///
  void faultreport_callback_umrr9f_v3_2_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v3_2_0::faultreports::FaultReports> &
    fault_report_umrr9f_v3_2_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9d_v1_0_3 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9d_v1_0_3  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9d_v1_0_3(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_0_3,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9d_v1_2_2 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_port_umrr9d_v1_2_2  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void targetlist_callback_umrr9d_v1_2_2(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_2_2,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9d_v1_4_1 T152 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrr9d_v1_4_1  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrr9d_v1_4_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_4_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9d_v1_5_0 T152 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrr9d_v1_5_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrr9d_v1_5_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_5_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrr9d_v1_7_0 T152 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrr9d_v1_7_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrr9d_v1_7_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_7_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_7_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new fault report for
  /// umrr9d_v1_7_0 T152 arrives.
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  fault_report_umrr9d_v1_5_0  The fault report port.
  /// @param[in]  client_id The client_id of the sensor
  ///
  void faultreport_callback_umrr9d_v1_7_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_7_0::faultreports::FaultReports> &
    fault_report_umrr9d_v1_5_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v1_0_1 T171 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra4_v1_0_1  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra4_v1_0_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_0_1::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_0_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v1_2_1 T171 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra4_v1_2_1  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra4_v1_2_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_2_1::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_2_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v1_4_0 T171 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra4_v1_4_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra4_v1_4_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_4_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_4_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra4_v1_6_0 T171 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra4_v1_6_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra4_v1_6_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_6_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_6_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new fault report for
  /// umrra4_v1_6_0 T171 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  fault_report_umrra4_v1_6_0  The fault list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void faultreport_callback_umrra4_v1_6_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_6_0::faultreports::FaultReports> &
    fault_report_umrra4_v1_6_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra1_v1_0_0 T166 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra1_v1_0_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra1_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra1_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra1_v2_0_0 T166 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra1_v2_0_0  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra1_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra1_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new target list port for
  /// umrra1_v2_0_1 T166 arrives.
  /// @param[in]  sensor_idx  The sensor id for the respected published topic.
  /// @param[in]  targetlist_port_umrra1_v2_0_1  The target list port
  /// @param[in]  client_id The client_id of the sensor
  ///
  void targetlist_callback_umrra1_v2_0_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra1_t166_b_automotive_v2_0_1::comtargetlist::ComTargetList> &
    targetlist_port_umrra1_v2_0_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN object list for
  /// umrra4_v1_0_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_can_umrra4_mse_v1_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  ///
  void CAN_objectlist_callback_umrra4_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrra4_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief    A callback that is called when a new CAN target list for
  /// umrra4_mse_v1_0_0 T171 MSE arrives.
  /// @param[in] sensor_idx The sensor id for respective published topic.
  /// @param[in] targetlist_can_umrra4_mse_v1_0_0 The target list port
  /// @param[in] client_id  The client_id of the sensor.
  ///
  ///
  void CAN_targetlist_callback_umrra4_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN object list for
  /// umrra4_v2_1_0 T171 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_can_umrra4_mse_v2_1_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  ///
  void CAN_objectlist_callback_umrra4_mse_v2_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v2_1_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrra4_mse_v2_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief    A callback that is called when a new CAN target list for
  /// umrra4_mse_v2_1_0 T171 MSE arrives.
  /// @param[in] sensor_idx The sensor id for respective published topic.
  /// @param[in] targetlist_can_umrra4_mse_v2_1_0 The target list port
  /// @param[in] client_id  The client_id of the sensor.
  ///
  ///
  void CAN_targetlist_callback_umrra4_mse_v2_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v2_1_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_mse_v2_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN object list for
  /// umrr9f_v1_0_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_can_umrr9f_mse_v1_0_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  ///
  void CAN_objectlist_callback_umrr9f_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief    A callback that is called when a new CAN target list for
  /// umrr9f_mse_v1_0_0 T169 MSE arrives.
  ///
  /// @param[in] sensor_idx The sensor id for respective published topic.
  /// @param[in] targetlist_can_umrr9f_mse_v1_0_0 The target list port
  /// @param[in] client_id  The client_id of the sensor.
  ///
  ///
  void CAN_targetlist_callback_umrr9f_mse_v1_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN object list for
  /// umrr9f_v1_1_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_can_umrr9f_mse_v1_1_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  ///
  void CAN_objectlist_callback_umrr9f_mse_v1_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief    A callback that is called when a new CAN target list for
  /// umrr9f_mse_v1_1_0 T169 MSE arrives.
  ///
  /// @param[in] sensor_idx The sensor id for respective published topic.
  /// @param[in] targetlist_can_umrr9f_mse_v1_1_0 The target list port
  /// @param[in] client_id  The client_id of the sensor.
  ///
  ///
  void CAN_targetlist_callback_umrr9f_mse_v1_1_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_1_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN object list for
  /// umrr9f_v1_3_0 T169 MSE arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  objectlist_can_umrr9f_mse_v1_3_0  The object list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  ///
  void CAN_objectlist_callback_umrr9f_mse_v1_3_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_3_0,
    const com::types::ClientId client_id);

  ///
  /// @brief    A callback that is called when a new CAN target list for
  /// umrr9f_mse_v1_3_0 T169 MSE arrives.
  /// @param[in] sensor_idx The sensor id for respective published topic.
  /// @param[in] targetlist_can_umrr9f_mse_v1_3_0 The target list port
  /// @param[in] client_id  The client_id of the sensor.
  ///
  ///
  void CAN_targetlist_callback_umrr9f_mse_v1_3_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_3_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN object list from an UMRRA4 MSE v3.0.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  objectlist_can_umrra4_mse_v3_0_0  The received CAN object list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_objectlist_callback_umrra4_mse_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v3_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrra4_mse_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN target list from an UMRRA4 MSE v3.0.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  targetlist_can_umrra4_mse_v3_0_0  The received CAN target list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_targetlist_callback_umrra4_mse_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_mse_v3_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_mse_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN object list from an UMRR9F MSE v2.0.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  objectlist_can_umrr9f_mse_v2_0_0  The received CAN object list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_objectlist_callback_umrr9f_mse_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v2_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN target list from an UMRR9F MSE v2.0.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  targetlist_can_umrr9f_mse_v2_0_0  The received CAN target list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_targetlist_callback_umrr9f_mse_v2_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_mse_v2_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v2_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr96_v1_2_2 T153 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr96  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr96(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr96,
    const com::types::ClientId client_id);

  /// @brief      A callback that is called when a new CAN target list for
  /// umrr11_v1_1_2 T132 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr11  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr11(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr11,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9f_v2_1_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9f_v2_1_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9f_v2_1_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_1_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9f_v2_2_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9f_v2_2_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9f_v2_2_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_2_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9f_v2_4_1 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9f_v2_4_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9f_v2_4_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_4_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9f_v3_0_0 T169 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9f_v3_0_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9f_v3_0_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v3_0_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN target list from an UMRR9F v3.2.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  targetlist_can_umrr9f_v3_2_0  The received CAN target list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_targetlist_callback_umrr9f_v3_2_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9f_t169_automotive_v3_2_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v3_2_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9d_v1_0_3 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9d_v1_0_3  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9d_v1_0_3(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_0_3,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9d_v1_2_2 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9d_v1_2_2  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9d_v1_2_2(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_2_2,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9d_v1_4_1 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9d_v1_4_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9d_v1_4_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_4_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrr9d_v1_5_0 T152 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrr9d_v1_5_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrr9d_v1_5_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_5_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN target list from an UMRR9D v1.7.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  targetlist_can_umrr9d_v1_7_0  The received CAN target list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_targetlist_callback_umrr9d_v1_7_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrr9d_t152_automotive_v1_7_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_7_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrra4_v1_0_1 T171 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrra4_v1_0_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrra4_v1_0_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_0_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_0_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrra4_v1_2_1 T171 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrra4_v1_2_1  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrra4_v1_2_1(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_2_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_2_1,
    const com::types::ClientId client_id);

  ///
  /// @brief      A callback that is called when a new CAN target list for
  /// umrra4_v1_4_0 T171 arrives.
  ///
  /// @param[in]  sensor_idx   The sensor id for respective published topic.
  /// @param[in]  targetlist_can_umrra4_v1_4_0  The target list port
  /// @param[in]  client_id  The client_id of the sensor
  ///
  void CAN_targetlist_callback_umrra4_v1_4_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_4_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_4_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Handles a CAN target list from an UMRRA4 v1.6.0 radar.
  ///
  /// @param[in]  sensor_idx  The sensor index for the respective published topic.
  /// @param[in]  targetlist_can_umrra4_v1_6_0  The received CAN target list.
  /// @param[in]  client_id  The client identifier of the sensor.
  ///
  void CAN_targetlist_callback_umrra4_v1_6_0(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<
      com::master::umrra4_automotive_v1_6_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_6_0,
    const com::types::ClientId client_id);

  ///
  /// @brief      Read parameters and update the json config files required by
  /// Smart Access C++ API.
  ///
  void update_config_files_from_params();

  ///
  /// @brief      Creates publishers for sensors using ports.
  ///
  /// @param[in]  sensor       The sensor configuration.
  /// @param[in]  sensor_idx   The sensor index.
  ///
  void port_publishers(const detail::SensorConfig & sensor, size_t sensor_idx);

  ///
  /// @brief      Creates publishers for sensors using CAN.
  ///
  /// @param[in]  sensor       The sensor configuration.
  /// @param[in]  sensor_idx   The sensor index.
  ///
  void can_publishers(const detail::SensorConfig & sensor, size_t sensor_idx);

  ///
  /// @brief      Callback for getting the parameter response.
  ///
  /// @param[in]  client_id         The client identifier.
  /// @param[in]  response          The response batch.
  /// @param[in]  instruction_names  The instruction names.
  /// @param[in]  section_name       The interface section name.
  ///
  void mode_response(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> & response,
    const std::vector<std::string> & instruction_names,
    const std::string & section_name);

  ///
  /// @brief      Callback for getting the command response.
  ///
  /// @param[in]  client_id     The client identifier.
  /// @param[in]  response      The response batch.
  /// @param[in]  command_name  The command name.
  ///
  void command_response(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> & response, const std::string command_name,
    const std::string & section_name);

  ///
  /// @brief      Callback for changing IP address.
  ///
  /// @param[in]  client_id  The client identifier.
  /// @param[in]  response   The response batch.
  ///
  void sensor_response_ip(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> & response);

  ///
  /// @brief      Sends instructions to the sensor.
  ///
  /// @param[in]  request   The request.
  /// @param[out] response  The response.
  ///
  void set_radar_mode(
    const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> response);

  ///
  /// @brief      Configures the sensor IP address.
  ///
  /// @param[in]  request   The request.
  /// @param[out] response  The response.
  ///
  void ip_address(
    const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> response);

  ///
  /// @brief      Sends command to the sensor.
  ///
  /// @param[in]  request   The request.
  /// @param[out] response  The response.
  ///
  void radar_command(
    const std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Response> response);

  ///
  /// @brief      Service for firmware download.
  ///
  /// @param[in]  request   The request.
  /// @param[out] result    The result.
  ///
  void firmware_download(
    const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Response> result);

  ///
  /// @brief Converts a timestamp from microseconds to seconds and nanoseconds.
  ///
  /// @param timestamp The input timestamp in microseconds as a `std::chrono::microseconds`.
  /// @return A `std::pair` where:
  ///         - `first` is the number of seconds (`int32_t`).
  ///         - `second` is the number of nanoseconds (`uint32_t`).
  ///
  inline std::pair<int32_t, uint32_t> convert_timestamp(std::chrono::microseconds timestamp)
  {
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    return {sec.count(), nanosec.count()};
  }

  ///
  /// @brief Fills the ROS timestamp for the PointCloud2 message and the custom header message.
  ///
  /// @tparam HeaderMsgT One of the custom header message types:
  ///         PortTargetHeader, PortObjectHeader, CanTargetHeader, CanObjectHeader.
  ///
  /// @param[in,out] msg         The PointCloud2 message.
  /// @param[in,out] header      The custom ROS header message.
  /// @param[in]     timestamp_us Sensor timestamp in microseconds.
  /// @param[in]     sensor_idx   Sensor index used to select the frame_id.
  ///
  template<typename HeaderMsgT>
  void fill_ros_header_stamp(
    sensor_msgs::msg::PointCloud2 & msg,
    HeaderMsgT & header,
    const std::uint64_t timestamp_us,
    const std::uint32_t sensor_idx)
  {
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{timestamp_us});

    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    stamp.nanosec = nanosec;

    msg.header.stamp = stamp;
    header.header.stamp = stamp;
    header.header.frame_id = m_sensors[sensor_idx].frame_id;
  }

  ///
  /// @brief      Fills the ROS header timestamp and frame ID of a message.
  ///
  /// @tparam MsgT  A ROS message type that contains a standard header member.
  /// @param[in,out] msg  The ROS message to update.
  /// @param[in] timestamp_us  Sensor timestamp in microseconds.
  /// @param[in] sensor_idx  Sensor index used to select the frame ID.
  ///
  template<typename MsgT>
  void fill_ros_header_stamp(
    MsgT & msg,
    const std::uint64_t timestamp_us,
    const std::uint32_t sensor_idx)
  {
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{timestamp_us});

    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    stamp.nanosec = nanosec;

    msg.header.stamp = stamp;
    msg.header.frame_id = m_sensors[sensor_idx].frame_id;
  }

  ///
  /// @brief      Initializes all the smart access and ros2 services.
  ///
  void initialize_services();

  ///
  /// @brief      Check and set up publsihers w.r.t defined radar parameters.
  ///
  void setup_publishers();

  ///
  /// @brief      Service to get sensor status.
  ///
  /// @param[in]  request   The request.
  /// @param[out] response  The response.
  ///
  void get_radar_status(
    const std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Response> response);

  ///
  /// @brief      Service to get sensor modes.
  ///
  /// @param[in]  request   The request.
  /// @param[out] response  The response.
  ///
  void get_radar_mode(
    const std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Response> response);

  ///
  /// @brief      Callback for getting the status response.
  ///
  /// @param[in]  client_id         The client identifier.
  /// @param[in]  response          The response batch.
  /// @param[in]  statuses          The status name of the sensor.
  /// @param[in]  section_name      The section name of the interface.
  ///
  void status_response(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> & response,
    const std::vector<std::string> & statuses,
    const std::string & section_name);

  ///
  /// @brief      Callback for getting the reading param response.
  ///
  /// @param[in]  client_id         The client identifier.
  /// @param[in]  response          The response batch.
  /// @param[in]  params            The parameter names of the sensor.
  /// @param[in]  section_name      The interface section name.
  ///
  void param_response(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> & response,
    const std::vector<std::string> & params,
    const std::string & section_name);

  rclcpp::Service<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SetIp>::SharedPtr ip_addr_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::SendCommand>::SharedPtr command_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::FirmwareDownload>::SharedPtr download_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::GetStatus>::SharedPtr status_srv_;
  rclcpp::Service<umrr_ros2_msgs::srv::GetMode>::SharedPtr read_mode_srv_;

  std::array<detail::SensorConfig, detail::kMaxSensorCount> m_sensors{};
  std::array<detail::HWConfig, detail::kMaxHwCount> m_adapters{};

  std::array<
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_obj{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortObjectHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_port_obj_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::CanObjectHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_can_obj_header{};

  std::array<
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, detail::kMaxSensorCount>
  m_publishers{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortTargetHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_port_target_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::CanTargetHeader>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_can_target_header{};

  std::array<
    rclcpp::Publisher<umrr_ros2_msgs::msg::PortFaultReportsMsg>::SharedPtr, detail::kMaxSensorCount>
  m_publishers_fault_report_msg{};

  std::size_t m_number_of_sensors{};
  std::size_t m_number_of_adapters{};
  com::types::ClientId client_id;
  std::uint64_t response_type{};
  std::shared_ptr<UpdateService> update_service;
  std::atomic_bool m_shutdown_requested{false};
};

std::shared_ptr<com::master::CommunicationServicesIface> m_services{};
std::shared_ptr<
  com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface>
data_umrra4_v1_0_1{};
std::shared_ptr<
  com::master::umrra4_automotive_v1_2_1::DataStreamServiceIface>
data_umrra4_v1_2_1{};
std::shared_ptr<
  com::master::umrra4_automotive_v1_4_0::DataStreamServiceIface>
data_umrra4_v1_4_0{};
std::shared_ptr<
  com::master::umrra4_automotive_v1_6_0::DataStreamServiceIface>
data_umrra4_v1_6_0{};
std::shared_ptr<
  com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface>
data_umrr11{};
std::shared_ptr<
  com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface>
data_umrr96{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface>
data_umrr9f_v1_1_1{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface>
data_umrr9f_v2_0_0{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface>
data_umrr9f_v2_1_1{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface>
data_umrr9f_v2_2_1{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v2_4_1::DataStreamServiceIface>
data_umrr9f_v2_4_1{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v3_0_0::DataStreamServiceIface>
data_umrr9f_v3_0_0{};
std::shared_ptr<
  com::master::umrr9f_t169_automotive_v3_2_0::DataStreamServiceIface>
data_umrr9f_v3_2_0{};
std::shared_ptr<
  com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface>
data_umrr9d_v1_0_3{};
std::shared_ptr<
  com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface>
data_umrr9d_v1_2_2{};
std::shared_ptr<
  com::master::umrr9d_t152_automotive_v1_4_1::DataStreamServiceIface>
data_umrr9d_v1_4_1{};
std::shared_ptr<
  com::master::umrr9d_t152_automotive_v1_5_0::DataStreamServiceIface>
data_umrr9d_v1_5_0{};
std::shared_ptr<
  com::master::umrr9d_t152_automotive_v1_7_0::DataStreamServiceIface>
data_umrr9d_v1_7_0{};
std::shared_ptr<
  com::master::umrr9f_t169_mse_v1_0_0::DataStreamServiceIface>
data_umrr9f_mse_v1_0_0{};
std::shared_ptr<
  com::master::umrr9f_t169_mse_v1_1_0::DataStreamServiceIface>
data_umrr9f_mse_v1_1_0{};
std::shared_ptr<
  com::master::umrr9f_t169_mse_v1_3_0::DataStreamServiceIface>
data_umrr9f_mse_v1_3_0{};
std::shared_ptr<
  com::master::umrr9f_t169_mse_v2_0_0::DataStreamServiceIface>
data_umrr9f_mse_v2_0_0{};
std::shared_ptr<
  com::master::umrra4_mse_v1_0_0::DataStreamServiceIface>
data_umrra4_mse_v1_0_0{};
std::shared_ptr<
  com::master::umrra4_mse_v2_1_0::DataStreamServiceIface>
data_umrra4_mse_v2_1_0{};
std::shared_ptr<
  com::master::umrra4_mse_v3_0_0::DataStreamServiceIface>
data_umrra4_mse_v3_0_0{};
std::shared_ptr<
  com::master::umrra1_t166_b_automotive_v1_0_0::DataStreamServiceIface>
data_umrra1_v1_0_0{};
std::shared_ptr<
  com::master::umrra1_t166_b_automotive_v2_0_0::DataStreamServiceIface>
data_umrra1_v2_0_0{};
std::shared_ptr<
  com::master::umrra1_t166_b_automotive_v2_0_1::DataStreamServiceIface>
data_umrra1_v2_0_1{};

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

#endif  // UMRR_ROS2_DRIVER__SMARTMICRO_RADAR_NODE_HPP_
