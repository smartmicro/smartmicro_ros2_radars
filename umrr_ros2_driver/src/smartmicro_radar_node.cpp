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

#include "umrr_ros2_driver/smartmicro_radar_node.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <umrr11_t132_automotive_v1_1_2/comtargetlist/PortHeader.h>
#include <umrr11_t132_automotive_v1_1_2/comtargetlist/Target.h>
#include <umrr96_t153_automotive_v1_2_2/comtargetlist/PortHeader.h>
#include <umrr96_t153_automotive_v1_2_2/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_0_3/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_0_3/comtargetlist/Target.h>
#include <umrr9d_t152_automotive_v1_2_2/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_2_2/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/Target.h>

#include <signal.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>

#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "umrr_ros2_driver/UpdateService.hpp"
#include "umrr_ros2_driver/config_path.hpp"

using com::common::Instruction;
using com::master::CmdRequest;
using com::master::CommunicationServicesIface;
using com::master::GetParamRequest;
using com::master::GetStatusRequest;
using com::master::InstructionBatch;
using com::master::InstructionServiceIface;
using com::master::Response;
using com::master::ResponseBatch;
using com::master::SetParamRequest;
using point_cloud_msg_wrapper::PointCloud2Modifier;

namespace
{
constexpr auto kDefaultClientId = 0;
constexpr auto kDefaultInterfaceName = "lo";
constexpr auto kDefaultIp = "127.0.0.1";
constexpr auto kDefaultPort = 55555;
constexpr auto kDefaultHistorySize = 10;
constexpr auto kDefaultFrameId = "umrr";
constexpr auto kDefaultSensorType = "umrr11";
constexpr auto kDefaultInstType = "port_based";
constexpr auto kDefaultDataType = "port_based";

constexpr auto kDefaultHwDevId = 1;
constexpr auto kDefaultHwDevIface = "slcan";
constexpr auto kDefaultHwLinkType = "can";

constexpr auto kHwDevLinkTag = "type";
constexpr auto kClientLinkTag = "link_type";
constexpr auto kHwDevIdTag = "dev_id";
constexpr auto kHwDevIfaceNameTag = "iface_name";

constexpr auto kHwDevPortTag = "hw_port";

constexpr auto kClientIdTag = "client_id";
constexpr auto kPortTag = "port";
constexpr auto kBaudRateTag = "baudrate";
constexpr auto kIpTag = "ip";

constexpr auto kInstSerialTypeTag = "master_inst_serial_type";
constexpr auto kDataSerialTypeTag = "master_data_serial_type";

constexpr auto kInstSerialTypeJsonTag = "instruction_serialization_type";
constexpr auto kDataSerialTypeJsonTag = "data_serialization_type";

constexpr auto kClientsJsonTag = "clients";
constexpr auto kHwItemsJsonTag = "hwItems";
constexpr auto kUINameTag = "user_interface_name";
constexpr auto kUIMajorVTag = "user_interface_major_v";
constexpr auto kUIMinorVTag = "user_interface_minor_v";
constexpr auto kUIPatchVTag = "user_interface_patch_v";

constexpr bool float_eq(const float a, const float b) noexcept
{
  const auto maximum = std::max(std::fabs(a), std::fabs(b));
  return std::fabs(a - b) <= maximum * std::numeric_limits<float>::epsilon();
}

struct RadarPoint
{
  float x{};
  float y{};
  float z{};
  float radial_speed{};
  float power{};
  float rcs{};
  float noise{};
  float snr{};
  constexpr friend bool operator==(const RadarPoint & p1, const RadarPoint & p2) noexcept
  {
    return float_eq(p1.x, p2.x) && float_eq(p1.y, p2.y) && float_eq(p1.z, p2.z) &&
           float_eq(p1.radial_speed, p2.radial_speed) && float_eq(p1.power, p2.power) &&
           float_eq(p1.rcs, p2.rcs) && float_eq(p1.noise, p2.noise) && float_eq(p1.snr, p2.snr);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(radial_speed);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(power);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(rcs);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(noise);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(snr);
using Generators = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_radial_speed_generator, field_rcs_generator,
  field_noise_generator, field_power_generator, field_snr_generator>;
using RadarCloudModifier = PointCloud2Modifier<RadarPoint, Generators>;

}  // namespace

namespace smartmicro
{
namespace drivers
{
namespace radar
{
SmartmicroRadarNode::SmartmicroRadarNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"smartmicro_radar_node", node_options}
{
  update_config_files_from_params();

  const auto override = false;
  setenv("SMART_ACCESS_CFG_FILE_PATH", kConfigFilePath, override);

  // Getting the communication services
  m_services = CommunicationServicesIface::Get();
  if (!m_services->Init()) {
    throw std::runtime_error("Initialization failed");
  }

  // Getting the data stream service
  data_umrra4_v1_0_1 = com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface::Get();
  data_umrr11 = com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface::Get();
  data_umrr96 = com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9f_v1_1_1 = com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_0_0 = com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface::Get();
  data_umrr9f_v2_1_1 = com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_2_1 = com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface::Get();
  data_umrr9d_v1_0_3 = com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface::Get();
  data_umrr9d_v1_2_2 = com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface::Get();

  // Wait for initailization
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->get_logger(), "Data stream services have been received!");

  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];

    m_publishers[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/targets_" + std::to_string(i), sensor.history_size);
    m_targetlist_publishers[i] = create_publisher<umrr_ros2_msgs::msg::TargetList>(
      "smart_radar/targetlists_" + std::to_string(i), sensor.history_size);

    if (
      sensor.model == "umrra4_v1_0_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrra4_v1_0_1->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrra4" << std::endl;
    }
    if (
      sensor.model == "umrr96" &&
      com::types::ERROR_CODE_OK !=
        data_umrr96->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr96, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr96" << std::endl;
    }
    if (
      sensor.model == "umrr11" &&
      com::types::ERROR_CODE_OK !=
        data_umrr11->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr11, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr11" << std::endl;
    }
    if (
      sensor.model == "umrr9f_v1_1_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v1_1_1->RegisterComTargetListPortReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f_v1_1_1" << std::endl;
    }
    if (
      sensor.model == "umrr9f_v2_0_0" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v2_0_0->RegisterComTargetListPortReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f_v2_0_0" << std::endl;
    }
    if (
      sensor.model == "umrr9f_v2_1_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v2_1_1->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f" << std::endl;
    }
    if (
      sensor.model == "umrr9f_v2_2_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v2_2_1->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f_v2_2_1" << std::endl;
    }
    if (
      sensor.model == "umrr9d_v1_0_3" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9d_v1_0_3->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9d_v1_0_3" << std::endl;
    }
    if (
      sensor.model == "umrr9d_v1_2_2" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9d_v1_2_2->RegisterComTargetListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9d_v1_2_2" << std::endl;
    }
    if (
      sensor.model == "umrra4_can_v1_0_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrra4_v1_0_1->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrra4" << std::endl;
    }
    if (
      sensor.model == "umrr96_can" &&
      com::types::ERROR_CODE_OK !=
        data_umrr96->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr96, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr96" << std::endl;
    }
    if (
      sensor.model == "umrr11_can" &&
      com::types::ERROR_CODE_OK !=
        data_umrr11->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr11, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr11" << std::endl;
    }
    if (
      sensor.model == "umrr9d_can_v1_0_3" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9d_v1_0_3->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr9d_v1_0_3" << std::endl;
    }
    if (
      sensor.model == "umrr9d_can_v1_2_2" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9d_v1_2_2->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr9d_v1_2_2" << std::endl;
    }
    if (
      sensor.model == "umrr9f_can_v2_1_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v2_1_1->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr9f_v2_1_1" << std::endl;
    }
    if (
      sensor.model == "umrr9f_can_v2_2_1" &&
      com::types::ERROR_CODE_OK !=
        data_umrr9f_v2_2_1->RegisterComTargetBaseListReceiveCallback(
          sensor.id, std::bind(
                       &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1, this, i,
                       std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register CAN targetlist for sensor umrr9f_v2_2_1" << std::endl;
    }
  }

  // create a ros2 service to change the radar parameters
  mode_srv_ = create_service<umrr_ros2_msgs::srv::SetMode>(
    "smart_radar/set_radar_mode",
    std::bind(
      &SmartmicroRadarNode::radar_mode, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to change the IP address
  ip_addr_srv_ = create_service<umrr_ros2_msgs::srv::SetIp>(
    "smart_radar/set_ip_address",
    std::bind(
      &SmartmicroRadarNode::ip_address, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to send command to radar
  command_srv_ = create_service<umrr_ros2_msgs::srv::SendCommand>(
    "smart_radar/send_command",
    std::bind(
      &SmartmicroRadarNode::radar_command, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to perform firmware download
  download_srv_ = create_service<umrr_ros2_msgs::srv::FirmwareDownload>(
    "smart_radar/firmware_download",
    std::bind(
      &SmartmicroRadarNode::firmware_download, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Radar services are ready.");

  rclcpp::on_shutdown(std::bind(&SmartmicroRadarNode::on_shutdown_callback, this));
}

void SmartmicroRadarNode::on_shutdown_callback()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown called!!!");
  check_signal = true;
  rclcpp::Rate sleepRate(std::chrono::milliseconds(100));
  sleepRate.sleep();
  m_services.reset();
}

void SmartmicroRadarNode::firmware_download(
  const std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::FirmwareDownload::Response> result)
{
  bool check_flag_id = false;
  client_id = request->sensor_id;
  update_image = request->file_path;

  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  StartSoftwareUpdate(client_id, update_image);
  result->res = "Service ended, check the console output for status! ";
}

void SmartmicroRadarNode::radar_mode(
  const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> result)
{
  std::string instruction_name{};
  bool check_flag_id = false;

  instruction_name = request->param;
  client_id = request->sensor_id;

  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;

  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Failed to allocate instruction batch! ";
    return;
  }

  std::shared_ptr<SetParamRequest<uint8_t>> radar_mode_01 =
    std::make_shared<SetParamRequest<uint8_t>>(
      "auto_interface_0dim", request->param, request->value);

  std::shared_ptr<SetParamRequest<float>> radar_mode_02 =
    std::make_shared<SetParamRequest<float>>("auto_interface_0dim", request->param, request->value);

  if (!batch->AddRequest(radar_mode_01)) {
    result->res = "Failed to add instruction to the batch! ";
  }
  if (!batch->AddRequest(radar_mode_02)) {
    result->res = "Failed to add instruction to the batch! ";
  }

  if (
    com::types::ERROR_CODE_OK != inst->SendInstructionBatch(
                                   batch, std::bind(
                                            &SmartmicroRadarNode::mode_response, this, client_id,
                                            std::placeholders::_2, instruction_name))) {
    result->res = "Check param is listed for sensor type and the min/max values!";
    return;
  }
  result->res = "Service conducted successfully";
}

void SmartmicroRadarNode::ip_address(
  const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> result)
{
  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));
  bool check_flag = false;
  client_id = request->sensor_id;
  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag = true;
      break;
    }
  }
  if (!check_flag) {
    result->res_ip = "Sensor ID entered is not listed in the param file! ";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res_ip = "Failed to allocate instruction batch! ";
    return;
  }

  std::shared_ptr<SetParamRequest<uint32_t>> ip_address =
    std::make_shared<SetParamRequest<uint32_t>>(
      "auto_interface_0dim", "ip_source_address", request->value_ip);

  std::shared_ptr<CmdRequest> cmd =
    std::make_shared<CmdRequest>("auto_interface_command", "comp_eeprom_ctrl_save_param_sec", 2010);

  if (!batch->AddRequest(ip_address)) {
    result->res_ip = "Failed to add instruction to batch! ";
    return;
  }
  if (!batch->AddRequest(cmd)) {
    result->res_ip = "Failed to add instruction to batch! ";
    return;
  }
  // send instruction batch to the device
  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, std::bind(
               &SmartmicroRadarNode::sensor_response_ip, this, client_id, std::placeholders::_2))) {
    result->res_ip = "Service not conducted";
    return;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Radar must be restarted and the parameters in the param file "
      "must be updated !!.");
    result->res_ip = "Service conducted successfully";
  }
}

void SmartmicroRadarNode::radar_command(
  const std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SendCommand::Response> result)
{
  std::string command_name{};
  bool check_flag_id = false;

  command_name = request->command;
  client_id = request->sensor_id;

  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;

  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Failed to allocate instruction batch! ";
    return;
  }

  std::shared_ptr<CmdRequest> radar_command =
    std::make_shared<CmdRequest>("auto_interface_command", request->command, request->value);

  if (!batch->AddRequest(radar_command)) {
    result->res = "Failed to add instruction to the batch! ";
    return;
  }

  if (
    com::types::ERROR_CODE_OK != inst->SendInstructionBatch(
                                   batch, std::bind(
                                            &SmartmicroRadarNode::command_response, this, client_id,
                                            std::placeholders::_2, command_name))) {
    result->res = "Error in sending command to the sensor!";
    return;
  }
  result->res = "Service conducted successfully";
}

void SmartmicroRadarNode::mode_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response, const std::string instruction_name)
{
  std::vector<std::shared_ptr<Response<uint8_t>>> myResp_1;
  std::vector<std::shared_ptr<Response<float>>> myResp_2;

  if (response->GetResponse<uint8_t>("auto_interface_0dim", instruction_name.c_str(), myResp_1)) {
    for (auto & resp : myResp_1) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(
        this->get_logger(), "Response from '%s' : %i", instruction_name.c_str(), response_type);
    }
  }
  if (response->GetResponse<float>("auto_interface_0dim", instruction_name.c_str(), myResp_2)) {
    for (auto & resp : myResp_2) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(
        this->get_logger(), "Response from '%s' : %i", instruction_name.c_str(), response_type);
    }
  }
}

void SmartmicroRadarNode::sensor_response_ip(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> myResp_2;
  if (response->GetResponse<uint32_t>("auto_interface_0dim", "ip_source_address", myResp_2)) {
    for (auto & resp : myResp_2) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(this->get_logger(), "Response from sensor for ip change: %i", response_type);
    }
  }
}

void SmartmicroRadarNode::command_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response, const std::string command_name)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> command_resp;
  if (response->GetResponse<uint32_t>(
        "auto_interface_command", command_name.c_str(), command_resp)) {
    for (auto & resp : command_resp) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(this->get_logger(), "Response from sensor to command: %i", response_type);
    }
  }
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_0_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra4_v1_0_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrra4_v1_0_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrra4_v1_0_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.power = target->GetPower();
      point.rcs = target->GetRcs();
      point.noise = target->GetNoise();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr96(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr96,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr96_v1_2_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr96->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr96->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.power = target->GetPower();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr11(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr11,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr11_v1_1_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr11->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr11->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_0_3,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_0_3" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9d_v1_0_3->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9d_v1_0_3->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRcs();
      point.noise = target->GetNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_2_2,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_2_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9d_v1_2_2->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9d_v1_2_2->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRcs();
      point.noise = target->GetNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::ComTargetListPort> &
    targetlist_port_umrr9f_v1_1_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v1_1_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<
      com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::GenericPortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v1_1_1->GetGenericPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9f_v1_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetTgtNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetTgtNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::ComTargetListPort> &
    targetlist_port_umrr9f_v2_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist callback is being called for umrr9f_v2_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<
      com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::GenericPortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v2_0_0->GetGenericPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9f_v2_0_0->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetTgtNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetTgtNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_1_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v2_1_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v2_1_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9f_v2_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRcs();
      point.noise = target->GetNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_2_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f v2_2_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v2_2_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_port_umrr9f_v2_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRcs();
      point.noise = target->GetNoise();
      point.power = target->GetPower();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_0_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_0_1,
  const com::types::ClientId client_id)
{
  std::cout << "CAN Targetlist for umrra4_v1_0_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrra4_v1_0_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrra4_v1_0_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr96(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr96,
  const com::types::ClientId client_id)
{
  std::cout << "CAN Targetlist for umrr96_v1_2_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr96->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr96->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr11(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr11,
  const com::types::ClientId client_id)
{
  std::cout << "CAN Targetlist for umrr11_v1_1_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr11->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr11->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_0_3,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_0_3" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9d_v1_0_3->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr9d_v1_0_3->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_2_2,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_2_2" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9d_v1_2_2->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr9d_v1_2_2->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_1_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v2_1_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9f_v2_1_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr9f_v2_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_2_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v2_2_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9f_v2_2_1->GetPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::TargetList point;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    point.header.stamp.sec = sec.count();
    point.header.stamp.nanosec = nanosec.count();
    point.header.frame_id = m_sensors[sensor_idx].frame_id;
    for (const auto & target : targetlist_can_umrr9f_v2_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      point.azimuth_angle = azimuth_angle;
      point.elevation_angle = elevation_angle;
      point.range = range;
      point.radial_speed = target->GetSpeedRadial();
      point.rcs = target->GetRCS();
      point.noise = target->GetNoise();
      point.power = target->GetSignalLevel();
      point.snr = snr;
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), point.radial_speed, point.power, point.rcs, point.noise,
         point.snr});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_targetlist_publishers[sensor_idx]->publish(point);
  }
}

void SmartmicroRadarNode::update_config_files_from_params()
{
  const auto master_inst_serial_type = declare_parameter(kInstSerialTypeTag, std::string{});
  const auto master_data_serial_type = declare_parameter(kDataSerialTypeTag, std::string{});

  auto read_adapter_params_if_possible = [&](const std::uint32_t index) {
    RCLCPP_INFO(this->get_logger(), "Updating ADAPTER params.");
    auto & current_adapter = m_adapters[index];
    const auto prefix_2 = "adapters.adapter_" + std::to_string(index);
    current_adapter.hw_dev_id = this->declare_parameter(prefix_2 + ".hw_dev_id", kDefaultHwDevId);
    if (current_adapter.hw_dev_id == kDefaultHwDevId) {
      // The id was not set, so the adapter with this index was not defined.
      // Stop here.
      return false;
    }
    current_adapter.hw_iface_name =
      this->declare_parameter(prefix_2 + ".hw_iface_name", kDefaultHwDevIface);
    current_adapter.hw_type = this->declare_parameter(prefix_2 + ".hw_type", kDefaultHwLinkType);
    current_adapter.baudrate = this->declare_parameter(prefix_2 + ".baudrate", 500000);
    current_adapter.port = this->declare_parameter(prefix_2 + ".port", kDefaultPort);

    return true;
  };

  auto read_sensor_params_if_possible = [&](const std::uint32_t index) {
    RCLCPP_INFO(this->get_logger(), "Updating SENSOR params.");
    auto & sensor = m_sensors[index];
    const auto prefix_3 = "sensors.sensor_" + std::to_string(index);
    sensor.dev_id = this->declare_parameter(prefix_3 + ".dev_id", kDefaultHwDevId);
    sensor.uifname = this->declare_parameter(prefix_3 + ".uifname", "");
    sensor.uifmajorv = this->declare_parameter(prefix_3 + ".uifmajorv", 0);
    sensor.uifminorv = this->declare_parameter(prefix_3 + ".uifminorv", 0);
    sensor.uifpatchv = this->declare_parameter(prefix_3 + ".uifpatchv", 0);
    sensor.model = this->declare_parameter(prefix_3 + ".model", kDefaultSensorType);
    sensor.id = this->declare_parameter(prefix_3 + ".id", kDefaultClientId);
    if (sensor.id == kDefaultClientId) {
      // The id was not set, so the sensor with this index was not defined. Stop
      // here.
      return false;
    }
    sensor.ip = this->declare_parameter(prefix_3 + ".ip", "");
    sensor.port = this->declare_parameter(prefix_3 + ".port", 0);
    sensor.frame_id = this->declare_parameter(prefix_3 + ".frame_id", kDefaultFrameId);
    sensor.history_size = this->declare_parameter(prefix_3 + ".history_size", kDefaultHistorySize);
    sensor.inst_type = this->declare_parameter(prefix_3 + ".inst_type", "");
    sensor.data_type = this->declare_parameter(prefix_3 + ".data_type", "");
    sensor.link_type = this->declare_parameter(prefix_3 + ".link_type", kDefaultHwLinkType);
    return true;
  };

  for (auto j = 0UL; j < m_adapters.size(); ++j) {
    if (!read_adapter_params_if_possible(j)) {
      m_number_of_adapters = j;
      break;
    }
  }

  if (!m_number_of_adapters) {
    throw std::runtime_error("At least one adapter must be configured.");
  }

  for (auto i = 0UL; i < m_sensors.size(); ++i) {
    if (!read_sensor_params_if_possible(i)) {
      m_number_of_sensors = i;
      break;
    }
  }
  if (!m_number_of_sensors) {
    throw std::runtime_error("At least one sensor must be configured.");
  }

  auto config = nlohmann::json::parse(std::ifstream{kConfigFilePath});
  config[kDataSerialTypeJsonTag] = master_data_serial_type;
  config[kInstSerialTypeJsonTag] = master_inst_serial_type;
  std::ofstream{kConfigFilePath, std::ios::trunc} << config;

  auto hw_inventory = nlohmann::json::parse(std::ifstream{kHwInventoryFilePath});
  auto & hw_items = hw_inventory[kHwItemsJsonTag];
  if (hw_items.empty()) {
    throw std::runtime_error("There are no 'hwItems' defined in the hw_inventory.json file.");
  }
  auto hw_item = hw_items.front();
  hw_items.clear();
  for (auto j = 0UL; j < m_number_of_adapters; ++j) {
    const auto & adapter = m_adapters[j];
    hw_item[kPortTag] = adapter.port;
    hw_item[kHwDevLinkTag] = adapter.hw_type;
    hw_item[kHwDevIdTag] = adapter.hw_dev_id;
    hw_item[kHwDevIfaceNameTag] = adapter.hw_iface_name;
    hw_item[kBaudRateTag] = adapter.baudrate;
    hw_items.push_back(hw_item);
  }
  std::ofstream{kHwInventoryFilePath, std::ios::trunc} << hw_inventory;

  auto routing_table = nlohmann::json::parse(std::ifstream{kRoutingTableFilePath});
  auto & clients = routing_table[kClientsJsonTag];
  if (clients.empty()) {
    throw std::runtime_error("There are no 'clients' defined in the routing_table.json file.");
  }
  auto client = clients.front();  // Make a copy of the first client.
  clients.clear();
  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];
    client[kClientLinkTag] = sensor.link_type;
    client[kClientIdTag] = sensor.id;  //
    client[kHwDevIdTag] = sensor.dev_id;
    client[kPortTag] = sensor.port;
    client[kIpTag] = sensor.ip;
    client[kInstSerialTypeJsonTag] = sensor.inst_type;
    client[kDataSerialTypeJsonTag] = sensor.data_type;
    client[kUINameTag] = sensor.uifname;
    client[kUIMajorVTag] = sensor.uifmajorv;
    client[kUIMinorVTag] = sensor.uifminorv;
    client[kUIPatchVTag] = sensor.uifpatchv;
    clients.push_back(client);
  }

  std::ofstream{kRoutingTableFilePath, std::ios::trunc} << std::setw(4) << routing_table;
}

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::SmartmicroRadarNode)
