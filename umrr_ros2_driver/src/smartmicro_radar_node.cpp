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

#include <umrr_ros2_driver/config_path.hpp>
#include <umrr_ros2_driver/sensor_params.hpp>
#include <umrr_ros2_driver/smartmicro_radar_node.hpp>

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>

#include <umrr11_t132_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr11_t132_automotive_v1_1_1/comtargetlistport/ComTargetListPort.h>
#include <umrr11_t132_automotive_v1_1_1/comtargetlistport/GenericPortHeader.h>
#include <umrr11_t132_automotive_v1_1_1/comtargetlistport/Target.h>

#include <umrr96_t153_automotive_v1_2_1/DataStreamServiceIface.h>
#include <umrr96_t153_automotive_v1_2_1/comtargetlistport/ComTargetListPort.h>
#include <umrr96_t153_automotive_v1_2_1/comtargetlistport/GenericPortHeader.h>
#include <umrr96_t153_automotive_v1_2_1/comtargetlistport/Target.h>

#include <umrr9f_t169_automotive_v1_1_1/DataStreamServiceIface.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/ComTargetListPort.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/Target.h>

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

namespace {

constexpr auto kDefaultClientId = 0;
constexpr auto kDefaultInterfaceName = "lo";
constexpr auto kDefaultIp = "127.0.0.1";
constexpr auto kDefaultPort = 55555;
constexpr auto kDefaultHistorySize = 10;
constexpr auto kDefaultFrameId = "umrr";
constexpr auto kDefaultSensorType = "umrr11";

constexpr auto kClientIdTag = "client_id";
constexpr auto kPortTag = "port";
constexpr auto kIpTag = "ip";
constexpr auto kIfaceNameTag = "iface_name";
constexpr auto kMasterClientIdTag = "master_client_id";
constexpr auto kDevIdTag = "hw_dev_id";
constexpr auto kDevIfaceNameTag = "hw_iface_name";
constexpr auto kDevPortTag = "hw_port";

constexpr auto kDevIdJsonTag = "dev_id";
constexpr auto kClientsJsonTag = "clients";
constexpr auto kHwItemsJsonTag = "hwItems";

constexpr bool float_eq(const float a, const float b) noexcept {
  const auto maximum = std::max(std::fabs(a), std::fabs(b));
  return std::fabs(a - b) <= maximum * std::numeric_limits<float>::epsilon();
}

struct RadarPoint {
  float x{};
  float y{};
  float z{};
  float radial_speed{};
  float power{};
  float RCS{};
  float Noise{};
  constexpr friend bool operator==(const RadarPoint &p1,
                                   const RadarPoint &p2) noexcept {
    return float_eq(p1.x, p2.x) && float_eq(p1.y, p2.y) &&
           float_eq(p1.z, p2.z) && float_eq(p1.radial_speed, p2.radial_speed) &&
           float_eq(p1.power, p2.power) && float_eq(p1.RCS, p2.RCS) &&
           float_eq(p1.Noise, p2.Noise);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(radial_speed);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(power);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(RCS);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(Noise);
using Generators = std::tuple<point_cloud_msg_wrapper::field_x_generator,
                              point_cloud_msg_wrapper::field_y_generator,
                              point_cloud_msg_wrapper::field_z_generator,
                              field_radial_speed_generator, field_RCS_generator,
                              field_Noise_generator, field_power_generator>;
using RadarCloudModifier = PointCloud2Modifier<RadarPoint, Generators>;

} // namespace


namespace smartmicro {
namespace drivers {
namespace radar {

SmartmicroRadarNode::SmartmicroRadarNode(
    const rclcpp::NodeOptions &node_options)
    : rclcpp::Node{"smartmicro_radar_node", node_options} {

  update_config_files_from_params();

  const auto override = false;
  setenv("SMART_ACCESS_CFG_FILE_PATH", kConfigFilePath, override);

  // Getting the communication services
  m_services = CommunicationServicesIface::Get();

  if (!m_services->Init()) {
    throw std::runtime_error("Initialization failed");
  }

  // Getting the data stream service
  std::shared_ptr<
      com::master::umrr11_t132_automotive_v1_1_1::DataStreamServiceIface>
      data_umrr11 = com::master::umrr11_t132_automotive_v1_1_1::
          DataStreamServiceIface::Get();
  std::shared_ptr<
      com::master::umrr96_t153_automotive_v1_2_1::DataStreamServiceIface>
      data_umrr96 = com::master::umrr96_t153_automotive_v1_2_1::
          DataStreamServiceIface::Get();
  std::shared_ptr<
      com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface>
      data_umrr9f = com::master::umrr9f_t169_automotive_v1_1_1::
          DataStreamServiceIface::Get();
  RCLCPP_INFO(this->get_logger(), "Data stream services have been received!");
  // Wait init time
  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto &sensor = m_sensors[i];

    if (sensor.model == "umrr11" && com::types::ERROR_CODE_OK !=
            data_umrr11->RegisterComTargetListPortReceiveCallback(
                sensor.id,
                std::bind(&SmartmicroRadarNode::targetlist_callback_umrr11,
                          this, i, std::placeholders::_1))) {
      RCLCPP_INFO(this->get_logger(), "Failed to register targetlist callback for sensor umrr11");
    }
    if (sensor.model == "umrr96" && com::types::ERROR_CODE_OK !=
            data_umrr96->RegisterComTargetListPortReceiveCallback(
                sensor.id,
                std::bind(&SmartmicroRadarNode::targetlist_callback_umrr96,
                          this, i, std::placeholders::_1))) {
      RCLCPP_INFO(this->get_logger(), "Failed to register targetlist callback for sensor umrr96");
    }
    if (sensor.model == "umrr9f" && com::types::ERROR_CODE_OK !=
            data_umrr9f->RegisterComTargetListPortReceiveCallback(
                sensor.id,
                std::bind(&SmartmicroRadarNode::targetlist_callback_umrr9f,
                          this, i, std::placeholders::_1))) {
      RCLCPP_INFO(this->get_logger(), "Failed to register targetlist callback for sensor umrr9f");
    }

    m_publishers[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "umrr/targets_" + std::to_string(i), sensor.history_size);
    
  }

  // create a ros2 service to change the radar parameters
  mode_srv_ = create_service<umrr_ros2_msgs::srv::SetMode>(
      "smartmicro_radar_node/set_radar_mode",
      std::bind(&SmartmicroRadarNode::radar_mode, this, std::placeholders::_1,
                std::placeholders::_2));

  // create a ros2 service to change the IP address
  ip_addr_srv_ = create_service<umrr_ros2_msgs::srv::SetIp>(
      "smartmicro_radar_node/set_ip_address",
      std::bind(&SmartmicroRadarNode::ip_address, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Radar services are ready.");

}

void SmartmicroRadarNode::radar_mode(
    const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> result) {

  std::string instruction_name{};
  bool check_flag_param = false;
  bool check_flag_id = false;

  std::ifstream instr_file(KSensorParamFilePath);
  auto param_table = nlohmann::json::parse(instr_file);

  instruction_name = request->param;
  client_id = request->sensor_id;

  for (const auto &item : param_table.items()) {
    for (const auto &param : item.value().items()) {
      if (instruction_name == param.value()["name"]) {
          check_flag_param = true;
          break;
      }
    }
  }

  if (!check_flag_param) {
    result->res = "Invalid instruction name or value! ";
    return;
  }

  for (auto &sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "The sensor ID value entered is invalid! ");
    result->res = "The sensor ID value entered is invalid! ";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{
      m_services->GetInstructionService()};
  timer = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;

  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Failed to allocate instruction batch! ";
    return;
  }
  
  std::shared_ptr<SetParamRequest<uint8_t>> radar_mode_01 = std::make_shared<SetParamRequest<uint8_t>>
  ("auto_interface_0dim", request->param, request->value);
  
  std::shared_ptr<SetParamRequest<float>> radar_mode_02 = std::make_shared<SetParamRequest<float>>
  ("auto_interface_0dim", request->param, request->value);
  
  if (!batch->AddRequest(radar_mode_01)) {
    result->res = "Failed to add instruction to the batch! ";
  }
  if (!batch->AddRequest(radar_mode_02)) {
    result->res = "Failed to add instruction to the batch! ";
  }

  if (com::types::ERROR_CODE_OK !=
      inst->SendInstructionBatch(
          batch, std::bind(&SmartmicroRadarNode::sensor_response, this,
                           client_id, std::placeholders::_2, instruction_name))) {
    result->res = "Check param is listed for sensor type and the min/max values!";
    return;
  }
  result->res = "Service conducted successfully";
}

void SmartmicroRadarNode::ip_address(
    const std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Request> request,
    std::shared_ptr<umrr_ros2_msgs::srv::SetIp::Response> result) {

  std::shared_ptr<InstructionServiceIface> inst{
      m_services->GetInstructionService()};
  timer = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SmartmicroRadarNode::my_timer_callback, this));
  bool check_flag = false;
  client_id = request->sensor_id;
  for (auto &sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag = true;
      break;
    }
  }
  if (!check_flag) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Sensor ID entered is not listed in the param file! ");
    result->res_ip = "Sensor ID entered is not listed in the param file! ";
    return;
  }

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res_ip = "Failed to allocate instruction batch! ";
    return;
  }
  
  std::shared_ptr<SetParamRequest<uint32_t>> ip_address = std::make_shared<SetParamRequest<uint32_t>>
  ("auto_interface_0dim", "ip_source_address", request->value_ip);
  
  std::shared_ptr<CmdRequest> cmd = std::make_shared<CmdRequest>
  ("auto_interface_command", "comp_eeprom_ctrl_save_param_sec", 2010);

  if (!batch->AddRequest(ip_address)) {
    result->res_ip = "Failed to add instruction to batch! ";
    return;
  }
  if (!batch->AddRequest(cmd)) {
    result->res_ip = "Failed to add instruction to batch! ";
    return;
  }
  // send instruction batch to the device
  if (com::types::ERROR_CODE_OK !=
      inst->SendInstructionBatch(
          batch, std::bind(&SmartmicroRadarNode::sensor_response_ip, this,
                           client_id, std::placeholders::_2))) {
    result->res_ip = "Service not conducted";
    return;
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Radar must be restarted and the parameters in the param file "
                "must be updated !!.");
    result->res_ip = "Service conducted successfully";
  }
}

void SmartmicroRadarNode::sensor_response(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> &response,
    const std::string instruction_name) {
  std::vector<std::shared_ptr<Response<uint8_t>>> myResp_1;
  std::vector<std::shared_ptr<Response<float>>> myResp_2;
  
  if (response->GetResponse<uint8_t>("auto_interface_0dim", instruction_name.c_str(), myResp_1))
  { 
    for (auto &resp : myResp_1) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(this->get_logger(), "Response from '%s' : %i",instruction_name.c_str(), response_type);
    }
  }
  if (response->GetResponse<float>("auto_interface_0dim", instruction_name.c_str(), myResp_2))
  {
    for (auto &resp : myResp_2) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(this->get_logger(), "Response from '%s' : %i",instruction_name.c_str(), response_type);
    }
  }
}

void SmartmicroRadarNode::sensor_response_ip(
    const com::types::ClientId client_id,
    const std::shared_ptr<com::master::ResponseBatch> &response) {
  std::vector<std::shared_ptr<Response<uint32_t>>> myResp_2;
  if (response->GetResponse<uint32_t>("auto_interface_0dim",
                                      "ip_source_address", myResp_2)) {
    for (auto &resp : myResp_2) {
      response_type = resp->GetResponseType();
      RCLCPP_INFO(this->get_logger(), "Response from sensor for ip change: %i",
                  response_type);
    }
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr11(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_1::
                              comtargetlistport::ComTargetListPort>
        &targetlist_port_umrr11) {
  
  std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_1::comtargetlistport::GenericPortHeader> port_header;
  port_header = targetlist_port_umrr11->GetGenericPortHeader();
  sensor_msgs::msg::PointCloud2 msg;
  RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
  const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
  const auto nanosec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
  msg.header.stamp.sec = sec.count();
  msg.header.stamp.nanosec = nanosec.count();

  for (const auto &target : targetlist_port_umrr11->GetTargetList()) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(),
         target->GetRCS(), target->GetTgtNoise(), target->GetPower()});
  }

  m_publishers[sensor_idx]->publish(msg);

}

void SmartmicroRadarNode::targetlist_callback_umrr96(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_1::
                              comtargetlistport::ComTargetListPort>
        &targetlist_port_umrr96) {
  std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_1::
                      comtargetlistport::GenericPortHeader>
      port_header = targetlist_port_umrr96->GetGenericPortHeader();
  sensor_msgs::msg::PointCloud2 msg;
  RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
  const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
  const auto nanosec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
  msg.header.stamp.sec = sec.count();
  msg.header.stamp.nanosec = nanosec.count();

  for (const auto &target : targetlist_port_umrr96->GetTargetList()) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(),
         target->GetRCS(), target->GetTgtNoise(), target->GetPower()});
  }

  m_publishers[sensor_idx]->publish(msg);

}

void SmartmicroRadarNode::targetlist_callback_umrr9f(
    const std::uint32_t sensor_idx,
    const std::shared_ptr<com::master::umrr9f_t169_automotive_v1_1_1::
                              comtargetlistport::ComTargetListPort>
        &targetlist_port_umrr9f) {

  std::shared_ptr<com::master::umrr9f_t169_automotive_v1_1_1::
                      comtargetlistport::GenericPortHeader>
      port_header = targetlist_port_umrr9f->GetGenericPortHeader();
  sensor_msgs::msg::PointCloud2 msg;
  RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
  const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
  const auto nanosec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
  msg.header.stamp.sec = sec.count();
  msg.header.stamp.nanosec = nanosec.count();

  for (const auto &target : targetlist_port_umrr9f->GetTargetList()) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(),
         target->GetRCS(), target->GetTgtNoise(), target->GetPower()});
  }

  m_publishers[sensor_idx]->publish(msg);

}

void SmartmicroRadarNode::update_config_files_from_params() {
  const auto dev_id = declare_parameter(kDevIdTag, 0);
  if (!dev_id) {
    throw std::runtime_error("Parameter dev_id not set.");
  }
  const auto dev_port = declare_parameter(kDevPortTag, 0);
  if (!dev_port) {
    throw std::runtime_error("Parameter dev_port not set.");
  }
  const auto dev_iface_name =
      declare_parameter(kDevIfaceNameTag, std::string{});
  if (dev_iface_name.empty()) {
    throw std::runtime_error("Parameter dev_iface_name not set.");
  }
  const auto master_client_id = declare_parameter(kMasterClientIdTag, 0);
  if (!master_client_id) {
    throw std::runtime_error("Parameter master_client_id not set.");
  }

  auto read_sensor_params_if_possible = [&](const std::uint32_t index) {
    auto &current_sensor = m_sensors[index];
    const auto prefix = "sensors.sensor_" + std::to_string(index);
    current_sensor.model =
        this->declare_parameter(prefix + ".model", kDefaultSensorType);
    current_sensor.id =
        this->declare_parameter(prefix + ".id", kDefaultClientId);
    if (current_sensor.id == kDefaultClientId) {
      // The id was not set, so the sensor with this index was not defined. Stop
      // here.
      return false;
    }
    current_sensor.ip = this->declare_parameter(prefix + ".ip", kDefaultIp);
    current_sensor.port =
        this->declare_parameter(prefix + ".port", kDefaultPort);
    current_sensor.frame_id =
        this->declare_parameter(prefix + ".frame_id", kDefaultFrameId);
    current_sensor.history_size =
        this->declare_parameter(prefix + ".history_size", kDefaultHistorySize);

    return true;
  };

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
  config[kClientIdTag] = master_client_id;
  std::ofstream{kConfigFilePath, std::ios::trunc} << config;

  auto hw_inventory =
      nlohmann::json::parse(std::ifstream{kHwInventoryFilePath});
  auto &hw_items = hw_inventory[kHwItemsJsonTag];
  if (hw_items.empty()) {
    throw std::runtime_error(
        "There are no 'hwItems' defined in the hw_inventory.json file.");
  }
  auto &hw_item = hw_items.front();
  hw_item[kDevIdJsonTag] = dev_id;
  hw_item[kPortTag] = dev_port;
  hw_item[kIfaceNameTag] = dev_iface_name;
  std::ofstream{kHwInventoryFilePath, std::ios::trunc} << hw_inventory;

  auto routing_table =
      nlohmann::json::parse(std::ifstream{kRoutingTableFilePath});
  auto &clients = routing_table[kClientsJsonTag];
  if (clients.empty()) {
    throw std::runtime_error(
        "There are no 'clients' defined in the routing_table.json file.");
  }
  auto client = clients.front(); // Make a copy of the first client.
  clients.clear();
  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto &sensor = m_sensors[i];
    client[kClientIdTag] = sensor.id;
    client[kPortTag] = sensor.port;
    client[kIpTag] = sensor.ip;
    clients.push_back(client);
  }
  std::ofstream{kRoutingTableFilePath, std::ios::trunc} << routing_table;
}

} // namespace radar
} // namespace drivers
} // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::SmartmicroRadarNode)
