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

#include <umrr_ros2_driver/smartmicro_radar_node.hpp>
#include <umrr_ros2_driver/config_path.hpp>

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <CommDataStreamServiceIface.h>
#include <CommunicationServicesIface.h>
#include <Instruction.h>
#include <InstructionBatch.h>
#include <InstructionServiceIface.h>

#include <comtargetlistport/ComTargetListPort.h>
#include <comtargetlistport/GenericPortHeader.h>
#include <comtargetlistport/StaticPortHeader.h>
#include <comtargetlistport/Target.h>

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
using com::master::GetStatusRequest;
using com::master::InstructionBatch;
using com::master::comtargetlistport::GenericPortHeader;
using com::master::comtargetlistport::StaticPortHeader;
using com::master::ResponseBatch;
using com::master::Response;
using com::master::CommunicationServicesIface;
using com::master::InstructionServiceIface;
using com::master::CommDataStreamServiceIface;
using point_cloud_msg_wrapper::PointCloud2Modifier;

namespace
{

constexpr auto kDefaultClientId = 0;
constexpr auto kDefaultInterfaceName = "lo";
constexpr auto kDefaultIp = "127.0.0.1";
constexpr auto kDefaultPort = 55555;
constexpr auto kDefaultHistorySize = 10;
constexpr auto kDefaultFrameId = "umrr";

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
  float RCS{};
  float TgtNoise{};
  constexpr friend bool operator==(const RadarPoint & p1, const RadarPoint & p2) noexcept
  {
    return
      float_eq(p1.x, p2.x) &&
      float_eq(p1.y, p2.y) &&
      float_eq(p1.z, p2.z) &&
      float_eq(p1.radial_speed, p2.radial_speed) &&
      float_eq(p1.power, p2.power) &&
      float_eq(p1.RCS, p2.RCS) &&
      float_eq(p1.TgtNoise, p2.TgtNoise);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(radial_speed);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(power);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(RCS);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(TgtNoise);
using Generators = std::tuple<
  point_cloud_msg_wrapper::field_x_generator,
  point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator,
  field_radial_speed_generator,
  field_RCS_generator,
  field_TgtNoise_generator,
  field_power_generator>;
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
  std::shared_ptr<CommDataStreamServiceIface> data{m_services->GetCommDataStreamServiceIface()};
  // Getting the instruction service
  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  // Wait init time
  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];
    if (ERROR_CODE_OK != data->RegisterComTargetListPortReceiveCallback(
        sensor.id,
        std::bind(&SmartmicroRadarNode::target_list_callback, this, i, std::placeholders::_1)))
    {
      throw std::runtime_error("Failed to register ComTargetListPort port callback");
    }
    m_publishers[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "umrr/targets_" + std::to_string(i), sensor.history_size);
  }
}

void SmartmicroRadarNode::target_list_callback(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::comtargetlistport::ComTargetListPort> & target_list_port)
{
  const auto port_header = target_list_port->GetGenericPortHeader();

  sensor_msgs::msg::PointCloud2 msg;
  RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
  const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
  const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
  msg.header.stamp.sec = sec.count();
  msg.header.stamp.nanosec = nanosec.count();

  for (const auto & target : target_list_port->GetTargetList()) {
    const auto range = target->GetRange();
    const auto elevation_angle = target->GetElevationAngle();
    const auto range_2d = range * std::cos(elevation_angle);
    const auto azimuth_angle = target->GetAzimuthAngle();
    modifier.push_back(
          {
            range_2d * std::cos(azimuth_angle),
            range_2d * std::sin(azimuth_angle),
            range * std::sin(elevation_angle),
            target->GetSpeedRadial(),
            target->GetRCS(),
            target->GetTgtNoise(),
            target->GetPower()
          });
  }

  m_publishers[sensor_idx]->publish(msg);
}

void SmartmicroRadarNode::update_config_files_from_params()
{
  const auto dev_id = declare_parameter(kDevIdTag, 0);
  if (!dev_id) {
    throw std::runtime_error("Parameter dev_id not set.");
  }
  const auto dev_port = declare_parameter(kDevPortTag, 0);
  if (!dev_port) {
    throw std::runtime_error("Parameter dev_port not set.");
  }
  const auto dev_iface_name = declare_parameter(kDevIfaceNameTag, std::string{});
  if (dev_iface_name.empty()) {
    throw std::runtime_error("Parameter dev_iface_name not set.");
  }
  const auto master_client_id = declare_parameter(kMasterClientIdTag, 0);
  if (!master_client_id) {
    throw std::runtime_error("Parameter master_client_id not set.");
  }

  auto read_sensor_params_if_possible = [&](const std::uint32_t index) {
      auto & current_sensor = m_sensors[index];
      const auto prefix = "sensors.sensor_" + std::to_string(index);
      current_sensor.id = this->declare_parameter(prefix + ".id", kDefaultClientId);
      if (current_sensor.id == kDefaultClientId) {
        // The id was not set, so the sensor with this index was not defined. Stop here.
        return false;
      }
      current_sensor.ip = this->declare_parameter(prefix + ".ip", kDefaultIp);
      current_sensor.port = this->declare_parameter(prefix + ".port", kDefaultPort);
      current_sensor.iface_name = this->declare_parameter(
        prefix + ".iface_name",
        kDefaultInterfaceName);
      current_sensor.frame_id = this->declare_parameter(prefix + ".frame_id", kDefaultFrameId);
      current_sensor.history_size = this->declare_parameter(
        prefix + ".history_size",
        kDefaultHistorySize);
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

  auto hw_inventory = nlohmann::json::parse(std::ifstream{kHwInventoryFilePath});
  auto & hw_items = hw_inventory[kHwItemsJsonTag];
  if (hw_items.empty()) {
    throw std::runtime_error("There are no 'hwItems' defined in the hw_inventory.json file.");
  }
  auto & hw_item = hw_items.front();
  hw_item[kDevIdJsonTag] = dev_id;
  hw_item[kPortTag] = dev_port;
  hw_item[kIfaceNameTag] = dev_iface_name;
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
    client[kClientIdTag] = sensor.id;
    client[kPortTag] = sensor.port;
    client[kIpTag] = sensor.ip;
    clients.push_back(client);
  }
  std::ofstream{kRoutingTableFilePath, std::ios::trunc} << routing_table;
}


}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::SmartmicroRadarNode)
