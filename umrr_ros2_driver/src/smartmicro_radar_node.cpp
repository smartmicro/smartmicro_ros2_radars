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
#include <umrr9d_t152_automotive_v1_4_1/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_4_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v1_1_1/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/GenericPortHeader.h>
#include <umrr9f_t169_automotive_v2_0_0/comtargetlistport/Target.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_1_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_2_1/comtargetlist/Target.h>
#include <umrr9f_t169_automotive_v2_4_1/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v2_4_1/comtargetlist/Target.h>
#include <umrr9f_t169_mse_v1_0_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_0_0/comobjectlist/Object.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/Target.h>

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
  float azimuth_angle{};
  float elevation_angle{};
  float range{};
  constexpr friend bool operator==(const RadarPoint & p1, const RadarPoint & p2) noexcept
  {
    return float_eq(p1.x, p2.x) && float_eq(p1.y, p2.y) && float_eq(p1.z, p2.z) &&
           float_eq(p1.radial_speed, p2.radial_speed) && float_eq(p1.power, p2.power) &&
           float_eq(p1.rcs, p2.rcs) && float_eq(p1.noise, p2.noise) && float_eq(p1.snr, p2.snr) &&
           float_eq(p1.azimuth_angle, p2.azimuth_angle) &&
           float_eq(p1.elevation_angle, p2.elevation_angle) && float_eq(p1.range, p2.range);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(radial_speed);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(power);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(rcs);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(noise);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(snr);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth_angle);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(elevation_angle);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(range);
using Generators = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_radial_speed_generator, field_power_generator,
  field_rcs_generator, field_noise_generator, field_snr_generator, field_azimuth_angle_generator,
  field_elevation_angle_generator, field_range_generator>;
using RadarCloudModifier = PointCloud2Modifier<RadarPoint, Generators>;

struct ObjectPoint
{
  float x{};
  float y{};
  float z{};
  float speed_absolute{};
  float heading{};
  float length{};
  float mileage{};
  float quality{};
  float acceleration{};
  int16_t object_id{};
  uint16_t idle_cycles{};
  uint16_t spline_idx{};
  uint8_t object_class{};
  uint16_t status{};

  constexpr friend bool operator==(const ObjectPoint & p1, const ObjectPoint & p2) noexcept
  {
    return float_eq(p1.x, p2.x) && float_eq(p1.y, p2.y) && float_eq(p1.z, p2.z) &&
           float_eq(p1.speed_absolute, p2.speed_absolute) && float_eq(p1.heading, p2.heading) &&
           float_eq(p1.length, p2.length) && float_eq(p1.mileage, p2.mileage) &&
           float_eq(p1.quality, p2.quality) && float_eq(p1.acceleration, p2.acceleration);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(speed_absolute);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(heading);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(length);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(mileage);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(quality);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(acceleration);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(object_id);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(idle_cycles);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(spline_idx);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(object_class);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(status);
using GeneratorsObjectPoint = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_speed_absolute_generator,
  field_heading_generator, field_length_generator, field_mileage_generator, field_quality_generator,
  field_acceleration_generator, field_object_id_generator, field_idle_cycles_generator,
  field_spline_idx_generator, field_object_class_generator, field_status_generator>;

using ObjectPointCloudModifier = PointCloud2Modifier<ObjectPoint, GeneratorsObjectPoint>;
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
  data_umrra4_v1_2_1 = com::master::umrra4_automotive_v1_2_1::DataStreamServiceIface::Get();
  data_umrra4_v1_0_1 = com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface::Get();
  data_umrr11 = com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface::Get();
  data_umrr96 = com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9f_v1_1_1 = com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_0_0 = com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface::Get();
  data_umrr9f_v2_1_1 = com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_2_1 = com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_4_1 = com::master::umrr9f_t169_automotive_v2_4_1::DataStreamServiceIface::Get();
  data_umrr9d_v1_0_3 = com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface::Get();
  data_umrr9d_v1_2_2 = com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9d_v1_4_1 = com::master::umrr9d_t152_automotive_v1_4_1::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_0_0 = com::master::umrr9f_t169_mse_v1_0_0::DataStreamServiceIface::Get();

  // Wait for initailization
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->get_logger(), "Data stream services have been received!");

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

  for (auto i = 0UL; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];

    if (sensor.pub_type == "mse") {
      if (sensor.model.find("mse") == std::string::npos) {
        throw std::runtime_error("Model name must contain 'mse' when pub_type is 'mse'");
      }
    } else if (sensor.pub_type == "target") {
      if (sensor.model.find("mse") != std::string::npos) {
        throw std::runtime_error("Model name must not contain 'mse' when pub_type is 'target'");
      }
    }

    if (sensor.link_type == "eth") {
      port_publishers(sensor, i);
    } else if (sensor.link_type == "can") {
      can_publishers(sensor, i);
    } else {
      RCLCPP_INFO(this->get_logger(), "Link type for sensor unknown!");
    }
  }
  rclcpp::on_shutdown(std::bind(&SmartmicroRadarNode::on_shutdown_callback, this));
}

void SmartmicroRadarNode::port_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  if (m_sensors[sensor_idx].pub_type == "mse") {
    m_publishers_obj[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/port_objects_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_port_obj_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::PortObjectHeader>(
        "smart_radar/port_objectheader_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_port_target_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
        "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
  } else if (m_sensors[sensor_idx].pub_type == "target") {
    m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_port_target_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
        "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
  } else {
    RCLCPP_INFO(this->get_logger(), "Unkwon publish type!");
  }

  RCLCPP_INFO(this->get_logger(), "Inside PORT publishers");

  if (sensor.model == "umrr9f_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register objectlist callback for sensor umrr9f_mse" << std::endl;
    }

    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f_mse" << std::endl;
    }
  }
  if (
    sensor.model == "umrr96_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr96->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr96, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr96" << std::endl;
  }
  if (
    sensor.model == "umrr11_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr11->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr11, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr11" << std::endl;
  }
  if (
    sensor.model == "umrr9f_v1_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v1_1_1->RegisterComTargetListPortReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9f_v1_1_1" << std::endl;
  }
  if (
    sensor.model == "umrr9f_v2_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_0_0->RegisterComTargetListPortReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9f_v2_0_0" << std::endl;
  }
  if (
    sensor.model == "umrr9f_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_1_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9f" << std::endl;
  }
  if (
    sensor.model == "umrr9f_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_2_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9f_v2_2_1" << std::endl;
  }
  if (
    sensor.model == "umrr9f_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_4_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9f_v2_4_1" << std::endl;
  }
  if (
    sensor.model == "umrr9d_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_0_3->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9d_v1_0_3" << std::endl;
  }
  if (
    sensor.model == "umrr9d_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_2_2->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrr9d_v1_2_2" << std::endl;
  }
  if (
    sensor.model == "umrr9d_v1_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_4_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_4_1");
    std::cout << "Failed to register targetlist callback for sensor umrr9d_v1_4_1" << std::endl;
  }
  if (
    sensor.model == "umrra4_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_0_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrra4_v1_0_1" << std::endl;
  }
  if (
    sensor.model == "umrra4_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_2_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register targetlist callback for sensor umrra4_v1_2_1" << std::endl;
  }
}

void SmartmicroRadarNode::can_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  if (m_sensors[sensor_idx].pub_type == "mse") {
    m_publishers_obj[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/can_objects_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_can_obj_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::CanObjectHeader>(
        "smart_radar/can_objectheader_" + std::to_string(sensor_idx), sensor.history_size);

    m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/can_targets_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_can_target_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
        "smart_radar/can_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
  } else if (m_sensors[sensor_idx].pub_type == "target") {
    m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "smart_radar/can_targets_" + std::to_string(sensor_idx), sensor.history_size);
    m_publishers_can_target_header[sensor_idx] =
      create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
        "smart_radar/can_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
  } else {
    RCLCPP_INFO(this->get_logger(), "Unkwon publish type!");
  }

  RCLCPP_INFO(this->get_logger(), "Inside CAN publisher");

  if (sensor.model == "umrr9f_can_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_0_0"
                << std::endl;
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      std::cout << "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_0_0"
                << std::endl;
    }
  }
  if (
    sensor.model == "umrr96_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr96->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr96, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr96" << std::endl;
  }
  if (
    sensor.model == "umrr11_can_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr11->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr11, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr11" << std::endl;
  }
  if (
    sensor.model == "umrr9f_can_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_1_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9f_v2_1_1" << std::endl;
  }
  if (
    sensor.model == "umrr9f_can_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_2_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9f_v2_2_1" << std::endl;
  }
  if (
    sensor.model == "umrr9f_can_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_4_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9f_v2_4_1" << std::endl;
  }
  if (
    sensor.model == "umrr9d_can_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_0_3->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9d_v1_0_3" << std::endl;
  }
  if (
    sensor.model == "umrr9d_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_2_2->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9d_v1_2_2" << std::endl;
  }
  if (
    sensor.model == "umrr9d_can_v1_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_4_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrr9d_v1_4_1" << std::endl;
  }
  if (
    sensor.model == "umrra4_can_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_0_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrra4_v1_0_1" << std::endl;
  }
  if (
    sensor.model == "umrra4_can_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_2_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    std::cout << "Failed to register CAN targetlist for sensor umrra4_v1_2_1" << std::endl;
  }
}

void SmartmicroRadarNode::on_shutdown_callback()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown called!");
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
      uint32_t value = resp->GetValue();
      RCLCPP_INFO(this->get_logger(), "Response from sensor to command: %i", value);
    }
  }
}

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::PortHeader> port_header;
    port_header = objectlist_port_umrr9f_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectlist::ObjectListHeader>
      object_header;
    object_header = objectlist_port_umrr9f_mse_v1_0_0->GetObjectListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();
    header.cycle_time = object_header->GetCycleTime();
    header.number_of_objects = object_header->GetNumberOfObjects();
    header.ts_measurement = object_header->GetTimestampOfMeasurement();
    for (const auto & object : objectlist_port_umrr9f_mse_v1_0_0->GetObjectList()) {
      const auto x_pos = object->GetPosX();
      const auto y_pos = object->GetPosY();
      const auto z_pos = object->GetPosZ();
      const auto speed_abs = object->GetSpeedAbs();
      const auto heading = object->GetHeading();
      const auto length = object->GetLength();
      const auto mileage = object->GetMileage();
      const auto quality = object->GetQuality();
      const auto acceleration = object->GetAcceleration();
      const auto object_id = object->GetObjectId();
      const auto idle_cycles = object->GetIdleCycles();
      const auto status = object->GetStatus();

      modifier.push_back(
        {x_pos, y_pos, z_pos, speed_abs, heading, length, mileage, quality, acceleration, object_id,
         idle_cycles, status});
    }

    m_publishers_obj[sensor_idx]->publish(msg);
    m_publishers_port_obj_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Port Targetlist for umrr9f_mse_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrr9f_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_mse_v1_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();
    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9f_mse_v1_0_0->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_2_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra4_v1_2_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrra4_v1_2_1->GetPortHeader();
    std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrra4_v1_2_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrra4_v1_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrra4_v1_0_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrra4_v1_0_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr96->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    for (const auto & target : targetlist_port_umrr96->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr11->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    for (const auto & target : targetlist_port_umrr11->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9d_v1_0_3->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9d_v1_0_3->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9d_v1_2_2->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9d_v1_2_2->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v1_1_1::comtargetlistport::StaticPortHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v1_1_1->GetStaticPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortId();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    for (const auto & target : targetlist_port_umrr9f_v1_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetTgtNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRCS(), target->GetTgtNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_0_0::comtargetlistport::StaticPortHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v2_0_0->GetStaticPortHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortId();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAnt();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweep();
    header.acquisition_cf_idx = target_header->GetAcquisitionTx();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9f_v2_0_0->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetTgtNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRCS(), target->GetTgtNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v2_1_1->GetTargetListHeader();
    umrr_ros2_msgs::msg::PortTargetHeader header;
    sensor_msgs::msg::PointCloud2 msg;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9f_v2_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v2_2_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9f_v2_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_4_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_4_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9d_v1_4_1->GetPortHeader();
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9d_v1_4_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9d_v1_4_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::targetlist_callback_umrr9f_v2_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v2_4_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f v2_4_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v2_4_1->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v2_4_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;

    header.port_identifier = port_header->GetPortIdentifier();
    header.port_ver_major = port_header->GetPortVersionMajor();
    header.port_ver_minor = port_header->GetPortVersionMinor();
    header.port_size = port_header->GetPortSize();
    header.body_endianness = port_header->GetBodyEndianness();
    header.port_index = port_header->GetPortIndex();
    header.header_ver_major = port_header->GetHeaderVersionMajor();
    header.header_ver_minor = port_header->GetHeaderVersionMinor();

    header.cycle_time = target_header->GetCycleTime();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_tx_ant_idx = target_header->GetAcquisitionTxAntIdx();
    header.acquisition_sweep_idx = target_header->GetAcquisitionSweepIdx();
    header.acquisition_cf_idx = target_header->GetAcquisitionCfIdx();
    header.prf = target_header->GetPrf();
    header.umambiguous_speed = target_header->GetUmambiguousSpeed();
    header.acquisition_start = target_header->GetAcquisitionStart();
    for (const auto & target : targetlist_port_umrr9f_v2_4_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetPower() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetPower(),
         target->GetRcs(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_port_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_can_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::PortHeader> port_header;
    port_header = objectlist_can_umrr9f_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comobjectbaselist::ComObjectBaseListHeader>
      object_header;
    object_header = objectlist_can_umrr9f_mse_v1_0_0->GetComObjectBaseListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = object_header->GetCycleDuration();
    header.cycle_count = object_header->GetCycleCount();
    header.number_of_objects = object_header->GetNoOfObjects();
    header.ego_speed = object_header->GetSpeed();
    header.ego_speed_quality = object_header->GetSpeedQuality();
    header.ego_yaw_rate = object_header->GetYawRate();
    header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
    header.dyn_source = object_header->GetDynamicSource();
    for (const auto & object : objectlist_can_umrr9f_mse_v1_0_0->GetObjectList()) {
      const auto x_pos = object->GetXPoint1();
      const auto y_pos = object->GetYPoint1();
      const auto z_pos = object->GetZPoint1();
      const auto speed_abs = object->GetSpeedAbs();
      const auto heading = object->GetHeadingDeg();
      const auto length = object->GetObjectLen();
      const auto quality = object->GetQuality();
      const auto acceleration = object->GetAcceleration();
      const auto object_id = static_cast<float>(object->GetObjectId());
      modifier.push_back(
        {x_pos, y_pos, z_pos, speed_abs, heading, length, quality, acceleration, object_id});
    }

    m_publishers_obj[sensor_idx]->publish(msg);
    m_publishers_can_obj_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_mse_can_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::PortHeader> port_header;
    port_header = targetlist_can_umrr9f_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_0_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_mse_v1_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrr9f_mse_v1_0_0->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_2_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_2_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_2_1,
  const com::types::ClientId client_id)
{
  std::cout << "CAN Targetlist for umrra4_v1_2_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrra4_v1_2_1->GetPortHeader();
    std::shared_ptr<com::master::umrra4_automotive_v1_2_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrra4_v1_2_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrra4_v1_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrra4_automotive_v1_0_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrra4_v1_0_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrra4_v1_0_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr96_t153_automotive_v1_2_2::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr96->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr96->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr11_t132_automotive_v1_1_2::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr11->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr11->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_0_3::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9d_v1_0_3->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9d_v1_0_3->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_2_2::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9d_v1_2_2->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9d_v1_2_2->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_1_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_v2_1_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9f_v2_1_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_2_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_v2_2_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9f_v2_2_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_4_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_4_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9d_v1_4_1->GetPortHeader();
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_4_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9d_v1_4_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9d_v1_4_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
  }
}

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_4_1(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v2_4_1,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v2_4_1" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9f_v2_4_1->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_automotive_v2_4_1::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_v2_4_1->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto timestamp = std::chrono::microseconds{port_header->GetTimestamp()};
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    const auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp - sec);
    msg.header.stamp.sec = sec.count();
    msg.header.stamp.nanosec = nanosec.count();
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9f_v2_4_1->GetTargetList()) {
      const auto range = target->GetRange();
      const auto elevation_angle = target->GetElevationAngle();
      const auto range_2d = range * std::cos(elevation_angle);
      const auto azimuth_angle = target->GetAzimuthAngle();
      const auto snr = target->GetSignalLevel() - target->GetNoise();
      modifier.push_back(
        {range_2d * std::cos(azimuth_angle), range_2d * std::sin(azimuth_angle),
         range * std::sin(elevation_angle), target->GetSpeedRadial(), target->GetSignalLevel(),
         target->GetRCS(), target->GetNoise(), snr, azimuth_angle, elevation_angle, range});
    }

    m_publishers[sensor_idx]->publish(msg);
    m_publishers_can_target_header[sensor_idx]->publish(header);
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
    sensor.pub_type = this->declare_parameter(prefix_3 + ".pub_type", "");
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
    client[kClientIdTag] = sensor.id;
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
