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

#include <signal.h>

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

#include <nlohmann/json.hpp>
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
#include <umrr9d_t152_automotive_v1_5_0/comtargetlist/PortHeader.h>
#include <umrr9d_t152_automotive_v1_5_0/comtargetlist/Target.h>
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
#include <umrr9f_t169_automotive_v3_0_0/comtargetlist/PortHeader.h>
#include <umrr9f_t169_automotive_v3_0_0/comtargetlist/Target.h>
#include <umrr9f_t169_mse_v1_0_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_0_0/comobjectlist/Object.h>
#include <umrr9f_t169_mse_v1_1_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_1_0/comobjectlist/Object.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/ComObjectList.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/Object.h>
#include <umrra1_t166_b_automotive_v1_0_0/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v1_0_0/comtargetlist/Target.h>
#include <umrra1_t166_b_automotive_v2_0_0/comtargetlist/PortHeader.h>
#include <umrra1_t166_b_automotive_v2_0_0/comtargetlist/Target.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_0_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_2_1/comtargetlist/Target.h>
#include <umrra4_automotive_v1_4_0/comtargetlist/PortHeader.h>
#include <umrra4_automotive_v1_4_0/comtargetlist/Target.h>
#include <umrra4_mse_v1_0_0/comobjectlist/ComObjectList.h>
#include <umrra4_mse_v1_0_0/comobjectlist/Object.h>
#include <umrra4_mse_v2_1_0/comobjectlist/ComObjectList.h>
#include <umrra4_mse_v2_1_0/comobjectlist/Object.h>

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
using std::literals::string_view_literals::operator""sv;

namespace
{
constexpr auto kMseType = "mse"sv;
constexpr auto kTargetType = "target"sv;
constexpr auto kEthLink = "eth"sv;
constexpr auto kCanLink = "can"sv;

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
  update_service = std::make_shared<UpdateService>();

  const auto override = false;
  setenv("SMART_ACCESS_CFG_FILE_PATH", kConfigFilePath, override);

  initialize_services();
  setup_publishers();

  rclcpp::on_shutdown(std::bind(&SmartmicroRadarNode::on_shutdown_callback, this));
}

void SmartmicroRadarNode::initialize_services()
{
  // Getting the communication services
  m_services = CommunicationServicesIface::Get();
  if (!m_services->Init()) {
    throw std::runtime_error("Communication Service initialization failed");
  }

  // Getting the data stream service
  data_umrra4_v1_0_1 = com::master::umrra4_automotive_v1_0_1::DataStreamServiceIface::Get();
  data_umrra4_v1_2_1 = com::master::umrra4_automotive_v1_2_1::DataStreamServiceIface::Get();
  data_umrra4_v1_4_0 = com::master::umrra4_automotive_v1_4_0::DataStreamServiceIface::Get();
  data_umrr11 = com::master::umrr11_t132_automotive_v1_1_2::DataStreamServiceIface::Get();
  data_umrr96 = com::master::umrr96_t153_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9f_v1_1_1 = com::master::umrr9f_t169_automotive_v1_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_0_0 = com::master::umrr9f_t169_automotive_v2_0_0::DataStreamServiceIface::Get();
  data_umrr9f_v2_1_1 = com::master::umrr9f_t169_automotive_v2_1_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_2_1 = com::master::umrr9f_t169_automotive_v2_2_1::DataStreamServiceIface::Get();
  data_umrr9f_v2_4_1 = com::master::umrr9f_t169_automotive_v2_4_1::DataStreamServiceIface::Get();
  data_umrr9f_v3_0_0 = com::master::umrr9f_t169_automotive_v3_0_0::DataStreamServiceIface::Get();
  data_umrr9d_v1_0_3 = com::master::umrr9d_t152_automotive_v1_0_3::DataStreamServiceIface::Get();
  data_umrr9d_v1_2_2 = com::master::umrr9d_t152_automotive_v1_2_2::DataStreamServiceIface::Get();
  data_umrr9d_v1_4_1 = com::master::umrr9d_t152_automotive_v1_4_1::DataStreamServiceIface::Get();
  data_umrr9d_v1_5_0 = com::master::umrr9d_t152_automotive_v1_5_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_0_0 = com::master::umrr9f_t169_mse_v1_0_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_1_0 = com::master::umrr9f_t169_mse_v1_1_0::DataStreamServiceIface::Get();
  data_umrr9f_mse_v1_3_0 = com::master::umrr9f_t169_mse_v1_3_0::DataStreamServiceIface::Get();
  data_umrra4_mse_v1_0_0 = com::master::umrra4_mse_v1_0_0::DataStreamServiceIface::Get();
  data_umrra4_mse_v2_1_0 = com::master::umrra4_mse_v2_1_0::DataStreamServiceIface::Get();
  data_umrra1_v1_0_0 = com::master::umrra1_t166_b_automotive_v1_0_0::DataStreamServiceIface::Get();
  data_umrra1_v2_0_0 = com::master::umrra1_t166_b_automotive_v2_0_0::DataStreamServiceIface::Get();

  // Wait for initailization
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(this->get_logger(), "Data stream services have been received!");

  // create a ros2 service to change the radar parameters
  mode_srv_ = create_service<umrr_ros2_msgs::srv::SetMode>(
    "smart_radar/set_radar_mode",
    std::bind(
      &SmartmicroRadarNode::set_radar_mode, this, std::placeholders::_1, std::placeholders::_2));

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

  // create a ros2 service to read the radar status
  status_srv_ = create_service<umrr_ros2_msgs::srv::GetStatus>(
    "smart_radar/get_radar_status",
    std::bind(
      &SmartmicroRadarNode::get_radar_status, this, std::placeholders::_1, std::placeholders::_2));

  // create a ros2 service to read the radar modes
  read_mode_srv_ = create_service<umrr_ros2_msgs::srv::GetMode>(
    "smart_radar/get_radar_mode",
    std::bind(
      &SmartmicroRadarNode::get_radar_mode, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Radar services are ready.");
}

void SmartmicroRadarNode::setup_publishers()
{
  const auto validate_sensor = [](const auto & sensor) {
    std::string_view pub_type{sensor.pub_type};
    std::string_view model{sensor.model};

    const bool is_mse = (pub_type == kMseType);
    const bool has_mse = (model.find(kMseType) != std::string_view::npos);

    if (is_mse == !has_mse) {
      throw std::runtime_error(
        std::string("Model name ") + (is_mse ? "must" : "must not") +
        " contain 'mse' when pub_type is '" + std::string(pub_type) + "'");
    }
  };

  for (size_t i = 0; i < m_number_of_sensors; ++i) {
    const auto & sensor = m_sensors[i];

    validate_sensor(sensor);

    std::string_view link_type{sensor.link_type};
    if (link_type == kEthLink) {
      port_publishers(sensor, i);
    } else if (link_type == kCanLink) {
      can_publishers(sensor, i);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown link type for sensor %zu", i);
    }
  }
}

void SmartmicroRadarNode::port_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  std::string_view pub_type{m_sensors[sensor_idx].pub_type};

  try {
    if (pub_type == kMseType) {
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

    } else if (pub_type == kTargetType) {
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/port_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_port_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::PortTargetHeader>(
          "smart_radar/port_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown publish type: %s", sensor.pub_type.c_str());
      throw std::invalid_argument("Unknown publish type");
    }

    RCLCPP_INFO(get_logger(), "Successfully created PORT publishers for sensor %zu", sensor_idx);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create publishers for sensor %zu: %s", sensor_idx, e.what());
    throw;
  }

  if (sensor.model == "umrra4_mse_v2_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrra4_mse_v2_1_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrra4_mse_v2_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_mse_v2_1_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_mse_v2_1_0");
    }
  }
  if (sensor.model == "umrra4_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrra4_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrra4_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrra4_mse_v1_0_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_3_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_3_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_3_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_3_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_3_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_1_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_1_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_1_0");
    }
  }
  if (sensor.model == "umrr9f_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register objectlist callback for sensor umrr9f_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_mse_v1_0_0");
    }
  }
  if (
    sensor.model == "umrr96_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr96->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr96, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr96_v1_2_2");
  }
  if (
    sensor.model == "umrr11_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr11->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr11, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr11_v1_1_2");
  }
  if (
    sensor.model == "umrr9f_v1_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v1_1_1->RegisterComTargetListPortReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v1_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v1_1_1");
  }
  if (
    sensor.model == "umrr9f_v2_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_0_0->RegisterComTargetListPortReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_0_0");
  }
  if (
    sensor.model == "umrr9f_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_1_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_1_1");
  }
  if (
    sensor.model == "umrr9f_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_2_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_2_1");
  }
  if (
    sensor.model == "umrr9f_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_4_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v2_4_1");
  }
  if (
    sensor.model == "umrr9f_v3_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v3_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9f_v3_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9f_v3_0_0");
  }
  if (
    sensor.model == "umrr9d_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_0_3->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_0_3");
  }
  if (
    sensor.model == "umrr9d_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_2_2->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_2_2");
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
  }
  if (
    sensor.model == "umrr9d_v1_5_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_5_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrr9d_v1_5_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrr9d_v1_5_0");
  }
  if (
    sensor.model == "umrra4_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_0_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_0_1");
  }
  if (
    sensor.model == "umrra4_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_2_1->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_2_1");
  }
  if (
    sensor.model == "umrra4_v1_4_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_4_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra4_v1_4_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra4_v1_4_0");
  }
  if (
    sensor.model == "umrra1_v1_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrra1_v1_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra1_v1_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v1_0_0");
  }
  if (
    sensor.model == "umrra1_v2_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrra1_v2_0_0->RegisterComTargetListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register targetlist callback for sensor umrra1_v2_0_0");
  }
}

void SmartmicroRadarNode::can_publishers(const detail::SensorConfig & sensor, size_t sensor_idx)
{
  std::string_view pub_type{m_sensors[sensor_idx].pub_type};

  try {
    if (pub_type == kMseType) {
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
    } else if (pub_type == kTargetType) {
      m_publishers[sensor_idx] = create_publisher<sensor_msgs::msg::PointCloud2>(
        "smart_radar/can_targets_" + std::to_string(sensor_idx), sensor.history_size);
      m_publishers_can_target_header[sensor_idx] =
        create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
          "smart_radar/can_targetheader_" + std::to_string(sensor_idx), sensor.history_size);
    } else {
      RCLCPP_INFO(this->get_logger(), "Unkwon publish type!");
    }

    RCLCPP_INFO(get_logger(), "Successfully created CAN publishers for sensor %zu", sensor_idx);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create publishers for sensor %zu: %s", sensor_idx, e.what());
    throw;
  }

  if (sensor.model == "umrra4_can_mse_v2_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v2_1_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrra4_can_mse_v2_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v2_1_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v2_1_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrra4_can_mse_v2_1_0");
    }
  }
  if (sensor.model == "umrra4_can_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrra4_can_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrra4_mse_v1_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrra4_can_mse_v1_0_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_3_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_3_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_3_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_3_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_3_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_3_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_1_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_1_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_1_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_1_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_1_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_1_0");
    }
  }
  if (sensor.model == "umrr9f_can_mse_v1_0_0") {
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComObjectBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register objectlist callback for sensor umrr9f_can_mse_v1_0_0");
    }
    if (
      com::types::ERROR_CODE_OK !=
      data_umrr9f_mse_v1_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_0_0, this,
                     sensor_idx, std::placeholders::_1, std::placeholders::_2))) {
      RCLCPP_INFO(
        this->get_logger(),
        "Failed to register targetlist callback for sensor umrr9f_can_mse_v1_0_0");
    }
  }
  if (
    sensor.model == "umrr96_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr96->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr96, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr96_can_v1_2_2");
  }
  if (
    sensor.model == "umrr11_can_v1_1_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr11->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr11, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr11_can_v1_1_2");
  }
  if (
    sensor.model == "umrr9f_can_v2_1_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_1_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_1_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_1_1");
  }
  if (
    sensor.model == "umrr9f_can_v2_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_2_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_2_1");
  }
  if (
    sensor.model == "umrr9f_can_v2_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v2_4_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v2_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v2_4_1");
  }
  if (
    sensor.model == "umrr9f_can_v3_0_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9f_v3_0_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_0_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9f_can_v3_0_0");
  }
  if (
    sensor.model == "umrr9d_can_v1_0_3" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_0_3->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_0_3, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_0_3");
  }
  if (
    sensor.model == "umrr9d_can_v1_2_2" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_2_2->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_2_2, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_2_2");
  }
  if (
    sensor.model == "umrr9d_can_v1_4_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_4_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_4_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_4_1");
  }
  if (
    sensor.model == "umrr9d_can_v1_5_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrr9d_v1_5_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_5_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrr9d_can_v1_5_0");
  }
  if (
    sensor.model == "umrra4_can_v1_0_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_0_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_0_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_0_1");
  }
  if (
    sensor.model == "umrra4_can_v1_2_1" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_2_1->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_2_1, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_2_1");
  }
  if (
    sensor.model == "umrra4_can_v1_4_0" &&
    com::types::ERROR_CODE_OK !=
      data_umrra4_v1_4_0->RegisterComTargetBaseListReceiveCallback(
        sensor.id, std::bind(
                     &SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_4_0, this, sensor_idx,
                     std::placeholders::_1, std::placeholders::_2))) {
    RCLCPP_INFO(
      this->get_logger(), "Failed to register CAN targetlist for sensor umrra4_can_v1_4_0");
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

  update_service->StartSoftwareUpdate(client_id, update_image);
  result->res = "Service ended, check the console output for status! ";
}

void SmartmicroRadarNode::set_radar_mode(
  const std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::SetMode::Response> result)
{
  // Validate sensor ID
  bool check_flag_id = false;
  client_id = request->sensor_id;
  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_0dim" && section_name != "auto_interface_rrm" &&
    section_name != "Parameter") {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_0dim', "
      "'auto_interface_rrm', or 'Parameter'.";
    return;
  }

  // Check arrays have same length
  if (
    request->params.size() != request->values.size() ||
    request->params.size() != request->value_types.size()) {
    result->res = "Error: param, values and value_types arrays must have same length";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->params.size(); i++) {
    const auto & param = request->params[i];
    const auto & value = request->values[i];
    const auto & value_type = request->value_types[i];
    bool request_added = false;
    try {
      switch (value_type) {
        case 0: {
          float float_value = std::stof(value);
          auto radar_mode_float =
            std::make_shared<SetParamRequest<float>>(section_name, param, float_value);
          request_added = batch->AddRequest(radar_mode_float);
          break;
        }
        case 1: {
          if (value.find('.') != std::string::npos) {
            result->res = "Error: uint32 value cannot contain decimal points";
            return;
          }
          uint32_t u32_value = static_cast<uint32_t>(std::stoul(value));
          auto radar_mode_u32 =
            std::make_shared<SetParamRequest<uint32_t>>(section_name, param, u32_value);
          request_added = batch->AddRequest(radar_mode_u32);
          break;
        }
        case 2: {
          if (value.find('.') != std::string::npos) {
            result->res = "Error: uint16 value cannot contain decimal points";
            return;
          }
          uint64_t temp = std::stoul(value);
          if (temp > 65535) {
            result->res = "Error: uint16 value must be between 0 and 65535";
            return;
          }
          uint16_t u16_value = static_cast<uint16_t>(temp);
          auto radar_mode_u16 =
            std::make_shared<SetParamRequest<uint16_t>>(section_name, param, u16_value);
          request_added = batch->AddRequest(radar_mode_u16);
          break;
        }
        case 3: {
          if (value.find('.') != std::string::npos) {
            result->res = "Error: uint8 value cannot contain decimal points";
            return;
          }
          uint64_t temp = std::stoul(value);
          if (temp > 255) {
            result->res = "Error: uint8 value must be between 0 and 255";
            return;
          }
          uint8_t u8_value = static_cast<uint8_t>(temp);
          auto radar_mode_u8 =
            std::make_shared<SetParamRequest<uint8_t>>(section_name, param, u8_value);
          request_added = batch->AddRequest(radar_mode_u8);
          break;
        }
        default:
          result->res =
            "Error: Invalid value_type specified. Must be 0 (f32), 1 (u32), 2 (u16), 3 (u8)";
          return;
      }
    } catch (const std::invalid_argument & e) {
      result->res = "Error: Failed to convert value string, invalid format";
      return;
    } catch (const std::out_of_range & e) {
      result->res = "Error: Value is out of range for the specified type";
      return;
    }

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + param + "'! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, std::bind(
               &SmartmicroRadarNode::mode_response, this, client_id, std::placeholders::_2,
               request->params, section_name))) {
    result->res = "Error: Check params are valid for this sensor and values within range!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
  RCLCPP_INFO(this->get_logger(), "Service call result: %s", result->res.c_str());
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
    result->res_ip = "Failed to allocate instruction! ";
    return;
  }

  std::shared_ptr<SetParamRequest<uint32_t>> ip_address =
    std::make_shared<SetParamRequest<uint32_t>>(
      "auto_interface_0dim", "ip_source_address", request->value_ip);

  std::shared_ptr<CmdRequest> cmd =
    std::make_shared<CmdRequest>("auto_interface_command", "comp_eeprom_ctrl_save_param_sec", 2010);

  if (!batch->AddRequest(ip_address)) {
    result->res_ip = "Failed to add instruction! ";
    return;
  }
  if (!batch->AddRequest(cmd)) {
    result->res_ip = "Failed to add instruction! ";
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
    result->res_ip =
      "Success: IP change executed successfully. Radar must be restarted "
      "and the parameters in the param file must be updated";
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

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_command" && section_name != "auto_interface_rrm_command" &&
    section_name != "Command") {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_command', "
      "'auto_interface_rrm_command', or 'Command'.";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;

  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Failed to allocate instruction! ";
    return;
  }

  std::shared_ptr<CmdRequest> radar_command =
    std::make_shared<CmdRequest>(section_name, request->command, request->value);

  if (!batch->AddRequest(radar_command)) {
    result->res = "Failed to add instruction! ";
    return;
  }

  if (
    com::types::ERROR_CODE_OK != inst->SendInstructionBatch(
                                   batch, std::bind(
                                            &SmartmicroRadarNode::command_response, this, client_id,
                                            std::placeholders::_2, command_name, section_name))) {
    result->res = "Error in sending command to the sensor!";
    return;
  }
  result->res = "Success: Request sent successfully.";
}

void SmartmicroRadarNode::get_radar_status(
  const std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::GetStatus::Response> result)
{
  // Validate sensor ID
  bool check_flag_id = false;
  client_id = request->sensor_id;
  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface" && section_name != "auto_interface_rrm" &&
    section_name != "Status") {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface', 'auto_interface_rrm', or "
      "'Status'.";
    return;
  }

  // Check arrays have same length
  if (request->statuses.size() != request->status_types.size()) {
    result->res = "Error: status and status_types arrays must have same length";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->statuses.size(); i++) {
    const auto & status = request->statuses[i];
    const auto & status_type = request->status_types[i];
    bool request_added = false;

    switch (status_type) {
      case 0: {
        auto radar_status_u32 = std::make_shared<GetStatusRequest<uint32_t>>(section_name, status);
        request_added = batch->AddRequest(radar_status_u32);
        break;
      }
      case 1: {
        auto radar_status_u16 = std::make_shared<GetStatusRequest<uint16_t>>(section_name, status);
        request_added = batch->AddRequest(radar_status_u16);
        break;
      }
      default:
        result->res = "Error: Invalid value_type specified. Must be 0 (u32) or 1 (u8)";
        return;
    }

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + status + "' ! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, std::bind(
               &SmartmicroRadarNode::status_response, this, client_id, std::placeholders::_2,
               request->statuses, section_name))) {
    result->res = "Error: Check status are valid for this sensor!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
}

void SmartmicroRadarNode::get_radar_mode(
  const std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Request> request,
  std::shared_ptr<umrr_ros2_msgs::srv::GetMode::Response> result)
{
  // Validate sensor ID
  bool check_flag_id = false;
  client_id = request->sensor_id;
  for (auto & sensor : m_sensors) {
    if (client_id == sensor.id) {
      check_flag_id = true;
      break;
    }
  }
  if (!check_flag_id) {
    result->res = "Error: Sensor ID is invalid! ";
    return;
  }

  auto section_name = request->section_name;
  if (
    section_name != "auto_interface_0dim" && section_name != "auto_interface_rrm" &&
    section_name != "Parameter") {
    result->res =
      "Error: Invalid section name specified! Must be 'auto_interface_0dim', 'auto_interface_rrm', "
      "or 'Parameter'.";
    return;
  }

  // Check arrays have same length
  if (request->params.size() != request->param_types.size()) {
    result->res = "Error: param and value_types arrays must have same length";
    return;
  }

  std::shared_ptr<InstructionServiceIface> inst{m_services->GetInstructionService()};
  if (!inst) {
    result->res = "Error: Failed to get instruction service";
    return;
  }

  timer = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&SmartmicroRadarNode::my_timer_callback, this));

  std::shared_ptr<InstructionBatch> batch;
  if (!inst->AllocateInstructionBatch(client_id, batch)) {
    result->res = "Error: Failed to allocate instruction! ";
    return;
  }

  for (size_t i = 0; i < request->params.size(); i++) {
    const auto & param = request->params[i];
    const auto & param_type = request->param_types[i];
    bool request_added = false;

    switch (param_type) {
      case 0: {
        auto radar_param_float = std::make_shared<GetParamRequest<float>>(section_name, param);
        request_added = batch->AddRequest(radar_param_float);
        break;
      }
      case 1: {
        auto radar_param_u32 = std::make_shared<GetParamRequest<uint32_t>>(section_name, param);
        request_added = batch->AddRequest(radar_param_u32);
        break;
      }
      case 2: {
        auto radar_param_u16 = std::make_shared<GetParamRequest<uint16_t>>(section_name, param);
        request_added = batch->AddRequest(radar_param_u16);
        break;
      }
      case 3: {
        auto radar_param_u8 = std::make_shared<GetParamRequest<uint8_t>>(section_name, param);
        request_added = batch->AddRequest(radar_param_u8);
        break;
      }
      default:
        result->res =
          "Error: Invalid value_type specified. Must be 0(u32), 1(u16), 2(u8) or 3(float)";
        return;
    }

    if (!request_added) {
      result->res = "Error: Failed to add instruction '" + param + "' Check param types match! ";
      return;
    }
  }

  if (
    com::types::ERROR_CODE_OK !=
    inst->SendInstructionBatch(
      batch, std::bind(
               &SmartmicroRadarNode::param_response, this, client_id, std::placeholders::_2,
               request->params, section_name))) {
    result->res = "Error: Check params are valid for this sensor!";
    return;
  }
  result->res = "Success: Request sent successfully. Check main terminal for sensor response!";
}

void SmartmicroRadarNode::mode_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & instruction_names, const std::string & section_name)
{
  for (const auto & instruction_name : instruction_names) {
    std::vector<std::shared_ptr<Response<uint8_t>>> resp_u8;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    std::vector<std::shared_ptr<Response<float>>> resp_f;
    bool response_found = false;

    if (response->GetResponse<uint8_t>(section_name, instruction_name.c_str(), resp_u8)) {
      response_found = true;
      for (auto & resp : resp_u8) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<float>(section_name, instruction_name.c_str(), resp_f)) {
      response_found = true;
      for (auto & resp : resp_f) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %f\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::sensor_response_ip(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> resp_ip;
  if (response->GetResponse<uint32_t>("auto_interface_0dim", "ip_source_address", resp_ip)) {
    for (auto & resp : resp_ip) {
      RCLCPP_INFO(
        this->get_logger(),
        "Response details:\n"
        "   Instruction: %s\n"
        "   Response type: %u\n"
        "   Value: %u\n",
        resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
    }
  }
}

void SmartmicroRadarNode::command_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response, const std::string command_name,
  const std::string & section_name)
{
  std::vector<std::shared_ptr<Response<uint32_t>>> command_resp;
  if (response->GetResponse<uint32_t>(section_name, command_name.c_str(), command_resp)) {
    for (auto & resp : command_resp) {
      RCLCPP_INFO(
        this->get_logger(),
        "Response details:\n"
        "   Instruction: %s\n"
        "   Response type: %u\n"
        "   Value: %u\n",
        resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
    }
  }
}

void SmartmicroRadarNode::status_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & statuses, const std::string & section_name)
{
  for (const auto & instruction_name : statuses) {
    std::vector<std::shared_ptr<Response<uint16_t>>> resp_u16;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    bool response_found = false;

    if (response->GetResponse<uint16_t>(section_name, instruction_name.c_str(), resp_u16)) {
      response_found = true;
      for (auto & resp : resp_u16) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::param_response(
  const com::types::ClientId client_id,
  const std::shared_ptr<com::master::ResponseBatch> & response,
  const std::vector<std::string> & statuses, const std::string & section_name)
{
  for (const auto & instruction_name : statuses) {
    std::vector<std::shared_ptr<Response<uint16_t>>> resp_u16;
    std::vector<std::shared_ptr<Response<uint32_t>>> resp_u32;
    std::vector<std::shared_ptr<Response<uint8_t>>> resp_u8;
    std::vector<std::shared_ptr<Response<float>>> resp_f;
    bool response_found = false;

    if (response->GetResponse<uint16_t>(section_name, instruction_name.c_str(), resp_u16)) {
      response_found = true;
      for (auto & resp : resp_u16) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint32_t>(section_name, instruction_name.c_str(), resp_u32)) {
      response_found = true;
      for (auto & resp : resp_u32) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<uint8_t>(section_name, instruction_name.c_str(), resp_u8)) {
      response_found = true;
      for (auto & resp : resp_u8) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %u\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (response->GetResponse<float>(section_name, instruction_name.c_str(), resp_f)) {
      response_found = true;
      for (auto & resp : resp_f) {
        RCLCPP_INFO(
          this->get_logger(),
          "Response details:\n"
          "   Instruction: %s\n"
          "   Response type: %u\n"
          "   Value: %f\n",
          resp->GetInstructionName().c_str(), resp->GetResponseType(), resp->GetValue());
      }
    }

    if (!response_found) {
      RCLCPP_WARN(this->get_logger(), "No response received!");
    }
  }
}

void SmartmicroRadarNode::objectlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::ComObjectList> &
    objectlist_port_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrra4_mse_v2_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::PortHeader> port_header;
    port_header = objectlist_port_umrra4_mse_v2_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectlist::ObjectListHeader> object_header;
    object_header = objectlist_port_umrra4_mse_v2_1_0->GetObjectListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & object : objectlist_port_umrra4_mse_v2_1_0->GetObjectList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Port Targetlist for umrra4_mse_v2_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrra4_mse_v2_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetlist::TargetListHeader> target_header;
    target_header = targetlist_port_umrra4_mse_v2_1_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrra4_mse_v2_1_0->GetTargetList()) {
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

void SmartmicroRadarNode::objectlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::ComObjectList> &
    objectlist_port_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrra4_mse_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::PortHeader> port_header;
    port_header = objectlist_port_umrra4_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectlist::ObjectListHeader> object_header;
    object_header = objectlist_port_umrra4_mse_v1_0_0->GetObjectListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & object : objectlist_port_umrra4_mse_v1_0_0->GetObjectList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Port Targetlist for umrra4_mse_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrra4_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetlist::TargetListHeader> target_header;
    target_header = targetlist_port_umrra4_mse_v1_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrra4_mse_v1_0_0->GetTargetList()) {
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

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_v1_3_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::PortHeader> port_header;
    port_header = objectlist_port_umrr9f_mse_v1_3_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectlist::ObjectListHeader>
      object_header;
    object_header = objectlist_port_umrr9f_mse_v1_3_0->GetObjectListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & object : objectlist_port_umrr9f_mse_v1_3_0->GetObjectList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::cout << "Port Targetlist for umrr9f_mse_v1_3_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrr9f_mse_v1_3_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_mse_v1_3_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrr9f_mse_v1_3_0->GetTargetList()) {
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

void SmartmicroRadarNode::objectlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::ComObjectList> &
    objectlist_port_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_v1_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::PortHeader> port_header;
    port_header = objectlist_port_umrr9f_mse_v1_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectlist::ObjectListHeader>
      object_header;
    object_header = objectlist_port_umrr9f_mse_v1_1_0->GetObjectListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & object : objectlist_port_umrr9f_mse_v1_1_0->GetObjectList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Port Targetlist for umrr9f_mse_v1_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrr9f_mse_v1_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_mse_v1_1_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrr9f_mse_v1_1_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::targetlist_callback_umrr9f_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9f_v3_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f v3_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9f_v3_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9f_v3_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrr9f_v3_0_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::targetlist_callback_umrr9d_v1_5_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::ComTargetList> &
    targetlist_port_umrr9d_v1_5_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_5_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrr9d_v1_5_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrr9d_v1_5_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrr9d_v1_5_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::targetlist_callback_umrra4_v1_4_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra4_v1_4_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra4_v1_4_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::PortHeader> port_header;
    port_header = targetlist_port_umrra4_v1_4_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrra4_v1_4_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrra4_v1_4_0->GetTargetList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrra1_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra1_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra1_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrra1_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra1_t166_b_automotive_v1_0_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrra1_v1_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrra1_v1_0_0->GetTargetList()) {
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

void SmartmicroRadarNode::targetlist_callback_umrra1_v2_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::ComTargetList> &
    targetlist_port_umrra1_v2_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra1_v2_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::PortHeader>
      port_header;
    port_header = targetlist_port_umrra1_v2_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra1_t166_b_automotive_v2_0_0::comtargetlist::TargetListHeader>
      target_header;
    target_header = targetlist_port_umrra1_v2_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::PortTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    for (const auto & target : targetlist_port_umrra1_v2_0_0->GetTargetList()) {
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

void SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrra4_mse_can_v2_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::PortHeader> port_header;
    port_header = objectlist_can_umrra4_mse_v2_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comobjectbaselist::ComObjectBaseListHeader>
      object_header;
    object_header = objectlist_can_umrra4_mse_v2_1_0->GetComObjectBaseListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = object_header->GetCycleDuration();
    header.cycle_count = object_header->GetCycleCount();
    header.number_of_objects = object_header->GetNoOfObjects();
    header.ego_speed = object_header->GetSpeed();
    header.ego_speed_quality = object_header->GetSpeedQuality();
    header.ego_yaw_rate = object_header->GetYawRate();
    header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
    header.dyn_source = object_header->GetDynamicSource();
    for (const auto & object : objectlist_can_umrra4_mse_v2_1_0->GetObjectList()) {
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v2_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_mse_v2_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra4_mse_can_v2_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::PortHeader> port_header;
    port_header = targetlist_can_umrra4_mse_v2_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v2_1_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrra4_mse_v2_1_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrra4_mse_v2_1_0->GetTargetList()) {
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

void SmartmicroRadarNode::CAN_objectlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrra4_mse_can_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::PortHeader> port_header;
    port_header = objectlist_can_umrra4_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comobjectbaselist::ComObjectBaseListHeader>
      object_header;
    object_header = objectlist_can_umrra4_mse_v1_0_0->GetComObjectBaseListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = object_header->GetCycleDuration();
    header.cycle_count = object_header->GetCycleCount();
    header.number_of_objects = object_header->GetNoOfObjects();
    header.ego_speed = object_header->GetSpeed();
    header.ego_speed_quality = object_header->GetSpeedQuality();
    header.ego_yaw_rate = object_header->GetYawRate();
    header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
    header.dyn_source = object_header->GetDynamicSource();
    for (const auto & object : objectlist_can_umrra4_mse_v1_0_0->GetObjectList()) {
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_mse_v1_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_mse_v1_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrra4_mse_can_v1_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::PortHeader> port_header;
    port_header = targetlist_can_umrra4_mse_v1_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_mse_v1_0_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrra4_mse_v1_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrra4_mse_v1_0_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_can_v1_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::PortHeader> port_header;
    port_header = objectlist_can_umrr9f_mse_v1_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comobjectbaselist::ComObjectBaseListHeader>
      object_header;
    object_header = objectlist_can_umrr9f_mse_v1_1_0->GetComObjectBaseListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = object_header->GetCycleDuration();
    header.cycle_count = object_header->GetCycleCount();
    header.number_of_objects = object_header->GetNoOfObjects();
    header.ego_speed = object_header->GetSpeed();
    header.ego_speed_quality = object_header->GetSpeedQuality();
    header.ego_yaw_rate = object_header->GetYawRate();
    header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
    header.dyn_source = object_header->GetDynamicSource();
    for (const auto & object : objectlist_can_umrr9f_mse_v1_1_0->GetObjectList()) {
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_1_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_1_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_mse_can_v1_1_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::PortHeader> port_header;
    port_header = targetlist_can_umrr9f_mse_v1_1_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_1_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_mse_v1_1_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrr9f_mse_v1_1_0->GetTargetList()) {
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

void SmartmicroRadarNode::CAN_objectlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::ComObjectBaseList> &
    objectlist_can_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::cout << "Objectlist for umrr9f_mse_can_v1_3_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::PortHeader> port_header;
    port_header = objectlist_can_umrr9f_mse_v1_3_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comobjectbaselist::ComObjectBaseListHeader>
      object_header;
    object_header = objectlist_can_umrr9f_mse_v1_3_0->GetComObjectBaseListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanObjectHeader header;
    ObjectPointCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = object_header->GetCycleDuration();
    header.cycle_count = object_header->GetCycleCount();
    header.number_of_objects = object_header->GetNoOfObjects();
    header.ego_speed = object_header->GetSpeed();
    header.ego_speed_quality = object_header->GetSpeedQuality();
    header.ego_yaw_rate = object_header->GetYawRate();
    header.ego_yaw_rate_quality = object_header->GetYawRateQuality();
    header.dyn_source = object_header->GetDynamicSource();
    for (const auto & object : objectlist_can_umrr9f_mse_v1_3_0->GetObjectList()) {
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_mse_v1_3_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_mse_v1_3_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_mse_can_v1_3_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::PortHeader> port_header;
    port_header = targetlist_can_umrr9f_mse_v1_3_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_mse_v1_3_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_mse_v1_3_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrr9f_mse_v1_3_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9d_v1_5_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9d_v1_5_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9d_v1_5_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9d_v1_5_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9d_t152_automotive_v1_5_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9d_v1_5_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9d_v1_5_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrr9f_v3_0_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrr9f_v3_0_0,
  const com::types::ClientId client_id)
{
  std::cout << "Targetlist for umrr9f_v3_0_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrr9f_v3_0_0->GetPortHeader();
    std::shared_ptr<com::master::umrr9f_t169_automotive_v3_0_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrr9f_v3_0_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    header.time_stamp = target_header->GetTimeStamp();
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    for (const auto & target : targetlist_can_umrr9f_v3_0_0->GetTargetList()) {
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
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

void SmartmicroRadarNode::CAN_targetlist_callback_umrra4_v1_4_0(
  const std::uint32_t sensor_idx,
  const std::shared_ptr<
    com::master::umrra4_automotive_v1_4_0::comtargetbaselist::ComTargetBaseList> &
    targetlist_can_umrra4_v1_4_0,
  const com::types::ClientId client_id)
{
  std::cout << "CAN Targetlist for umrra4_v1_4_0" << std::endl;
  if (!check_signal) {
    std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetbaselist::PortHeader>
      port_header;
    port_header = targetlist_can_umrra4_v1_4_0->GetPortHeader();
    std::shared_ptr<com::master::umrra4_automotive_v1_4_0::comtargetbaselist::TargetListHeader>
      target_header;
    target_header = targetlist_can_umrra4_v1_4_0->GetTargetListHeader();
    sensor_msgs::msg::PointCloud2 msg;
    umrr_ros2_msgs::msg::CanTargetHeader header;
    RadarCloudModifier modifier{msg, m_sensors[sensor_idx].frame_id};
    const auto [sec, nanosec] =
      convert_timestamp(std::chrono::microseconds{port_header->GetTimestamp()});
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    header.frame_id = m_sensors[sensor_idx].frame_id;
    header.cycle_time = target_header->GetCycleDuration();
    header.number_of_targets = target_header->GetNumberOfTargets();
    header.acquisition_setup = target_header->GetAcquisitionSetup();
    header.cycle_count = target_header->GetCycleCount();
    header.time_stamp = target_header->GetTimeStamp();
    header.acq_ts_fraction = target_header->GetAcqTimeStampFraction();
    for (const auto & target : targetlist_can_umrra4_v1_4_0->GetTargetList()) {
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
