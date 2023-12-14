#include "smart_rviz_plugin/smart_services.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

namespace smart_rviz_plugin
{
SmartRadarService::SmartRadarService(QWidget * parent) : rviz_common::Panel(parent)
{
  initialize();
}

void SmartRadarService::initialize()
{
  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Declare node_ and clients
  client_node = rclcpp::Node::make_shared("smart_service_gui");
  mode_client =
    client_node->create_client<umrr_ros2_msgs::srv::SetMode>("smart_radar/set_radar_mode");
  command_client =
    client_node->create_client<umrr_ros2_msgs::srv::SendCommand>("smart_radar/send_command");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client node created!");

  // GUI setup
  param_name = new QLineEdit(this);
  param_value = new QLineEdit(this);
  sensor_id_value = new QLineEdit(this);
  command_name = new QLineEdit(this);
  command_value = new QLineEdit(this);
  command_sensor_id_value = new QLineEdit(this);
  send_param_button = new QPushButton("Send Param", this);
  send_command_button = new QPushButton("Send Command", this);

  tab_widget = new QTabWidget(this);
  set_mode_tab = new QWidget(tab_widget);
  send_command_tab = new QWidget(tab_widget);

  // Create layouts for each tab
  send_mode_layout = new QVBoxLayout(set_mode_tab);
  send_command_layout = new QVBoxLayout(send_command_tab);

  // Add widgets to the layouts
  send_mode_layout->addWidget(new QLabel("Radar Params:"));
  send_mode_layout->addWidget(new QLabel("Param:"));
  send_mode_layout->addWidget(param_name);
  send_mode_layout->addWidget(new QLabel("Value:"));
  send_mode_layout->addWidget(param_value);
  send_mode_layout->addWidget(new QLabel("Sensor ID:"));
  send_mode_layout->addWidget(sensor_id_value);
  send_mode_layout->addWidget(send_param_button);

  send_command_layout->addWidget(new QLabel("Radar Commands:"));
  send_command_layout->addWidget(new QLabel("Command:"));
  send_command_layout->addWidget(command_name);
  send_command_layout->addWidget(new QLabel("Value:"));
  send_command_layout->addWidget(command_value);
  send_command_layout->addWidget(new QLabel("Sensor ID:"));
  send_command_layout->addWidget(command_sensor_id_value);
  send_command_layout->addWidget(send_command_button);

  // Set default values
  param_name->setText("dummy_check_interface_file");
  param_value->setText("1");
  sensor_id_value->setText("100");

  command_name->setText("dummy_check_interface_file");
  command_value->setText("1");
  command_sensor_id_value->setText("100");

  tab_widget->addTab(set_mode_tab, "Set Radar Mode");
  tab_widget->addTab(send_command_tab, "Send Command");

  QHBoxLayout * mainLayout = new QHBoxLayout(this);
  mainLayout->addWidget(tab_widget);

  // Connect signals and slots
  connect(send_param_button, &QPushButton::clicked, this, &SmartRadarService::onSendParam);
  connect(send_command_button, &QPushButton::clicked, this, &SmartRadarService::onSendCommand);
}

void SmartRadarService::onSendParam()
{
  if (!mode_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create mode_client");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SetMode::Request>();
  request->param = param_name->text().toStdString();
  request->value = std::stoi(param_value->text().toUtf8().constData());
  request->sensor_id = std::stoi(sensor_id_value->text().toUtf8().constData());

  auto result = mode_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request successful. Response: %s", result.get()->res.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
}

void SmartRadarService::onSendCommand()
{
  if (!command_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create command client");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SendCommand::Request>();
  request->command = command_name->text().toStdString();
  request->value = std::stoi(command_value->text().toUtf8().constData());
  request->sensor_id = std::stoi(command_sensor_id_value->text().toUtf8().constData());

  auto result = command_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request successful. Response: %s", result.get()->res.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarService, rviz_common::Panel)
