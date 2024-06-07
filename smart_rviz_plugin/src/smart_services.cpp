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

  client_node = rclcpp::Node::make_shared("smart_service_gui");
  mode_client =
    client_node->create_client<umrr_ros2_msgs::srv::SetMode>("smart_radar/set_radar_mode");
  command_client =
    client_node->create_client<umrr_ros2_msgs::srv::SendCommand>("smart_radar/send_command");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client node created!");

  param_name_line_edit = new QLineEdit(this);
  param_value_line_edit = new QLineEdit(this);
  param_sensor_id = new QLineEdit(this);
  send_param_button = new QPushButton("Send Param", this);
  param_table_widget = new QTableWidget(this);

  command_name_line_edit = new QLineEdit(this);
  command_comment_line_edit = new QLineEdit(this);
  command_value_line_edit = new QLineEdit(this);
  command_sensor_id = new QLineEdit(this);
  send_command_button = new QPushButton("Send Command", this);
  command_table_widget = new QTableWidget(this);

  tab_widget = new QTabWidget(this);
  param_tab = new QWidget(tab_widget);
  command_tab = new QWidget(tab_widget);

  QVBoxLayout * param_layout = new QVBoxLayout(param_tab);
  QVBoxLayout * command_layout = new QVBoxLayout(command_tab);

  param_layout->addWidget(param_table_widget);
  param_layout->addWidget(new QLabel("Enter Param:"));
  param_layout->addWidget(param_name_line_edit);
  param_layout->addWidget(new QLabel("Enter Value:"));
  param_layout->addWidget(param_value_line_edit);
  param_layout->addWidget(new QLabel("Enter Sensor ID:"));
  param_layout->addWidget(param_sensor_id);
  param_layout->addWidget(send_param_button);

  command_layout->addWidget(command_table_widget);
  command_layout->addWidget(new QLabel("Enter Command:"));
  command_layout->addWidget(command_name_line_edit);
  command_layout->addWidget(new QLabel("Enter Value:"));
  command_layout->addWidget(command_value_line_edit);
  command_layout->addWidget(new QLabel("Enter Sensor ID:"));
  command_layout->addWidget(command_sensor_id);
  command_layout->addWidget(send_command_button);

  tab_widget->addTab(param_tab, "Parameter");
  tab_widget->addTab(command_tab, "Command");

  // Add dropdown menu for file selection
  file_selector_combo_box = new QComboBox(this);
  populate_file_menu();

  QVBoxLayout * main_layout = new QVBoxLayout(this);
  main_layout->addWidget(file_selector_combo_box);
  main_layout->addWidget(tab_widget);

  response_text_edit = new QTextEdit(this);
  response_text_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  response_text_edit->setFixedSize(1200, 100);
  main_layout->addWidget(response_text_edit);

  // Connect dropdown menu signal to slot
  connect(
    file_selector_combo_box, QOverload<int>::of(&QComboBox::activated), this,
    &SmartRadarService::on_file_selected);

  read_param_json_data();
  read_command_json_data();
  connect(send_param_button, SIGNAL(clicked()), this, SLOT(on_send_param()));
  connect(send_command_button, SIGNAL(clicked()), this, SLOT(on_send_command()));
}

void SmartRadarService::populate_file_menu()
{
  // Add file names to the dropdown menu
  file_selector_combo_box->clear();
  file_selector_combo_box->addItem("Choose Sensor Type");
  file_selector_combo_box->addItem("UMRR9F MSE");
  file_selector_combo_box->addItem("UMRR9F");
  file_selector_combo_box->addItem("UMRR9D");
  file_selector_combo_box->addItem("UMRRA4");
}

void SmartRadarService::on_file_selected(int index)
{
  // Update file paths based on selected file
  if (index == 0) {
    // Clear param tab
    param_table_widget->setRowCount(0);
    param_name_line_edit->clear();
    param_value_line_edit->clear();
    param_sensor_id->clear();

    // Clear command tab
    command_table_widget->setRowCount(0);
    command_name_line_edit->clear();
    command_argument_line_edit->clear();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Index Zero!");
  } else if (index == 1) {
    param_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9f_t169_mseV1.0.0/instructions/params/auto_interface_0dim.param";
    command_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9f_t169_mseV1.0.0/instructions/command/auto_interface.command";
  } else if (index == 2) {
    param_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9f_t169_automotiveV2.4.1/instructions/params/auto_interface_0dim.param";
    command_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9f_t169_automotiveV2.4.1/instructions/command/auto_interface.command";
  } else if (index == 3) {
    param_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9d_t152_automotiveV1.4.1/instructions/params/auto_interface_0dim.param";
    command_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrr9d_t152_automotiveV1.4.1/instructions/command/auto_interface.command";
  } else if (index == 4) {
    param_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrra4_automotiveV1.2.1/instructions/params/auto_interface_0dim.param";
    command_json_file_path =
      current_directory +
      "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/"
      "UserInterfaceUmrra4_automotiveV1.2.1/instructions/command/auto_interface.command";
  }

  // Update tables with data from selected files
  if (index != 0) {
    read_param_json_data();
    read_command_json_data();
  }
}

void SmartRadarService::read_command_json_data()
{
  QFile command_json_file(command_json_file_path);
  if (!command_json_file.open(QIODevice::ReadOnly)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parse Failed!");
    return;
  }

  QByteArray json_data = command_json_file.readAll();
  QJsonDocument doc(QJsonDocument::fromJson(json_data));
  QJsonObject json_object = doc.object();

  // Extract the "commands" array
  QJsonArray commands_array = json_object["commands"].toArray();

  // Set up table headers
  QStringList command_header_labels = {"Name", "Argument", "Comment"};
  command_table_widget->setColumnCount(3);
  command_table_widget->setHorizontalHeaderLabels(command_header_labels);

  // Populate the table with command data
  command_table_widget->setRowCount(commands_array.size());
  for (int i = 0; i < commands_array.size(); ++i) {
    QJsonObject command_object = commands_array[i].toObject();
    QString name = command_object["name"].toString();
    QString argument = command_object["argument"].toString();
    QString comment = command_object["comment"].toString();

    QTableWidgetItem * name_item = new QTableWidgetItem(name);
    QTableWidgetItem * argument_item = new QTableWidgetItem(argument);
    QTableWidgetItem * comment_item = new QTableWidgetItem(comment);

    command_table_widget->setItem(i, 0, name_item);
    command_table_widget->setItem(i, 1, argument_item);
    command_table_widget->setItem(i, 2, comment_item);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Commands parsed successfully!");
}

void SmartRadarService::read_param_json_data()
{
  QFile param_json_file(param_json_file_path);
  if (!param_json_file.open(QIODevice::ReadOnly)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parse Failed!");
    return;
  }

  QByteArray json_data = param_json_file.readAll();
  QJsonDocument doc(QJsonDocument::fromJson(json_data));
  QJsonObject json_object = doc.object();

  // Extract the "parameters" array
  QJsonArray params_array = json_object["parameters"].toArray();

  // Set up table headers
  QStringList param_header_labels = {"Name", "Comment"};
  param_table_widget->setColumnCount(2);
  param_table_widget->setHorizontalHeaderLabels(param_header_labels);

  // Populate the table with parameter data
  param_table_widget->setRowCount(params_array.size());
  for (int i = 0; i < params_array.size(); ++i) {
    QJsonObject param_object = params_array[i].toObject();
    QString name = param_object["name"].toString();
    QString comment = param_object["comment"].toString();

    QTableWidgetItem * name_item = new QTableWidgetItem(name);
    QTableWidgetItem * comment_item = new QTableWidgetItem(comment);

    param_table_widget->setItem(i, 0, name_item);
    param_table_widget->setItem(i, 1, comment_item);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameters parsed successfully!");
}

void SmartRadarService::on_send_param()
{
  if (!mode_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create mode_client");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SetMode::Request>();
  request->param = param_name_line_edit->text().toStdString();
  request->value = std::stoi(param_value_line_edit->text().toUtf8().constData());
  request->sensor_id = std::stoi(param_sensor_id->text().toUtf8().constData());

  auto result = mode_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    QString response_msg = QString::fromStdString(result.get()->res);
    response_text_edit->append("Request sent. Response from sensor: " + response_msg);
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request sent. Response from sensor: %s",
      result.get()->res.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service!");
    response_text_edit->append("<font color=\"red\">Failed to call service!</font>");
  }
}

void SmartRadarService::on_send_command()
{
  if (!command_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create command client");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SendCommand::Request>();
  request->command = command_name_line_edit->text().toStdString();
  request->value = std::stoi(command_value_line_edit->text().toUtf8().constData());
  request->sensor_id = std::stoi(command_sensor_id->text().toUtf8().constData());

  auto result = command_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    QString response_msg = QString::fromStdString(result.get()->res);
    response_text_edit->append("Request sent. Response from sensor: " + response_msg);
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request sent. Response: %s", result.get()->res.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service!");
    response_text_edit->append("<font color=\"red\">Failed to call service!</font>");
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarService, rviz_common::Panel)
