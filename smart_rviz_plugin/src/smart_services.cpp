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

  setup_ros_clients();
  create_widgets();
  setup_layout();
  setup_connections();
}

void SmartRadarService::setup_ros_clients()
{
  client_node = rclcpp::Node::make_shared("smart_service_gui");
  mode_client = client_node->create_client<umrr_ros2_msgs::srv::SetMode>("smart_radar/set_radar_mode");
  command_client = client_node->create_client<umrr_ros2_msgs::srv::SendCommand>("smart_radar/send_command");
  status_client = client_node->create_client<umrr_ros2_msgs::srv::GetStatus>("/smart_radar/get_radar_status");
  get_param_client = client_node->create_client<umrr_ros2_msgs::srv::GetMode>("smart_radar/get_radar_mode");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client node created!");
}

void SmartRadarService::create_widgets()
{
  // Parameter tab widget
  param_name_line_edit = new QLineEdit(this);
  param_name_line_edit->setPlaceholderText("Enter parameter name");

  param_value_line_edit = new QLineEdit(this);
  param_value_line_edit->setPlaceholderText("Enter value");

  param_sensor_id = new QLineEdit(this);
  param_sensor_id->setPlaceholderText("Enter sensor ID");

  param_section_name = new QLineEdit(this);
  param_section_name->setPlaceholderText("Enter param section name");

  param_action_combo = new QComboBox(this);
  param_action_combo->addItem("Write Parameter");
  param_action_combo->addItem("Read Parameter");
  
  param_value_type = new QComboBox(this);
  param_value_type->addItem("float32, (0)");
  param_value_type->addItem("uint32, (1)");
  param_value_type->addItem("uint16, (2)");
  param_value_type->addItem("uint8, (3)");

  send_param_button = new QPushButton("Send Parameter", this);
  param_table_widget = new QTableWidget(this);
  param_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  param_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);

  // Command tab widgets
  command_name_line_edit = new QLineEdit(this);
  command_name_line_edit->setPlaceholderText("Enter command name");

  command_value_line_edit = new QLineEdit(this);
  command_value_line_edit->setPlaceholderText("Enter command value");
  
  command_sensor_id = new QLineEdit(this);
  command_sensor_id->setPlaceholderText("Enter sensor ID");

  command_section_name = new QLineEdit(this);
  command_section_name->setPlaceholderText("Enter command sectionn name");
  
  send_command_button = new QPushButton("Send Command", this);
  command_table_widget = new QTableWidget(this);
  command_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  command_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);

  // Status tab widget
  status_name_line_edit = new QLineEdit(this);
  status_name_line_edit->setPlaceholderText("Enter status name");
  
  status_sensor_id = new QLineEdit(this);
  status_sensor_id->setPlaceholderText("Enter sensor ID");

  status_section_name = new QLineEdit(this);
  status_section_name->setPlaceholderText("Enter status section name");

  status_value_type = new QComboBox(this);
  status_value_type->addItem("uint32, (0)");
  status_value_type->addItem("uint16, (1)");

  send_status_button = new QPushButton("Get Status", this);
  status_table_widget = new QTableWidget(this);
  status_table_widget->setSelectionBehavior(QAbstractItemView::SelectRows);
  status_table_widget->setSelectionMode(QAbstractItemView::SingleSelection);
  
  // Create tabs
  tab_widget = new QTabWidget(this);
  param_tab = new QWidget(tab_widget);
  command_tab = new QWidget(tab_widget);
  status_tab = new QWidget(tab_widget);
  
  // Add dropdown menu for file selection
  file_selector_combo_box = new QComboBox(this);
  populate_file_menu();
  
  response_text_edit = new QTextEdit(this);
  response_text_edit->setReadOnly(true);
  response_text_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  response_text_edit->setFixedHeight(100);

}

void SmartRadarService::setup_layout()
{
  // Parameter tab layout
  QVBoxLayout * param_layout = new QVBoxLayout(param_tab);
  param_layout->setSpacing(10);
  param_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * param_form = new QFormLayout();
  param_form->addRow("Action:", param_action_combo);
  param_form->addRow("Parameter Name:", param_name_line_edit);
  param_form->addRow("Section Name:", param_section_name); 
  param_form->addRow("Value Type:", param_value_type);
  param_form->addRow("Value:", param_value_line_edit);
  param_form->addRow("Sensor ID:", param_sensor_id);

  param_layout->addWidget(param_table_widget);
  param_layout->addLayout(param_form);
  param_layout->addWidget(send_param_button);
  
  // Command tab widget
  QVBoxLayout * command_layout = new QVBoxLayout(command_tab);
  command_layout->setSpacing(10);
  command_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * command_form = new QFormLayout();
  command_form->addRow("Command Name:", command_name_line_edit);
  command_form->addRow("Section Name:", command_section_name);
  command_form->addRow("Value:", command_value_line_edit);
  command_form->addRow("Sensor ID:", command_sensor_id);

  command_layout->addWidget(command_table_widget);
  command_layout->addLayout(command_form);
  command_layout->addWidget(send_command_button);

  // Status tab layout
  QVBoxLayout * status_layout = new QVBoxLayout(status_tab);
  status_layout->setSpacing(10);
  status_layout->setContentsMargins(10, 10, 10, 10);

  QFormLayout * status_form = new QFormLayout();
  status_form->addRow("Status Name:", status_name_line_edit);
  status_form->addRow("Section Name:", status_section_name);
  status_form->addRow("Value Type:", status_value_type);
  status_form->addRow("Sensor ID:", status_sensor_id);

  status_layout->addWidget(status_table_widget);
  status_layout->addLayout(status_form);
  status_layout->addWidget(send_status_button);

  // Add tabs to tab widget
  tab_widget->addTab(param_tab, "Parameter");
  tab_widget->addTab(command_tab, "Command");
  tab_widget->addTab(status_tab, "Status");

  // Main layout
  QVBoxLayout * main_layout = new QVBoxLayout(this);
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  main_layout->addWidget(file_selector_combo_box);
  main_layout->addWidget(tab_widget);
  main_layout->addWidget(response_text_edit);
}

void SmartRadarService::setup_connections()
{
  // Connect dropdown menu signal to slot
  connect(file_selector_combo_box, QOverload<int>::of(&QComboBox::activated), this, &SmartRadarService::on_file_selected);
  connect(send_param_button, &QPushButton::clicked, this, &SmartRadarService::on_send_param);
  connect(send_command_button, &QPushButton::clicked, this, &SmartRadarService::on_send_command);
  connect(send_status_button, &QPushButton::clicked, this, &SmartRadarService::on_get_status);
  connect(param_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_param_selection);
  connect(command_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_command_selection);
  connect(status_table_widget, &QTableWidget::itemSelectionChanged, this, &SmartRadarService::on_status_selection);
  connect(param_action_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
    [this](int index) {
      // Disable value input when "Read Parameter"
      param_value_line_edit->setEnabled(index == 0);
    });
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
  file_selector_combo_box->addItem("UMRRA4 MSE");
  file_selector_combo_box->addItem("UMRRA1");
}

void SmartRadarService::on_param_selection()
{
  QList<QTableWidgetItem*> selected = param_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    param_name_line_edit->setText(param_table_widget->item(row, 1)->text());
    param_section_name->setText(param_table_widget->item(row, 0)->text());
  }
}

void SmartRadarService::on_command_selection()
{
  QList<QTableWidgetItem*> selected = command_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    command_name_line_edit->setText(command_table_widget->item(row, 1)->text());
    command_section_name->setText(command_table_widget->item(row, 0)->text());
  }
}

void SmartRadarService::on_status_selection()
{
  QList<QTableWidgetItem*> selected = status_table_widget->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected[0]->row();
    status_name_line_edit->setText(status_table_widget->item(row, 1)->text());
    status_section_name->setText(status_table_widget->item(row, 0)->text());
  }
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
    param_section_name->clear();

    // Clear command tab
    command_table_widget->setRowCount(0);
    command_name_line_edit->clear();
    command_value_line_edit->clear();
    command_sensor_id->clear();
    command_section_name->clear();

    // Clear status tab
    status_table_widget->setRowCount(0);
    status_name_line_edit->clear();
    status_sensor_id->clear();
    status_section_name->clear();

    return;
  } 
  
  QString base_path = current_directory +
    "/src/smartmicro_ros2_radars/umrr_ros2_driver/smartmicro/user_interfaces/";
  
  switch (index) {
    case 1:
      param_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_mseV1.0.0/instructions/params/auto_interface_0dim.param";
      command_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_mseV1.0.0/instructions/command/auto_interface.command";
      status_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_mseV1.0.0/instructions/status/auto_interface.status";
      break;
    case 2:
      param_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_automotiveV2.4.1/instructions/params/auto_interface_0dim.param";
      command_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_automotiveV2.4.1/instructions/command/auto_interface.command";
      status_json_file_path = base_path +
        "UserInterfaceUmrr9f_t169_automotiveV2.4.1/instructions/status/auto_interface.status";
      break;
    case 3:
      param_json_file_path = base_path +
        "UserInterfaceUmrr9d_t152_automotiveV1.4.1/instructions/params/auto_interface_0dim.param";
      command_json_file_path = base_path +
        "UserInterfaceUmrr9d_t152_automotiveV1.4.1/instructions/command/auto_interface.command";
      status_json_file_path = base_path +
        "UserInterfaceUmrr9d_t152_automotiveV1.4.1/instructions/status/auto_interface.status";
      break;
    case 4:
      param_json_file_path = base_path +
        "UserInterfaceUmrra4_automotiveV1.2.1/instructions/params/auto_interface_0dim.param";
      command_json_file_path = base_path +
        "UserInterfaceUmrra4_automotiveV1.2.1/instructions/command/auto_interface.command";
      status_json_file_path = base_path +
        "UserInterfaceUmrra4_automotiveV1.2.1/instructions/status/auto_interface.status";
      break;
    case 5:
      param_json_file_path = base_path +
        "UserInterfaceUmrra4_mseV1.0.0/instructions/params/auto_interface_0dim.param";
      command_json_file_path = base_path +
        "UserInterfaceUmrra4_mseV1.0.0/instructions/command/auto_interface.command";
      status_json_file_path = base_path +
        "UserInterfaceUmrra4_mseV1.0.0/instructions/status/auto_interface.status";
      break;
    case 6:
      param_json_file_path = base_path +
        "user_interface_umrra1_t166_b_automotive_v2_0_0/instructions/params/auto_interface_rrm.param";
      command_json_file_path = base_path +
        "user_interface_umrra1_t166_b_automotive_v2_0_0/instructions/command/auto_interface_rrm.command";
      status_json_file_path = base_path +
        "user_interface_umrra1_t166_b_automotive_v2_0_0/instructions/status/auto_interface_rrm.status";
      break;
  }

  read_param_json_data();
  read_command_json_data();
  read_status_json_data();
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

  // Extract gloabl section name
  QString global_section_name = json_object["name"].toString();

  // Extract the "commands" array
  QJsonArray commands_array = json_object["commands"].toArray();

  // Set up table headers
  QStringList command_header_labels = {"Section", "Name", "Argument", "Comment"};
  command_table_widget->setColumnCount(3);
  command_table_widget->setHorizontalHeaderLabels(command_header_labels);

  // Populate the table with command data
  command_table_widget->setRowCount(commands_array.size());
  for (int i = 0; i < commands_array.size(); ++i) {
    QJsonObject command_object = commands_array[i].toObject();
    QString name = command_object["name"].toString();
    QString argument = command_object["argument"].toString();
    QString comment = command_object["comment"].toString();

    QTableWidgetItem * section_item = new QTableWidgetItem(global_section_name);
    QTableWidgetItem * name_item = new QTableWidgetItem(name);
    QTableWidgetItem * argument_item = new QTableWidgetItem(argument);
    QTableWidgetItem * comment_item = new QTableWidgetItem(comment);

    command_table_widget->setItem(i, 0, section_item);
    command_table_widget->setItem(i, 1, name_item);
    command_table_widget->setItem(i, 2, argument_item);
    command_table_widget->setItem(i, 3, comment_item);
  }
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

  // Extract gloabl section name
  QString global_section_name = json_object["name"].toString();

  // Extract the "parameters" array
  QJsonArray params_array = json_object["parameters"].toArray();

  // Set up table headers
  QStringList param_header_labels = {"Section", "Name", "Comment", "Type"};
  param_table_widget->setColumnCount(3);
  param_table_widget->setHorizontalHeaderLabels(param_header_labels);

  // Populate the table with parameter data
  param_table_widget->setRowCount(params_array.size());
  for (int i = 0; i < params_array.size(); ++i) {
    QJsonObject param_object = params_array[i].toObject();
    QString name = param_object["name"].toString();
    QString comment = param_object["comment"].toString();
    QString type = param_object["type"].toString();

    QTableWidgetItem * section_item = new QTableWidgetItem(global_section_name);
    QTableWidgetItem * name_item = new QTableWidgetItem(name);
    QTableWidgetItem * comment_item = new QTableWidgetItem(comment);
    QTableWidgetItem * type_item = new QTableWidgetItem(type);

    param_table_widget->setItem(i, 0, section_item);
    param_table_widget->setItem(i, 1, name_item);
    param_table_widget->setItem(i, 2, comment_item);
    param_table_widget->setItem(i, 3, type_item);
  }
}

void SmartRadarService::read_status_json_data()
{
  QFile status_json_file(status_json_file_path);
  if (!status_json_file.open(QIODevice::ReadOnly)) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parse Failed!");
    return;
  }

  QByteArray json_data = status_json_file.readAll();
  QJsonDocument doc(QJsonDocument::fromJson(json_data));
  QJsonObject json_object = doc.object();

  // Extract gloabl section name
  QString global_section_name = json_object["name"].toString();

  // Extract the "status" array
  QJsonArray status_array = json_object["status"].toArray();

  // Set up table headers
  QStringList status_header_labels = {"Section", "Name", "Comment", "Type"};
  status_table_widget->setColumnCount(3);
  status_table_widget->setHorizontalHeaderLabels(status_header_labels);

  // Populate the table with status data
  status_table_widget->setRowCount(status_array.size());
  for (int i = 0; i < status_array.size(); ++i) {
    QJsonObject status_object = status_array[i].toObject();
    QString name = status_object["name"].toString();
    QString comment = status_object["comment"].toString();
    QString type = status_object["type"].toString();

    QTableWidgetItem * section_item = new QTableWidgetItem(global_section_name);
    QTableWidgetItem * name_item = new QTableWidgetItem(name);
    QTableWidgetItem * comment_item = new QTableWidgetItem(comment);
    QTableWidgetItem * type_item = new QTableWidgetItem(type);

    status_table_widget->setItem(i, 0, section_item);
    status_table_widget->setItem(i, 1, name_item);
    status_table_widget->setItem(i, 2, comment_item);
    status_table_widget->setItem(i, 3, type_item);
  }
}
 
void SmartRadarService::on_send_param()
{
  // Validate common inputs
  if (param_name_line_edit->text().isEmpty() || param_sensor_id->text().isEmpty()) {
    response_text_edit->append("<font color=\"red\">Error: Parameter name and sensor ID fields must be filled!</font>");
    return;
  }

  if (param_action_combo->currentIndex() == 0) {
    // Writing param 
    if (param_value_line_edit->text().isEmpty()) {
      response_text_edit->append("<font color=\"red\">Error: Value must be provided for write!</font>");
      return;
    }
    if (!mode_client->wait_for_service(SERVICE_AVAILABILITY_TIMEOUT)) {
      response_text_edit->append("<font color=\"red\">SetMode service not available!</font>");
      return;
    }
    auto request = std::make_shared<umrr_ros2_msgs::srv::SetMode::Request>();
    request->section_name = param_section_name->text().toStdString();
    request->params.push_back(param_name_line_edit->text().toStdString());
    request->sensor_id = std::stoi(param_sensor_id->text().toStdString());
    request->value_types.push_back(param_value_type->currentIndex());
    request->values.push_back(param_value_line_edit->text().toStdString());
    auto result = mode_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(client_node, result);
    switch (status) {
      case rclcpp::FutureReturnCode::SUCCESS:
        {
          auto response = result.get();
          QString response_msg = QString::fromStdString(response->res);
          response_text_edit->append(response_msg);
        }
        break;

      case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service timed out!");
        response_text_edit->append("<font color=\"red\">Service timed out!</font>");
        break;

      case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call was interrupted!");
        response_text_edit->append("<font color=\"red\">Service call was interrupted!</font>");
        break;

      default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed due to unknown reason!");
        response_text_edit->append("<font color=\"red\">Service call failed due to unknown reason!</font>");
        break;
    }
  }
  else {
    // Reading param
    if (!get_param_client->wait_for_service(SERVICE_AVAILABILITY_TIMEOUT)) {
      response_text_edit->append("<font color=\"red\">GetMode service not available!</font>");
      return;
    }
    auto request = std::make_shared<umrr_ros2_msgs::srv::GetMode::Request>();
    request->section_name = param_section_name->text().toStdString();
    request->params.push_back(param_name_line_edit->text().toStdString());
    request->sensor_id = std::stoi(param_sensor_id->text().toStdString());
    request->param_types.push_back(param_value_type->currentIndex());
    auto result = get_param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(client_node, result);
    switch (status) {
      case rclcpp::FutureReturnCode::SUCCESS:
        {
          auto response = result.get();
          QString response_msg = QString::fromStdString(response->res);
          response_text_edit->append(response_msg);
        }
        break;

      case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service timed out!");
        response_text_edit->append("<font color=\"red\">Service timed out!</font>");
        break;

      case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call was interrupted!");
        response_text_edit->append("<font color=\"red\">Service call was interrupted!</font>");
        break;

      default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed due to unknown reason!");
        response_text_edit->append("<font color=\"red\">Service call failed due to unknown reason!</font>");
        break;
    }
  }
}

void SmartRadarService::on_send_command()
{
  // Validate inputs
  if (command_name_line_edit->text().isEmpty() || command_value_line_edit->text().isEmpty() || command_sensor_id->text().isEmpty()) {
    response_text_edit->append("<font color=\"red\">Error: All command fields must be filled. Add dummy value of command if not specified!</font>");
    return;
  }

  if (!command_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create command client");
    return;
  }

  if(!command_client->wait_for_service(SERVICE_AVAILABILITY_TIMEOUT)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available!");
    response_text_edit->append("<font color=\"red\">Service not available! Is the radar node running?</font>");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::SendCommand::Request>();
  request->section_name = command_section_name->text().toStdString();
  request->command = command_name_line_edit->text().toStdString();
  request->value = std::stoi(command_value_line_edit->text().toUtf8().constData());
  request->sensor_id = std::stoi(command_sensor_id->text().toUtf8().constData());

  auto result = command_client->async_send_request(request);
  auto status = rclcpp::spin_until_future_complete(client_node, result); 

  switch (status) {
    case rclcpp::FutureReturnCode::SUCCESS:
      {
        auto response = result.get();
        QString response_msg = QString::fromStdString(response->res);
        response_text_edit->append(response_msg);
      }
      break;

    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service timed out!");
      response_text_edit->append("<font color=\"red\">Service timed out!</font>");
      break;
      
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call was interrupted!");
      response_text_edit->append("<font color=\"red\">Service call was interrupted!</font>");
      break;
      
    default:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed due to unknown reason!");
      response_text_edit->append("<font color=\"red\">Service call failed due to unknown reason!</font>");
      break;
  }
}

void SmartRadarService::on_get_status()
{
  // Validate inputs
  if (status_name_line_edit->text().isEmpty() || status_sensor_id->text().isEmpty()) {
    response_text_edit->append("<font color=\"red\">Error: Status name and sensor ID must be filled!</font>");
    return;
  }
  
  if (!status_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create status_client");
    return;
  }

  if(!status_client->wait_for_service(SERVICE_AVAILABILITY_TIMEOUT)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available!");
    response_text_edit->append("<font color=\"red\">Service not available! Is the radar node running?</font>");
    return;
  }

  auto request = std::make_shared<umrr_ros2_msgs::srv::GetStatus::Request>();

  request->section_name = status_section_name->text().toStdString();
  request->statuses.push_back(status_name_line_edit->text().toStdString());
  request->sensor_id = std::stoi(status_sensor_id->text().toUtf8().constData());
  request->status_types.push_back(status_value_type->currentIndex());
  
  auto result = status_client->async_send_request(request);
  auto status = rclcpp::spin_until_future_complete(client_node, result); 
  
  switch (status) {
    case rclcpp::FutureReturnCode::SUCCESS:
      {
        auto response = result.get();
        QString response_msg = QString::fromStdString(response->res);
        response_text_edit->append(response_msg);
      }
      break;

    case rclcpp::FutureReturnCode::TIMEOUT:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service timed out!");
      response_text_edit->append("<font color=\"red\">Service timed out!</font>");
      break;
      
    case rclcpp::FutureReturnCode::INTERRUPTED:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call was interrupted!");
      response_text_edit->append("<font color=\"red\">Service call was interrupted!</font>");
      break;
      
    default:
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed due to unknown reason!");
      response_text_edit->append("<font color=\"red\">Service call failed due to unknown reason!</font>");
      break;
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarService, rviz_common::Panel)
