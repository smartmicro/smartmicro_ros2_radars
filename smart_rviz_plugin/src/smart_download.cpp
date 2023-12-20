#include "smart_rviz_plugin/smart_download.hpp"

namespace smart_rviz_plugin
{
SmartDownloadService::SmartDownloadService(QWidget * parent) : rviz_common::Panel(parent)
{
  initialize();
}

void SmartDownloadService::initialize()
{
  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Declare node and client
  download_node = rclcpp::Node::make_shared("smart_download_gui");
  download_client = download_node->create_client<umrr_ros2_msgs::srv::FirmwareDownload>(
    "smart_radar/firmware_download");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Download node created!");

  // GUI setup
  file_path_input = new QLineEdit(this);
  sensor_id_input = new QLineEdit(this);
  start_download_button = new QPushButton("Start Download", this);
  browse_button = new QPushButton("Browse", this);

  QVBoxLayout * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("File Path:"));
  layout->addWidget(file_path_input);
  layout->addWidget(browse_button);
  layout->addWidget(new QLabel("Sensor ID:"));
  layout->addWidget(sensor_id_input);
  layout->addWidget(start_download_button);

  // Connect signals and slots
  connect(start_download_button, &QPushButton::clicked, this, &SmartDownloadService::onDownload);
  connect(browse_button, &QPushButton::clicked, this, &SmartDownloadService::onBrowse);
}

void SmartDownloadService::onDownload()
{
  if (!download_client) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to create download_client");
    return;
  }

  // Get file path and sensor ID
  QString filePath = file_path_input->text();
  int sensorId = sensor_id_input->text().toInt();

  if (filePath.isEmpty()) {
    QMessageBox::warning(this, "Warning", "Please select a file to download.", QMessageBox::Ok);
    return;
  }

  // Set up the request
  auto request = std::make_shared<umrr_ros2_msgs::srv::FirmwareDownload::Request>();
  request->file_path = filePath.toStdString();
  request->sensor_id = sensorId;

  auto result = download_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(download_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request successful. Response: %s", result.get()->res.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service firmware_download");
  }
}

void SmartDownloadService::onBrowse()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("All Files (*)"));
  if (!filePath.isEmpty()) {
    file_path_input->setText(filePath);
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartDownloadService, rviz_common::Panel)
