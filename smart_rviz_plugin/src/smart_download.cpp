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

  response_text_edit = new QTextEdit(this);
  response_text_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  response_text_edit->setFixedSize(1200, 100);

  QVBoxLayout * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("File Path:"));
  layout->addWidget(file_path_input);
  layout->addWidget(browse_button);
  layout->addWidget(new QLabel("Sensor ID:"));
  layout->addWidget(sensor_id_input);
  layout->addWidget(start_download_button);
  layout->addWidget(response_text_edit);

  // Connect signals and slots
  connect(start_download_button, SIGNAL(clicked()), this, SLOT(download_firmware()));
  connect(browse_button, SIGNAL(clicked()), this, SLOT(browse_file()));
}

void SmartDownloadService::download_firmware()
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

  start_download_button->setText("Downloading...");

  // Set up the request
  auto request = std::make_shared<umrr_ros2_msgs::srv::FirmwareDownload::Request>();
  request->file_path = filePath.toStdString();
  request->sensor_id = sensorId;

  auto result = download_client->async_send_request(request);

  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(download_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    QString response_msg = QString::fromStdString(result.get()->res);
    response_text_edit->append("Download request sent.\n Response from sensor: " + response_msg);
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Request successful\n. Response: %s ",
      result.get()->res.c_str());
    start_download_button->setText("Start Download");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to download sensor firmware");
    response_text_edit->append("<font color=\"red\">Failed to downlaod sensor firmware!</font>");
    start_download_button->setText("Start Download");
  }
}

void SmartDownloadService::browse_file()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("All Files (*)"));
  if (!filePath.isEmpty()) {
    file_path_input->setText(filePath);
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartDownloadService, rviz_common::Panel)
