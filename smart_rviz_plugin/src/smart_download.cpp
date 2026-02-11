#include "smart_rviz_plugin/smart_download.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace smart_rviz_plugin
{

SmartDownloadService::SmartDownloadService(QWidget * parent)
: rviz_common::Panel(parent)
{
  initialize_ros();
  setup_ui();
}

SmartDownloadService::~SmartDownloadService()
{
  if (executor_) {
    executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
  rclcpp::shutdown();
}

void SmartDownloadService::initialize_ros()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  download_node_ = rclcpp::Node::make_shared("smart_download_gui");
  download_client_ = download_node_->create_client<umrr_ros2_msgs::srv::FirmwareDownload>(
    "smart_radar/firmware_download");

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(download_node_);

  ros_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(download_node_->get_logger(), "SmartDownloadService node initialized.");
}

void SmartDownloadService::setup_ui()
{
  file_path_input_ = new QLineEdit(this);
  sensor_id_input_ = new QLineEdit(this);
  start_download_button_ = new QPushButton("Start Download", this);
  browse_button_ = new QPushButton("Browse", this);
  response_text_edit_ = new QTextEdit(this);
  response_text_edit_->setReadOnly(true);
  response_text_edit_->setFixedHeight(120);

  // Compact file path layout
  auto * file_layout = new QHBoxLayout;
  file_layout->addWidget(file_path_input_);
  file_layout->addWidget(browse_button_);

  // Main layout
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Firmware File Path:"));
  layout->addLayout(file_layout);
  layout->addWidget(new QLabel("Sensor ID:"));
  layout->addWidget(sensor_id_input_);
  layout->addWidget(start_download_button_);
  layout->addWidget(response_text_edit_);
  setLayout(layout);

  // Connect signals and slots
  connect(start_download_button_, &QPushButton::clicked, this, &SmartDownloadService::download_firmware);
  connect(browse_button_, &QPushButton::clicked, this, &SmartDownloadService::browse_file);
}

void SmartDownloadService::download_firmware()
{
  if (!download_client_) {
    RCLCPP_ERROR(download_node_->get_logger(), "Download client not created.");
    return;
  }

  QString file_path = file_path_input_->text().trimmed();
  QString sensor_id_str = sensor_id_input_->text().trimmed();

  if (file_path.isEmpty()) {
    QMessageBox::warning(this, "Warning", "Please select a firmware file.", QMessageBox::Ok);
    return;
  }
  if (sensor_id_str.isEmpty() || !sensor_id_str.toInt()) {
    QMessageBox::warning(this, "Warning", "Please enter a valid numeric sensor ID.", QMessageBox::Ok);
    return;
  }

  int sensor_id = sensor_id_str.toInt();
  start_download_button_->setEnabled(false);
  start_download_button_->setText("Downloading...");

  auto request = std::make_shared<umrr_ros2_msgs::srv::FirmwareDownload::Request>();
  request->file_path = file_path.toStdString();
  request->sensor_id = sensor_id;

  if (!download_client_->wait_for_service(std::chrono::seconds(2))) {
    QMessageBox::critical(this, "Error", "Firmware download service not available.");
    start_download_button_->setEnabled(true);
    start_download_button_->setText("Start Download");
    return;
  }

  auto future = download_client_->async_send_request(request);

  // Async callback, do non block rviz
  std::thread([this, future]() mutable {
    try {
      auto result = future.get();
      QString response_msg = QString::fromStdString(result->res);
      QMetaObject::invokeMethod(this, [this, response_msg]() {
        response_text_edit_->append("Response: " + response_msg);
        start_download_button_->setEnabled(true);
        start_download_button_->setText("Start Download");
      });
      RCLCPP_INFO(download_node_->get_logger(), "Firmware download succeeded: %s", result->res.c_str());
    } catch (const std::exception & e) {
      QMetaObject::invokeMethod(this, [this, e]() {
        response_text_edit_->append(QString("<font color='red'>Error: %1</font>").arg(e.what()));
        start_download_button_->setEnabled(true);
        start_download_button_->setText("Start Download");
      });
      RCLCPP_ERROR(download_node_->get_logger(), "Firmware download failed: %s", e.what());
    }
  }).detach();
}

void SmartDownloadService::browse_file()
{
  QString file_path = QFileDialog::getOpenFileName(this, tr("Select Firmware File"), "", tr("All Files (*)"));
  if (!file_path.isEmpty()) {
    file_path_input_->setText(file_path);
  }
}

}  // namespace smart_rviz_plugin

PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartDownloadService, rviz_common::Panel)
