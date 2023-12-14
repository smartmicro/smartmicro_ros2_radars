#ifndef SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
#define SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_

#include <QDebug>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/firmware_download.hpp"

namespace smart_rviz_plugin
{
class SmartDownloadService : public rviz_common::Panel
{
  Q_OBJECT

public:
  SmartDownloadService(QWidget * parent = nullptr);

private slots:
  void onDownload();
  void onBrowse();

private:
  void initialize();

private:
  rclcpp::Node::SharedPtr download_node;
  rclcpp::Client<umrr_ros2_msgs::srv::FirmwareDownload>::SharedPtr download_client;

  QLineEdit * file_path_input;
  QLineEdit * sensor_id_input;
  QPushButton * start_download_button;
  QPushButton * browse_button;
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
