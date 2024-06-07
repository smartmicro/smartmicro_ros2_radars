#ifndef SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
#define SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_

#include <QDebug>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/firmware_download.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the firmware download onto the sensor.
///
/// This class provides a graphical user interface (GUI) panel for downloading
/// firmware to a sensor within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for browsing firmware files, initiating the
/// download process, and displaying responses from the download service.
///
class SmartDownloadService : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartDownloadService class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  SmartDownloadService(QWidget * parent = nullptr);

private slots:
  ///
  /// @brief      Slot function to handle firmware download action.
  ///
  /// This function is triggered when the download button is pressed. It sends
  /// a request to the firmware download service with the specified file path
  /// and sensor ID.
  ///
  void download_firmware();

  ///
  /// @brief      Slot function to handle file browsing action.
  ///
  /// This function is triggered when the browse button is pressed. It opens a
  /// file dialog to allow the user to select a firmware file, and sets the file
  /// path input field with the selected file.
  ///
  void browse_file();

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the firmware download service, and connects the signals and
  /// slots.
  ///
  void initialize();

  rclcpp::Node::SharedPtr download_node;
  rclcpp::Client<umrr_ros2_msgs::srv::FirmwareDownload>::SharedPtr download_client;

  QLineEdit * file_path_input;
  QLineEdit * sensor_id_input;
  QPushButton * start_download_button;
  QPushButton * browse_button;
  QTextEdit * response_text_edit;
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_DOWNLOAD_HPP_
