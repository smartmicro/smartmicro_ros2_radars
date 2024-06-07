#ifndef SMART_RVIZ_PLUGIN__SMART_STATUS_HPP_
#define SMART_RVIZ_PLUGIN__SMART_STATUS_HPP_

#include <cv_bridge/cv_bridge.h>

#include <QComboBox>
#include <QDebug>
#include <QDockWidget>
#include <QFile>
#include <QFileDialog>
#include <QHeaderView>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "umrr_ros2_msgs/msg/can_object_header.hpp"
#include "umrr_ros2_msgs/msg/can_target_header.hpp"
#include "umrr_ros2_msgs/msg/port_object_header.hpp"
#include "umrr_ros2_msgs/msg/port_target_header.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the target and object headers.
///
/// This class provides a graphical user interface (GUI) panel for viewing
/// the sensor header data within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for selecting, viewing
/// the sensor header data.
///
class SmartRadarStatus : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartRadarStatus class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  SmartRadarStatus(QWidget * parent = nullptr);

private slots:
  ///
  /// @brief      Slot function to update the table based on the topic selected.
  ///
  void update_table();

  ///
  /// @brief      Slot function to check the data is being published.
  ///
  void check_data();

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the status, and connects the signals and
  /// slots.
  ///
  void initialize();

  ///
  /// @brief      Subscribe to the port target headers publisheb by the smartmicro_radar_node.
  ///
  void port_targetheader_callback(
    const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can target headers publisheb by the smartmicro_radar_node.
  ///
  void can_targetheader_callback(
    const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the port object headers publisheb by the smartmicro_radar_node.
  ///
  void port_objectheader_callback(
    const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can object headers publisheb by the smartmicro_radar_node.
  ///
  void can_objectheader_callback(
    const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg, const std::string topic_name);

  QTableWidget * table_data_;
  QTableWidget * table_data_2_;
  QTableWidget * table_timestamps_;
  QSplitter * splitter_;
  QComboBox * topic_dropdown_;
  QVBoxLayout * gui_layout_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QPushButton * save_button_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  std::string selected_topic_;
  std::unordered_map<
    std::string, rclcpp::Subscription<umrr_ros2_msgs::msg::PortTargetHeader>::SharedPtr>
    port_header_target_subscribers_{};
  std::unordered_map<
    std::string, rclcpp::Subscription<umrr_ros2_msgs::msg::CanTargetHeader>::SharedPtr>
    can_header_target_subscribers_{};
  std::unordered_map<
    std::string, rclcpp::Subscription<umrr_ros2_msgs::msg::PortObjectHeader>::SharedPtr>
    port_header_object_subscribers_{};
  std::unordered_map<
    std::string, rclcpp::Subscription<umrr_ros2_msgs::msg::CanObjectHeader>::SharedPtr>
    can_header_object_subscribers_{};
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
