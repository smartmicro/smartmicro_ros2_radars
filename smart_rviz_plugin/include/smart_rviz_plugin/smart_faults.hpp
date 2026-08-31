#ifndef SMART_RVIZ_PLUGIN__SMART_FAULTS_HPP_
#define SMART_RVIZ_PLUGIN__SMART_FAULTS_HPP_

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <unordered_map>

#include "umrr_ros2_msgs/msg/port_fault_reports_msg.hpp"

namespace smart_rviz_plugin
{
///
/// @brief Panel for displaying fault reports published by the smartmicro_radar_node.
///
/// Subscribes to one sensor's `smart_radar/port_faultreport_<N>` topic at a time.
/// Shows the PortFaultReportHeader fields and the per-fault PortFaultReport[] array.
/// The sensor is selected via a dropdown that is populated dynamically from live topics.
///
class SmartFaultReports : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SmartFaultReports(QWidget * parent = nullptr);

private slots:
  /// Periodic ROS spinning.
  void check_data();

  /// Periodic topic-list refresh.
  void refresh_topics_tick();

  /// Called when the user picks a different topic in the dropdown.
  void on_topic_selected(int index);

private:
  void initialize();

  /// (Re-)subscribe to the currently selected fault-report topic.
  void subscribe_to_selected_topic();

  /// Refresh the dropdown with currently available port_faultreport_* topics.
  void refresh_topic_list();

  /// Incoming message handler.
  void fault_report_callback(
    const umrr_ros2_msgs::msg::PortFaultReportsMsg::SharedPtr msg);

  /// Widgets 
  QComboBox  * topic_dropdown_{nullptr};
  QTableWidget * header_table_{nullptr};
  QTableWidget * reports_table_{nullptr};
  QSplitter  * splitter_{nullptr};
  QVBoxLayout * layout_{nullptr};
  QTimer     * spin_timer_{nullptr};
  QTimer     * topic_refresh_timer_{nullptr};

  /// ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<umrr_ros2_msgs::msg::PortFaultReportsMsg>::SharedPtr subscription_;

  std::string selected_topic_;
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_FAULTS_HPP_
