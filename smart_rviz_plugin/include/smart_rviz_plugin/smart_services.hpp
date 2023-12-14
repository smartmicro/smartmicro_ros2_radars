#ifndef SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
#define SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_

#include <QDebug>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/send_command.hpp"
#include "umrr_ros2_msgs/srv/set_mode.hpp"

namespace smart_rviz_plugin
{
class SmartRadarService : public rviz_common::Panel
{
  Q_OBJECT

public:
  SmartRadarService(QWidget * parent = nullptr);

private slots:
  void onSendParam();
  void onSendCommand();

private:
  void initialize();

private:
  rclcpp::Node::SharedPtr client_node;
  QLineEdit * param_name;
  QLineEdit * param_value;
  QLineEdit * sensor_id_value;
  QLineEdit * command_name;
  QLineEdit * command_value;
  QLineEdit * command_sensor_id_value;
  QPushButton * send_param_button;
  QPushButton * send_command_button;
  QTabWidget * tab_widget;
  QWidget * set_mode_tab;
  QWidget * send_command_tab;
  QVBoxLayout * send_mode_layout;
  QVBoxLayout * send_command_layout;
  rclcpp::Client<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_client;
  rclcpp::Client<umrr_ros2_msgs::srv::SendCommand>::SharedPtr command_client;
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
