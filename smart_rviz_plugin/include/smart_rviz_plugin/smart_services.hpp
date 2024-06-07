#ifndef SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
#define SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_

#include <QComboBox>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "umrr_ros2_msgs/srv/send_command.hpp"
#include "umrr_ros2_msgs/srv/set_mode.hpp"

namespace smart_rviz_plugin
{
///
/// @brief      The class for the sending instructions to the sensors.
///
/// This class provides a graphical user interface (GUI) panel for sending
/// instructions to the sensor within the RViz environment. It extends the
/// rviz_common::Panel class and includes functionalities for selecting, viewing
/// and sending the sensor instructions.
///
class SmartRadarService : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartRadarService class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  SmartRadarService(QWidget * parent = nullptr);

private slots:
  ///
  /// @brief      Slot function to send a parameter instruction to the sensor.
  ///
  void on_send_param();

  ///
  /// @brief      Slot function to send a command instruction to the sensor.
  ///
  void on_send_command();

  ///
  /// @brief      Slot function to display instruction set of selected sensor model.
  ///
  void on_file_selected(int index);

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the instruction services, and connects the signals and
  /// slots.
  ///
  void initialize();

  ///
  /// @brief      Read parameters from user interface json file.
  ///
  void read_param_json_data();

  ///
  /// @brief      Read commands from user interface json file.
  ///
  void read_command_json_data();

  ///
  /// @brief      Populate the table with the read json files.
  ///
  void populate_file_menu();

  const QString current_directory = QDir::currentPath();
  QString param_json_file_path;
  QString command_json_file_path;

  QLineEdit * command_value_line_edit;
  QLineEdit * command_argument_line_edit;

  QVBoxLayout * main_layout;
  QTabWidget * tab_widget;
  QWidget * param_tab;
  QLineEdit * param_name_line_edit;
  QLineEdit * param_value_line_edit;
  QLineEdit * param_min_line_edit;
  QLineEdit * param_max_line_edit;
  QLineEdit * param_sensor_id;
  QLineEdit * command_name_line_edit;
  QLineEdit * command_comment_line_edit;
  QLineEdit * command_sensor_id;

  QPushButton * send_param_button;
  QPushButton * send_command_button;
  QTableWidget * param_table_widget;
  QTableWidget * command_table_widget;
  QWidget * command_tab;
  QComboBox * file_selector_combo_box;
  QTextEdit * response_text_edit;

  rclcpp::Node::SharedPtr client_node;
  rclcpp::Client<umrr_ros2_msgs::srv::SetMode>::SharedPtr mode_client;
  rclcpp::Client<umrr_ros2_msgs::srv::SendCommand>::SharedPtr command_client;
};
}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_SERVICES_HPP_
