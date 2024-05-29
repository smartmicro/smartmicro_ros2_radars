#ifndef SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
#define SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_

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
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "std_msgs/msg/string.hpp"

namespace smart_rviz_plugin
{
struct TargetData
{
  float range;
  float power;
  float azimuth_deg;
  float elevation_deg;
  float rcs;
  float noise;
  float snr;
  float radial_speed;
  float azimuth_angle;
  float elevation_angle;
  uint32_t timestamp_sec;
  uint32_t timestamp_nanosec;
};

struct ObjectData
{
  float x_pos;
  float y_pos;
  float z_pos;
  float speed_abs;
  float heading;
  float length;
  float quality;
  float acceleration;
  uint16_t object_id;
  uint32_t timestamp_sec;
  uint32_t timestamp_nanosec;
};

///
/// @brief      The class for the target and object recorder.
///
/// This class provides a graphical user interface (GUI) panel for viewing
/// the sensor data within the RViz environment. It extends the rviz_common::Panel
/// class and includes functionalities for selecting, recording
/// and saving the sensor data as csv format.
///
class SmartRadarRecorder : public rviz_common::Panel
{
  Q_OBJECT

public:
  ///
  /// @brief      Constructor for the SmartRadarRecorder class.
  ///
  /// @param      parent  The parent widget. Defaults to nullptr.
  ///
  SmartRadarRecorder(QWidget * parent = nullptr);

private slots:
  ///
  /// @brief      Slot function to start recording the sensor data being viewed.
  ///
  void start_recording();

  ///
  /// @brief      Slot function to stop recording the sensor data.
  ///
  void stop_recording();

  ///
  /// @brief      Slot function to save the recorded sensor data as csv fomrat.
  ///
  void save_data();

  ///
  /// @brief      Slot function to check the data is being published.
  ///
  void check_data();

  ///
  /// @brief      Slot function to update the table based on the topic selected.
  ///
  void update_table();

private:
  ///
  /// @brief      Initializes the panel's components and ROS2 client.
  ///
  /// This function sets up the GUI elements, initializes the ROS2 node and
  /// client for the recorder, and connects the signals and
  /// slots.
  ///
  void initialize();

  ///
  /// @brief      Subscribe to the port targets topics publisheb by the smartmicro_radar_node.
  ///
  void port_target_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can targets topics publisheb by the smartmicro_radar_node.
  ///
  void can_target_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the port objects topics publisheb by the smartmicro_radar_node.
  ///
  void port_object_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the can objects topics publisheb by the smartmicro_radar_node.
  ///
  void can_object_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);

  ///
  /// @brief      Subscribe to the raw image topic.
  ///
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  ///
  /// @brief      Function to handle the data recording for target topics.
  ///
  void update_target_recorded_data(
    float range, float power, float azimuth_deg, float elevation_deg, float rcs, float noise,
    float snr, float radial_speed, float azimuth_angle, float elevation_angle,
    uint32_t timestamp_sec, uint32_t timestamp_nanosec);

  ///
  /// @brief      Function to handle the data recording for objects topics.
  ///
  void update_object_recorded_data(
    float x_pos, float y_pos, float z_pos, float speed_abs, float heading, float length,
    float quality, float acceleration, uint16_t object_id, uint32_t timestamp_sec,
    uint32_t timestamp_nanosec);

  QTableWidget * table_data_;
  QTableWidget * table_data_2_;
  QTableWidget * table_timestamps_;
  QWidget * widget_1;
  QWidget * widget_2;
  QSplitter * splitter_;
  QSplitter * horiz_splitter_;
  QComboBox * topic_dropdown_;
  QComboBox * topic_dropdown_2_;
  QVBoxLayout * gui_layout_;
  QVBoxLayout * vert_layout_1;
  QVBoxLayout * vert_layout_2;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QPushButton * save_button_;
  QString selectedTopic;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
    subscribers_{};
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
    object_subscribers_{};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  std::vector<TargetData> target_recorded_data;
  std::vector<ObjectData> object_recorded_data;
  std::string selected_topic_;
  std::string selected_topic_2_;
  bool recording_active_{false};
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
