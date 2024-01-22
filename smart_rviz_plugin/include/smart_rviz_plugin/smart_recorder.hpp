#ifndef SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
#define SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_

#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QHeaderView>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "std_msgs/msg/string.hpp"

namespace smart_rviz_plugin
{

struct RecordedData {
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

class SmartRadarRecorder : public rviz_common::Panel
{
  Q_OBJECT

public:
  SmartRadarRecorder(QWidget * parent = nullptr);

private slots:
  void startRecording();
  void stopRecording();
  void saveDataToCSV();
  void checkForData();
  void updateTable();

private:
  void initializeRecorder();
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name);
  void updateRecordedData(
    float range, float power, float azimuth_deg, float elevation_deg, float rcs,
    float noise, float snr, float radial_speed, float azimuth_angle,
    float elevation_angle, uint32_t timestamp_sec, uint32_t timestamp_nanosec);

private:
  QTableWidget * table_data_;
  QTableWidget * table_timestamps_;
  QSplitter * splitter_;
  QSplitter * horiz_splitter_;
  QComboBox * topic_dropdown_;
  QVBoxLayout * gui_layout_;
  QPushButton * start_button_;
  QPushButton * stop_button_;
  QPushButton * save_button_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
    subscribers_{};
  std::vector<RecordedData> recorded_data;
  std::string selected_topic_;
  bool recording_active_{false};
};

}  // namespace smart_rviz_plugin

#endif  // SMART_RVIZ_PLUGIN__SMART_RECORDER_HPP_
