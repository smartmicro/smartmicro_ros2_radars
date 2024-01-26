#include "smart_rviz_plugin/smart_recorder.hpp"

#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace smart_rviz_plugin
{
SmartRadarRecorder::SmartRadarRecorder(QWidget * parent) : rviz_common::Panel(parent)
{
  initializeRecorder();
}

void SmartRadarRecorder::initializeRecorder()
{
  node_ = rclcpp::Node::make_shared("smart_radar_gui_node");

  // Retrieve available topics
  auto topic_names_and_types = node_->get_topic_names_and_types();
  std::vector<std::string> available_topics;
  for (const auto & topic : topic_names_and_types) {
    available_topics.push_back(topic.first);
  }

  // Reorder setup
  gui_layout_ = new QVBoxLayout();
  topic_dropdown_ = new QComboBox();
  topic_dropdown_->addItem("Select a Topic");
  // Create subscribers for selected topics
  for (const auto & topic : available_topics) {
    subscribers_[topic] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, 10,
      [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { callback(msg, topic); });

    topic_dropdown_->addItem(QString::fromStdString(topic));
  }
  connect(topic_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateTable()));

  table_data_ = new QTableWidget();
  table_data_->setColumnCount(10);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_data_->setHorizontalHeaderLabels(
    {"Range [m]", "Power [dB]", "AzimuthAngle [Deg]", "ElevationAngle [Deg]", "RCS [dB]",
     "Noise [dB]", "SNR [dB]", "RadialSpeed [m/s]", "AzimuthAngle [rad]", "ElevationAngle [rad]"});
  table_timestamps_ = new QTableWidget();
  table_timestamps_->setColumnCount(2);
  table_timestamps_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_timestamps_->setHorizontalHeaderLabels({"TsSec", "TsNanoSec"});

  splitter_ = new QSplitter(Qt::Vertical);
  splitter_->addWidget(topic_dropdown_);
  splitter_->addWidget(table_data_);

  horiz_splitter_ = new QSplitter(Qt::Horizontal);
  horiz_splitter_->addWidget(splitter_);
  horiz_splitter_->addWidget(table_timestamps_);
  horiz_splitter_->setSizes(QList<int>({400, 100}));

  gui_layout_->addWidget(horiz_splitter_);

  timer_ = new QTimer();
  connect(timer_, SIGNAL(timeout()), this, SLOT(checkForData()));
  timer_->start(20);

  start_button_ = new QPushButton("Record");
  connect(start_button_, SIGNAL(clicked()), this, SLOT(startRecording()));

  stop_button_ = new QPushButton("Stop Recording");
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopRecording()));

  save_button_ = new QPushButton("Save Data as CSV");
  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveDataToCSV()));

  gui_layout_->addWidget(start_button_);
  gui_layout_->addWidget(stop_button_);
  gui_layout_->addWidget(save_button_);

  setLayout(gui_layout_);
}

void SmartRadarRecorder::updateRecordedData(
  float range, float power, float azimuth_deg, float elevation_deg, float rcs, float noise,
  float snr, float radial_speed, float azimuth_angle, float elevation_angle,
  uint32_t timestamp_sec, uint32_t timestamp_nanosec)
{
  RecordedData data;
  data.range = range;
  data.power = power;
  data.azimuth_deg = azimuth_deg;
  data.elevation_deg = elevation_deg;
  data.rcs = rcs;
  data.noise = noise;
  data.snr = snr;
  data.radial_speed = radial_speed;
  data.azimuth_angle = azimuth_angle;
  data.elevation_angle = elevation_angle;
  data.timestamp_sec = timestamp_sec;
  data.timestamp_nanosec = timestamp_nanosec;

  recorded_data.push_back(data);
}

void SmartRadarRecorder::callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(*msg, "range");
    sensor_msgs::PointCloud2ConstIterator<float> iter_power(*msg, "power");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth_angle(*msg, "azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_elevation_angle(*msg, "elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(*msg, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> iter_noise(*msg, "noise");
    sensor_msgs::PointCloud2ConstIterator<float> iter_snr(*msg, "snr");
    sensor_msgs::PointCloud2ConstIterator<float> iter_radial_speed(*msg, "radial_speed");

    table_data_->setRowCount(0);

    // Conversion from radians to degrees
    const double radToDeg = 180.0 / M_PI;
    double azimuth_deg, elevation_deg;

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_range, ++iter_power,
                ++iter_azimuth_angle, ++iter_elevation_angle, ++iter_rcs, ++iter_noise,
                ++iter_snr) {
      azimuth_deg = *iter_azimuth_angle * radToDeg;
      elevation_deg = *iter_elevation_angle * radToDeg;

      // Update the recorded data
      if (recording_active_) {
        updateRecordedData(
          *iter_range, *iter_power, azimuth_deg, elevation_deg, *iter_rcs, *iter_noise, *iter_snr,
          *iter_radial_speed, *iter_azimuth_angle, *iter_elevation_angle, timestamp_sec,
          timestamp_nanosec);
      }
      // Add items to the table
      table_data_->insertRow(0);
      table_data_->setItem(0, 0, new QTableWidgetItem(QString::number(*iter_range, 'f', 2)));
      table_data_->setItem(0, 1, new QTableWidgetItem(QString::number(*iter_power, 'f', 2)));
      table_data_->setItem(0, 2, new QTableWidgetItem(QString::number(azimuth_deg, 'f', 2)));
      table_data_->setItem(0, 3, new QTableWidgetItem(QString::number(elevation_deg, 'f', 2)));
      table_data_->setItem(0, 4, new QTableWidgetItem(QString::number(*iter_rcs, 'f', 2)));
      table_data_->setItem(0, 5, new QTableWidgetItem(QString::number(*iter_noise, 'f', 2)));
      table_data_->setItem(0, 6, new QTableWidgetItem(QString::number(*iter_snr, 'f', 2)));
      table_data_->setItem(0, 7, new QTableWidgetItem(QString::number(*iter_radial_speed, 'f', 2)));
      table_data_->setItem(
        0, 8, new QTableWidgetItem(QString::number(*iter_azimuth_angle, 'f', 2)));
      table_data_->setItem(
        0, 9, new QTableWidgetItem(QString::number(*iter_elevation_angle, 'f', 2)));
    }

    // Update the timestamp table
    table_timestamps_->setRowCount(0);
    table_timestamps_->insertRow(0);
    table_timestamps_->setItem(0, 0, new QTableWidgetItem(QString::number(timestamp_sec)));
    table_timestamps_->setItem(0, 1, new QTableWidgetItem(QString::number(timestamp_nanosec)));
  }
}

void SmartRadarRecorder::updateTable()
{
  // Clear the table when a new topic is selected
  table_data_->setRowCount(0);
  selected_topic_ = topic_dropdown_->currentText().toStdString();
}

void SmartRadarRecorder::startRecording()
{
  qDebug() << "Recording started!";
  recording_active_ = true;
  start_button_->setText("Recording...");
}

void SmartRadarRecorder::stopRecording()
{
  qDebug() << "Recording stopped!";
  recording_active_ = false;
  start_button_->setText("Record");
}

void SmartRadarRecorder::saveDataToCSV()
{
  qDebug() << "Saving data to CSV!";
  if (!recorded_data.empty()) {
    QFileDialog file_dialog;
    QString file_path =
      file_dialog.getSaveFileName(this, "Save Data", "", "CSV Files (*.csv);;All Files (*)");

    if (!file_path.isEmpty()) {
      QFile csvfile(file_path);
      if (csvfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream csv_writer(&csvfile);
        csv_writer << "Range [m], Power [dB], AzimuthAngle [Deg], ElevationAngle [Deg], RCS [dB], "
                      "Noise [dB], SNR [dB],  "
                      "RadialSpeed [m/s], ElevationAngle [rad], AzimuthAngle [rad], "
                      "TimestampSec, TimestampNanoSec\n";

        for (const auto & data_row : recorded_data) {
          QStringList data_str_list;
          data_str_list << QString::number(data_row.range, 'f', 2);
          data_str_list << QString::number(data_row.power, 'f', 2);
          data_str_list << QString::number(data_row.azimuth_angle * 180.0 / M_PI, 'f', 2);
          data_str_list << QString::number(data_row.elevation_angle * 180.0 / M_PI, 'f', 2);
          data_str_list << QString::number(data_row.rcs, 'f', 2);
          data_str_list << QString::number(data_row.noise, 'f', 2);
          data_str_list << QString::number(data_row.snr, 'f', 2);
          data_str_list << QString::number(data_row.radial_speed, 'f', 2);
          data_str_list << QString::number(data_row.azimuth_angle, 'f', 2);
          data_str_list << QString::number(data_row.elevation_angle, 'f', 2);
          data_str_list << QString::number(data_row.timestamp_sec);
          data_str_list << QString::number(data_row.timestamp_nanosec);

          csv_writer << data_str_list.join(", ") << "\n";
        }

        csvfile.close();
      } else {
        qDebug() << "Error: Could not open the file for writing.";
      }
    }
  } else {
    qDebug() << "No recorded data to save.";
  }
  // Clear recorded_data after saving
  recorded_data.clear();
}

void SmartRadarRecorder::checkForData()
{
  if (rclcpp::ok())  // Check if ROS2 is still running
  {
    rclcpp::spin_some(node_);
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarRecorder, rviz_common::Panel)
