#include "smart_rviz_plugin/smart_recorder.hpp"

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
  table_data_->setColumnCount(8);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_data_->setHorizontalHeaderLabels(
    {"RadialSpeed [m/s]", "Power [dB]", "RCS [dB]", "Noise [dB]", "SNR", "AzimuthAngle [rad]",
     "ElevationAngle [rad]", "Range [m]"});

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

void SmartRadarRecorder::callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_radial_speed(*msg, "radial_speed");
    sensor_msgs::PointCloud2ConstIterator<float> iter_power(*msg, "power");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(*msg, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> iter_noise(*msg, "noise");
    sensor_msgs::PointCloud2ConstIterator<float> iter_snr(*msg, "snr");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth_angle(*msg, "azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_elevation_angle(*msg, "elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(*msg, "range");

    // Initialize variables for recorded_data
    QString radial_speed_value_str;
    QString power_value_str;
    QString rcs_value_str;
    QString noise_value_str;
    QString snr_value_str;
    QString azimuth_value_str;
    QString elevation_value_str;
    QString range_value_str;

    table_data_->setRowCount(0);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_radial_speed, ++iter_power,
                ++iter_rcs, ++iter_noise, ++iter_snr, ++iter_azimuth_angle, ++iter_elevation_angle,
                ++iter_range) {
      // Add items to the table
      table_data_->insertRow(0);
      table_data_->setItem(0, 0, new QTableWidgetItem(QString::number(*iter_radial_speed, 'f', 2)));
      table_data_->setItem(0, 1, new QTableWidgetItem(QString::number(*iter_power, 'f', 2)));
      table_data_->setItem(0, 2, new QTableWidgetItem(QString::number(*iter_rcs, 'f', 2)));
      table_data_->setItem(0, 3, new QTableWidgetItem(QString::number(*iter_noise, 'f', 2)));
      table_data_->setItem(0, 4, new QTableWidgetItem(QString::number(*iter_snr, 'f', 2)));
      table_data_->setItem(
        0, 5, new QTableWidgetItem(QString::number(*iter_azimuth_angle, 'f', 2)));
      table_data_->setItem(
        0, 6, new QTableWidgetItem(QString::number(*iter_elevation_angle, 'f', 2)));
      table_data_->setItem(0, 7, new QTableWidgetItem(QString::number(*iter_range, 'f', 2)));
      // Update variables for recorded_data
      radial_speed_value_str = QString::number(*iter_radial_speed, 'f', 2);
      power_value_str = QString::number(*iter_power, 'f', 2);
      rcs_value_str = QString::number(*iter_rcs, 'f', 2);
      noise_value_str = QString::number(*iter_noise, 'f', 2);
      snr_value_str = QString::number(*iter_snr, 'f', 2);
      azimuth_value_str = QString::number(*iter_azimuth_angle, 'f', 2);
      elevation_value_str = QString::number(*iter_elevation_angle, 'f', 2);
      range_value_str = QString::number(*iter_range, 'f', 2);
    }

    // Append data to recorded_data
    if (recording_active_) {
      recorded_data.append(
        {radial_speed_value_str, power_value_str, rcs_value_str, noise_value_str, snr_value_str,
         azimuth_value_str, elevation_value_str, range_value_str});
      recorded_data.back().append(QString::number(timestamp_sec));
      recorded_data.back().append(QString::number(timestamp_nanosec));
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
  if (!recorded_data.isEmpty()) {
    QFileDialog file_dialog;
    QString file_path =
      file_dialog.getSaveFileName(this, "Save Data", "", "CSV Files (*.csv);;All Files (*)");

    if (!file_path.isEmpty()) {
      QFile csvfile(file_path);
      if (csvfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream csv_writer(&csvfile);
        csv_writer << "RadialSpeed [m/s], Power [dB], RCS [dB], Noise [dB], SNR, AzimuthAngle "
                      "[rad], ElevationAngle [rad], Range [m], TimestampSec, TimestampNanoSec \n";

        for (const auto & data_row : recorded_data) {
          QStringList data_str_list = QStringList::fromVector(data_row);
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
