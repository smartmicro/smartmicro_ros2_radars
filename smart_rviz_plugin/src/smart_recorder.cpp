#include "smart_rviz_plugin/smart_recorder.hpp"

#include <cmath>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

const double radToDeg = 180.0 / M_PI;
namespace smart_rviz_plugin
{
SmartRadarRecorder::SmartRadarRecorder(QWidget * parent) : rviz_common::Panel(parent)
{
  initialize();
}

void SmartRadarRecorder::initialize()
{
  node_ = rclcpp::Node::make_shared("smart_radar_gui_node");

  subscription_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/ip_camera_front_right/image_raw/compressed", 10,
    [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { image_callback(msg); });

  publisher_ =
    node_->create_publisher<sensor_msgs::msg::Image>("/ip_camera_front_right/image_raw", 10);

  // Reorder setup
  gui_layout_ = new QVBoxLayout();
  topic_dropdown_ = new QComboBox();
  topic_dropdown_->addItem("Select a Topic");

  // Retrieve available topics
  auto topic_names_and_types = node_->get_topic_names_and_types();

  // Create subscribers for selected topics
  for (const auto & topic : topic_names_and_types) {
    if (topic.first.find("port_targets") != std::string::npos) {
      subscribers_[topic.first] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic.first, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          port_target_callback(msg, topic.first);
        });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("can_targets") != std::string::npos) {
      subscribers_[topic.first] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic.first, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          can_target_callback(msg, topic.first);
        });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("port_objects") != std::string::npos) {
      object_subscribers_[topic.first] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic.first, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          port_object_callback(msg, topic.first);
        });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("can_objects") != std::string::npos) {
      object_subscribers_[topic.first] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic.first, 10, [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          can_object_callback(msg, topic.first);
        });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    }
  }

  connect(topic_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(update_table()));

  // Table one layout
  table_data_ = new QTableWidget();
  table_data_->setColumnCount(13);
  table_data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

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
  connect(timer_, SIGNAL(timeout()), this, SLOT(check_data()));
  timer_->start(20);

  start_button_ = new QPushButton("Record");
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start_recording()));

  stop_button_ = new QPushButton("Stop Recording");
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop_recording()));

  save_button_ = new QPushButton("Save Data as CSV");
  connect(save_button_, SIGNAL(clicked()), this, SLOT(save_data()));

  gui_layout_->addWidget(start_button_);
  gui_layout_->addWidget(stop_button_);
  gui_layout_->addWidget(save_button_);

  setLayout(gui_layout_);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recorder Plugin Created!");
}

void SmartRadarRecorder::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    return;
  }

  sensor_msgs::msg::Image ros_image;
  ros_image.header = msg->header;
  ros_image.height = cv_ptr->image.rows;
  ros_image.width = cv_ptr->image.cols;
  ros_image.encoding = sensor_msgs::image_encodings::BGR8;
  ros_image.is_bigendian = false;
  ros_image.step = sizeof(unsigned char) * cv_ptr->image.cols * cv_ptr->image.channels();
  ros_image.data.assign(
    cv_ptr->image.data, cv_ptr->image.data + cv_ptr->image.total() * cv_ptr->image.elemSize());

  publisher_->publish(ros_image);
}

void SmartRadarRecorder::update_target_recorded_data(
  float range, float power, float azimuth_deg, float elevation_deg, float rcs, float noise,
  float snr, float radial_speed, float azimuth_angle, float elevation_angle, uint32_t timestamp_sec,
  uint32_t timestamp_nanosec)
{
  TargetData data;
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

  target_recorded_data.push_back(data);
}

void SmartRadarRecorder::update_object_recorded_data(
  float x_pos, float y_pos, float z_pos, float speed_abs, float heading, float length,
  float quality, float acceleration, uint16_t object_id, uint32_t timestamp_sec,
  uint32_t timestamp_nanosec)
{
  ObjectData data;
  data.x_pos = x_pos;
  data.y_pos = y_pos;
  data.z_pos = z_pos;
  data.speed_abs = speed_abs;
  data.heading = heading;
  data.length = length;
  data.quality = quality;
  data.object_id = object_id;
  data.acceleration = acceleration;
  data.timestamp_sec = timestamp_sec;
  data.timestamp_nanosec = timestamp_nanosec;

  object_recorded_data.push_back(data);
}

void SmartRadarRecorder::port_target_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_radial_speed(*msg, "radial_speed");
    sensor_msgs::PointCloud2ConstIterator<float> iter_power(*msg, "power");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(*msg, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> iter_noise(*msg, "noise");
    sensor_msgs::PointCloud2ConstIterator<float> iter_snr(*msg, "snr");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth_angle(*msg, "azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_elevation_angle(*msg, "elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(*msg, "range");

    table_data_->setRowCount(0);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z,
                ++iter_radial_speed, ++iter_power, ++iter_rcs, ++iter_noise, ++iter_snr,
                ++iter_azimuth_angle, ++iter_elevation_angle, ++iter_range) {
      double azimuth_deg = *iter_azimuth_angle * radToDeg;
      double elevation_deg = *iter_elevation_angle * radToDeg;

      // Update the recorded data
      if (recording_active_) {
        update_target_recorded_data(
          *iter_range, *iter_power, azimuth_deg, elevation_deg, *iter_rcs, *iter_noise, *iter_snr,
          *iter_radial_speed, *iter_azimuth_angle, *iter_elevation_angle, timestamp_sec,
          timestamp_nanosec);
      }

      // Add items to the table
      int row_index = table_data_->rowCount();
      table_data_->insertRow(row_index);
      table_data_->setItem(row_index, 0, new QTableWidgetItem(QString::number(*iter_x, 'f', 2)));
      table_data_->setItem(row_index, 1, new QTableWidgetItem(QString::number(*iter_y, 'f', 2)));
      table_data_->setItem(row_index, 2, new QTableWidgetItem(QString::number(*iter_z, 'f', 2)));
      table_data_->setItem(
        row_index, 3, new QTableWidgetItem(QString::number(*iter_radial_speed, 'f', 2)));
      table_data_->setItem(
        row_index, 4, new QTableWidgetItem(QString::number(*iter_power, 'f', 2)));
      table_data_->setItem(row_index, 5, new QTableWidgetItem(QString::number(*iter_rcs, 'f', 2)));
      table_data_->setItem(
        row_index, 6, new QTableWidgetItem(QString::number(*iter_noise, 'f', 2)));
      table_data_->setItem(row_index, 7, new QTableWidgetItem(QString::number(*iter_snr, 'f', 2)));
      table_data_->setItem(
        row_index, 8, new QTableWidgetItem(QString::number(azimuth_deg, 'f', 2)));
      table_data_->setItem(
        row_index, 9, new QTableWidgetItem(QString::number(elevation_deg, 'f', 2)));
      table_data_->setItem(
        row_index, 10, new QTableWidgetItem(QString::number(*iter_range, 'f', 2)));
      table_data_->setItem(
        row_index, 11, new QTableWidgetItem(QString::number(*iter_azimuth_angle, 'f', 2)));
      table_data_->setItem(
        row_index, 12, new QTableWidgetItem(QString::number(*iter_elevation_angle, 'f', 2)));
    }

    // Update the timestamp table
    table_timestamps_->setRowCount(0);
    table_timestamps_->insertRow(0);
    table_timestamps_->setItem(0, 0, new QTableWidgetItem(QString::number(timestamp_sec)));
    table_timestamps_->setItem(0, 1, new QTableWidgetItem(QString::number(timestamp_nanosec)));
  }
}

void SmartRadarRecorder::can_target_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_radial_speed(*msg, "radial_speed");
    sensor_msgs::PointCloud2ConstIterator<float> iter_power(*msg, "power");
    sensor_msgs::PointCloud2ConstIterator<float> iter_rcs(*msg, "rcs");
    sensor_msgs::PointCloud2ConstIterator<float> iter_noise(*msg, "noise");
    sensor_msgs::PointCloud2ConstIterator<float> iter_snr(*msg, "snr");
    sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth_angle(*msg, "azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_elevation_angle(*msg, "elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_range(*msg, "range");

    table_data_->setRowCount(0);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z,
                ++iter_radial_speed, ++iter_power, ++iter_rcs, ++iter_noise, ++iter_snr,
                ++iter_azimuth_angle, ++iter_elevation_angle, ++iter_range) {
      double azimuth_deg = *iter_azimuth_angle * radToDeg;
      double elevation_deg = *iter_elevation_angle * radToDeg;

      // Update the recorded data
      if (recording_active_) {
        update_target_recorded_data(
          *iter_range, *iter_power, azimuth_deg, elevation_deg, *iter_rcs, *iter_noise, *iter_snr,
          *iter_radial_speed, *iter_azimuth_angle, *iter_elevation_angle, timestamp_sec,
          timestamp_nanosec);
      }

      // Add items to the table
      int row_index = table_data_->rowCount();
      table_data_->insertRow(row_index);
      table_data_->setItem(row_index, 0, new QTableWidgetItem(QString::number(*iter_x, 'f', 2)));
      table_data_->setItem(row_index, 1, new QTableWidgetItem(QString::number(*iter_y, 'f', 2)));
      table_data_->setItem(row_index, 2, new QTableWidgetItem(QString::number(*iter_z, 'f', 2)));
      table_data_->setItem(
        row_index, 3, new QTableWidgetItem(QString::number(*iter_radial_speed, 'f', 2)));
      table_data_->setItem(
        row_index, 4, new QTableWidgetItem(QString::number(*iter_power, 'f', 2)));
      table_data_->setItem(row_index, 5, new QTableWidgetItem(QString::number(*iter_rcs, 'f', 2)));
      table_data_->setItem(
        row_index, 6, new QTableWidgetItem(QString::number(*iter_noise, 'f', 2)));
      table_data_->setItem(row_index, 7, new QTableWidgetItem(QString::number(*iter_snr, 'f', 2)));
      table_data_->setItem(
        row_index, 8, new QTableWidgetItem(QString::number(azimuth_deg, 'f', 2)));
      table_data_->setItem(
        row_index, 9, new QTableWidgetItem(QString::number(elevation_deg, 'f', 2)));
      table_data_->setItem(
        row_index, 10, new QTableWidgetItem(QString::number(*iter_range, 'f', 2)));
      table_data_->setItem(
        row_index, 11, new QTableWidgetItem(QString::number(*iter_azimuth_angle, 'f', 2)));
      table_data_->setItem(
        row_index, 12, new QTableWidgetItem(QString::number(*iter_elevation_angle, 'f', 2)));
    }

    // Update the timestamp table
    table_timestamps_->setRowCount(0);
    table_timestamps_->insertRow(0);
    table_timestamps_->setItem(0, 0, new QTableWidgetItem(QString::number(timestamp_sec)));
    table_timestamps_->setItem(0, 1, new QTableWidgetItem(QString::number(timestamp_nanosec)));
    // }
  }
}

void SmartRadarRecorder::port_object_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_speed_absolute(*msg, "speed_absolute");
    sensor_msgs::PointCloud2ConstIterator<float> iter_heading(*msg, "heading");
    sensor_msgs::PointCloud2ConstIterator<float> iter_length(*msg, "length");
    sensor_msgs::PointCloud2ConstIterator<float> iter_mileage(*msg, "mileage");
    sensor_msgs::PointCloud2ConstIterator<float> iter_quality(*msg, "quality");
    sensor_msgs::PointCloud2ConstIterator<float> iter_acceleration(*msg, "acceleration");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_object_id(*msg, "object_id");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_idle_cycles(*msg, "idle_cycles");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_spline_idx(*msg, "spline_idx");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_object_class(*msg, "object_class");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_status(*msg, "status");

    table_data_->setRowCount(0);
    size_t num_points = msg->height * msg->width;

    for (size_t i = num_points; i > 0; --i, ++iter_x, ++iter_y, ++iter_z, ++iter_speed_absolute,
                ++iter_heading, ++iter_length, ++iter_mileage, ++iter_quality, ++iter_acceleration,
                ++iter_object_id, ++iter_idle_cycles, ++iter_spline_idx, ++iter_object_class,
                ++iter_status) {
      double heading_deg = *iter_heading * radToDeg;

      // Update the recorded data
      if (recording_active_) {
        update_object_recorded_data(
          *iter_x, *iter_y, *iter_z, *iter_speed_absolute, *iter_heading, *iter_length,
          *iter_quality, *iter_acceleration, *iter_object_id, timestamp_sec, timestamp_nanosec);
      }

      // Add items to the table
      int row_index = table_data_->rowCount();
      table_data_->insertRow(row_index);
      table_data_->setItem(row_index, 0, new QTableWidgetItem(QString::number(*iter_x, 'f', 2)));
      table_data_->setItem(row_index, 1, new QTableWidgetItem(QString::number(*iter_y, 'f', 2)));
      table_data_->setItem(row_index, 2, new QTableWidgetItem(QString::number(*iter_z, 'f', 2)));
      table_data_->setItem(
        row_index, 3, new QTableWidgetItem(QString::number(*iter_speed_absolute, 'f', 2)));
      table_data_->setItem(
        row_index, 4, new QTableWidgetItem(QString::number(heading_deg, 'f', 2)));
      table_data_->setItem(
        row_index, 5, new QTableWidgetItem(QString::number(*iter_length, 'f', 2)));
      table_data_->setItem(
        row_index, 6, new QTableWidgetItem(QString::number(*iter_quality, 'f', 2)));
      table_data_->setItem(
        row_index, 7, new QTableWidgetItem(QString::number(*iter_acceleration, 'f', 2)));
      table_data_->setItem(row_index, 8, new QTableWidgetItem(QString::number(*iter_object_id)));
    }

    // Update the timestamp table
    table_timestamps_->setRowCount(0);
    table_timestamps_->insertRow(0);
    table_timestamps_->setItem(0, 0, new QTableWidgetItem(QString::number(timestamp_sec)));
    table_timestamps_->setItem(0, 1, new QTableWidgetItem(QString::number(timestamp_nanosec)));
  }
}

void SmartRadarRecorder::can_object_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    auto timestamp_sec = msg->header.stamp.sec;
    auto timestamp_nanosec = msg->header.stamp.nanosec;

    // Create iterators for the pc2 fields
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_speed_abs(*msg, "speed_absolute");
    sensor_msgs::PointCloud2ConstIterator<float> iter_heading(*msg, "heading");
    sensor_msgs::PointCloud2ConstIterator<float> iter_length(*msg, "length");
    sensor_msgs::PointCloud2ConstIterator<float> iter_quality(*msg, "quality");
    sensor_msgs::PointCloud2ConstIterator<float> iter_acceleration(*msg, "acceleration");
    sensor_msgs::PointCloud2ConstIterator<float> iter_object_id(*msg, "object_id");
    sensor_msgs::PointCloud2ConstIterator<float> iter_status(*msg, "status");

    table_data_->setRowCount(0);

    size_t num_points = msg->height * msg->width;

    for (size_t i = num_points; i > 0; --i, ++iter_x, ++iter_y, ++iter_z, ++iter_speed_abs,
                ++iter_heading, ++iter_length, ++iter_quality, ++iter_acceleration,
                ++iter_object_id) {
      // Update the recorded data
      if (recording_active_) {
        update_object_recorded_data(
          *iter_x, *iter_y, *iter_z, *iter_speed_abs, *iter_heading, *iter_length, *iter_quality,
          *iter_acceleration, *iter_object_id, timestamp_sec, timestamp_nanosec);
      }

      int row_index = table_data_->rowCount();
      table_data_->insertRow(row_index);
      table_data_->setItem(row_index, 0, new QTableWidgetItem(QString::number(*iter_x, 'f', 2)));
      table_data_->setItem(row_index, 1, new QTableWidgetItem(QString::number(*iter_y, 'f', 2)));
      table_data_->setItem(row_index, 2, new QTableWidgetItem(QString::number(*iter_z, 'f', 2)));
      table_data_->setItem(
        row_index, 3, new QTableWidgetItem(QString::number(*iter_speed_abs, 'f', 2)));
      table_data_->setItem(
        row_index, 4, new QTableWidgetItem(QString::number(*iter_heading, 'f', 2)));
      table_data_->setItem(
        row_index, 5, new QTableWidgetItem(QString::number(*iter_length, 'f', 2)));
      table_data_->setItem(
        row_index, 6, new QTableWidgetItem(QString::number(*iter_quality, 'f', 2)));
      table_data_->setItem(
        row_index, 7, new QTableWidgetItem(QString::number(*iter_acceleration, 'f', 2)));
      table_data_->setItem(
        row_index, 8, new QTableWidgetItem(QString::number(*iter_object_id, 'f', 2)));
    }

    // Update the timestamp table
    table_timestamps_->setRowCount(0);
    table_timestamps_->insertRow(0);
    table_timestamps_->setItem(0, 0, new QTableWidgetItem(QString::number(timestamp_sec)));
    table_timestamps_->setItem(0, 1, new QTableWidgetItem(QString::number(timestamp_nanosec)));
  }
}

void SmartRadarRecorder::update_table()
{
  table_data_->setRowCount(0);
  selected_topic_ = topic_dropdown_->currentText().toStdString();
  if (selected_topic_.find("port_targets") != std::string::npos) {
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "RadialSpeed [m/s]", "Power [dB]", "RCS [m^2]",
       "Noise [dB]", "SNR [dB]", "AzimuthAngle [Deg]", "ElevationAngle [Deg]", "Range [m]",
       "AzimuthAngle [rad]", "ElevationAngle [rad]"});
  } else if (selected_topic_.find("can_targets") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "RadialSpeed [m/s]", "Power [dB]", "RCS [dB]",
       "Noise [dB]", "SNR [dB]", "AzimuthAngle [Deg]", "ElevationAngle [Deg]", "Range [m]",
       "AzimuthAngle [rad]", "ElevationAngle [rad]"});
  } else if (selected_topic_.find("can_objects") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "AbsoluteSpeed [m/s]", "Heading [Deg]",
       "ObjectLength [m]", "Quality", "Acceleration [m/s^2]", "Object_ID", "", "", "", ""});
  } else if (selected_topic_.find("port_objects") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"PosX [m]", "PosY [m]", "PosZ [m]", "AbsoluteSpeed [m/s]", "Heading [Deg]",
       "ObjectLength [m]", "Quality", "Acceleration [m/s^2]", "ObjectId", "", "", "", ""});
  }
}

void SmartRadarRecorder::start_recording()
{
  qDebug() << "Recording started!";
  recording_active_ = true;
  start_button_->setText("Recording...");
}

void SmartRadarRecorder::stop_recording()
{
  qDebug() << "Recording stopped!";
  recording_active_ = false;
  start_button_->setText("Record");
}

void SmartRadarRecorder::save_data()
{
  qDebug() << "Saving data to CSV!";
  if (!target_recorded_data.empty() || !object_recorded_data.empty()) {
    QFileDialog file_dialog;
    QString file_path =
      file_dialog.getSaveFileName(this, "Save Data", "", "CSV Files (*.csv);;All Files (*)");

    if (!file_path.isEmpty()) {
      QFile csvfile(file_path);
      if (csvfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream csv_writer(&csvfile);
        csv_writer
          << "Type, Range [m], Power [dB], AzimuthAngle [Deg], ElevationAngle [Deg], RCS [dB], "
             "Noise [dB], SNR [dB], "
             "RadialSpeed [m/s], ElevationAngle [rad], AzimuthAngle [rad], "
             "TimestampSec, TimestampNanoSec\n";

        for (const auto & data_row : target_recorded_data) {
          QStringList data_str_list;
          data_str_list << "Target";
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

        csv_writer << "Type, PosX [m], PosY [m], PosZ [m], AbsoluteSpeed [m/s], Heading [Deg], "
                      "ObjectLength [m], Quality, Acceleration [m/s^2], ObjectId, "
                      "TimestampSec, TimestampNanoSec\n";
        // Write object data
        for (const auto & object : object_recorded_data) {
          QStringList data_str_list;
          data_str_list << "Object";
          data_str_list << QString::number(object.x_pos, 'f', 2);
          data_str_list << QString::number(object.y_pos, 'f', 2);
          data_str_list << QString::number(object.z_pos, 'f', 2);
          data_str_list << QString::number(object.speed_abs, 'f', 2);
          data_str_list << QString::number(object.heading * 180.0 / M_PI, 'f', 2);
          data_str_list << QString::number(object.length, 'f', 2);
          data_str_list << QString::number(object.quality, 'f', 2);
          data_str_list << QString::number(object.acceleration, 'f', 2);
          data_str_list << QString::number(object.object_id);
          data_str_list << QString::number(object.timestamp_sec);
          data_str_list << QString::number(object.timestamp_nanosec);

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
  target_recorded_data.clear();
  object_recorded_data.clear();
}

void SmartRadarRecorder::check_data()
{
  if (rclcpp::ok())  // Check if ROS2 is still running
  {
    rclcpp::spin_some(node_);
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarRecorder, rviz_common::Panel)