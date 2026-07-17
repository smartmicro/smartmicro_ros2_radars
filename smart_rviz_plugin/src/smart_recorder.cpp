#include "smart_rviz_plugin/smart_recorder.hpp"

#include <cmath>
#include <QMessageBox>
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
  node_ = rclcpp::Node::make_shared("smart_radar_recorder_gui_node");

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
  table_data_->setColumnCount(20);
  table_data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  table_data_->horizontalHeader()->setStretchLastSection(true);

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

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(check_data()));
  timer_->start(50);

  start_button_ = new QPushButton("Record");
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start_recording()));

  stop_button_ = new QPushButton("Stop Recording");
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop_recording()));
  stop_button_->setEnabled(false);

  save_button_ = new QPushButton("Save Data as CSV");
  connect(save_button_, SIGNAL(clicked()), this, SLOT(save_data()));
  save_button_->setEnabled(false);

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
  const std::string & topic_name, float range, float power, float azimuth_deg,
  float elevation_deg, float rcs, float noise, float snr, float radial_speed,
  float azimuth_angle, float elevation_angle, float variance_range, float variance_speed,
  float variance_azimuth_angle, float variance_elevation_angle, float false_alarm_probability,
  uint32_t flags, uint16_t peak_idx, uint32_t timestamp_sec, uint32_t timestamp_nanosec)
{
  TargetData data;
  data.topic_name = topic_name;
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
  data.variance_range = variance_range;
  data.variance_speed = variance_speed;
  data.variance_azimuth_angle = variance_azimuth_angle;
  data.variance_elevation_angle = variance_elevation_angle;
  data.false_alarm_probability = false_alarm_probability;
  data.flags = flags;
  data.peak_idx = peak_idx;
  data.timestamp_sec = timestamp_sec;
  data.timestamp_nanosec = timestamp_nanosec;

  target_recorded_data.push_back(data);
}

void SmartRadarRecorder::update_object_recorded_data(
  const std::string & topic_name, float x_pos, float y_pos, float z_pos, float speed_abs,
  float heading, float length, float mileage, float quality, float acceleration,
  int16_t object_id, uint16_t idle_cycles, uint16_t spline_idx, uint8_t object_class,
  uint16_t status, uint32_t timestamp_sec, uint32_t timestamp_nanosec)
{
  ObjectData data;
  data.topic_name = topic_name;
  data.x_pos = x_pos;
  data.y_pos = y_pos;
  data.z_pos = z_pos;
  data.speed_abs = speed_abs;
  data.heading = heading;
  data.length = length;
  data.mileage = mileage;
  data.quality = quality;
  data.object_id = object_id;
  data.acceleration = acceleration;
  data.idle_cycles = idle_cycles;
  data.spline_idx = spline_idx;
  data.object_class = object_class;
  data.status = status;
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
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_range(*msg, "variance_range");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_speed(*msg, "variance_speed");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_azimuth_angle(
      *msg, "variance_azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_elevation_angle(
      *msg, "variance_elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_false_alarm_probability(
      *msg, "false_alarm_probability");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_flags(*msg, "flags");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_peak_idx(*msg, "peak_idx");

    table_data_->setRowCount(0);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z,
                ++iter_radial_speed, ++iter_power, ++iter_rcs, ++iter_noise, ++iter_snr,
                ++iter_azimuth_angle, ++iter_elevation_angle, ++iter_range,
                ++iter_variance_range, ++iter_variance_speed, ++iter_variance_azimuth_angle,
                ++iter_variance_elevation_angle, ++iter_false_alarm_probability, ++iter_flags,
                ++iter_peak_idx) {
      double azimuth_deg = *iter_azimuth_angle * radToDeg;
      double elevation_deg = *iter_elevation_angle * radToDeg;

      // Update the recorded data
      if (recording_active_ && topic_name == recording_topic_) {
        update_target_recorded_data(
          topic_name, *iter_range, *iter_power, azimuth_deg, elevation_deg, *iter_rcs,
          *iter_noise, *iter_snr, *iter_radial_speed, *iter_azimuth_angle,
          *iter_elevation_angle, *iter_variance_range, *iter_variance_speed,
          *iter_variance_azimuth_angle, *iter_variance_elevation_angle,
          *iter_false_alarm_probability, *iter_flags, *iter_peak_idx, timestamp_sec,
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
      table_data_->setItem(
        row_index, 13, new QTableWidgetItem(QString::number(*iter_variance_range, 'f', 4)));
      table_data_->setItem(
        row_index, 14, new QTableWidgetItem(QString::number(*iter_variance_speed, 'f', 4)));
      table_data_->setItem(
        row_index, 15,
        new QTableWidgetItem(QString::number(*iter_variance_azimuth_angle, 'f', 4)));
      table_data_->setItem(
        row_index, 16,
        new QTableWidgetItem(QString::number(*iter_variance_elevation_angle, 'f', 4)));
      table_data_->setItem(
        row_index, 17, new QTableWidgetItem(QString::number(*iter_false_alarm_probability, 'f', 4)));
      table_data_->setItem(row_index, 18, new QTableWidgetItem(QString::number(*iter_flags)));
      table_data_->setItem(row_index, 19, new QTableWidgetItem(QString::number(*iter_peak_idx)));
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
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_range(*msg, "variance_range");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_speed(*msg, "variance_speed");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_azimuth_angle(
      *msg, "variance_azimuth_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_variance_elevation_angle(
      *msg, "variance_elevation_angle");
    sensor_msgs::PointCloud2ConstIterator<float> iter_false_alarm_probability(
      *msg, "false_alarm_probability");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_flags(*msg, "flags");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_peak_idx(*msg, "peak_idx");

    table_data_->setRowCount(0);

    for (size_t i = 0; i < msg->height * msg->width; ++i, ++iter_x, ++iter_y, ++iter_z,
                ++iter_radial_speed, ++iter_power, ++iter_rcs, ++iter_noise, ++iter_snr,
                ++iter_azimuth_angle, ++iter_elevation_angle, ++iter_range,
                ++iter_variance_range, ++iter_variance_speed, ++iter_variance_azimuth_angle,
                ++iter_variance_elevation_angle, ++iter_false_alarm_probability, ++iter_flags,
                ++iter_peak_idx) {
      double azimuth_deg = *iter_azimuth_angle * radToDeg;
      double elevation_deg = *iter_elevation_angle * radToDeg;

      // Update the recorded data
      if (recording_active_ && topic_name == recording_topic_) {
        update_target_recorded_data(
          topic_name, *iter_range, *iter_power, azimuth_deg, elevation_deg, *iter_rcs,
          *iter_noise, *iter_snr, *iter_radial_speed, *iter_azimuth_angle,
          *iter_elevation_angle, *iter_variance_range, *iter_variance_speed,
          *iter_variance_azimuth_angle, *iter_variance_elevation_angle,
          *iter_false_alarm_probability, *iter_flags, *iter_peak_idx, timestamp_sec,
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
      table_data_->setItem(
        row_index, 13, new QTableWidgetItem(QString::number(*iter_variance_range, 'f', 4)));
      table_data_->setItem(
        row_index, 14, new QTableWidgetItem(QString::number(*iter_variance_speed, 'f', 4)));
      table_data_->setItem(
        row_index, 15,
        new QTableWidgetItem(QString::number(*iter_variance_azimuth_angle, 'f', 4)));
      table_data_->setItem(
        row_index, 16,
        new QTableWidgetItem(QString::number(*iter_variance_elevation_angle, 'f', 4)));
      table_data_->setItem(
        row_index, 17, new QTableWidgetItem(QString::number(*iter_false_alarm_probability, 'f', 4)));
      table_data_->setItem(row_index, 18, new QTableWidgetItem(QString::number(*iter_flags)));
      table_data_->setItem(row_index, 19, new QTableWidgetItem(QString::number(*iter_peak_idx)));

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
      if (recording_active_ && topic_name == recording_topic_) {
        update_object_recorded_data(
          topic_name, *iter_x, *iter_y, *iter_z, *iter_speed_absolute, *iter_heading,
          *iter_length, *iter_mileage, *iter_quality, *iter_acceleration,
          static_cast<int16_t>(*iter_object_id), *iter_idle_cycles, *iter_spline_idx,
          *iter_object_class, *iter_status, timestamp_sec, timestamp_nanosec);
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
      table_data_->setItem(
        row_index, 9, new QTableWidgetItem(QString::number(*iter_mileage, 'f', 2)));
      table_data_->setItem(row_index, 10, new QTableWidgetItem(QString::number(*iter_idle_cycles)));
      table_data_->setItem(row_index, 11, new QTableWidgetItem(QString::number(*iter_spline_idx)));
      table_data_->setItem(
        row_index, 12, new QTableWidgetItem(QString::number(*iter_object_class)));
      table_data_->setItem(row_index, 13, new QTableWidgetItem(QString::number(*iter_status)));
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
    sensor_msgs::PointCloud2ConstIterator<int16_t> iter_object_id(*msg, "object_id");
    sensor_msgs::PointCloud2ConstIterator<float> iter_mileage(*msg, "mileage");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_idle_cycles(*msg, "idle_cycles");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_spline_idx(*msg, "spline_idx");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_object_class(*msg, "object_class");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_status(*msg, "status");

    table_data_->setRowCount(0);

    size_t num_points = msg->height * msg->width;

    for (size_t i = num_points; i > 0; --i, ++iter_x, ++iter_y, ++iter_z, ++iter_speed_abs,
                ++iter_heading, ++iter_length, ++iter_quality, ++iter_acceleration,
                ++iter_object_id, ++iter_mileage, ++iter_idle_cycles, ++iter_spline_idx,
                ++iter_object_class, ++iter_status) {
      // Update the recorded data
      if (recording_active_ && topic_name == recording_topic_) {
        update_object_recorded_data(
          topic_name, *iter_x, *iter_y, *iter_z, *iter_speed_abs, *iter_heading, *iter_length,
          *iter_mileage, *iter_quality, *iter_acceleration, *iter_object_id, *iter_idle_cycles,
          *iter_spline_idx, *iter_object_class, *iter_status, timestamp_sec,
          timestamp_nanosec);
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
        row_index, 8, new QTableWidgetItem(QString::number(*iter_object_id)));
      table_data_->setItem(
        row_index, 9, new QTableWidgetItem(QString::number(*iter_mileage, 'f', 2)));
      table_data_->setItem(row_index, 10, new QTableWidgetItem(QString::number(*iter_idle_cycles)));
      table_data_->setItem(row_index, 11, new QTableWidgetItem(QString::number(*iter_spline_idx)));
      table_data_->setItem(
        row_index, 12, new QTableWidgetItem(QString::number(*iter_object_class)));
      table_data_->setItem(row_index, 13, new QTableWidgetItem(QString::number(*iter_status)));
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
  for (int col = 0; col < table_data_->columnCount(); ++col) {
    table_data_->setColumnHidden(col, false);
  }
  if (selected_topic_.find("port_targets") != std::string::npos) {
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "RadialSpeed [m/s]", "Power [dB]", "RCS [m^2]",
       "Noise [dB]", "SNR [dB]", "AzimuthAngle [Deg]", "ElevationAngle [Deg]", "Range [m]",
       "AzimuthAngle [rad]", "ElevationAngle [rad]", "VarRange", "VarSpeed",
       "VarAzimuthAngle", "VarElevationAngle", "FalseAlarmProb", "Flags", "PeakIdx"});
  } else if (selected_topic_.find("can_targets") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "RadialSpeed [m/s]", "Power [dB]", "RCS [dB]",
       "Noise [dB]", "SNR [dB]", "AzimuthAngle [Deg]", "ElevationAngle [Deg]", "Range [m]",
       "AzimuthAngle [rad]", "ElevationAngle [rad]", "VarRange", "VarSpeed",
       "VarAzimuthAngle", "VarElevationAngle", "FalseAlarmProb", "Flags", "PeakIdx"});
    // Driver sets CAN target extras to sentinels (NaN / max): hide unavailable fields.
    table_data_->setColumnHidden(13, true);
    table_data_->setColumnHidden(14, true);
    table_data_->setColumnHidden(15, true);
    table_data_->setColumnHidden(16, true);
    table_data_->setColumnHidden(17, true);
    table_data_->setColumnHidden(18, true);
    table_data_->setColumnHidden(19, true);
  } else if (selected_topic_.find("can_objects") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"X_pos [m]", "Y_pos [m]", "Z_pos [m]", "AbsoluteSpeed [m/s]", "Heading [Deg]",
       "ObjectLength [m]", "Quality", "Acceleration [m/s^2]", "Object_ID", "Mileage",
       "IdleCycles", "SplineIdx", "ObjectClass", "Status", "", "", "", "", "", ""});
    table_data_->setColumnHidden(9, true);
    table_data_->setColumnHidden(10, true);
    table_data_->setColumnHidden(11, true);
    table_data_->setColumnHidden(12, true);
    table_data_->setColumnHidden(13, true);
  } else if (selected_topic_.find("port_objects") != std::string::npos) {
    table_data_->setRowCount(0);
    table_data_->setHorizontalHeaderLabels(
      {"PosX [m]", "PosY [m]", "PosZ [m]", "AbsoluteSpeed [m/s]", "Heading [Deg]",
       "ObjectLength [m]", "Quality", "Acceleration [m/s^2]", "ObjectId", "Mileage",
       "IdleCycles", "SplineIdx", "ObjectClass", "Status", "", "", "", "", "", ""});
  }
}

void SmartRadarRecorder::start_recording()
{
  if (pending_save_) {
    qDebug() << "Save or discard the completed recording before starting another one.";
    return;
  }

  selected_topic_ = topic_dropdown_->currentText().toStdString();
  if (
    selected_topic_.empty() || selected_topic_ == "Select a Topic" ||
    selected_topic_.find("/smart_radar/") == std::string::npos)
  {
    qDebug() << "Please select a valid /smart_radar topic before recording.";
    return;
  }

  recording_topic_ = selected_topic_;
  qDebug() << "Recording started!";
  recording_active_ = true;
  start_button_->setText("Recording...");
  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  save_button_->setEnabled(false);
  topic_dropdown_->setEnabled(false);
}

void SmartRadarRecorder::stop_recording()
{
  if (pending_save_) {
    QMessageBox save_prompt(this);
    save_prompt.setWindowTitle("Completed Recording");
    save_prompt.setText("Save or discard the completed recording before starting another one.");
    auto * save = save_prompt.addButton("Save", QMessageBox::AcceptRole);
    auto * discard = save_prompt.addButton("Discard", QMessageBox::DestructiveRole);
    save_prompt.addButton(QMessageBox::Cancel);
    save_prompt.exec();

    if (save_prompt.clickedButton() == save) {
      save_data();
    } else if (save_prompt.clickedButton() == discard) {
      clear_recorded_data();
      return_to_ready_state();
    }
    return;
  }

  if (!recording_active_) {
    return;
  }

  qDebug() << "Recording stopped!";
  recording_active_ = false;

  if (target_recorded_data.empty() && object_recorded_data.empty()) {
    return_to_ready_state();
    return;
  }

  pending_save_ = true;
  stop_button_->setText("Save or Discard...");
  save_button_->setEnabled(true);
  stop_recording();
}

void SmartRadarRecorder::clear_recorded_data()
{
  target_recorded_data.clear();
  object_recorded_data.clear();
}

void SmartRadarRecorder::return_to_ready_state()
{
  pending_save_ = false;
  recording_topic_.clear();
  start_button_->setText("Record");
  start_button_->setEnabled(true);
  stop_button_->setText("Stop Recording");
  stop_button_->setEnabled(false);
  save_button_->setEnabled(false);
  topic_dropdown_->setEnabled(true);
}

void SmartRadarRecorder::save_data()
{
  qDebug() << "Saving data to CSV!";
  bool data_saved = false;
  if (!target_recorded_data.empty() || !object_recorded_data.empty()) {
    QFileDialog file_dialog;
    QString file_path =
      file_dialog.getSaveFileName(this, "Save Data", "", "CSV Files (*.csv);;All Files (*)");

    if (!file_path.isEmpty()) {
      QFile csvfile(file_path);
      if (csvfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream csv_writer(&csvfile);
        bool wrote_port_target_header = false;
        bool wrote_can_target_header = false;
        for (const auto & data_row : target_recorded_data) {
          const bool is_port_target_topic =
            data_row.topic_name.find("port_targets") != std::string::npos;

          if (is_port_target_topic) {
            if (!wrote_port_target_header) {
              csv_writer
                << "Type, Topic, Range [m], Power [dB], AzimuthAngle [Deg], ElevationAngle [Deg], "
                   "RCS [dB], Noise [dB], SNR [dB], RadialSpeed [m/s], AzimuthAngle [rad], "
                   "ElevationAngle [rad], VarianceRange, VarianceSpeed, VarianceAzimuthAngle, "
                   "VarianceElevationAngle, FalseAlarmProbability, Flags, PeakIdx, "
                   "TimestampSec, TimestampNanoSec\n";
              wrote_port_target_header = true;
            }
          } else {
            if (!wrote_can_target_header) {
              csv_writer
                << "Type, Topic, Range [m], Power [dB], AzimuthAngle [Deg], ElevationAngle [Deg], "
                   "RCS [dB], Noise [dB], SNR [dB], RadialSpeed [m/s], AzimuthAngle [rad], "
                   "ElevationAngle [rad], TimestampSec, TimestampNanoSec\n";
              wrote_can_target_header = true;
            }
          }

          QStringList data_str_list;
          data_str_list << "Target";
          data_str_list << QString::fromStdString(data_row.topic_name);
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
          if (is_port_target_topic) {
            data_str_list << QString::number(data_row.variance_range, 'f', 4);
            data_str_list << QString::number(data_row.variance_speed, 'f', 4);
            data_str_list << QString::number(data_row.variance_azimuth_angle, 'f', 4);
            data_str_list << QString::number(data_row.variance_elevation_angle, 'f', 4);
            data_str_list << QString::number(data_row.false_alarm_probability, 'f', 4);
            data_str_list << QString::number(data_row.flags);
            data_str_list << QString::number(data_row.peak_idx);
          }
          data_str_list << QString::number(data_row.timestamp_sec);
          data_str_list << QString::number(data_row.timestamp_nanosec);

          csv_writer << data_str_list.join(", ") << "\n";
        }

        bool wrote_port_object_header = false;
        bool wrote_can_object_header = false;

        // Write object data grouped by schema to avoid exporting CAN sentinel-only columns.
        for (const auto & object : object_recorded_data) {
          const bool is_port_object_topic =
            object.topic_name.find("port_objects") != std::string::npos;

          if (is_port_object_topic) {
            if (!wrote_port_object_header) {
              csv_writer << "Type, Topic, PosX [m], PosY [m], PosZ [m], AbsoluteSpeed [m/s], "
                            "Heading [Deg], ObjectLength [m], Quality, Acceleration [m/s^2], "
                            "ObjectId, Mileage, IdleCycles, SplineIdx, ObjectClass, Status, "
                            "TimestampSec, TimestampNanoSec\n";
              wrote_port_object_header = true;
            }
          } else {
            if (!wrote_can_object_header) {
              csv_writer << "Type, Topic, PosX [m], PosY [m], PosZ [m], AbsoluteSpeed [m/s], "
                            "Heading [Deg], ObjectLength [m], Quality, Acceleration [m/s^2], "
                            "ObjectId, TimestampSec, TimestampNanoSec\n";
              wrote_can_object_header = true;
            }
          }

          QStringList data_str_list;
          data_str_list << "Object";
          data_str_list << QString::fromStdString(object.topic_name);
          data_str_list << QString::number(object.x_pos, 'f', 2);
          data_str_list << QString::number(object.y_pos, 'f', 2);
          data_str_list << QString::number(object.z_pos, 'f', 2);
          data_str_list << QString::number(object.speed_abs, 'f', 2);
          data_str_list << QString::number(object.heading * 180.0 / M_PI, 'f', 2);
          data_str_list << QString::number(object.length, 'f', 2);
          data_str_list << QString::number(object.quality, 'f', 2);
          data_str_list << QString::number(object.acceleration, 'f', 2);
          data_str_list << QString::number(object.object_id);

          if (is_port_object_topic) {
            data_str_list << QString::number(object.mileage, 'f', 2);
            data_str_list << QString::number(object.idle_cycles);
            data_str_list << QString::number(object.spline_idx);
            data_str_list << QString::number(object.object_class);
            data_str_list << QString::number(object.status);
          }

          data_str_list << QString::number(object.timestamp_sec);
          data_str_list << QString::number(object.timestamp_nanosec);

          csv_writer << data_str_list.join(", ") << "\n";
        }

        csvfile.close();
        data_saved = true;
      } else {
        qDebug() << "Error: Could not open the file for writing.";
      }
    }
  } else {
    qDebug() << "No recorded data to save.";
  }

  if (data_saved) {
    clear_recorded_data();
    return_to_ready_state();
  }
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