#include "smart_rviz_plugin/smart_status.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace smart_rviz_plugin
{
SmartRadarStatus::SmartRadarStatus(QWidget * parent) : rviz_common::Panel(parent) { initialize(); }

void SmartRadarStatus::initialize()
{
  node_ = rclcpp::Node::make_shared("smart_radar_gui_node");

  // Status setup
  gui_layout_ = new QVBoxLayout();
  topic_dropdown_ = new QComboBox();
  topic_dropdown_->addItem("Select a Topic");

  // Retrieve available topics
  auto topic_names_and_types = node_->get_topic_names_and_types();

  // Create subscribers for selected topics
  for (const auto & topic : topic_names_and_types) {
    if (topic.first.find("port_targetheader") != std::string::npos) {
      port_header_target_subscribers_[topic.first] =
        node_->create_subscription<umrr_ros2_msgs::msg::PortTargetHeader>(
          topic.first, 10,
          [this, topic](const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg) {
            port_targetheader_callback(msg, topic.first);
          });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("can_targetheader") != std::string::npos) {
      can_header_target_subscribers_[topic.first] =
        node_->create_subscription<umrr_ros2_msgs::msg::CanTargetHeader>(
          topic.first, 10,
          [this, topic](const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg) {
            can_targetheader_callback(msg, topic.first);
          });

      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("port_objectheader") != std::string::npos) {
      port_header_object_subscribers_[topic.first] =
        node_->create_subscription<umrr_ros2_msgs::msg::PortObjectHeader>(
          topic.first, 10,
          [this, topic](const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg) {
            port_objectheader_callback(msg, topic.first);
          });
      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    } else if (topic.first.find("can_objectheader") != std::string::npos) {
      can_header_object_subscribers_[topic.first] =
        node_->create_subscription<umrr_ros2_msgs::msg::CanObjectHeader>(
          topic.first, 10,
          [this, topic](const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg) {
            can_objectheader_callback(msg, topic.first);
          });
      topic_dropdown_->addItem(QString::fromStdString(topic.first));
    }
  }

  connect(topic_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(update_table()));

  table_data_ = new QTableWidget();
  table_data_->setRowCount(16);
  table_data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  table_data_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  splitter_ = new QSplitter(Qt::Vertical);
  splitter_->addWidget(topic_dropdown_);
  splitter_->addWidget(table_data_);

  gui_layout_->addWidget(splitter_);

  timer_ = new QTimer();
  connect(timer_, SIGNAL(timeout()), this, SLOT(check_data()));
  timer_->start(20);

  setLayout(gui_layout_);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status Plugin Created!");
}

void SmartRadarStatus::port_targetheader_callback(
  const umrr_ros2_msgs::msg::PortTargetHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    std::uint64_t ntp_timestamp = msg->acquisition_start;
    // Extract the first 4 bytes (seconds) - most significant 32 bits
    std::uint32_t seconds = static_cast<uint32_t>(ntp_timestamp >> 32);
    // Extract the last 4 bytes (fraction of a second) - least significant 32 bits
    std::uint32_t fraction_sec = static_cast<uint32_t>(ntp_timestamp & 0xFFFFFFFF);
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_targets)));
    table_data_->setItem(
      2, col_index, new QTableWidgetItem(QString::number(msg->acquisition_tx_ant_idx)));
    table_data_->setItem(
      3, col_index, new QTableWidgetItem(QString::number(msg->acquisition_sweep_idx)));
    table_data_->setItem(
      4, col_index, new QTableWidgetItem(QString::number(msg->acquisition_cf_idx)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(seconds)));
    table_data_->setItem(6, col_index, new QTableWidgetItem(QString::number(fraction_sec)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->prf)));
    table_data_->setItem(
      8, col_index, new QTableWidgetItem(QString::number(msg->umambiguous_speed, 'f', 2)));
    table_data_->setItem(9, col_index, new QTableWidgetItem(QString::number(msg->port_identifier)));
    table_data_->setItem(10, col_index, new QTableWidgetItem(QString::number(msg->port_ver_major)));
    table_data_->setItem(11, col_index, new QTableWidgetItem(QString::number(msg->port_ver_minor)));
    table_data_->setItem(12, col_index, new QTableWidgetItem(QString::number(msg->port_size)));
    table_data_->setItem(
      13, col_index, new QTableWidgetItem(QString::number(msg->body_endianness)));
    table_data_->setItem(14, col_index, new QTableWidgetItem(QString::number(msg->port_index)));
    table_data_->setItem(
      15, col_index, new QTableWidgetItem(QString::number(msg->header_ver_major)));
    table_data_->setItem(
      16, col_index, new QTableWidgetItem(QString::number(msg->header_ver_minor)));
  }
}

void SmartRadarStatus::can_targetheader_callback(
  const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_targets)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(msg->cycle_count)));
    table_data_->setItem(
      3, col_index, new QTableWidgetItem(QString::number(msg->acquisition_setup)));
    table_data_->setItem(4, col_index, new QTableWidgetItem(QString::number(msg->time_stamp)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->acq_ts_fraction)));
  }
}

void SmartRadarStatus::port_objectheader_callback(
  const umrr_ros2_msgs::msg::PortObjectHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    float_t cycle_time = msg->cycle_time;
    std::uint16_t no_objs = msg->number_of_objects;
    std::uint64_t ntp_timestamp = msg->ts_measurement;
    std::uint32_t seconds = static_cast<uint32_t>(ntp_timestamp >> 32);
    std::uint32_t fraction_sec = static_cast<uint32_t>(ntp_timestamp & 0xFFFFFFFF);

    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(0, col_index, new QTableWidgetItem(QString::number(cycle_time, 'f', 3)));
    table_data_->setItem(1, col_index, new QTableWidgetItem(QString::number(no_objs)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(seconds)));
    table_data_->setItem(3, col_index, new QTableWidgetItem(QString::number(fraction_sec)));
    table_data_->setItem(4, col_index, new QTableWidgetItem(QString::number(msg->port_identifier)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->port_ver_major)));
    table_data_->setItem(6, col_index, new QTableWidgetItem(QString::number(msg->port_ver_minor)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->port_size)));
    table_data_->setItem(8, col_index, new QTableWidgetItem(QString::number(msg->body_endianness)));
    table_data_->setItem(9, col_index, new QTableWidgetItem(QString::number(msg->port_index)));
    table_data_->setItem(
      10, col_index, new QTableWidgetItem(QString::number(msg->header_ver_major)));
    table_data_->setItem(
      11, col_index, new QTableWidgetItem(QString::number(msg->header_ver_minor)));
  }
}

void SmartRadarStatus::can_objectheader_callback(
  const umrr_ros2_msgs::msg::CanObjectHeader::SharedPtr msg, const std::string topic_name)
{
  if (!selected_topic_.empty() && topic_name == selected_topic_) {
    table_data_->setColumnCount(0);
    int col_index = 0;
    table_data_->insertColumn(col_index);
    table_data_->setItem(
      0, col_index, new QTableWidgetItem(QString::number(msg->cycle_time, 'f', 3)));
    table_data_->setItem(
      1, col_index, new QTableWidgetItem(QString::number(msg->number_of_objects)));
    table_data_->setItem(2, col_index, new QTableWidgetItem(QString::number(msg->cycle_count)));
    table_data_->setItem(3, col_index, new QTableWidgetItem(QString::number(msg->ego_speed)));
    table_data_->setItem(
      4, col_index, new QTableWidgetItem(QString::number(msg->ego_speed_quality)));
    table_data_->setItem(5, col_index, new QTableWidgetItem(QString::number(msg->ego_yaw_rate)));
    table_data_->setItem(
      6, col_index, new QTableWidgetItem(QString::number(msg->ego_yaw_rate_quality)));
    table_data_->setItem(7, col_index, new QTableWidgetItem(QString::number(msg->dyn_source)));
  }
}

void SmartRadarStatus::update_table()
{
  table_data_->setColumnCount(0);
  selected_topic_ = topic_dropdown_->currentText().toStdString();
  if (selected_topic_.find("port_targetheader") != std::string::npos) {
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfTargets", "AcquisitionTxAntIdx", "AcquisitionSweepIdx",
       "AcquisitionCfIdx", "AcqTimeStamp [s]", "AcqTimeStampfrac [NTP]", "PRF", "UmambiguousSpeed",
       "PortIdentifier", "PortVersionMajor", "PortVersionMinor", "PortSize", "BodyEndianness",
       "PortIndex", "HeaderVersionMajor", "HeaderVersionMinor"});
  } else if (selected_topic_.find("can_targetheader") != std::string::npos) {
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfTargets", "CycleCount", "AcquisitionSetup", "AcqTimeStamp [s]",
       "AcqTimeStampfrac [NTP]", "", "", "", "", "", "", "", "", "", "", ""});
  } else if (selected_topic_.find("can_objectheader") != std::string::npos) {
    table_data_->setColumnCount(0);
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfObjects", "CycleCount", "Speed [km/h]", "SpeedQuality",
       "YawRate [rad/s]", "YawRateQuality", "DynamicSource", "", "", "", "", "", "", "", "", ""});
  } else if (selected_topic_.find("port_objectheader") != std::string::npos) {
    table_data_->setColumnCount(0);
    table_data_->setVerticalHeaderLabels(
      {"CycleDuration [s]", "NumOfObjects", "AcqTimeStamp [s]", "AcqTimeStampfrac [NTP]",
       "PortIdentifier", "PortVersionMajor", "PortVersionMinor", "PortSize", "BodyEndianness",
       "PortIndex", "HeaderVersionMajor", "HeaderVersionMinor", "", "", "", "", ""});
  }
}

void SmartRadarStatus::check_data()
{
  if (rclcpp::ok())  // Check if ROS2 is still running
  {
    rclcpp::spin_some(node_);
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartRadarStatus, rviz_common::Panel)
