#include "smart_rviz_plugin/smart_faults.hpp"

#include <QHeaderView>
#include <QSizePolicy>

namespace smart_rviz_plugin
{

SmartFaultReports::SmartFaultReports(QWidget * parent)
: rviz_common::Panel(parent)
{
  initialize();
}

void SmartFaultReports::initialize()
{
  node_ = rclcpp::Node::make_shared("smart_fault_reports_gui_node");

  layout_ = new QVBoxLayout();
  layout_->setContentsMargins(4, 4, 4, 4);
  layout_->setSpacing(4);

  topic_dropdown_ = new QComboBox();
  topic_dropdown_->addItem("Select a Fault Report Topic");

  header_table_ = new QTableWidget();
  header_table_->setColumnCount(1);
  header_table_->setRowCount(14);
  header_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  header_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  header_table_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  header_table_->setHorizontalHeaderLabels({"Value"});
  header_table_->setVerticalHeaderLabels({
    "stamp.sec", "stamp.nanosec", "frame_id",
    "port_identifier", "port_ver_major", "port_ver_minor",
    "port_size", "body_endianness", "port_index",
    "header_ver_major", "header_ver_minor",
    "num_max_reports", "num_valid_reports", "faults_time_line"
  });
  
  for (int row = 0; row < 14; ++row) {
    header_table_->setItem(row, 0, new QTableWidgetItem("-"));
  }

  reports_table_ = new QTableWidget();
  reports_table_->setColumnCount(9);
  reports_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  reports_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  reports_table_->horizontalHeader()->setStretchLastSection(true);
  reports_table_->setHorizontalHeaderLabels({
    "module_id", "fault_group", "fault_code", "fault_errno",
    "fault_time_stamp", "cycle_count", "instance_id",
    "criticality", "occurence_count"
  });

  splitter_ = new QSplitter(Qt::Horizontal);
  splitter_->addWidget(header_table_);
  splitter_->addWidget(reports_table_);
  splitter_->setSizes({520, 900});

  layout_->addWidget(topic_dropdown_);
  layout_->addWidget(splitter_);
  setLayout(layout_);

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, SIGNAL(timeout()), this, SLOT(check_data()));
  spin_timer_->start(50);

  topic_refresh_timer_ = new QTimer(this);
  connect(topic_refresh_timer_, SIGNAL(timeout()), this, SLOT(refresh_topics_tick()));
  topic_refresh_timer_->start(500);

  connect(
    topic_dropdown_,
    QOverload<int>::of(&QComboBox::currentIndexChanged),
    this,
    &SmartFaultReports::on_topic_selected);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SmartFaultReports plugin created.");
}

void SmartFaultReports::refresh_topic_list()
{
  const auto all_topics = node_->get_topic_names_and_types();

  std::vector<std::string> fault_topics;
  for (const auto & t : all_topics) {
    if (t.first.find("port_faultreport_") != std::string::npos) {
      fault_topics.push_back(t.first);
    }
  }

  std::vector<std::string> current;
  for (int i = 1; i < topic_dropdown_->count(); ++i) {
    current.push_back(topic_dropdown_->itemText(i).toStdString());
  }
  if (current == fault_topics) {
    return;
  }

  const std::string prev_selection = selected_topic_;

  topic_dropdown_->blockSignals(true);
  topic_dropdown_->clear();
  topic_dropdown_->addItem("Select a Fault Report Topic");
  for (const auto & name : fault_topics) {
    topic_dropdown_->addItem(QString::fromStdString(name));
  }

  if (!prev_selection.empty()) {
    int idx = topic_dropdown_->findText(QString::fromStdString(prev_selection));
    if (idx >= 0) {
      topic_dropdown_->setCurrentIndex(idx);
    }
  }
  topic_dropdown_->blockSignals(false);
}

void SmartFaultReports::on_topic_selected(int /*index*/)
{
  const std::string default_choice = "Select a Fault Report Topic";
  const std::string choice = topic_dropdown_->currentText().toStdString();

  if (choice == default_choice) {
    selected_topic_.clear();
    subscription_.reset();

    for (int row = 0; row < 14; ++row) {
      header_table_->item(row, 0)->setText("-");
    }
    reports_table_->setRowCount(0);
    return;
  }

  if (choice == selected_topic_) {
    return;
  }

  selected_topic_ = choice;

  for (int row = 0; row < 14; ++row) {
    header_table_->item(row, 0)->setText("-");
  }
  reports_table_->setRowCount(0);

  subscribe_to_selected_topic();
}

void SmartFaultReports::subscribe_to_selected_topic()
{
  subscription_.reset();

  if (selected_topic_.empty()) {
    return;
  }

  subscription_ = node_->create_subscription<umrr_ros2_msgs::msg::PortFaultReportsMsg>(
    selected_topic_, 10,
    [this](const umrr_ros2_msgs::msg::PortFaultReportsMsg::SharedPtr msg) {
      fault_report_callback(msg);
    });
}

void SmartFaultReports::fault_report_callback(
  const umrr_ros2_msgs::msg::PortFaultReportsMsg::SharedPtr msg)
{
  const auto & fh = msg->fault_report_header;
  const auto & ros_hdr = msg->header;

  auto set_cell = [this](int row, const QString & text) {
    if (!header_table_->item(row, 0)) {
      header_table_->setItem(row, 0, new QTableWidgetItem(text));
    } else {
      header_table_->item(row, 0)->setText(text);
    }
  };

  set_cell(0,  QString::number(ros_hdr.stamp.sec));
  set_cell(1,  QString::number(ros_hdr.stamp.nanosec));
  set_cell(2,  QString::fromStdString(ros_hdr.frame_id));
  set_cell(3,  QString::number(fh.port_identifier));
  set_cell(4,  QString::number(fh.port_ver_major));
  set_cell(5,  QString::number(fh.port_ver_minor));
  set_cell(6,  QString::number(fh.port_size));
  set_cell(7,  QString::number(fh.body_endianness));
  set_cell(8,  QString::number(fh.port_index));
  set_cell(9,  QString::number(fh.header_ver_major));
  set_cell(10, QString::number(fh.header_ver_minor));
  set_cell(11, QString::number(fh.num_max_reports));
  set_cell(12, QString::number(fh.num_valid_reports));
  set_cell(13, QString::number(fh.faults_time_line));

  const auto & reports = msg->reports;
  reports_table_->setRowCount(static_cast<int>(reports.size()));

  for (int row = 0; row < static_cast<int>(reports.size()); ++row) {
    const auto & r = reports[row];
    reports_table_->setItem(row, 0, new QTableWidgetItem(QString::number(r.module_id)));
    reports_table_->setItem(row, 1, new QTableWidgetItem(QString::number(r.fault_group)));
    reports_table_->setItem(row, 2, new QTableWidgetItem(QString::number(r.fault_code)));
    reports_table_->setItem(row, 3, new QTableWidgetItem(QString::number(r.fault_errno)));
    reports_table_->setItem(row, 4, new QTableWidgetItem(QString::number(r.fault_time_stamp)));
    reports_table_->setItem(row, 5, new QTableWidgetItem(QString::number(r.cycle_count)));
    reports_table_->setItem(row, 6, new QTableWidgetItem(QString::number(r.instance_id)));
    reports_table_->setItem(row, 7, new QTableWidgetItem(QString::number(r.criticality)));
    reports_table_->setItem(row, 8, new QTableWidgetItem(QString::number(r.occurence_count)));
  }
}

void SmartFaultReports::check_data()
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }
}

void SmartFaultReports::refresh_topics_tick()
{
  if (rclcpp::ok()) {
    refresh_topic_list();
  }
}

}  // namespace smart_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smart_rviz_plugin::SmartFaultReports, rviz_common::Panel)
