#include "rviz2_plugins/state_trigger_panel.h"

namespace rviz2_plugins
{

StateTriggerPanel::StateTriggerPanel(QWidget* parent)
  : rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>("state_trigger_panel"))
{
  // Create GUI layout
  start_nav_button_ = new QPushButton("Start Navigation", this);
  resume_nav_button_ = new QPushButton("Resume Navigation", this);

  // Vertical layout for buttons
  auto* layout = new QVBoxLayout;
  layout->addWidget(start_nav_button_);
  layout->addWidget(resume_nav_button_);
  setLayout(layout);

  // ROS2 service clients
  start_client_ = node_->create_client<std_srvs::srv::Trigger>("start_wp_nav");
  resume_client_ = node_->create_client<std_srvs::srv::Trigger>("resume_nav");

  // Connect buttons to slots
  connect(start_nav_button_, SIGNAL(clicked()), this, SLOT(pushStartNavigation()));
  connect(resume_nav_button_, SIGNAL(clicked()), this, SLOT(pushResumeNavigation()));
}

StateTriggerPanel::~StateTriggerPanel() = default;

void StateTriggerPanel::pushStartNavigation()
{
  RCLCPP_INFO(node_->get_logger(), "Service call: start waypoints navigation");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  start_client_->async_send_request(request);
}

void StateTriggerPanel::pushResumeNavigation()
{
  RCLCPP_INFO(node_->get_logger(), "Service call: resume waypoints navigation");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  resume_client_->async_send_request(request);
}

void StateTriggerPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void StateTriggerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace rviz2_plugins

CLASS_LOADER_REGISTER_CLASS(rviz2_plugins::StateTriggerPanel, rviz_common::Panel)
