#include "rviz2_plugins/set_finish_panel.h"

namespace rviz2_plugins
{

SetFinishPanel::SetFinishPanel(QWidget* parent)
  : rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>("set_finish_panel"))
{
  // Create GUI layout
  finish_button_ = new QPushButton("Set Finish Pose Here", this);

  // Vertical layout for buttons
  auto* layout = new QVBoxLayout;
  layout->addWidget(finish_button_);
  setLayout(layout);

  // ROS2 service clients
  finish_client_ = node_->create_client<std_srvs::srv::Trigger>("set_finish_pose");

  // Connect buttons to slots
  connect(finish_button_, SIGNAL(clicked()), this, SLOT(pushSetFinishPose()));
}

SetFinishPanel::~SetFinishPanel() = default;

void SetFinishPanel::pushSetFinishPose()
{
  RCLCPP_INFO(node_->get_logger(), "Service call: Set finish pose");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  finish_client_->async_send_request(request);
}

void SetFinishPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void SetFinishPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace rviz2_plugins

CLASS_LOADER_REGISTER_CLASS(rviz2_plugins::SetFinishPanel, rviz_common::Panel)
