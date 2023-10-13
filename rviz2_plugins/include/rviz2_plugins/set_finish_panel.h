#pragma once

#include <memory>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/load_resource.hpp>

#include <class_loader/class_loader.hpp>

class QPushButton;

namespace rviz2_plugins
{

class SetFinishPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  SetFinishPanel(QWidget* parent = nullptr);
  ~SetFinishPanel() override;

  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void pushSetFinishPose();

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_client_;
  QPushButton* finish_button_;

  rclcpp::Node::SharedPtr node_;
};

}  // namespace rviz2_plugins
