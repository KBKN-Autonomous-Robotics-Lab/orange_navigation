#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>
#include <vector>
#include "waypoint_navigation/waypoint.h"

using namespace std::chrono_literals;
using PoseArray = geometry_msgs::msg::PoseArray;
using Trigger = std_srvs::srv::Trigger;

class WaypointsNavigation : public rclcpp::Node
{
public:
  WaypointsNavigation();

private:
  rclcpp::Service<Trigger>::SharedPtr start_server_, resume_server_;
  rclcpp::Publisher<PoseArray>::SharedPtr wp_vis_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Waypoint> waypoint_list_;
  PoseArray pose_array_;
  std::vector<geometry_msgs::msg::Pose>::iterator current_wp_, finish_pose_;
  std::string robot_frame_, world_frame_;
  bool has_activate_;

  bool startNavCallback(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> responce);
  bool resumeNavCallback(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> responce);

  bool readFile(const std::string& wp_file_path);
  bool contains(const YAML::Node& node, const std::string& key);
  void setWpOrientation();
  void run();
};
