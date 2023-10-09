#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>
#include <vector>
#include "waypoint_navigation/waypoint.h"

using namespace std::chrono_literals;

class WaypointsNavigation : public rclcpp::Node
{
public:
  WaypointsNavigation();
  bool readFile(const std::string& wp_file_path);
  void run();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wp_vis_pub_;
  std::vector<Waypoint> waypoint_list_;
  geometry_msgs::msg::PoseArray pose_array_;
  std::vector<geometry_msgs::msg::Pose>::iterator current_wp_, finish_pose_;
  std::string robot_frame_, world_frame_;

  bool contains(const YAML::Node& node, const std::string& key);
  void setWpOrientation();
};
