#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>
#include <vector>
#include "waypoint_navigation/waypoint.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

using NavToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;

class WaypointsNavigation : public rclcpp::Node
{
public:
  WaypointsNavigation();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_server_, resume_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wp_vis_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr wp_num_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavToPose>::SharedPtr client_ptr_;
  rclcpp_action::Client<NavToPose>::SendGoalOptions send_goal_opts_;
  rclcpp_action::ResultCode nav_status_;

  std::vector<Waypoint> waypoint_list_;
  geometry_msgs::msg::PoseArray pose_array_;
  geometry_msgs::msg::Pose current_pose_;
  std::vector<geometry_msgs::msg::Pose>::iterator current_wp_, finish_pose_;
  std::string robot_frame_, world_frame_;
  int32_t nav_time_;
  uint16_t wp_num_;
  float target_yaw_, min_dist_err_, min_yaw_err_;
  bool has_activate_;

  // Service callback functions
  bool startNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> responce);
  bool resumeNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> responce);
  // Nav2 action callback functions
  void responseCallback(const GoalHandleNavToPose::SharedPtr& future);
  void feedbackCallback(GoalHandleNavToPose::SharedPtr p, const std::shared_ptr<const NavToPose::Feedback>& feedback);
  void resultCallback(const GoalHandleNavToPose::WrappedResult& result);
  // Other functions
  bool readFile(const std::string& wp_file_path);
  bool contains(const YAML::Node& node, const std::string& key);
  void setWpOrientation();
  void waitActionServer();
  void sendGoal(const geometry_msgs::msg::Pose& goal_pose);
  bool onNavPoint(const geometry_msgs::msg::Pose& goal_pose);
  void execLoop();
};
