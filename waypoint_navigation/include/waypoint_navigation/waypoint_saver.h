#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include "waypoint.h"  // include Waypoint class

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
namespace fs = std::filesystem;

class WaypointsSaver : public rclcpp::Node
{
public:
  WaypointsSaver();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_server_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr waypoints_joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoints_viz_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_saved_time_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::vector<Waypoint> waypoints_;
  std::vector<visualization_msgs::msg::Marker> markers_;
  std::string filename_;
  std::string world_frame_;
  std::string robot_frame_;
  int save_joy_button_;
  float default_rad_;

  bool finishPoseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void waypointsJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void waypointsVizCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  bool getCurrentPose(geometry_msgs::msg::TransformStamped& tf_stamped);
  void addWaypointMarker(Waypoint& point);
  void publishMarkerArray();
  void save(geometry_msgs::msg::TransformStamped& finish_pose);
  void copyToSrc();
};
