#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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
    bool readFile();
    void run();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoint_list_;
    geometry_msgs::msg::PoseArray pose_array_;
    
};
