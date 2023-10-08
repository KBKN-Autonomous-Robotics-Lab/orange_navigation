#include "waypoint_navigation/waypoint_nav.h"


WaypointsNavigation::WaypointsNavigation() : Node("waypoint_nav")
{
    // Parameters
    this->declare_parameter<std::string>("waypoints_file", "");

    // Main loop runs at 10hz
    timer_ = this->create_wall_timer(
        1s, std::bind(&WaypointsNavigation::run, this));
}


bool WaypointsNavigation::readFile()
{
    // Load YAML file
    std::string waypoints_file = "";
    this->get_parameter("waypoints_file", waypoints_file);
    YAML::Node node;
    try
    {
        node = YAML::LoadFile(waypoints_file);
    }
    catch(const YAML::BadFile &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read waypoints file: %s", waypoints_file.c_str());
        return false;
    }
    // Read waypoints
    const YAML::Node &waypoints = node["waypoints"];
    size_t num_wp = waypoints.size();
    if (num_wp == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No \"waypoints\" key in yaml file.");
        return false;
    }
    waypoint_list_.resize(num_wp+1);
    pose_array_.poses.resize(num_wp+1);
    for (size_t i = 0; i < num_wp; i++)
    {
        waypoint_list_[i].x = waypoints[i]["point"]["x"].as<float>();
        waypoint_list_[i].y = waypoints[i]["point"]["y"].as<float>();
        waypoint_list_[i].z = waypoints[i]["point"]["z"].as<float>();
        waypoint_list_[i].vel = waypoints[i]["point"]["vel"].as<float>();
        waypoint_list_[i].rad = waypoints[i]["point"]["rad"].as<float>();
        waypoint_list_[i].stop = waypoints[i]["point"]["stop"].as<bool>();
    }
    
    return true;
}


void WaypointsNavigation::run()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsNavigation>();
    if (node->readFile())
    {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}