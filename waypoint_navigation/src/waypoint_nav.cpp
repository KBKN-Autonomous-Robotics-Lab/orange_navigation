#include "waypoint_navigation/waypoint_nav.h"

WaypointsNavigation::WaypointsNavigation() : Node("waypoint_nav")
{
  // Parameters
  this->declare_parameter<std::string>("waypoints_file", "");
  this->declare_parameter<std::string>("world_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_footprint");

  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("robot_frame", robot_frame_);

  // Load YAML file
  std::string waypoints_file = "";
  this->get_parameter("waypoints_file", waypoints_file);
  if (!readFile(waypoints_file))
  {
    throw std::runtime_error("Failed to read waypoints yaml file");
  }
  current_wp_ = pose_array_.poses.begin();
  finish_pose_ = pose_array_.poses.end() - 1;
  setWpOrientation();

  // Publisher
  wp_vis_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/waypoints", 10);

  // Main loop runs at 10hz
  timer_ = this->create_wall_timer(1000ms, std::bind(&WaypointsNavigation::run, this));
}

bool WaypointsNavigation::readFile(const std::string& wp_file_path)
{
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(wp_file_path);
  }
  catch (const YAML::BadFile& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to read waypoints file: %s", wp_file_path.c_str());
    return false;
  }

  // Read waypoints
  const YAML::Node& wp_node = node["waypoints"];
  size_t num_wp = wp_node.size();
  if (num_wp == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No points in \"waypoints\".");
    return false;
  }
  waypoint_list_.resize(num_wp + 1);
  pose_array_.poses.resize(num_wp + 1);
  for (size_t i = 0; i < num_wp; i++)
  {
    waypoint_list_[i].x = wp_node[i]["point"]["x"].as<float>();
    waypoint_list_[i].y = wp_node[i]["point"]["y"].as<float>();
    waypoint_list_[i].z = wp_node[i]["point"]["z"].as<float>();
    if (contains(wp_node[i]["point"], "vel"))
    {
      waypoint_list_[i].vel = wp_node[i]["point"]["vel"].as<float>();
    }
    if (contains(wp_node[i]["point"], "rad"))
    {
      waypoint_list_[i].rad = wp_node[i]["point"]["rad"].as<float>();
    }
    if (contains(wp_node[i]["point"], "stop"))
    {
      waypoint_list_[i].stop = wp_node[i]["point"]["stop"].as<bool>();
    }
    pose_array_.poses[i].position.x = waypoint_list_[i].x;
    pose_array_.poses[i].position.y = waypoint_list_[i].y;
    pose_array_.poses[i].position.z = waypoint_list_[i].z;
  }

  // Read finish_pose
  const YAML::Node& fp_node = node["finish_pose"];
  waypoint_list_[num_wp].x = fp_node["pose"]["position"]["x"].as<float>();
  waypoint_list_[num_wp].y = fp_node["pose"]["position"]["y"].as<float>();
  waypoint_list_[num_wp].z = fp_node["pose"]["position"]["z"].as<float>();
  waypoint_list_[num_wp].stop = true;
  pose_array_.poses[num_wp].position.x = waypoint_list_[num_wp].x;
  pose_array_.poses[num_wp].position.y = waypoint_list_[num_wp].y;
  pose_array_.poses[num_wp].position.z = waypoint_list_[num_wp].z;
  pose_array_.poses[num_wp].orientation.x = fp_node["pose"]["orientation"]["x"].as<float>();
  pose_array_.poses[num_wp].orientation.y = fp_node["pose"]["orientation"]["y"].as<float>();
  pose_array_.poses[num_wp].orientation.z = fp_node["pose"]["orientation"]["z"].as<float>();
  pose_array_.poses[num_wp].orientation.w = fp_node["pose"]["orientation"]["w"].as<float>();

  return true;
}

bool WaypointsNavigation::contains(const YAML::Node& node, const std::string& key)
{
  try
  {
    node[key].as<std::string>();  // access element
  }
  catch (const YAML::InvalidNode& e)
  {
    return false;  // if the key is not in node, YAML::InvalidNode error is throwed
  }
  return true;
}

void WaypointsNavigation::setWpOrientation()
{
  std::vector<geometry_msgs::msg::Pose>::iterator poses_begin, it;
  poses_begin = pose_array_.poses.begin();
  float goal_yaw;
  tf2::Quaternion tf2_quat;
  for (it = poses_begin; it != finish_pose_; it++)
  {
    goal_yaw = atan2((it + 1)->position.y - (it)->position.y, (it + 1)->position.x - (it)->position.x);
    tf2_quat.setRPY(0, 0, goal_yaw);
    it->orientation = tf2::toMsg(tf2_quat);
  }
  pose_array_.header.frame_id = world_frame_;
}

void WaypointsNavigation::run()
{
  // RCLCPP_INFO(this->get_logger(), "Waiting for action server...");

  // Publish waypoints to be displayed as arrows on rviz2
  pose_array_.header.stamp = now();
  wp_vis_pub_->publish(pose_array_);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<WaypointsNavigation>();
    rclcpp::spin(node);
  }
  catch (const std::runtime_error& e)
  {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
