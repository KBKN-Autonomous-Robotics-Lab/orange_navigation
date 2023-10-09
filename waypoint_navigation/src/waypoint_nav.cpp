#include "waypoint_navigation/waypoint_nav.h"

WaypointsNavigation::WaypointsNavigation() : Node("waypoint_nav"), has_activate_(false)
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

  // Service
  start_server_ = create_service<std_srvs::srv::Trigger>(
      "start_wp_nav", std::bind(&WaypointsNavigation::startNavCallback, this, _1, _2));
  resume_server_ = create_service<std_srvs::srv::Trigger>(
      "resume_nav", std::bind(&WaypointsNavigation::resumeNavCallback, this, _1, _2));

  // Nav2 action
  client_ptr_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
  send_goal_opts_.goal_response_callback = std::bind(&WaypointsNavigation::responseCallback, this, _1);
  send_goal_opts_.feedback_callback = std::bind(&WaypointsNavigation::feedbackCallback, this, _1, _2);
  send_goal_opts_.result_callback = std::bind(&WaypointsNavigation::resultCallback, this, _1);

  // Main loop runs at 10hz
  timer_ = this->create_wall_timer(1000ms, std::bind(&WaypointsNavigation::exec_loop, this));
  waitActionServer();
  RCLCPP_INFO(this->get_logger(), "Successfully connected to server");
  RCLCPP_INFO(this->get_logger(), "Waiting for waypoint navigation to start");
}

bool WaypointsNavigation::startNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> responce)
{
  if (has_activate_ || (current_wp_ != pose_array_.poses.begin()))
  {
    RCLCPP_WARN(this->get_logger(), "Waypoint navigation is already started");
    responce->success = false;
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation is started now");
  has_activate_ = true;
  responce->success = true;
  return true;
}

bool WaypointsNavigation::resumeNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> responce)
{
  if (has_activate_)
  {
    RCLCPP_WARN(this->get_logger(), "Navigation is already active");
    responce->success = false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Navigation has resumed");
    responce->success = true;
    has_activate_ = true;
  }
  return true;
}

void WaypointsNavigation::responseCallback(const GoalHandleNavToPose::SharedPtr future)
{
}

void WaypointsNavigation::feedbackCallback(GoalHandleNavToPose::SharedPtr,
                                           const std::shared_ptr<const NavToPose::Feedback> feedback)
{
}

void WaypointsNavigation::resultCallback(const GoalHandleNavToPose::WrappedResult& result)
{
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
    RCLCPP_ERROR(this->get_logger(), "No points in \"waypoints\"");
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

void WaypointsNavigation::waitActionServer()
{
  while (rclcpp::ok() && (!client_ptr_->wait_for_action_server(5s)))
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    rclcpp::sleep_for(1s);
  }
}

void WaypointsNavigation::sendGoal(const geometry_msgs::msg::Pose& goal_pose)
{
  // Create action goal
  auto goal_msg = NavToPose::Goal();
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.header.frame_id = world_frame_;
  goal_msg.pose.pose.position.x = goal_pose.position.x;
  goal_msg.pose.pose.position.y = goal_pose.position.y;
  goal_msg.pose.pose.position.z = goal_pose.position.z;
  goal_msg.pose.pose.orientation.x = goal_pose.orientation.x;
  goal_msg.pose.pose.orientation.y = goal_pose.orientation.y;
  goal_msg.pose.pose.orientation.z = goal_pose.orientation.z;
  goal_msg.pose.pose.orientation.w = goal_pose.orientation.w;
  // Send goal to action server
  client_ptr_->async_send_goal(goal_msg, send_goal_opts_);
}

void WaypointsNavigation::exec_loop()
{
  // has_activate_ is false, nothing to do
  if (!has_activate_) {}
  // go to current waypoint
  else if (current_wp_ < finish_pose_) {}
  // go to fianal goal and finish process
  else if (current_wp_ == finish_pose_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Go to final goal");
  }

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
