#include "waypoint_navigation/waypoint_nav.h"

WaypointsNavigation::WaypointsNavigation() : Node("waypoint_nav"), nav_time_(0), wp_num_(0), has_activate_(false)
{
  // Parameters
  this->declare_parameter<std::string>("waypoints_file", "");
  this->declare_parameter<std::string>("world_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_footprint");
  this->declare_parameter<float>("min_dist_err", 0.3);
  this->declare_parameter<float>("min_yaw_err", 0.3);

  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("min_dist_err", min_dist_err_);
  this->get_parameter("min_yaw_err", min_yaw_err_);

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
  wp_num_pub_ = this->create_publisher<std_msgs::msg::UInt16>("waypoint_num", 10);

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
  nav_status_ = rclcpp_action::ResultCode::UNKNOWN;

  // Main loop runs at 10hz
  timer_ = this->create_wall_timer(100ms, std::bind(&WaypointsNavigation::execLoop, this));
  waitActionServer();
  RCLCPP_INFO(this->get_logger(), "Successfully connected to server");
  RCLCPP_INFO(this->get_logger(), "Waiting for waypoint navigation to start");
}

bool WaypointsNavigation::startNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (has_activate_ || (wp_num_ > 0))
  {
    RCLCPP_WARN(this->get_logger(), "Waypoint navigation is already started");
    response->success = false;
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Navigation is started now");
  wp_num_ = 1;
  current_wp_ = pose_array_.poses.begin();
  sendGoal(*current_wp_);
  response->success = true;
  has_activate_ = true;
  return true;
}

bool WaypointsNavigation::resumeNavCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (has_activate_)
  {
    RCLCPP_WARN(this->get_logger(), "Navigation is already active");
    response->success = false;
  }
  else if ((wp_num_ > 0) && (current_wp_ <= finish_pose_))
  {
    RCLCPP_INFO(this->get_logger(), "Navigation has resumed");
    sendGoal(*current_wp_);
    response->success = true;
    has_activate_ = true;
  }
  return true;
}

void WaypointsNavigation::responseCallback(const GoalHandleNavToPose::SharedPtr& future)
{
  auto goal_handle = future.get();
  bool is_valid_goal_handle = static_cast<bool>(goal_handle);
  if (!is_valid_goal_handle)
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
}

void WaypointsNavigation::feedbackCallback(GoalHandleNavToPose::SharedPtr /*p*/,
                                           const std::shared_ptr<const NavToPose::Feedback>& feedback)
{
  // Probably about 100 Hz
  current_pose_ = feedback->current_pose.pose;
  nav_time_ = feedback->navigation_time.sec;
}

void WaypointsNavigation::resultCallback(const GoalHandleNavToPose::WrappedResult& result)
{
  nav_status_ = result.code;
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
  if (waypoint_list_[wp_num_ - 1].stop)
  {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(goal_pose.orientation, tf2_quat);
    tf2::Matrix3x3 m(tf2_quat);
    double r, p, y;
    m.getRPY(r, p, y);
    target_yaw_ = y;
  }
  // Send goal to action server
  nav_status_ = rclcpp_action::ResultCode::UNKNOWN;
  client_ptr_->async_send_goal(goal_msg, send_goal_opts_);
  if (current_wp_ == finish_pose_)
    RCLCPP_INFO(this->get_logger(), "Go to final goal");
  else
    RCLCPP_INFO(this->get_logger(), "Go to waypoint %d", wp_num_);
}

bool WaypointsNavigation::onNavPoint(const geometry_msgs::msg::Pose& goal_pose)
{
  const geometry_msgs::msg::Pose robot_pose = current_pose_;
  float x_diff = goal_pose.position.x - robot_pose.position.x;
  float y_diff = goal_pose.position.y - robot_pose.position.y;
  float dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  float dist_err = waypoint_list_[wp_num_ - 1].rad;
  if (waypoint_list_[wp_num_ - 1].stop)
  {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(robot_pose.orientation, tf2_quat);
    tf2::Matrix3x3 m(tf2_quat);
    double r, p, robot_yaw;
    m.getRPY(r, p, robot_yaw);
    double yaw_diff = std::abs(target_yaw_ - robot_yaw);
    return ((dist < dist_err) && (yaw_diff < min_yaw_err_)) || (nav_status_ == rclcpp_action::ResultCode::SUCCEEDED);
  }
  return dist < dist_err;
}

void WaypointsNavigation::execLoop()
{
  // Publish waypoints to be displayed as arrows on rviz2
  pose_array_.header.stamp = now();
  wp_vis_pub_->publish(pose_array_);
  // Publish num of current waypoint
  std_msgs::msg::UInt16 msg;
  msg.data = wp_num_;
  wp_num_pub_->publish(msg);

  // If has_activate is false, nothing to do
  if (!has_activate_)
    return;

  if (onNavPoint(*current_wp_))
  {
    // Reached current waypoint
    if (current_wp_ < finish_pose_)
    {
      bool stop = waypoint_list_[wp_num_ - 1].stop;
      // If current waypoint is stop point
      if (stop)
      {
        client_ptr_->async_cancel_all_goals();
        has_activate_ = false;
        RCLCPP_INFO(this->get_logger(), "Waiting for navigation to resume...");
      }
      // Update current waypoint
      current_wp_++;
      wp_num_++;
      // Send next goal
      if (!stop)
        sendGoal(*current_wp_);
    }
    // Reached final goal
    else if (current_wp_ == finish_pose_)
    {
      RCLCPP_INFO(this->get_logger(), "Final goal reached!!");
      has_activate_ = false;
      rclcpp::shutdown();
    }
  }
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
