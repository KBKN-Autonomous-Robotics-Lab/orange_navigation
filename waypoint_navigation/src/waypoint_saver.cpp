#include "waypoint_navigation/waypoint_saver.h"

WaypointsSaver::WaypointsSaver() : Node("waypoint_saver")
{
  // Parameters
  this->declare_parameter<std::string>("filename", "waypoints.yaml");
  this->declare_parameter<std::string>("world_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_footprint");
  this->declare_parameter<int>("save_joy_button", 0);
  this->declare_parameter<float>("default_rad", 0.8);

  this->get_parameter("filename", filename_);
  this->get_parameter("save_joy_button", save_joy_button_);
  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("default_rad", default_rad_);

  // tf2 setup
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Server, Publisher, Subscriber
  finish_server_ = this->create_service<std_srvs::srv::Trigger>(
      "set_finish_pose", std::bind(&WaypointsSaver::finishPoseCallback, this, _1, _2));
  waypoints_viz_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "waypoints_viz", 1, std::bind(&WaypointsSaver::waypointsVizCallback, this, _1));
  waypoints_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "waypoints_joy", 1, std::bind(&WaypointsSaver::waypointsJoyCallback, this, _1));
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", 10);

  // Publish markers to rviz at 10hz
  timer_ = this->create_wall_timer(100ms, std::bind(&WaypointsSaver::publishMarkerArray, this));

  last_saved_time_ = now();
}

bool WaypointsSaver::finishPoseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /*request*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  if (!getCurrentPose(tf_stamped))
    return false;
  try
  {
    save(tf_stamped);
    RCLCPP_INFO(this->get_logger(), "Write success!");
    response->success = true;
    response->message = "Waypoints file saved at " + filename_;
    rclcpp::shutdown();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    response->success = false;
    return false;
  }
  return true;
}

void WaypointsSaver::waypointsJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // If save_joy_button is not pushed or within 3 seconds after last pushed
  if ((msg->buttons[save_joy_button_] != 1) || ((now() - last_saved_time_).seconds() < 3.0))
    return;
  // Get the coordinates of the current location using tf
  geometry_msgs::msg::TransformStamped tf_stamped;
  if (!getCurrentPose(tf_stamped))
    return;
  // Add waypoint
  Waypoint point;
  point.x = tf_stamped.transform.translation.x;
  point.y = tf_stamped.transform.translation.y;
  point.z = tf_stamped.transform.translation.z;
  point.rad = default_rad_;
  waypoints_.push_back(point);
  addWaypointMarker(point);
  last_saved_time_ = now();
}

void WaypointsSaver::waypointsVizCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Add waypoint
  Waypoint point;
  point.x = msg->point.x;
  point.y = msg->point.y;
  point.z = msg->point.z;
  point.rad = default_rad_;
  waypoints_.push_back(point);
  addWaypointMarker(point);
  last_saved_time_ = now();
}

bool WaypointsSaver::getCurrentPose(geometry_msgs::msg::TransformStamped& tf_stamped)
{
  try
  {
    tf_stamped = tf_buffer_->lookupTransform(world_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "tf2::TransformException: %s", e.what());
    return false;
  }
  return true;
}

void WaypointsSaver::addWaypointMarker(Waypoint point)
{
  float scale = 0.2;
  size_t number = waypoints_.size();
  std::stringstream name;
  name << "Waypoint " << number;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = now();
  marker.ns = name.str();
  marker.id = number;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = scale / 2;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 0.8f;
  marker.color.g = 0.2f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0f;

  markers_.push_back(marker);
}

void WaypointsSaver::publishMarkerArray()
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers = markers_;
  markers_pub_->publish(marker_array);
}

void WaypointsSaver::save(geometry_msgs::msg::TransformStamped& finish_pose)
{
  std::ofstream ofs(filename_.c_str(), std::ios::out);
  /*
  waypoints:
  - point: {x: *, y: *, z: *, ...}
  */
  ofs << "waypoints:" << std::endl;
  for (size_t i = 0; i < waypoints_.size(); i++)
  {
    ofs << "- point: " << waypoints_[i].getYAMLstr() << std::endl;
  }
  /*
  finish_pose:
    header: {seq: *, nanoseq: *, frame_id: *}
    pose:
      positioin: {x: *, y: *, z: *}
      orientation: {x: *, y: *, z: *, w: *}
  */
  ofs << "finish_pose:" << std::endl;
  ofs << "  header: {";
  ofs << "seq: " << finish_pose.header.stamp.sec << ", ";
  ofs << "stamp: " << finish_pose.header.stamp.nanosec << ", ";
  ofs << "frame_id: " << finish_pose.header.frame_id << "}" << std::endl;
  ofs << "  pose: " << std::endl;
  ofs << "    position: {";
  ofs << "x: " << finish_pose.transform.translation.x << ", ";
  ofs << "y: " << finish_pose.transform.translation.y << ", ";
  ofs << "z: " << finish_pose.transform.translation.z << "}" << std::endl;
  ofs << "    orientation: {";
  ofs << "x: " << finish_pose.transform.rotation.x << ", ";
  ofs << "y: " << finish_pose.transform.rotation.y << ", ";
  ofs << "z: " << finish_pose.transform.rotation.z << ", ";
  ofs << "w: " << finish_pose.transform.rotation.w << "}\n" << std::endl;

  ofs.close();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointsSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
