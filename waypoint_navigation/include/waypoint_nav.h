#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <waypoint.h>  // include Waypoint class

#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>


#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif


class SwitchRunningStatus : public std::exception {
public:
    SwitchRunningStatus() : std::exception() { }
};


class WaypointsNavigation {
public:
    WaypointsNavigation();
    
    bool readFile(const std::string &filename);
    void computeWpOrientation();
    bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
    bool resumeNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
    bool stopNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
    void laserscan_callback(const sensor_msgs::LaserScan &msg);
    void sendNoObstacleGoal(float first_thresh, float after_second_thresh, float goal_thresh, const geometry_msgs::Pose &dest);
    bool navigationFinished();
    bool onNavigationPoint(const geometry_msgs::Point &dest, double dist_err=1.0);
    tf::StampedTransform getRobotPosGL();
    void sleep();
    void publishPoseArray();
    void startNavigationGL(const geometry_msgs::Pose &dest);
    void setMaxVel();
    void run();

private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    geometry_msgs::PoseArray waypoints_;
    visualization_msgs::MarkerArray marker_;
    std::vector<Waypoint> waypoint_list_;
    
    //Scripts of "start from the middle"
    //Edited mori 2022/10/19
    //#############################################################
    bool StartFromTheMiddle;
    std::vector<geometry_msgs::Pose>::iterator init_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator compare_waypoint_;
    //#############################################################
    
    std::vector<geometry_msgs::Pose>::iterator current_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator finish_pose_;
    bool has_activate_, stopped_;
    std::string robot_frame_, world_frame_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, stop_server_, resume_server_;
    ros::Subscriber cmd_vel_sub_,scan_sub_, move_base_status_sub_;
    ros::Publisher wp_pub_, max_vel_pub_, wp_num_pub_;
    ros::ServiceClient clear_costmaps_srv_;
    tf::TransformListener tf_listener_;
    std_msgs::UInt16 wp_num_;
    std_msgs::Float32 max_vel_msg_;
    sensor_msgs::LaserScan laserscan;
    double last_moved_time_, dist_err_, min_dist_err_, target_yaw_, min_yaw_err_;
};
