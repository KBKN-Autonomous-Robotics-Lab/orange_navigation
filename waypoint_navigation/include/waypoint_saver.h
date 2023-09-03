#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <fstream>
#include <string>
#include <waypoint.h>  // include Waypoint class


class WaypointsSaver {
public:
    WaypointsSaver();
    
    void waypointsJoyCallback(const sensor_msgs::Joy &msg);
    void waypointsVizCallback(const geometry_msgs::PointStamped &msg);
    void finishPoseCallback(const geometry_msgs::PoseStamped &msg);
    void pushbackWaypoint();
    void addWaypointMarker(Waypoint point);
    void publishMarkerArray();
    void save();

private:
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::Subscriber finish_pose_sub_;
    ros::Publisher markers_pub_;
    std::vector<Waypoint> waypoints_;
    std::vector<visualization_msgs::Marker> markers_;
    geometry_msgs::PoseStamped finish_pose_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
    float default_rad_;
};
