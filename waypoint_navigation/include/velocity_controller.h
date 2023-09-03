#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>
#include <string>


class VelocityController
{
public:
    VelocityController();

    void maxVelCallback(const std_msgs::Float32 &msg);
    void cmdVelCallback(const geometry_msgs::Twist &msg);

private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    ros::Subscriber max_vel_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist pub_msg_;
    float standard_vel_, current_max_vel_, min_vel_;
};
