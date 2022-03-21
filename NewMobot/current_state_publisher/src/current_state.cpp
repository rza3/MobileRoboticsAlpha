#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//node to receive desired states and republish the twist as cmd_vel commands
ros::Publisher g_state_republisher;

//simply copy the desired twist and republish it to cmd_vel
void odomStateCallback(const nav_msgs::Odometry& odom_state) {
    nav_msgs::Odometry current_state = odom_state;
    g_state_republisher.publish(current_state);    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "current_state"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    g_state_republisher = n.advertise<nav_msgs::Odometry>("/current_state", 1);
    ros::Subscriber odom_state_subscriber = n.subscribe("/odom",1,odomStateCallback); 
    ros::spin();
}