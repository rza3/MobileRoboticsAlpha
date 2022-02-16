#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <traj_builder/traj_builder.h> 
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


geometry_msgs::Twist g_halt_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

void do_inits() { //similar to a constructor  
    //define a halt state; zero speed and spin, and fill with viable coords
    g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;

    //default values; can be overridden
    g_start_state.twist.twist = g_halt_twist;
    g_start_state.pose.pose.position.x = 0;
    g_start_state.pose.pose.position.y = 0;
    g_start_state.pose.pose.position.z = 0;
}

bool callback(des_state_publisher_service::NavSrvRequest& request, des_state_publisher_service::NavSrvResponse& response)
{
        ROS_INFO("callback_activated");
        double pose_x = request.x[0];
        double pose_y = request.y[0];
        double pose_psi = request.psi[0];
        
        ros::Rate looprate(1 / dt); //timer for fixed publication rate   
        TrajBuilder trajBuilder; //instantiate one of these
        trajBuilder.set_dt(dt); //make sure trajectory builder and main use the same time step
        trajBuilder.set_alpha_max(1.0);
        //hard code two poses; more generally, would get poses from a nav_msgs/Path message.
        double psi_start = 0.0;// what is the start point should be??
        double psi_end = pose_psi;
        g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
        g_end_state = g_start_state;
        g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);


        g_start_pose.pose.position.x = 0.0;
        g_start_pose.pose.position.y = 0.0;
        g_start_pose.pose.position.z = 0.0;
        g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
        g_end_pose = g_start_pose; //includes copying over twist with all zeros
        //don't really care about orientation, since this will follow from 
        // point-and-go trajectory; 
        g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
        g_end_pose.pose.position.x = pose_x; 
        g_end_pose.pose.position.y = pose_y; 

        double des_psi;
        std_msgs::Float64 psi_msg;
        std::vector<nav_msgs::Odometry> vec_of_states;
         //trajBuilder.build_triangular_spin_traj(g_start_pose,g_end_pose,vec_of_states);
        //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

        nav_msgs::Odometry des_state;
        nav_msgs::Odometry last_state;
        geometry_msgs::PoseStamped last_pose;

        while (ros::ok()) {
        ROS_INFO("building traj from start to end");
        trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
        ROS_INFO("publishing desired states ");
        for (int i = 0; i < vec_of_states.size(); i++) {
            des_state = vec_of_states[i];
            des_state.header.stamp = ros::Time::now();
            des_state_publisher.publish(des_state);
            des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
            psi_msg.data = des_psi;
            des_psi_publisher.publish(psi_msg);

            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
        last_state = vec_of_states.back();
        g_start_pose.header = last_state.header;
        g_start_pose.pose = last_state.pose.pose;
    }
}




int main (int argc, char **argv)
{
    ros::init(argc, argv, "des_state_pub");
    ros::NodeHandle n;
    double dt= 0.02;
    ros::Publisher des_state_publisher = n.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher des_psi_publisher = n.advertise<std_msgs::Float64>("/desPsi", 1);

    ros::ServiceServer service = n.advertiseService("des_pose_service",callback);

    ros::spin();
    return 0;
}