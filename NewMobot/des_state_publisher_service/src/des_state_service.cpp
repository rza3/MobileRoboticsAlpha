#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <traj_builder/traj_builder.h> 
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>



geometry_msgs::Twist g_halt_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

double g_dt= 0.75;
ros::Publisher g_des_state_publisher;
ros::Publisher g_des_psi_publisher;
ros::Publisher g_des_mode_publisher;
//ros::Publisher g_des_mode1_publisher;
//ros::Publisher g_des_mode0_publisher;
ros::Subscriber g_lidar_alarm_subscriber;
ros::Subscriber g_current_state_subscriber;
std_msgs::Int8 mode_msg;

bool g_lidar_alarm;

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
    TrajBuilder trajBuilder; //instantiate one of these
    g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(0);
        
}
void lidarAlarmCallback(const std_msgs::Bool& alarm_msg){
    g_lidar_alarm = alarm_msg.data;
    if(g_lidar_alarm){
        ROS_INFO("LIDAR alarm detected");
    }
}
bool callback(des_state_publisher_service::NavSrvRequest& request, des_state_publisher_service::NavSrvResponse& response)
{
        //ROS_INFO("callback_activated");
        double pose_x = request.x;
        double pose_y = request.y;
        double pose_psi = request.psi;
        int pose_mode = request.mode;
        if(pose_mode >-1)
            mode_msg.data = pose_mode;
        else
            mode_msg.data = 1;
        
        
        ros::Rate looprate(1 / g_dt); //timer for fixed publication rate   
        TrajBuilder trajBuilder; //instantiate one of these
        trajBuilder.set_dt(g_dt); //make sure trajectory builder and main use the same time step
        trajBuilder.set_alpha_max(1.0);
        //hard code two poses; more generally, would get poses from a nav_msgs/Path message.
        /*double psi_start = 0.0;// what is the start point should be??
        double psi_end = pose_psi;
        g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
        g_end_state = g_start_state;
        g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
*/
        //CHANGE THESE TO CURRENT POSE!!!
        /*
        g_start_pose.pose.position.x = 0.0;
        g_start_pose.pose.position.y = 0.0;
        g_start_pose.pose.position.z = 0.0;
        g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);*/
        g_end_pose = g_start_pose; //includes copying over twist with all zeros
        //don't really care about orientation, since this will follow from 
        // point-and-go trajectory; 
        //if (pose_mode==1){
        //    g_end_pose.pose.orientation=trajBuilder.convertPlanarPsi2Quaternion(psi_end);
        //    pose_mode1.data = true;

        //}
        //else if(pose_mode==0){

         //   g_end_pose.pose.position.x = pose_x; 
         //   g_end_pose.pose.position.y = pose_y; 
         //   pose_mode0.data = true;

       // }
       if(!g_lidar_alarm & pose_mode>-1){
           g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(pose_psi);
           g_end_pose.pose.position.x = pose_x; 
           g_end_pose.pose.position.y = pose_y; 
           ROS_INFO("Not Stopping");
       }
       if(g_lidar_alarm){
           ROS_INFO("STOPPING");
       }
        
        
        double des_psi;
        std_msgs::Float64 psi_msg;
        std::vector<nav_msgs::Odometry> vec_of_states;
         //trajBuilder.build_triangular_spin_traj(g_start_pose,g_end_pose,vec_of_states);
        //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

        nav_msgs::Odometry des_state;
        nav_msgs::Odometry last_state;
        geometry_msgs::PoseStamped last_pose;

        //while (ros::ok()) {
        //ROS_INFO("building traj from start to end");
        //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
        if(pose_mode == 1)
            trajBuilder.build_spin_traj(g_start_pose, g_end_pose, vec_of_states);
        else
            trajBuilder.build_travel_traj(g_start_pose, g_end_pose, vec_of_states);
        // Test if this works for backing up.
        //ROS_INFO("publishing desired states ");
        for (int i = 0; i < vec_of_states.size(); i++) {
            des_state = vec_of_states[i];
            des_state.header.stamp = ros::Time::now();
            g_des_state_publisher.publish(des_state);
            //des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
            //psi_msg.data = des_psi;
            //g_des_psi_publisher.publish(psi_msg);
            ROS_WARN("MODE IS [%i]",mode_msg.data);
            g_des_mode_publisher.publish(mode_msg);
            //g_des_mode1_publisher.publish(pose_mode1);
            //g_des_mode0_publisher.publish(pose_mode0);
            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
        last_state = vec_of_states.back();
        g_start_pose.header = last_state.header;
        g_start_pose.pose = last_state.pose.pose;
        //ROS_INFO("publishing desired states12345678 ");

    //}
        response.alarm = false;
        response.failed = false;

        if(g_lidar_alarm){
           response.alarm = true;
           response.failed = true;
       }

        return true;
}


void currStateCallback(const nav_msgs::Odometry& curr_state){
    g_start_pose.pose.position.x = curr_state.pose.pose.position.x;;
    g_start_pose.pose.position.y = curr_state.pose.pose.position.y;
    g_start_pose.pose.position.z = curr_state.pose.pose.position.z;
    g_start_pose.pose.orientation = curr_state.pose.pose.orientation;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "des_state_publisher_service");
    ros::NodeHandle n;
    g_current_state_subscriber = n.subscribe("/current_state",1,currStateCallback);
    g_lidar_alarm_subscriber = n.subscribe("lidar_alarm",1,lidarAlarmCallback);
    ros::ServiceServer service = n.advertiseService("des_pose_service",callback);
    g_des_state_publisher = n.advertise<nav_msgs::Odometry>("/desState", 1);
    g_des_psi_publisher = n.advertise<std_msgs::Float64>("/desPsi", 1);
    g_des_mode_publisher = n.advertise<std_msgs::Int8>("/desMode",1);
    //g_des_mode1_publisher = n.advertise<std_msgs::Bool>("/des_mode1",1);
    //g_des_mode0_publisher = n.advertise<std_msgs::Bool>("/des_mode0",1);
    
    
    ros::spin();
    return 0;
}
