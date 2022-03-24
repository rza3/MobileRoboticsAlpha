#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

double stop_time = 0.5; //time to stop if get lidar alarm
bool g_controller_lidar = true;
double g_curr_x = 0.0;
double g_curr_y = 0.0;
double g_curr_psi = 0.0;

double convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return psi;
}
void currStateCallback(const nav_msgs::Odometry& curr_state){
    geometry_msgs::Twist twist = curr_state.twist.twist;
    g_curr_x = curr_state.pose.pose.position.x;
    g_curr_y = curr_state.pose.pose.position.y;
    g_curr_psi = convertPlanarQuat2Psi(curr_state.pose.pose.orientation);
    ROS_ERROR("Changed current states in nav coord");
}
void controllerLidarCallback(const std_msgs::Bool& alarm_msg){
    g_controller_lidar = alarm_msg.data;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;
    ros::Subscriber current_state_subscriber = n.subscribe("/current_state",1,currStateCallback);
    ros::Subscriber controller_lidar_alarm_subscriber = n.subscribe("/controller_lidar",1,controllerLidarCallback);
    ros::ServiceClient client = n.serviceClient<des_state_publisher_service::NavSrv>("des_pose_service");
    
    des_state_publisher_service::NavSrv pose_srv;

    //Very simple motion for debugging des_state_service.cpp - just rotate
    int numGoals = 24; 
    double x[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0,   0.5,     0.5, 3.200,  3.000, 3.000, 3.000, 0.750, 0.750, 0.750, 0.750, 0.750, 0.750, 0.750, 0.750,  0.000, 0.000, 0.000};
    double y[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0,   0.5,     0.5, 0.500,  0.500, 0.500, 0.500, 0.500, 0.500, 2.200, 2.000, 2.000, 2.000, 0.000, 0.000,  0.000, 0.000, 0.000};
    double psi[] ={0.0, 1.57, 3.14,  4.71, 6.28, 0.0,  0.785, 0.785,   0.0, 0.000,  0.000, 1.57,  3.14,  3.14,  1.57,  1.57,  1.57,  0.000, -1.57, -1.57, -3.14,  -3.14, -4.71, -6.28};
    int mode[] =  {1,   1,    1,     1,    1,    1,     1,    0,       1,   0,          2,  1,    1,     0,     1,     0,     2,     1,      1,    0,      1,     0,     1,     1}; 


    double sample_dt = 0.01;
    //Is this necessary this time?
    //ros::Rate loop_timer(1/sample_dt);

    //Wait until the service is advertized and available before attempting to send goal positions.
    client.waitForExistence();
    
    for(int i = 0; i < numGoals;){
        //ROS_INFO("Reached inside for loop");
        ros::spinOnce();
        pose_srv.request.x = x[i];
        pose_srv.request.y = y[i];
        pose_srv.request.psi = psi[i];
        pose_srv.request.x_curr = g_curr_x;
        pose_srv.request.y_curr = g_curr_y;
        pose_srv.request.psi_curr = g_curr_psi;
        pose_srv.request.mode = mode[i];
        ROS_INFO("This is rot_current[%f]", g_curr_psi);
        ROS_INFO("This is des rot[%f]", psi[i]);
        ROS_INFO("This is x_current[%f]", g_curr_x);
        ROS_INFO("This is des x[%f]", x[i]);
        ROS_INFO("This is y_current[%f]", g_curr_y);
        ROS_INFO("This is des y[%f]", y[i]);
        //ROS_INFO("i is %i",i);
        //i++;
        if (client.call(pose_srv) && ros::ok()){
            while((pose_srv.response.alarm || pose_srv.response.failed)&&client.call(pose_srv))
                if(pose_srv.response.alarm){
                    //If had alarm from somewhere - set next goal pose to be the same as current pose plus where we will be in the next stop_time
                    ros::spinOnce(); //make sure we do not try to go backwards
                    pose_srv.request.x = g_curr_x;
                    pose_srv.request.y = g_curr_y;
                    pose_srv.request.psi = g_curr_psi;
                    pose_srv.request.x_curr = g_curr_x;
                    pose_srv.request.y_curr = g_curr_y;
                    pose_srv.request.psi_curr = g_curr_psi;
                    pose_srv.request.mode = mode[i];
                    
                }
               ros::spinOnce();
            ros::spinOnce();
            if(!g_controller_lidar){
                // implement a got close enough check
                i++;
            }
                
                
            //ros::spin();
        }
    }
    return 0;
}
