#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

double stop_distance = 0.25; //x,y distance to stop if get lidar alarm
double stop_distance_phi = 0.25; // same for phi
bool g_controller_lidar = true;
double g_curr_x = 0.0;
double g_curr_y = 0.0;
double g_curr_psi = 0.0;
bool g_lidar_alarm;

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
void lidarCallback(const std_msgs::Bool& alarm_msg){
    if(g_curr_x<2.6 && g_curr_y<1.7)
        g_lidar_alarm = alarm_msg.data;
    else
        g_lidar_alarm = false; //if we are near the tables, ignore lidar alarm
    if(g_lidar_alarm){
        ROS_INFO("LIDAR alarm detected");
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;
    ros::Subscriber current_state_subscriber = n.subscribe("/current_state",1,currStateCallback);
    ros::Subscriber controller_lidar_alarm_subscriber = n.subscribe("/lidar_alarm",1,lidarCallback);
    ros::ServiceClient client = n.serviceClient<des_state_publisher_service::NavSrv>("des_pose_service");
    des_state_publisher_service::NavSrv pose_srv;

    //Very simple motion for debugging des_state_service.cpp - just rotate
    int numGoals = 24; 
    double x[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0,   0.7,     0.7, 3.600,  3.000, 3.000, 3.000, 0.550, 0.550, 0.550, 0.550, 0.550, 0.550, 0.550, 0.550,  0.000, 0.000, 0.000};
    double y[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0,   0.7,     0.7, 0.70,  0.700, 0.70,     0.70, 0.70, 0.70, 2.400, 2.000, 2.000, 2.000, 0.000, 0.000,  0.000, 0.000, 0.000};
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
                    //If had alarm from somewhere - set next goal pose to be stop_dist ahead
                    ros::spinOnce(); //make sure we do not try to go backwards
                    if(mode[i]!=1){
                        pose_srv.request.x = g_curr_x + stop_distance*cosf(g_curr_psi);
                        pose_srv.request.y = g_curr_y + stop_distance*sinf(g_curr_psi);
                        pose_srv.request.psi = g_curr_psi;
                    }
                    else{
                        pose_srv.request.x = g_curr_x;
                        pose_srv.request.y = g_curr_y;
                        pose_srv.request.psi = g_curr_psi + stop_distance_phi;
                    }
                    pose_srv.request.x_curr = g_curr_x;
                    pose_srv.request.y_curr = g_curr_y;
                    pose_srv.request.psi_curr = g_curr_psi;
                    pose_srv.request.mode = mode[i];
                    client.call(pose_srv);
                }
               ros::spinOnce();
            //ros::spinOnce();
            if(!g_lidar_alarm && !pose_srv.response.alarm && !pose_srv.response.failed){
                // implement a got close enough check
                i++;
            }
            else{
                ROS_WARN("Got lidar controller or failure, going back to last i");
                if(g_controller_lidar)
                    ROS_WARN("Was lidar controller");
            }
            if(i<7 || i == 10 || i ==16)
                ros::Duration(5).sleep();        
                
            //ros::spin();
        }
    }
    return 0;
}
