#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>



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

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;
    ros::Subscriber current_state_subscriber = n.subscribe("/current_state",1,currStateCallback);
    ros::ServiceClient client = n.serviceClient<des_state_publisher_service::NavSrv>("des_pose_service");
    
    des_state_publisher_service::NavSrv pose_srv;

    //Very simple motion for debugging des_state_service.cpp - just rotate
   /* int numGoals = 7; 
    double x[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0};
    double y[] =  {0.0, 0.0,  0.0,   0.0, 0.0,  0.0,   0.0};
    double psi[] ={0.0, 1.57, 3.14,  4.71, 6.28, 0.0,  0.785};
    int mode[] =  {1,   1,    1,     1,    1,    1,     1 };*/

    //Simplifying full trajectory for debugging
    int numGoals = 27;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.825, 0.825, 3.000, 3.200, 3.000, 3.000, 3.000, 0.700, 0.700, 0.700, 0.700, 0.700,  0.700, 0.700,  0.700, 0.700, 0.000, 0.000, 0.000 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 1.464, 2.170, 1.464,  1.464, 1.464,  0.000, 0.000, 0.000, 0.000, 0.000 };
    double psi[] = {0.0, 0.00, 1.57, 3.14, 4.71, 6.28, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000, 0.000, 1.57, 3.142,  3.142, 1.571, 1.571, 1.571, 1.571,  0,    -1.571, -1.571, 3.142, 3.142, 1.571, 0.000 };
    int mode[] =   {-1,  0,      1,   1,    1,    1,    1,     1,     0,     1,     0,     0,     2,     1,    1,      0,     1,     0,     0,     2,      1,     1,      0,     1,     0,      1,    1     };
    
    
    // Simple forward motion
    /*int numGoals = 4; 
    double x[] = {0.0, 0.0, 0.0, 1.0, 2.0};
    double y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double psi[] = {0.0, 0.4, 0.0, 0.0, 0.0};
    int mode[] = {0, 1, 1, 0, 0};*/
    
    //double sample_dt = 0.02;
    double sample_dt = 0.01;
    //Is this necessary this time?
    //ros::Rate loop_timer(1/sample_dt);

    //Wait until the service is advertized and available before attempting to send goal positions.
    client.waitForExistence();
    
    /*pose_srv.request.x = x[0];
    pose_srv.request.y = y[0];
    pose_srv.request.psi = psi[0];
    pose_srv.request.mode = mode[0];*/
    
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
        //ROS_INFO("i is %i",i);
        //i++;
        if (client.call(pose_srv) && ros::ok()){
            while(pose_srv.response.alarm || pose_srv.response.failed)
               ros::spinOnce();
            i++;
            //ros::spin();
        }
    }
    return 0;
}
