#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <math.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<des_state_publisher_service::NavSrv>("des_pose_service");
    
    des_state_publisher_service::NavSrv pose_srv;

    //Values for all the goal positions
    //These have to be manually modified for now
    int numGoals = 6;
    double x[] = {0.0, 0.5, 0, 0.25, 0.0, 1.0};
    double y[] = {0.0, 0.5, 0, 0.25, 0.0, 1.0};
    double psi[] = {0.0, 0.0, 0.5, 0.0, -0.5, 0.0};
    int mode[] = {0, 0, 1, 0, 1, 0};

    double sample_dt = 0.02;

    

    //Is this necessary this time?
    //ros::Rate loop_timer(1/sample_dt);

    //Wait until the service is advertized and available before attempting to send goal positions.
    client.waitForExistence();
    
    pose_srv.request.x = x[0];
    pose_srv.request.y = y[0];
    pose_srv.request.psi = psi[0];
    pose_srv.request.mode = mode[0];
    
    for(int i = 1; i < numGoals;){
        pose_srv.request.x = x[i];
        pose_srv.request.y = y[i];
        pose_srv.request.psi = psi[i];
        pose_srv.request.mode = mode[i];
        
        if (client.call(pose_srv) && ros::ok()){
            if(pose_srv.response.alarm){
                ROS_INFO("LIDAR Alarm detected!");
                //What kind of responses to this do we want?
            }
            if (pose_srv.response.alarm && pose_srv.response.failed){
                ROS_INFO("Waiting for obstacle to move...");
                //Wait for obstacle to move out of the way for how long?
                //Will the obstacle ever move?
                //Should we send a new goal to backup and try a different goal?
            } else if(!pose_srv.response.alarm && pose_srv.response.failed){
                ROS_INFO("Failed to converge on goal pose. Stopping Mobot...");
                //Should we stop the Mobot if it fails to converge on the goal pose? What other actions could we take?
            } else if(!pose_srv.response.alarm && !pose_srv.response.failed){
                //Move to next goal in list.
                ROS_INFO("Successfully converged on goal pose. Sending next goal...");
                i++;
            }
        } else {
            ROS_ERROR("Failed to call service des_pose_service");
            return 1;
        }
    }
    return 0;
}