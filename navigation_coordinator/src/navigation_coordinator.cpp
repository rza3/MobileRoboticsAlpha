#include <ros/ros.h>
#include "des_state_publisher_service/NavSrv.h"
#include <std_msgs/Bool.h>
#include <math.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<des_state_publisher_service::NavSrv>("des_pose_service");
    
    des_state_publisher_service::NavSrv pose_srv;

    //Values for all the goal positions
    //These have to be manually modified for now
    int numGoals = 7;
    double x[] = {0.0, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0};
    double y[] = {0.0, 0.5, 0.5, 0.5, -0.5, 1.0, 0.0} ;
    double psi[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //Mode can be one of four states: 0 = "forward travel", 1 = "spin in place", 2 = "backup", or 4 = "halt"
    int mode[] = {0, 0, 0, 0, 0, 0, 1};

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
        ROS_INFO("Reached inside for loop");
        pose_srv.request.x = x[i];
        pose_srv.request.y = y[i];
        pose_srv.request.psi = psi[i];
        pose_srv.request.mode = mode[i];
        ROS_INFO("i is %i",i);
        //i++;
        if (client.call(pose_srv) && ros::ok()){
            if(pose_srv.response.alarm){
                ROS_INFO("LIDAR Alarm detected!");
                //What kind of responses to this do we want?
            }
            if (pose_srv.response.alarm && pose_srv.response.failed){
                ROS_INFO("Waiting for obstacle to move...");
                //sleep for 5 seconds
                ros::Duration(5.0).sleep();
                //check if alarm or failed variables have changed
                if(client.call(pose_srv)){
                    if(pose_srv.response.alarm){
                        ROS_INFO("Obstacle did not move. Backing up...");
                        //pose_srv_request.mode = 2;
                        //code for straight line backup motion request
                    } else {
                        ROS_INFO("Obstacle moved. Trying to reconverge on goal pose...");
                        //do not need to increment i, will use the same goal pose as last time
                    }
                } else {
                    ROS_ERROR("Failed to call service des_pose_service");
                    return 1;
                }
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
