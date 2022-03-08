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
    /*int numGoals = 9;
    double x[] = {0.0, 0.5, 0.5, 0.0, 0.5, 0.5, -5, 0.0,0.0};
    double y[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,5} ;
    double psi[] = {0.0, 0.0, 0.0, 0.0, 0.0, 3.1, 3.1, 1.57,1.57};

    //Mode can be one of four states: 0 = "forward travel", 1 = "spin in place", 2 = "backup", or 4 = "halt"
    int mode[] = {0, 0, 0, 2, 0, 1, 0,1,0};*/

    int numGoals = 29;
    double x[] =   {0,0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.825, 0.825, 0.825, 2.869,  3.100, 2.869, 2.869, 2.869, 2.869, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.308, 0.00, 0.00, 0.00};
    double y[] =   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.584, 0.584, 0.584, 0.584,  0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 1.864, 2.170, 1.864, 1.864, 1.864, 0.00,  0.00,  0.0,   0.00, 0.00, 0.00 };
    double psi[] = {0.0, 0.0, 0.1, 6.1, 0.0, 0.434, 0.434, 0.0,   0.0,    0.0,   0.0,   0.0,    1.57,  3.14,   3.14, 3.14,  1.57,  1.57, 1.57,  1.57,  1.57,  -1.57, -1.57, -1.57, 3.14,  3.14,  3.14, 0.00, 0.00};
    int mode[] =   {-1,   0,   1,     1,   1,  1,      0,     1,     1,     0,    0,      2,      1,    1,         1,   0,    1,    1,     0,      0,     2,      1,   1,      0,     1,     1,    0,     1,   1};
    //Mode can be one of five states: -1 = "initial pose", 0 = "forward travel", 1 = "spin in place", 2 = "backup", or 4 = "halt"
    //int mode[] = {0, 1, 1, 1, 1, 0, 1, 0, 0, 2, 1, 0, 1, 0, 0, 2, 1,0, 1, 0, 1};
    
    int numGoalsInit = 1;
    double xInit[] = {0.0};
    double yInit[] = {0.0};
    double psiInit[] = {0.0};
    int modeInit[] = {-1};
    
    int numGoalsAMCL = 11;
    double xAMCL[] =   {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00 };
    double yAMCL[] =   {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
    double psiAMCL[] = {0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00 };
    int modeAMCL[] =   {0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2    };
    
    int numGoalsDiag1 = 3;
    double xDiag1[] =   {0.000, 0.825, 0.825};
    double yDiag1[] =   {0.000, 0.584, 0.584};
    double psiDiag1[] = {0.434, 0.434, 0.000};
    int modeDiag1[] =   {1,     0,     1};
    //double sample_dt = 0.02;
    double sample_dt = 0.5;
    //Is this necessary this time?
    //ros::Rate loop_timer(1/sample_dt);

    //Wait until the service is advertized and available before attempting to send goal positions.
    client.waitForExistence();
    
    pose_srv.request.x = x[0];
    pose_srv.request.y = y[0];
    pose_srv.request.psi = psi[0];
    pose_srv.request.mode = mode[0];
    
    for(int i = 1; i < numGoals;){
        //ROS_INFO("Reached inside for loop");
        pose_srv.request.x = x[i];
        pose_srv.request.y = y[i];
        pose_srv.request.psi = psi[i];
        pose_srv.request.mode = mode[i];
        //ROS_INFO("i is %i",i);
        if(i==6)
            ROS_ERROR("SHOULD BE TURNING BACK TO 0 HERE!!");
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
                //ROS_INFO("Successfully converged on goal pose. Sending next goal...");
                i++;
            }
        } else {
            ROS_ERROR("Failed to call service des_pose_service");
            return 1;
        }
    }
    return 0;
}
