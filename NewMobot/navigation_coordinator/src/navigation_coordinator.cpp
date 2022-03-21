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


    /*Just initialize
    int numGoals = 1;
    double x[]= {0.0};
    double y[] = {0.0};
    double psi[] = {0.0};
    int mode[] = {-1};*/
    
    /*Initialize and localize
    int numGoalsAMCL = 11;
    double xAMCL[] =   {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00 };
    double yAMCL[] =   {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
    double psiAMCL[] = {0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00 };
    int modeAMCL[] =   {0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2    };
    
    int numGoals = 12;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2    };*/

    /*Initialize, localize, and move first diagonal
    int numGoalsDiag1 = 3;
    double xDiag1[] =   {0.000, 0.825, 0.825};
    double yDiag1[] =   {0.000, 0.584, 0.584};
    double psiDiag1[] = {0.434, 0.434, 0.000};
    int modeDiag1[] =   {1,     0,     1    };
    int numGoals = 15;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00, 0.000, 0.825, 0.825 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00, 0.434, 0.434, 0.000 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2,    1,     0,     1     };*/

    /*Initialize, localize, move diagonal, go to first table
    int numGoalsStraight1 = 2;
    double xStraight1[] =   {2.869, 3.100};
    double yStraight1[] =   {0.584, 0.584};
    double psiStraight1[] = {0.000, 0.000};
    int modeStraight1[] =   {0,     0    };
    int numGoals = 17;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00, 0.000, 0.825, 0.825, 2.869, 3.600 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2,    1,     0,     1,     0,     0     };
*/
    /*Initialize, localize, move diagonal, go to first table, go back
    int numGoalsBack1 = 3;
    double xBack1[] =   {2.869, 2.869, 0.308};
    double yBack1[] =   {0.584, 0.584, 0.584};
    double psiBack1[] = {0.000, 3.142, 3.142};
    int modeBack1[] =   {2,     1,     0  };
    int numGoals = 20;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00, 0.000, 0.825, 0.825, 2.869, 3.100, 2.869, 2.869, 0.308 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000, 0.000, 3.142, 3.142 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2,    1,     0,     1,     0,     0,     2,     1,     0     };*/
    
    /*Initialize, localize, move diagonal, go to first table, go back, go to left table
    int numGoalsLeft = 3;
    double xLeft[] =   {0.308, 0.308, 0.308};
    double yLeft[] =   {0.584, 1.864, 2.170};
    double psiLeft[] = {1.571, 1.571, 1.571};
    int modeLeft[] =   {1,     0,     0    };
    int numGoals = 23;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.00, 0.000, 0.825, 0.825, 2.869, 3.100, 2.869, 2.869, 0.308, 0.308, 0.308, 0.308 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 1.864, 2.170 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93, 4.71, 5.50, 0.00, 0.00, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000, 0.000, 3.142, 3.142, 1.571, 1.571, 1.571 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,    1,    1,    1,    0,    2,    1,     0,     1,     0,     0,     2,     1,     0,     1,     0,     0     };*/

    // For debugging set x,y,psi, mode, and numGoals to any of the above sections to see which works and where the errors are.
    
    /*Initialize, localize, move diagonal, go to first table, go back, go to left table, go back to origin (full motion)
    int numGoalsBackOrigin = 6;
    double xBackOrigin[] =   {0.308,  0.308,  0.308, 0.308, 0.000, 0.000};
    double yBackOrigin[] =   {1.864,  1.864,  0.000, 0.000, 0.000, 0.000};
    double psiBackOrigin[] = {1.571, -1.571, -1.571, 3.142, 3.142, 0.000};
    int modeBackOrigin[] =   {2,     1,       0,     1,     0,     1};
    int numGoals = 34;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  0.00,   0.00, 0.00, 0.10, 0.00, 0.000, 0.825, 0.825, 2.669, 3.600, 2.669, 2.669, 2.669, 2.669, 0.700, 0.700, 0.700, 0.700, 0.700,  0.700, 0.700, 0.700,  0.700,  0.700, 0.700, 0.000, 0.000, 0.000 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,  0.00,   0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 1.464, 2.170, 1.464,  1.464, 1.464, 1.464,  1.464,  0.000, 0.000, 0.000, 0.000, 0.000 };
    double psi[] = {0.0, 0.00, 0.78, 1.57, 2.36, 3.14, 3.93,  4.71,   5.50, 0.00, 0.00, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000, 0.000, 1.047, 2.094, 3.142, 3.142, 1.571, 1.571, 1.571, 1.571,  0.780, 0.000, -0.780, -1.571, -1.571, 3.142, 3.142,1.571, 0.000 };
    int mode[] =   {-1,  0,    1,    1,    1,    1,    1,       1,    1,    1,    0,    2,    1,     0,     1,     0,     0,     2,     1,     1,     1,     0,     1,     0,     0,     2,      1,      1,       1,     1,     0,     1,     0,      1,    1     };
    
    */

    //Very simple motion for debugging des_state_service.cpp
    int numGoals = 10; 
    double x[] =  {0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0,0.0,0.7,0.7};
    double y[] =  {0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0,0.0,0.7,0.7};
    double psi[] ={0.0, 1.57, 3.14, 4.71, 6.28, 0.0, 0.0,0.785,0.785,0};
    int mode[] =  {1,   1,    1,     1,    1,    1,  0,  1,     0,   1 };

    /*Simplifying full trajectory for debugging
    int numGoals = 27;
    double x[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.825, 0.825, 3.000, 3.200, 3.000, 3.000, 3.000, 0.700, 0.700, 0.700, 0.700, 0.700,  0.700, 0.700,  0.700, 0.700, 0.000, 0.000, 0.000 };
    double y[] =   {0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.000, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 0.584, 1.464, 2.170, 1.464,  1.464, 1.464,  0.000, 0.000, 0.000, 0.000, 0.000 };
    double psi[] = {0.0, 0.00, 1.57, 3.14, 4.71, 6.28, 0.00, 0.434, 0.434, 0.000, 0.000, 0.000, 0.000, 1.57, 3.142,  3.142, 1.571, 1.571, 1.571, 1.571,  0,    -1.571, -1.571, 3.142, 3.142, 1.571, 0.000 };
    int mode[] =   {-1,  0,      1,   1,    1,    1,    1,     1,     0,     1,     0,     0,     2,     1,    1,      0,     1,     0,     0,     2,      1,     1,      0,     1,     0,      1,    1     };
    */
    
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
