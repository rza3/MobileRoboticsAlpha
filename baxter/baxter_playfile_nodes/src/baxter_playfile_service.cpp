//baxter_playfile_service.cpp
//wsn, Sept, 2016
// service to accept pre-specified playfile codes and execute the corresponding playfile


#include<ros/ros.h>
#include <stdlib.h>     /* getenv */
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>

#include <std_msgs/Int32.h>
#include<baxter_trajectory_streamer/trajAction.h>
#include<baxter_playfile_nodes/playfileSrv.h>

#define VECTOR_DIM 7 // e.g., a 7-dof vector

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace std;
bool g_got_good_traj_left = false;

bool g_left_arm_done = false;



void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr & result) {

    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_left_arm_done = true;
}


std::string g_path_to_playfiles;
Baxter_traj_streamer *g_baxter_traj_streamer_ptr;
actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> *g_left_arm_action_client_ptr;


bool srv_callback(baxter_playfile_nodes::playfileSrvRequest& request, baxter_playfile_nodes::playfileSrvResponse& response) {
    ROS_INFO("srv_callback activated");
    std::string fname_full_path;
   double pose_q0= request.q0;
   double pose_q1= request.q1;
   double pose_q2= request.q2;
   double pose_q3= request.q3;
   double pose_q4= request.q4;
   double pose_q5= request.q5;
   double pose_q6= request.q6;
   double pose_q7= request.q7;


    Eigen::VectorXd q_right_state, q_right_firstpoint, q_left_state, q_left_firstpoint;
    q_right_firstpoint.resize(7);
    q_left_firstpoint.resize(7);

    Vectorq7x1  q_vec_left_arm;
    baxter_trajectory_streamer::trajGoal goal_left;
    std::vector<Eigen::VectorXd>  des_path_left;
    trajectory_msgs::JointTrajectory des_trajectory; // objects to hold trajectories
    trajectory_msgs::JointTrajectory approach_trajectory_left; // objects to hold trajectories    
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;

    //get right and left arm angles; start motion from here
    q_vec_left_arm = g_baxter_traj_streamer_ptr->get_qvec_left_arm();
    q_left_state = q_vec_left_arm; // start from here;  
    des_path_left.clear();
    des_path_left.push_back(q_left_state);


    g_got_good_traj_left = false;

    des_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    des_trajectory.joint_names.clear(); //could put joint names in...but I assume a fixed order and fixed size, so this is unnecessary
    des_trajectory.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint trajectory_point; //,trajectory_point2; 
    trajectory_point.positions.resize(7);
    double t_arrival;
        trajectory_point.positions[0] = pose_q0;
        trajectory_point.positions[1] = pose_q1;
        trajectory_point.positions[2] = pose_q2;
        trajectory_point.positions[3] = pose_q3;
        trajectory_point.positions[4] = pose_q4;
        trajectory_point.positions[5] = pose_q5;
        trajectory_point.positions[6] = pose_q6;

        t_arrival = pose_q7;
        trajectory_point.time_from_start = ros::Duration(t_arrival);
        des_trajectory.points.push_back(trajectory_point);



    //now have current arm poses and desired trajectories; splice in a motion from current arm pose
    // to first point of desired trajectory;

    if (g_got_good_traj_left) {
        trajectory_point0 = des_trajectory.points[0];
        for (int i = 0; i < 7; i++) { //copy from traj point to Eigen-type vector
            q_left_firstpoint[i] = trajectory_point0.positions[i];
        }
        //add this pt to path from current pose:
        des_path_left.push_back(q_left_firstpoint);
        //use traj stuffer to find build trajectory from current pose to first point of recorded traj
        g_baxter_traj_streamer_ptr->stuff_trajectory_left_arm(des_path_left, approach_trajectory_left);
    }


 
    g_left_arm_done = true;
    if (g_got_good_traj_left) {
        goal_left.trajectory = approach_trajectory_left;
        g_left_arm_done = false;
        g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
    }
    while (!g_left_arm_done) {
        ROS_INFO("waiting on arm server(s) to approach start of traj");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    // now send the desired trajectory from file, if there are any points left to execute
    if (g_got_good_traj_left && (des_trajectory.points.size() > 1)) {
        goal_left.trajectory = des_trajectory;
        g_left_arm_done = false; //reset status trigger, so can check when done
        g_left_arm_action_client_ptr->sendGoal(goal_left, &leftArmDoneCb); // we could also name additional callback functions here, if desired
    }

    while ( !g_left_arm_done) {
        ROS_INFO("waiting on arm server(s) to execute playfile(s)");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_playfile_service");
    ros::NodeHandle n;


    cout << "instantiating a traj streamer" << endl;
    Baxter_traj_streamer baxter_traj_streamer(&n); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }

    //instantiate clients of the two arm servers:
    
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> left_arm_action_client("leftArmTrajActionServer", true);

    // attempt to connect to the servers:
    ROS_INFO("waiting for left-arm server: ");
    bool server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on left-arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = left_arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to left-arm action server"); // if here, then we connected to the server; 
    //make these available to callback fnc
    g_baxter_traj_streamer_ptr = &baxter_traj_streamer;
    g_left_arm_action_client_ptr = &left_arm_action_client;


    ros::ServiceServer service = n.advertiseService("playfile_service", srv_callback);
    ROS_INFO("playfile_service is ready to accept requests");
    ros::spin();

    return 0;
}


