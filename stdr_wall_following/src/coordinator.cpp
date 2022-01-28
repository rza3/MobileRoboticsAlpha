#include <ros/ros.h>
#include "double_vec_srv/DoubleSrv.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <math.h>

bool g_lidar_alarm_front = false;
bool g_lidar_alarm_left = false;

void lidarAlarmFrontCallback(const std_msgs::Bool& front_alarm_msg){
    g_lidar_alarm_front = front_alarm_msg.data;
    if(g_lidar_alarm_front){
        ROS_INFO("Front LIDAR alarm detected");
    }
}

void lidarAlarmLeftCallback(const std_msgs::Bool& left_alarm_msg){
    g_lidar_alarm_left = left_alarm_msg.data;
    if(!g_lidar_alarm_left){
        ROS_INFO("Left LIDAR alarm not detected");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "stdr_coordinator");
    ros::NodeHandle n;
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ros::Subscriber alarm_front_subscriber = n.subscribe("front_lidar_alarm",1,lidarAlarmFrontCallback);
    ros::Subscriber alarm_left_subscriber = n.subscribe("left_lidar_alarm",1,lidarAlarmLeftCallback);
    ros::ServiceClient client = n.serviceClient<double_vec_srv::DoubleSrv>("stdr_rotation_service");

    double_vec_srv::DoubleSrv heading_srv; //heading service
    double desired_heading;
    desired_heading = M_PI/2;

    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds
    
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;   

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }

    twist_cmd.linear.x=speed; //command to move forward
    while(ros::ok()){
        ros::spinOnce();
        loop_timer.sleep();
        while(!g_lidar_alarm_front && g_lidar_alarm_left) { //keep moving forward until either the front alarm is detected or the left alarm is no longer detected
            twist_commander.publish(twist_cmd);
            timer+=sample_dt;
            ros::spinOnce();
            loop_timer.sleep();
            ROS_INFO("Front: %d,  Left: %d", g_lidar_alarm_front, g_lidar_alarm_left);
        }
        
        twist_cmd.linear.x=0.0; //command to stop moving
        twist_commander.publish(twist_cmd);
        timer+=sample_dt;
        ros::spinOnce();
        loop_timer.sleep();

        timer=0.0;
        if(g_lidar_alarm_front){
            while(g_lidar_alarm_front){ //repeat until alarm front no longer detects a wall
                desired_heading -= M_PI/2;
                heading_srv.request.vec_of_doubles.resize(1); //command rotation service to rotate CW 90 degrees
                heading_srv.request.vec_of_doubles[0]=desired_heading;
                client.call(heading_srv);
                timer+=sample_dt;
                ros::spinOnce();
                loop_timer.sleep();
            }
        } else if(!g_lidar_alarm_left){
            desired_heading -= M_PI/2;
            heading_srv.request.vec_of_doubles.resize(1);
            heading_srv.request.vec_of_doubles[0]=desired_heading;
            client.call(heading_srv); //command rotation service to rotate CW 90 degrees once
            timer+=sample_dt;
            ros::spinOnce();
            loop_timer.sleep();

            twist_cmd.linear.x=speed; //command to move forward
            while(!g_lidar_alarm_left){ //move forward until alarm left detects a wall
                twist_commander.publish(twist_cmd);
                timer+=sample_dt;
                ros::spinOnce();
                loop_timer.sleep();
            }
        }
    }
}