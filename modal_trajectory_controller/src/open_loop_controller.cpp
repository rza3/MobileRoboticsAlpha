#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>


/*#include "mode_srv/ModeSrv.h"*/

//node to receive desired states and republish the twist as cmd_vel commands
ros::Publisher g_twist_publisher;
double g_des_x = 0.0;
double g_des_y = 0.0;
double g_des_phi = 0.0;
double g_curr_x = 0.0;
double g_curr_y = 0.0;
double g_curr_phi = 0.0;
double g_des_vel = 0;
double g_des_omega = 0;
geometry_msgs::Twist g_pub_twist;
double K_TRIP_DIST = 0.0;
double K_PHI = 0.0;
double K_DISP = 0.0;
double MAX_OMEGA = 0.5; // 0.5 rad/sec corresponds to 1.5 rad in 3 sec so 90 degrees in about 3 seconds
//int g_des_mode = 5;
int g_speed_multiplier = 0;
int g_omega_multiplier = 0;
bool g_backing_up = false;


//utility fnc to compute min dang, accounting for periodicity
double min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


// saturation function, values -1 to 1
double sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}
//some conversion utilities:
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}
//simply copy the desired twist and republish it to cmd_vel
void desStateCallback(const nav_msgs::Odometry& des_state) {
    geometry_msgs::Twist twist = des_state.twist.twist;
    g_pub_twist = twist;
    g_des_x = des_state.pose.pose.position.x;
    g_des_y = des_state.pose.pose.position.y;
    g_des_phi = convertPlanarQuat2Phi(des_state.pose.pose.orientation);
    g_des_vel = des_state.twist.twist.linear.x;
    g_des_omega = des_state.twist.twist.angular.z;
}
void currStateCallback(const nav_msgs::Odometry& curr_state){
    geometry_msgs::Twist twist = curr_state.twist.twist;
    g_curr_x = curr_state.pose.pose.position.x;
    g_curr_y = curr_state.pose.pose.position.y;
    g_curr_phi = convertPlanarQuat2Phi(curr_state.pose.pose.orientation);
}

void desModeCallback(const std_msgs::Int8& des_mode) {
    g_omega_multiplier = 0;
    g_speed_multiplier = 0;
    if(des_mode.data == 0)
       g_speed_multiplier = 1;
    else{
        if(des_mode.data == 1)
            g_omega_multiplier = 1;
        else{
            if(des_mode.data == 2)
                g_backing_up = true;
                g_pub_twist.linear.x = -1*g_des_vel;
        }
    }
}

void open_loop_control(){
    //Put code for backing up open loop controller here.
}

void closed_loop_control(){
    double controller_speed;
    double controller_omega;
    double tx = cos(g_des_phi);
    double ty = sin(g_des_phi);
    double nx = -ty;
    double ny = tx;
    
    double heading_err;  
    double lateral_err;
    double trip_dist_err; // error is scheduling...are we ahead or behind?
    

    // have access to: des_state_vel_, des_state_omega_, des_state_x_, des_state_y_, des_state_phi_ and corresponding odom values    
    double dx = g_des_x- g_curr_x;
    double dy = g_des_y - g_curr_y;
    
    //pos_err_xy_vec_ = des_xy_vec_ - odom_xy_vec_; // vector pointing from odom x-y to desired x-y
    //lateral_err = n_vec.dot(pos_err_xy_vec_); //signed scalar lateral offset error; if positive, then desired state is to the left of odom
    lateral_err = dx*nx + dy*ny;
    trip_dist_err = dx*tx + dy*ty;
    
    //trip_dist_err = t_vec.dot(pos_err_xy_vec_); // progress error: if positive, then we are behind schedule
    heading_err = min_dang(g_des_phi - g_curr_phi); // if positive, should rotate +omega to align with desired heading
    
    
    // DEBUG OUTPUT...
    /*
    ROS_INFO("g_odom_tf_x, g_odom_tf_y,dx,dy = %f, %f, %f, %f",g_odom_tf_x, g_odom_tf_y,dx,dy);
    ROS_INFO("des_state_phi, odom_phi, heading err = %f, %f, %f", des_state_phi_,odom_phi_,heading_err);
    ROS_INFO("lateral err, trip dist err = %f, %f",lateral_err,trip_dist_err);
    */
    // DEFINITELY COMMENT OUT ALL cout<< OPERATIONS FOR REAL-TIME CODE
    //std::cout<<des_xy_vec_<<std::endl;
    //std::cout<<odom_xy_vec_<<std::endl;
    // let's put these in a message to publish, for rqt_plot to display
    //steering_errs_.data.clear();
    //steering_errs_.data.push_back(lateral_err);
    //steering_errs_.data.push_back(heading_err); 
    //steering_errs_.data.push_back(trip_dist_err);

    //steering_errs_publisher_.publish(steering_errs_); // suitable for plotting w/ rqt_plot
    //END OF DEBUG STUFF
    
     // do something clever with this information     
    controller_speed = g_des_vel + K_TRIP_DIST*trip_dist_err; //speed up/slow down to null out 
    //controller_omega = des_state_omega_; //ditto
    controller_omega = g_des_omega + K_PHI*heading_err + K_DISP*lateral_err;
    
    controller_omega = MAX_OMEGA*sat(controller_omega/MAX_OMEGA); // saturate omega command at specified limits
    
    //g_pub_twist.linear.x = 0.0;
    //g_pub_twist.angular.z = 0.0;
    // send out our very clever speed/spin commands:
    //if(g_des_mode == 1)
        g_pub_twist.angular.z = g_omega_multiplier*controller_omega;
   // else if(g_des_mode == 0)
        g_pub_twist.linear.x = g_speed_multiplier*controller_speed;
        
    
    //g_pub_twist.header.stamp = ros::Time::now(); // look up the time and put it in the header 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "open_loop_controller"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    g_twist_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber des_state_subscriber = n.subscribe("/desState",1,desStateCallback); 
    ros::Subscriber curr_state_subscriber = n.subscribe("/current_state",1,currStateCallback); 
    ros::Subscriber mode_subscriber = n.subscribe("/desMode",1,desModeCallback); 
    //ros::Subscriber mode_0_subscriber = n.subscribe("/des_mode0",1,desMode0Callbaccurr_state_subscriberk); 
    /*ros::ServiceClient client = n.ServiceClient<mode_srv::ModeSrv>("mode_determining_service");
    mode_srv::ModeSrv mode_service;
    client.call(mode_service);
    int mode;
    mode = mode_service.response.mode_type;
    if(mode == 0){
        //have a series of if statements based on value of mode.
    }*/

    //ros::spin();
    while(ros::ok()){
        
        ros::spinOnce();
        if(g_backing_up == false)
            closed_loop_control();
        // if g_backing_up is true, we just need open loop control so no need to modify g_pub_twist
        g_twist_publisher.publish(g_pub_twist);
        /*if(g_des_vel - g_pub_twist.linear.x != 0)
            ROS_ERROR("x error of %f",g_des_vel - g_pub_twist.linear.x);
        if(g_des_omega - g_pub_twist.angular.z != 0)
            ROS_ERROR("vel error of %f",g_des_omega - g_pub_twist.angular.z );*/
    }
}

