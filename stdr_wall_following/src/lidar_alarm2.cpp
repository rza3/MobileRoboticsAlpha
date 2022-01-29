# include <ros/ros.h>
# include <sensor_msgs/LaserScan.h>
# include <std_msgs/Float32.h>
# include <std_msgs/Bool.h>
# include <math.h>

const double MIN_SAFE_DISTANCE = 1.0;

float ping_dist_to_left_ = 3.0;
int ping_index_ = -1;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_ = true;
double tolerance = 0.5;
double radius = 0.1;
double stopping_distance = 1;
double detect_length = radius + stopping_distance;
double detect_width = radius + tolerance;
int num_rays;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
	
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	if (ping_index_ < 0) {
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;
		ping_index_ = (int) ((M_PI/2 - angle_min_) / angle_increment_);
		//ROS_INFO("LIDAR setup: ping_index = %d" , ping_index_);
		}
		num_rays = 100;
		int num_near = 0;
		laser_alarm_ = true;
		for (int index = 0; index<num_rays; index ++){
			ping_dist_to_left_ = laser_scan.ranges[ping_index_ + index];
			if(ping_dist_to_left_>detect_length){
				ROS_WARN("Away from wall");
			}
			else{
				ROS_WARN("Next to wall");
				num_near = num_near + 1;
			}
			ping_dist_to_left_ = laser_scan.ranges[ping_index_ - index];
			if(ping_dist_to_left_>detect_length){
				ROS_WARN("Away from wall");
			}
			else{
				ROS_WARN("Next to wall");
				num_near = num_near + 1;
			}
		}
	if(num_near<0.4*num_rays){
		laser_alarm_ = false;
	}

	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = laser_alarm_;
	lidar_alarm_publisher_.publish(lidar_alarm_msg);
	std_msgs::Float32 lidar_dist_msg;
	lidar_dist_msg.data = ping_dist_to_left_;
	lidar_dist_publisher_.publish(lidar_dist_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_alarm2");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Bool>("left_lidar_alarm", 1);
	lidar_alarm_publisher_ = pub;
	ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("left_lidar_dist", 1);
	lidar_dist_publisher_ = pub2;
	ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
	ros::spin();
	return 0;
}

