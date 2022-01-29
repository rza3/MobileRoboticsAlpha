# THEORY OF OPERATION
A node that coordinates with the lidar_alarm, lidar_alarm2, and heading_service nodes to control the stdr robot.  

When the radar detects obstacles ahead and on the left, the robot will turn right.  
When the radar detects that there are no obstacles in front and on the left, the robot will turn left as if.   
When the radar detects that there are no obstacles in front and obstacles on the left, the robot will go straight.

## lidar_alarm & lidar_alarm2
The two radar nodes will constantly return the distance from the obstacles in front of the robot and on the left.   
When it exceeds the tolerance, it will prompt the alarm and make the turning operation.

## heading_service nodes
When the robot makes a turn, it will inform the heading service of the angle to turn, and then compare the actual steering angle with the expected steering angle in this service. If it exceeds the error range, the service will correct the robot.

# Example usage
Start up the STDR simulator:  
`roslaunch stdr_wall_following stdr_north_facing_robot.launch`  
When STDR is turned on, execute the program：  
`roslaunch stdr_wall_following stdr_wall_following_robot.launch`  

    
