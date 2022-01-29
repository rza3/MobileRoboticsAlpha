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

# Observations
The parameters of tolerance and stopping distance have a large impact on robot behavior. The choices of field of view of the front box inspector and left box inspector are also very important. All these values have a huge impact on how the robot behaves. There are some corners that are easier to navigate than others. It is harder to navigate losing the wall to your left than an obstacle coming up in front.

# Suggestions
It may be helpful to have more nuanced behavior like if there is a corner where there are obstacles on many sides, it may be more convenient to turn a full 180&deg;. In general, it would be helpful to have some way to determine what angle to turn and then to turn that angle. This may be possible by looking at the entire 270 &deg; field of view of the LiDAR.

# Future Work
Implementing this in a physical system would be an appropriate next step. Many of the parameters that had a great impact on behavior will be determined by the system. Stopping distance and radius will be determined by the robot. Tolerance will have some user choice but it will also depend on the physical system. Furthermore, in future work it would be helpful to have (choosable) parameters chosen using an algorithmic approach rather than trial and error. Additionally, implementing more nuanced approached like turning an arbitrary angle would be done. This would make greater use of LiDAR data as well as provide more options for the system and perhaps increase speed of traversing the maze. However, this would be a more complex method and may involve more overhead; it would be interesting to see the trade-offs.
