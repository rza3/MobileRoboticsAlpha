# lidar_alarm
## Theory of Operation
This node is a lidar_alarm that sends out a warning when the robot is near an obstacle.
<br> This node constantly publishes whether or not there is an obstacle in front; it is false when there is no obstacle in front and becomes true when there is one (front is defined not just as intersecting the single ray from the center of the robot out but as intersecting a collection of rays corresponding to the front of the robot). The node also publishes the distance out from the last ray.
## Example usage
` rosrun lidar_alarm lidar_alarm `
