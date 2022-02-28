# current_state_publisher
## Theory of Operation
This node publishes the current state of the robot. Right now, it just subscribes to the odom topic and republishes the odometry. However, it will be updated to include absolute location information.
## Example usage
` rosrun current_state_publisher current_state `
