# lidar_alarm

This node handles the front box inspector for our wall-following algorithm.

This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.

The node evaluates the number of pings from the LiDAR to look at based on tolerance, stopping distance, and robot width values. If any of these pings return a distance less than the stopping distance + tolerance, the lidar listener publishers a warning signal on
topic `lidar_alarm`.  The distance of the forward ping is also published, on topic `lidar_dist`.


## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
 `rosrun lidar_alarm lidar_alarm`
The controller code monitors the `lidar_alarm` topic to prevent running into obstacles (head on or from the side).
    
