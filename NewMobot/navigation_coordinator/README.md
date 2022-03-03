# navigation_coordinator

This package coordinates with the des_state_publisher_service package to provide goal poses and movement modes to the mobot.
Takes reactions from the des_state_publisher_service to respond to the lidar_alarm or failing to reach the goal pose in order to decide on new goals.

## Example usage
start up a gazxebo simulation of the mobot in the starting pen with:
`roslaunch mobot_urdf mobot_in_pen.launch`

start up all the nodes with:
`roslaunch navigation_coordinator mobot_simple_navigation.launch`

## Running tests/demos
    