# modal_trajectory_controller
## Theory of Operation
This node is a controller that uses the current and desired states. It will determine the mode of operation (move forward, spin in place, move backwards). It then sends commands to cmd_vel based on this information and gain control values (K). The current implementation is an open-loop controller that just copies desired state to cmd_vel. However, it will be updated to be a closed-loop controller that compares desired and current states to determine what cmd_vel should be. It will also be updated to use mode information - there is code for that currently included but it is commented out and not fully implemented as of 2/18/22.
## Example usage
` rosrun modal_trajectory_controller open_loop_controller `
