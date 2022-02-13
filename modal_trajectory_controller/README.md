# modal_trajectory_controller
This node is a controller that uses the current and desired states. It also uses a service to determine the mode of operation (move forward, spin in place, move backwards). It then sends commands to cmd_vel based on this information and gain control values (K).
## Example usage
` rosrun modal_trajectory_controller open_loop_controller `