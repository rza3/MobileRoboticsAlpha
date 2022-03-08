# modal_trajectory_controller
## Theory of Operation
This node is a controller that uses the current and desired states. It will determine the mode of operation (move forward, spin in place, move backwards). It then sends commands to cmd_vel based on this information and gain control values (K). This is a closed-loop controller that compares desired and current states to determine what cmd_vel should be. It also uses mode information to determine if motion is spinning, moving forward, or backing up. It sets controller gains (and decides whether to use open-loop (for backing up) or closed-loop (for spinning or moving forward) control.
## Example usage
` rosrun modal_trajectory_controller open_loop_controller `
