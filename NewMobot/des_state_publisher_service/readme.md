# des_state_publisher_service
## Theory of Operation
As an intermediate layer, it needs to accept the service request from navigation_coordinator through service, so as to calculate the feasible speed and displacement planning through traj_builder, which should construct triangular and trapezoidal trajectory plans for either forward travel or spin-in-place motions. At the same time, we should also combine the feedback of lidar_alarm and the feasibility of path to feed back which is True and False, and give the feasible route to modal_trajectory_controller.
## Example usage
` rosrun des_state_publisher_service open_loop_controller des_state_publisher_service`
