# MPC Trajectory tracking for a quad-arm system
ROS Package to implement MPC for tracking of a reference trajectory using MPC

# 1. Dependencies
## 1.1 ACADO
This package uses ACADO for the solving the optimal control problem. To install acado follow the instructions here : acado.github.io

## 1.2. Gazebo aerial manipulation plugin
If you want to use this package with gazebo, a plugin for the same can be found at : https://github.com/jhu-asco/gazebo_aerial_manipulation_plugin.git

# 2. Installation
To install, just run catkin_make or catkin build from your ROS workspace

# 3. Running the mpc node
To run the mpc node, execute "rosrun quad_arm_trajectory_tracking mpc_controller_node" from a terminal. In its current form, the node will run the position controller to take it to the intial state.
To start mpc, you will need to publish to the topic "/toggle_mpc"

# 4. Filter
The package comes with a filter.py node which can be used to obtain accelerations and yaw rate from pose and velocity. In its current form the filter doesn't work well for state feedback but can be used for plotting these states.