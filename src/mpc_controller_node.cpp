#include <ros/ros.h>
#include <mpc_controller.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "mpc_trajectory_tracking");
  MPController controller;
  ros::AsyncSpinner spinner(0, &controller.pose_callback_queue);
  spinner.start();

  ros::spin();
  return 0;
}