#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>
#include <gazebo_aerial_manipulation_plugin/RollPitchYawThrust.h>
#include <gazebo_aerial_manipulation_plugin/RPYPose.h>
#include <gazebo_aerial_manipulation_plugin/JointCommand.h>

class VelocityController {
public:
  VelocityController() : last_control_time_(ros::Time::now()) {
    rpy_cmd_pub_ = nh_.advertise<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>("/rpyt_command",10);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 10);
    pose_sub_ = nh_.subscribe("/base_pose", 10, &VelocityController::poseCallBack);
    joint_state_sub_ = nh_.subscribe("/joint_state", 10, &);
    controller_run_timer_ = nh_.createTimer(ros::Duration(dt), &VelocityController::controllerRunTimerCallback);
    tf_br_timer_ = nh_.createTimer(ros::Duration(0.01), &VelocityController::tfBrTimerCallback);
  };

  void setGoal(std::tuple<geometry_msgs::Vector3,double> goal) {goal_ = goal};

private:
  ros::NodeHandle nh_;
  ros::Publisher rpy_cmd_pub_;
  ros::Publisher joint_state_pub_;
  ros::Subscriber pose_sub_;
  tf::TransformBroadcaster br_;
  geometry_msgs::Pose quad_state_;
  geometry_msgs::Vector3 quad_vel_;
  sensor_msgs::JointState joint_state_;
  double quad_yaw_ = 0.0;
  std::tuple<geometry_msgs::Vector3, double> goal_;

  const double kp_ = 1.0;
  const double ki_ = 0.01;
  const double kt_ = 0.16;
  const tolerance = 1e-4;

  double cumulative_error_x_ = 0.0;
  double cumulative_error_y_ = 0.0;
  double cumulative_error_z_ = 0.0;

  double dt = 0.02;
private:
  void poseCallBack(gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& msg) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position = msg->position;
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->orientation.x,
      msg->orientation.y, msg->orientation.z, msg->orientation.w);
    /* run KF */
  }

  void jointStateCallBack(sensor_msgs::JointState::ConstPtr& msg){
    joint_state_ = *msg;
  }

  void controllerRunTimerCallback() {
    gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

    double ex = goal_.x - quad_vel_.x;
    double ey = goal_.y - quad_vel_.y;
    double ez = goal_.z - quad_vel_.z;

    if(abs(ex) < tolerance && abs(ey) < tolerance && abs(ez) < tolerance) return;

    cumulative_error_x_ += ex;
    cumulative_error_y_ += ey;
    cumulative_error_z_ += ez;

    // Acceleration in world frame
    Eigen::Vector3d world_acc;
    world_acc(0) = kp * ex + ki*cumulative_error_x_*dt;
    world_acc(1) = kp * ey + ki*cumulative_error_y_*dt;
    world_acc(2) = kp * ez + ki*cumulative_error_z_*dt;

    // Limit acceleration magnitude
    double acc_norm = world_acc.norm();

    // Compensate for gravity after limiting the residual
    world_acc(2) += 9.81;

    // Acceleration in gravity aligned yaw-compensated frame
    Eigen::Vector3d rot_acc;
    rot_acc[0] = world_acc(0) * cos(yaw) + world_acc(1) * sin(yaw);
    rot_acc[1] = -world_acc(0) * sin(yaw) + world_acc(1) * cos(yaw);
    rot_acc[2] = world_acc(2);

    // thrust is magnitude of acceleration scaled by kt
    rpyt_msg.thrust = rot_acc.norm() / kt;

    rot_acc = rot_acc/rot_acc.norm();
    // yaw-compensated y-acceleration is sine of roll
    rpyt.roll = -asin(rot_acc[1]);
    rpyt.pitch = atan2(rot_acc[0], rot_acc[2]);
    rpyt.yaw = yaw;

    rpy_cmd_pub_.publish(rpyt_msg);
  }

  void tfBrTimerCallback() {
    br_.sendTransform(tf::Transform(tf::Vector3(
                                      quad_state_.position.x,
                                      quad_state_.position.y,
                                      quad_state_.position.z),
                                    tf::createQuaternionMsgFromRollPitchYaw(
                                      quad_state_.orientation.x,
                                      quad_state_.orientation.y,
                                      quad_state_.orientation.z,
                                      quad_state_.orientation.w)),
                      ros::Time::now(), "world", "base_link");
  }
}