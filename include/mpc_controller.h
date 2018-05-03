#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Path.h>
#include <quad_simulator_parser/quad_simulator_parser.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo_aerial_manipulation_plugin/RollPitchYawThrust.h>
#include <gazebo_aerial_manipulation_plugin/RPYPose.h>
#include <gazebo_aerial_manipulation_plugin/JointCommand.h>
#include <gazebo_aerial_manipulation_plugin/atomic.h>

#include <quad_arm_trajectory_tracking/MpcConfig.h>
#include <dynamic_reconfigure/server.h>

USING_NAMESPACE_ACADO

class MPController {

public:
  MPController();
  void setGoal(double vx, double vy, double vz, double yaw);
  void setAccGoal(double ax, double ay, double az, double yaw);
  void setPosGoal(double px, double py, double pz, double yaw);

  ros::CallbackQueue pose_callback_queue;
  ros::CallbackQueue controller_callback_queue;
private:
  void posSubCallback(const gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& pose_msg);
  void posControllerTimerCallback(const ros::TimerEvent& event);
  void velocityControllerTimer(const ros::TimerEvent& event);
  void rpyCmdTimerCallback(const ros::TimerEvent& event);
  void mpcTimerCallback(const ros::TimerEvent& event);
  void hoverCallback(const ros::TimerEvent& event);
  void mpcTimerToggleCallback(const std_msgs::Bool::ConstPtr& msg);

  void reconfigureCallback(quad_arm_trajectory_tracking::MpcConfig& config, uint32_t level);
  void reset();
private:
  const double tolerance = 1e-4;
  const double pos_tolerance = 5e-2;
  const double kp_p = 1.0;
  const double ki_p = 0.01;
  const double kp = 5.0;
  const double ki = 0.01;
  const double kt = 0.1;

  double cex = 0;
  double cey = 0;
  double cez = 0;
  double cex_p = 0;
  double cey_p = 0;
  double cez_p = 0;

  ros::NodeHandle nh;
  ros::SubscribeOptions pose_so;
  ros::Publisher rpyPub;
  tf::TransformBroadcaster br;
  ros::Timer controllerTimer;
  ros::Timer posControllerTimer;
  ros::Timer rpyCmdTimer;
  ros::Timer mpcTimer;
  ros::Timer hoverTimer;
  ros::Publisher refPub;
  ros::Publisher pathPub;
  ros::Publisher jointPub;
  ros::Publisher posPub;
  ros::Subscriber posSub;
  ros::Subscriber toggleSub;

  Atomic<geometry_msgs::PoseStamped> quadPose_;
  Atomic<geometry_msgs::TwistStamped> quadVel_;
  Atomic<geometry_msgs::Vector3> quadAcc_;
  Atomic<geometry_msgs::Vector3> quadJerk_;
  geometry_msgs::Vector3 goal;
  double goal_yaw;
  geometry_msgs::Vector3 goal_acc;
  geometry_msgs::Vector3 goal_pos;

  double ts = 0.0;
  double te = 1.0;
  int numSteps = 10;
  int totalSteps = 20;
  const int tfinal = 5;
  DifferentialEquation f;

  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1,
                    q1, q2;

  Control x4, y4, z4, ga2, qd1, qd2;

  std::vector<std::vector<double>> ref_states;
  std::vector<std::vector<double>> ref_controls;

  Grid timeGrid;
  VariablesGrid output_states;
  VariablesGrid output_controls;

  nav_msgs::Path ref_path;
  nav_msgs::Path actual_path;

  int to;
  ros::Time mpc_start_time;

  int prop_steps = 4;
  boost::shared_ptr<dynamic_reconfigure::Server<quad_arm_trajectory_tracking::MpcConfig> > reconfigserver;
  dynamic_reconfigure::Server<quad_arm_trajectory_tracking::MpcConfig>::CallbackType reconfigcallbacktype;
};