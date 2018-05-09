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
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <gazebo_aerial_manipulation_plugin/RollPitchYawThrust.h>
#include <gazebo_aerial_manipulation_plugin/RPYPose.h>
#include <gazebo_aerial_manipulation_plugin/JointCommand.h>
#include <gazebo_aerial_manipulation_plugin/atomic.h>

USING_NAMESPACE_ACADO

class MPController {

public:
  /**
  * Constructor
  */
  MPController();
  /**
  * Sets Position goal
  */
  void setPosGoal(double px, double py, double pz, double yaw);
  /**
  * Sets velocity goal
  */
  void setVelGoal(double vx, double vy, double vz, double yaw);
  /**
  * Sets acceleration goal
  */
  void setAccGoal(double ax, double ay, double az, double yaw);

  /**
  * Separate callback queue for sensor data
  */
  ros::CallbackQueue sensor_callback_queue;

private:
  /**
  * Pose data callback
  */
  void posSubCallback(const gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& pose_msg);
  /**
  * Velocity data callback
  */
  void velSubCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);
  /**
  * Joint state callback
  */
  void jointSubCallback(const sensor_msgs::JointState::ConstPtr& joint_msg);
  /**
  * Acceleration data callback
  */
  void accnSubCallback(const geometry_msgs::Vector3::ConstPtr& accn_msg);
  /**
  * Yaw rate callback
  */
  void yawRateSubCallback(const std_msgs::Float64::ConstPtr& yr_msg);
  /**
  * Timer callback for position controller
  */
  void posControllerTimerCallback(const ros::TimerEvent& event);
  /**
  * Timer callback for velocity controller
  */
  void velocityControllerTimer(const ros::TimerEvent& event);
  /**
  * Timer callback for MPC
  */
  void mpcTimerCallback(const ros::TimerEvent& event);
  /**
  * Timer callback for hovering state
  */
  void hoverCallback(const ros::TimerEvent& event);
  /**
  * Callback for toggling MPC
  */
  void mpcTimerToggleCallback(const std_msgs::Bool::ConstPtr& msg);

private:
  const double tolerance = 1e-4; 
  const double pos_tolerance = 1e-2;
  const double kp_p = 2.0; ///< kp for position controller
  const double ki_p = 0.01; ///< ki for positoion controller
  const double kp_z = 5.0; ///< kp in z for velocity controller
  const double kp_y = 5.0; ///< kp in y for velocity controller
  const double kp_x = 5.0; ///< kp in z for velocity controller
  const double ki = 0.01; // ki for velocity controller
  const double kt = 0.101; // Thrust gain 

  const double controller_timer_duration = 0.01; // Velocity controller timer duration
  const double mpc_timer_duration = 0.2; // MPC Timer duration

  /* Cumulative errors */
  double cex = 0;
  double cey = 0;
  double cez = 0;
  double cex_p = 0;
  double cey_p = 0;
  double cez_p = 0;

  ros::NodeHandle nh; ///< Nodehandle
  
  /* Publishers */
  ros::Publisher rpyPub;
  ros::Publisher jointPub;

  /* Timers */
  ros::Timer controllerTimer;
  ros::Timer posControllerTimer;
  ros::Timer rpyCmdTimer;
  ros::Timer mpcTimer;
  ros::Timer hoverTimer;

  /* Subscribers */
  ros::Subscriber posSub;
  ros::Subscriber velSub;
  ros::Subscriber accnSub;
  ros::Subscriber yawRateSub;
  ros::Subscriber jointSub;
  ros::Subscriber toggleSub;

  /* Sensor data */
  Atomic<geometry_msgs::PoseStamped> quadPose_;
  Atomic<geometry_msgs::TwistStamped> quadVel_;
  Atomic<geometry_msgs::Vector3> quadAcc_;
  Atomic<sensor_msgs::JointState> jointState_;
  Atomic<double> yaw_rate_;

  /* goals */
  geometry_msgs::Vector3 goal_pos;
  geometry_msgs::Vector3 goal_vel;
  double goal_yaw;

  /* ACADO Variables */
  const double ts = 0.0; ///< Start time
  const double te = 1.0; ///< End time
  const int numSteps = 10; ///< number of steps for output trajectory
  const int totalSteps = 40; ///< number of steps in reference trajectory
  const int tfinal = 10; ///< Final time of reference trajectory

  DifferentialEquation f; ///< Differential equation

  /* 
  * Differential states :
  * x{i} is the ith time derivative of x
  * x,y,z is position.
  * ga is yaw
  * q is joint angle
  *
  */
  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1,
                    q1, q2;

  /* Control variables */
  Control x4, y4, z4, ga2, qd1, qd2;

  /* Reference trajectory */
  std::vector<std::vector<double>> ref_states;
  std::vector<std::vector<double>> ref_controls;

  Grid timeGrid; ///< time grid
  VariablesGrid output_states; ///< Stores output states of optimization
  VariablesGrid output_controls; ///< Stores output controls of optimization

  /* 
  *
  * Variable to keep track of the start index of 
  * the reference trajectory corresponding to 
  * time passed.
  *
  */
  int to = 0;
  ros::Time mpc_start_time;

  /* Files to store the outputs of MPC*/
  std::ofstream output_file; // States found from optimization
  std::ofstream achieved_file; // States actually acheived

  /**
  * The timestep of the output trajectory might be smaller than 
  * mpc timestep. In this case, this variable gives the index of the
  * state in the output trajectory that corresponds to the mpc timestep
  */
  int prop_steps;
};