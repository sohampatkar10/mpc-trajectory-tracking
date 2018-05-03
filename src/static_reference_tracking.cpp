// Acado
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado/function/function.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <quad_simulator_parser/quad_simulator_parser.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo_aerial_manipulation_plugin/RollPitchYawThrust.h>
#include <gazebo_aerial_manipulation_plugin/RPYPose.h>

USING_NAMESPACE_ACADO
using namespace quad_simulator_parser;

const double tolerance = 1e-3;
const double kp = 1.0;
const double ki = 0.0;
const double kt = 0.16;
double cex = 0;
double cey = 0;
double cez = 0;

ros::Publisher rpyPub;
geometry_msgs::PoseStamped quadPose;
geometry_msgs::TwistStamped quadVel;
geometry_msgs::Vector3 quadAcc;
geometry_msgs::Vector3 goal;
double goal_yaw;

void posSubCallback(const gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& pose_msg) {

  ROS_INFO("Pose callback");
  geometry_msgs::PoseStamped lastPos = quadPose;
  geometry_msgs::TwistStamped lastVel = quadVel;
  quadPose.pose.position.x = pose_msg->position.x;
  quadPose.pose.position.y = pose_msg->position.y;
  quadPose.pose.position.z = pose_msg->position.z;
  quadPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_msg->rpy.x, pose_msg->rpy.y, pose_msg->rpy.z);
  quadPose.header.stamp = ros::Time::now();

  double dt = (quadPose.header.stamp - lastPos.header.stamp).toSec();

  quadVel.header.stamp = lastPos.header.stamp;
  quadVel.twist.linear.x = (quadPose.pose.position.x - lastPos.pose.position.x)/dt;
  quadVel.twist.linear.y = (quadPose.pose.position.y - lastPos.pose.position.y)/dt;
  quadVel.twist.linear.z = (quadPose.pose.position.z - lastPos.pose.position.z)/dt;

  tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);
  double r, p, y; tf::Matrix3x3(q).getRPY(r,p,y);

  tf::Quaternion ql; tf::quaternionMsgToTF(lastPos.pose.orientation, ql);
  double rl, pl, yl; tf::Matrix3x3(ql).getRPY(rl,pl,yl);

  quadVel.twist.angular.x = (r-rl)/dt;
  quadVel.twist.angular.y = (p-pl)/dt;
  quadVel.twist.angular.z = (y-yl)/dt;

  dt = (quadVel.header.stamp - lastVel.header.stamp).toSec();
  quadAcc.x = (quadVel.twist.linear.x - lastVel.twist.linear.x)/dt;
  quadAcc.y = (quadVel.twist.linear.y - lastVel.twist.linear.y)/dt;
  quadAcc.z = (quadVel.twist.linear.z - lastVel.twist.linear.z)/dt;
}

void velocityControllerTimer(const ros::TimerEvent& event) {

  ROS_INFO("velocity timer");
  double ex = goal.x - quadVel.twist.linear.x;
  double ey = goal.y - quadVel.twist.linear.y;
  double ez = goal.z - quadVel.twist.linear.z;

  if(abs(ex) < tolerance && abs(ey) < tolerance && abs(ez) < tolerance) return;

  gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

  cex += ex; cey += ey; cez += ez;

  // Acceleration in world frame
  Eigen::Vector3d world_acc;
  world_acc(0) = kp * ex + ki*cex*0.01;
  world_acc(1) = kp * ey + ki*cey*0.01;
  world_acc(2) = kp * ez + ki*cez*0.01;

  // Limit acceleration magnitude
  double acc_norm = world_acc.norm();

  // Compensate for gravity after limiting the residual
  world_acc(2) += 9.81;

  double yaw = tf::getYaw(quadPose.pose.orientation);
  // Acceleration in gravity aligned yaw-compensated frame
  Eigen::Vector3d rot_acc;
  rot_acc[0] = world_acc(0) * cos(yaw) + world_acc(1) * sin(yaw);
  rot_acc[1] = -world_acc(0) * sin(yaw) + world_acc(1) * cos(yaw);
  rot_acc[2] = world_acc(2);

  // thrust is magnitude of acceleration scaled by kt
  rpyt_msg.thrust = rot_acc.norm() / kt;

  rot_acc = rot_acc/rot_acc.norm();
  // yaw-compensated y-acceleration is sine of roll
  rpyt_msg.roll = -asin(rot_acc[1]);
  rpyt_msg.pitch = atan2(rot_acc[0], rot_acc[2]);
  rpyt_msg.yaw = goal_yaw;

  rpyPub.publish(rpyt_msg);
}

void setGoal(double vx, double vy, double vz, double yr, double yaw) {
  goal.x = vx;
  goal.y = vy;
  goal.z = vz;
  goal_yaw = yaw;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "tracking_node");
  ros::NodeHandle nh;

  double ts = 0.0;
  double te = 1.0;
  int numSteps = 10;
  int totalSteps = 20;
  DifferentialEquation  f(ts, te);

  DifferentialState x0, y0, z0, 
                    x1, y1, z1,
                    x2, y2, z2,
                    x3, y3, z3,
                    ga0, ga1,
                    q1, q2;

  Control x4, y4, z4, ga2, qd1, qd2;

  f << dot(x0) == x1;
  f << dot(y0) == y1;
  f << dot(z0) == z1;

  f << dot(x1) == x2;
  f << dot(y1) == y2;
  f << dot(z1) == z2;

  f << dot(x2) == x3;
  f << dot(y2) == y3;
  f << dot(z2) == z3;

  f << dot(x3) == x4;
  f << dot(y3) == y4;
  f << dot(z3) == z4;

  f << dot(ga0) == ga1;
  f << dot(ga1) == ga2;

  f << dot(q1) == qd1;
  f << dot(q2) == qd2;

  std::string states_file = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/reference_states.txt";
  std::string controls_file = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/reference_controls.txt";
  std::ifstream xref(states_file);
  std::ifstream uref(controls_file);

  std::ofstream output_file("/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/states.txt");

  std::vector<std::vector<double> > ref_states(totalSteps, std::vector<double>(16));
  std::vector<std::vector<double> > ref_controls(totalSteps, std::vector<double>(6));

  for (int i = 0; i < totalSteps; ++i) {
    std::string line;
    std::getline(uref, line);
    std::istringstream iss(line);
    for(int u=0; u < 4; u++) {
      iss >> ref_controls[i][u];
    }
  }

  nav_msgs::Path ref_path;
  ref_path.header.frame_id = "world";

  for (int i = 0; i < totalSteps; ++i) {
    std::string line;
    std::getline(xref, line);
    std::istringstream iss(line);
    for(int x=0; x < 16; x++) {
      iss >> ref_states[i][x];
    }
    iss >> ref_controls[i][4];
    iss >> ref_controls[i][5];

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = ref_states[i][0];
    pose_msg.pose.position.y = ref_states[i][1];
    pose_msg.pose.position.z = ref_states[i][2];

    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(ref_states[i][12]);
    ref_path.poses.push_back(pose_msg);
  }

  Grid timeGrid(ts, te, numSteps);
  VariablesGrid xi(16, timeGrid);
  VariablesGrid ui(6, timeGrid);

  tf::TransformBroadcaster br;
  ros::Timer controllerTimer = nh.createTimer(ros::Duration(0.01), &velocityControllerTimer);
  ros::Publisher refPub = nh.advertise<nav_msgs::Path>("/ref_path",1);
  ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/path",10);
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
  rpyPub = nh.advertise<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>("/rpyt_command",1);
  ros::Publisher posPub = nh.advertise<geometry_msgs::Pose>("/set_model_pose", 1);
  ros::Subscriber posSub = nh.subscribe("/base_pose", 1, &posSubCallback);

  nav_msgs::Path actual_path;
  actual_path.header.frame_id = "world";

  quadPose.pose.position.x = ref_states[0][0];
  quadPose.pose.position.y = ref_states[0][1];
  quadPose.pose.position.z = ref_states[0][2];
  double init_yaw = ref_states[0][11];

  quadPose.pose.orientation = tf::createQuaternionMsgFromYaw(init_yaw);
  posPub.publish(quadPose.pose);

  sensor_msgs::JointState init_joints;
  init_joints.name.push_back("airbasetolink1");
  init_joints.name.push_back("link1tolink2");

  init_joints.position.push_back(-ref_states[0][14] + 1.57);
  init_joints.position.push_back(-ref_states[0][15]);

  init_joints.header.stamp = ros::Time::now();

  jointPub.publish(init_joints);

  VariablesGrid output_states(16, timeGrid);
  for(int x=0; x <16; x++)
    output_states(4,x) = ref_states[0][x];

  VariablesGrid output_controls(6,timeGrid);
  for(int u=0; u < 6; u++)
    output_controls(4,u) = ref_controls[0][u];

  int to = 0;
  while(to <= 4*totalSteps) {
    ROS_INFO("At time step %i", to);
    ros::Time t1 = ros::Time::now();
    VariablesGrid reference_grid(22, timeGrid);
    VariablesGrid init_states(16, timeGrid);
    VariablesGrid init_controls(6, timeGrid);
    for(int t=0; t < numSteps; t++) {
      int rt = ((t+to)/4 < totalSteps) ? (t+to)/4 : (totalSteps-1);
      for(int x=0; x < 16; x++) {
        reference_grid(t,x) = ref_states[rt][x];
        if(x > 8 && x < 12)
          reference_grid(t,x) = 0.0;
      }
      for(int u = 4; u < 6; u++)
        reference_grid(t,16+u) = ref_controls[rt][u];
      for(int x = 0; x < 16; x++)
        init_states(t,x) = ref_states[rt][x];
      for(int u = 0; u < 6; u++)
        init_controls(t,u) = ref_controls[rt][u];
    }

    DMatrix Q(22,22); Q.setIdentity();
    Q *= 10.0;
    // DMatrix Q(10,10); Q.setIdentity();

    ACADO::Function eta;
    eta << x0 << y0 << z0
        << x1 << y1 << z1
        << x2 << y2 << z2
        << x3 << y3 << z3
        << ga0 << ga1
        << q1 << q2
        << x4 << y4 << z4
        << ga2 << qd1 << qd2;

    IntermediateState T;
    T = sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));

    OCP ocp(timeGrid);
    ocp.subjectTo(f);
    ocp.minimizeLSQ(Q, eta, reference_grid);

    ocp.subjectTo(AT_START, x0 == quadPose.pose.position.x);
    ocp.subjectTo(AT_START, y0 == quadPose.pose.position.y);
    ocp.subjectTo(AT_START, z0 == quadPose.pose.position.z);

    ocp.subjectTo(AT_START, x1 == quadVel.twist.linear.x);
    ocp.subjectTo(AT_START, y1 == quadVel.twist.linear.y);
    ocp.subjectTo(AT_START, z1 == quadVel.twist.linear.z);

    ocp.subjectTo(AT_START, x2 == quadAcc.x);
    ocp.subjectTo(AT_START, y2 == quadAcc.y);
    ocp.subjectTo(AT_START, z2 == quadAcc.z);

    ocp.subjectTo(AT_START, x3 == output_states(4,9));
    ocp.subjectTo(AT_START, y3 == output_states(4,10));
    ocp.subjectTo(AT_START, z3 == output_states(4,11));

    ocp.subjectTo(AT_START, ga0 == tf::getYaw(quadPose.pose.orientation));
    ocp.subjectTo(AT_START, ga1 == quadVel.twist.angular.z);

    ocp.subjectTo(AT_START, q1 == output_states(4,14));
    ocp.subjectTo(AT_START, q2 == output_states(4,15));

    ocp.subjectTo(AT_START, x4 == output_controls(4,0));
    ocp.subjectTo(AT_START, y4 == output_controls(4,1));
    ocp.subjectTo(AT_START, z4 == output_controls(4,2));
    ocp.subjectTo(AT_START, ga2 == output_controls(4,3));
    ocp.subjectTo(AT_START, qd1 == output_controls(4,4));
    ocp.subjectTo(AT_START, qd2 == output_controls(4,5));

    ocp.subjectTo(-1.57 <= q1 <= 0.0); // joint limits
    ocp.subjectTo(-1.57 <= q2 <= 1.57); // joint limits
    ocp.subjectTo(-1.57 <= ga0 <= 1.57); // joint limits

    ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
    ocp.subjectTo(-0.78 <= qd2 <= 0.78);

    ocp.subjectTo(-2.0 <= x1 <= 2.0);
    ocp.subjectTo(-2.0 <= y1 <= 2.0);
    ocp.subjectTo(-2.0 <= z1 <= 2.0);
    ocp.subjectTo(-1.0 <= ga1 <= 1.0);

    std::unique_ptr<OptimizationAlgorithm> algorithm;
    algorithm.reset(new OptimizationAlgorithm(ocp));
    algorithm->set(MAX_NUM_QP_ITERATIONS, 20);
    algorithm->set(MAX_NUM_ITERATIONS, 20);
    algorithm->set(INFEASIBLE_QP_HANDLING, IQH_STOP);
    algorithm->set( INTEGRATOR_TYPE, INT_RK45);
    algorithm->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    algorithm->set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    algorithm->set(KKT_TOLERANCE, 1e-7);

    algorithm->initializeDifferentialStates(init_states);
    algorithm->initializeControls(init_controls);
    algorithm->solve();
    algorithm->getDifferentialStates(output_states);
    algorithm->getControls(output_controls);
    for(int x=0; x < 16; x++) 
      output_file << output_states(0,x) <<" ";
    output_file <<"\n";
    // for(int u = 0; u < 5; u++)
      setGoal(output_states(4,3), output_states(4,4), 
                        output_states(4,5), output_states(4,13), output_states(4,12));

    actual_path.poses.push_back(quadPose);
    pathPub.publish(actual_path);
    refPub.publish(ref_path);
    tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);
    tf::Transform transform = tf::Transform(
        q,tf::Vector3(quadPose.pose.position.x, quadPose.pose.position.y, quadPose.pose.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "baselink"));

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("airbasetolink1");
    joint_state.name.push_back("link1tolink2");

    joint_state.position.push_back(-output_states(4,14) + 1.57);
    joint_state.position.push_back(-output_states(4,15));

    joint_state.header.stamp = ros::Time::now();

    jointPub.publish(joint_state);
    to+=4;
    ROS_INFO("Time : %f", (ros::Time::now() - t1).toSec());
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Final velocity : %f %f %f", quadVel.twist.linear.x, quadVel.twist.linear.y, quadVel.twist.linear.z);
  return 0;
}