#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <fstream>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <quad_dynamics.h>
#include <flat_space_conversions.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define NT        50
#define NUM_OBS   1
#define NUM_STEPS 10
#define NP        3
/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* A template for testing of the solver. */
int main(int argc, char** argv) {
  ros::init(argc, argv, "quad_sim");
  ros::NodeHandle nh;
  int    i, iter;
  acado_timer t;

  std::string states_file; nh.getParam("states_file", states_file);
  std::string controls_file; nh.getParam("controls_file", controls_file);

  std::string reference_states; nh.getParam("reference_states", reference_states);
  std::string reference_controls; nh.getParam("reference_controls", reference_controls);

  std::ofstream states(states_file);
  std::ofstream controls(controls_file);

  std::ifstream xref(reference_states);
  std::ifstream uref(reference_controls);

  vector<double> ref_states((NT+1)*NX);
  vector<double> ref_controls(NT*NU);

  /* Initialize the solver. */
  acado_initializeSolver();

  for (i = 0; i < NT ; ++i) {
    std::string line;
    std::getline(uref, line);
    std::istringstream iss(line);
    for(int u=0; u < NU; u++)
      iss >> ref_controls[i*NU + u];
  }

  for (i = 0; i < NT+1 ; ++i) {
    std::string line;
    std::getline(xref, line);
    std::istringstream iss(line);
    for(int x=0; x < NX; x++) {
      iss >> ref_states[i*NX + x];
    }
  }

  double l1 = 0.175; double l2 = 0.42;

  for (i = 0; i < NX; ++i) acadoVariables.x0[i] = 0.0;
  acadoVariables.x0[14] = -1.5;

  vector<double> quad_state(NX);
  for(int i=0; i < NX; i++) quad_state[i] = 0.0;
  quad_state[14] = -1.56;
  tf::Transform quadTf = tf::Transform(
                          tf::createQuaternionFromRPY(0,0,0),
                          tf::Vector3(0,0,0));
  QuadDynamics dynamicsPropogater(NP, 0.02, NX, NU, N);
  FlatSpaceConversions flateSpaceConverter;

  tf::TransformBroadcaster br;
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  int dt = 0;
  double gx = 2.0; double gy = 0.0; double gz = 2.0;
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/goal_marker", 1);
  ros::Publisher obsPub = nh.advertise<visualization_msgs::Marker>("/obs_marker", 1);
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odometry", 1);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = gx;
  marker.pose.position.y = gy;
  marker.pose.position.z = gz - 0.0875;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  double obs[] = {1.0};
  visualization_msgs::Marker obs_marker[NUM_OBS];

  for(int o = 0; o < NUM_OBS; o++) {
    obs_marker[o].header.frame_id = "world";
    obs_marker[o].id = o;
    obs_marker[o].type = visualization_msgs::Marker::SPHERE;
    obs_marker[o].action = visualization_msgs::Marker::ADD;
    obs_marker[o].pose.position.x = obs[o];
    obs_marker[o].pose.position.y = 0.0;
    obs_marker[o].pose.position.z = obs[o] + 0.2;
    obs_marker[o].pose.orientation.x = 0.0;
    obs_marker[o].pose.orientation.y = 0.0;
    obs_marker[o].pose.orientation.z = 0.0;
    obs_marker[o].pose.orientation.w = 1.0;
    obs_marker[o].scale.x = 0.3;
    obs_marker[o].scale.y = 0.3;
    obs_marker[o].scale.z = 0.3;
    obs_marker[o].color.a = 1.0; 
    obs_marker[o].color.r = 1.0;
    obs_marker[o].color.g = 0.0;
    obs_marker[o].color.b = 0.0;
  }

  while(dt < (NT-N)) {
    for(i=0; i < N; i++) {
      for(int x=0; x < NX; x++) {
        acadoVariables.y[i*NY + x] = ref_states[(i+dt)*NX + x];
        acadoVariables.x[i*NX + x] = ref_states[(i+dt)*NX + x];
      }
      for(int u=0; u < NU; u++) {
        acadoVariables.y[i*NY + NX + u] = ref_controls[(i+dt)*NU + u];
        acadoVariables.u[i*NU + u] = ref_controls[(i+dt)*NU + u];
      }
    }

    acado_preparationStep();

    for(iter = 0; iter < NUM_STEPS; ++iter) {
      acado_feedbackStep();
      acado_preparationStep();
    }

    for(int x=0; x < NX; x++) quad_state[x] = acadoVariables.x[NP*NX + x];

    quadTf = flateSpaceConverter.flat2tf(quad_state);

    for(int s=0; s < NX; s++) states << quad_state[s] <<" ";
    states <<"\n";

    ros::Duration(0.5).sleep();
    br.sendTransform(tf::StampedTransform(quadTf, ros::Time::now(), "world", "baselink"));
    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("airbasetolink1");
    joint_state.name.push_back("link1tolink2");

    joint_state.position.push_back(-quad_state[14] + 1.57);
    joint_state.position.push_back(-quad_state[15]);
    joint_state.header.stamp = ros::Time::now();

    jointPub.publish(joint_state);
    goalPub.publish(marker);
    obsPub.publish(obs_marker[0]);

    nav_msgs::Odometry odommsg;

    odommsg.header.stamp = ros::Time::now();
    odommsg.header.frame_id = "world";
    odommsg.pose.pose.position.x = quadTf.getOrigin().x();
    odommsg.pose.pose.position.y = quadTf.getOrigin().y();
    odommsg.pose.pose.position.z = quadTf.getOrigin().z();

    odommsg.pose.pose.orientation.x = quadTf.getRotation().x();
    odommsg.pose.pose.orientation.y = quadTf.getRotation().y();
    odommsg.pose.pose.orientation.z = quadTf.getRotation().z();
    odommsg.pose.pose.orientation.w = quadTf.getRotation().w();

    odommsg.twist.twist.linear.x = quad_state[3];
    odommsg.twist.twist.linear.y = quad_state[4];
    odommsg.twist.twist.linear.z = quad_state[5];

    odomPub.publish(odommsg);

    for(i = 0; i < NX; ++i) acadoVariables.x0[i] = acadoVariables.x[NP*NX + i];
    dt += NP;
  }
  return 0;
}
