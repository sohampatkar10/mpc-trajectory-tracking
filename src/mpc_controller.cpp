#include <mpc_controller.h>

USING_NAMESPACE_ACADO

MPController::MPController() {

  f = DifferentialEquation(ts, te);
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

  ref_states = std::vector<std::vector<double>>(totalSteps, std::vector<double>(16));
  ref_controls = std::vector<std::vector<double>>(totalSteps, std::vector<double>(6));

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

  timeGrid = Grid(ts, te, numSteps);
  VariablesGrid xi(16, timeGrid);
  VariablesGrid ui(6, timeGrid);

  output_states = VariablesGrid(16, timeGrid);
  output_controls = VariablesGrid(6, timeGrid);

  for(int x=0; x <16; x++)
    output_states(prop_steps,x) = ref_states[0][x];

  for(int u=0; u < 6; u++)
    output_controls(prop_steps,u) = ref_controls[0][u];

  controllerTimer = nh.createTimer(ros::Duration(0.01), &MPController::velocityControllerTimer, this);
  posControllerTimer = nh.createTimer(ros::Duration(0.05), &MPController::posControllerTimerCallback,this);
  mpcTimer = nh.createTimer(ros::Duration(te/double(numSteps)*double(prop_steps)), &MPController::mpcTimerCallback, this);
  hoverTimer = nh.createTimer(ros::Duration(0.02), &MPController::hoverCallback, this);
  rpyCmdTimer = nh.createTimer(ros::Duration(0.01), &MPController::rpyCmdTimerCallback, this);
  refPub = nh.advertise<nav_msgs::Path>("/ref_path",1);
  pathPub = nh.advertise<nav_msgs::Path>("/path",10);
  jointPub = nh.advertise<gazebo_aerial_manipulation_plugin::JointCommand>("/joint_command",1);
  rpyPub = nh.advertise<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>("/rpyt_command",1);
  posPub = nh.advertise<geometry_msgs::Pose>("/set_model_pose", 1);
  toggleSub = nh.subscribe("/toggle_mpc", 1, &MPController::mpcTimerToggleCallback, this);

  pose_so = ros::SubscribeOptions::create<gazebo_aerial_manipulation_plugin::RPYPose>("/base_pose",1,
      boost::bind(&MPController::posSubCallback, this, _1), ros::VoidPtr(), &pose_callback_queue);
  posSub = nh.subscribe(pose_so);

  controllerTimer.stop();
  mpcTimer.stop();
  rpyCmdTimer.stop();
  hoverTimer.stop();

  nav_msgs::Path actual_path;
  actual_path.header.frame_id = "world";
  geometry_msgs::PoseStamped quadPose;
  quadPose.pose.position.x = 0.0;
  quadPose.pose.position.y = 0.0;
  quadPose.pose.position.z = 0.0;
  double init_yaw = 0.0;

  quadPose.pose.orientation = tf::createQuaternionMsgFromYaw(init_yaw);
  quadPose_.set(quadPose);
  setPosGoal(ref_states[0][0], ref_states[0][1], ref_states[0][2], ref_states[0][12]);

  // reconfigserver.reset(new dynamic_reconfigure::Server<quad_arm_trajectory_tracking::MpcConfig>(nh));
  // reconfigcallbacktype = boost::bind(&MPController::reconfigureCallback,this,_1,_2);
  // reconfigserver->setCallback(reconfigcallbacktype);
}

void MPController::posSubCallback(const gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& pose_msg) {
  geometry_msgs::PoseStamped lastPos = quadPose_.get();
  geometry_msgs::TwistStamped lastVel = quadVel_.get();
  geometry_msgs::PoseStamped quadPose;
  quadPose.pose.position.x = pose_msg->position.x;
  quadPose.pose.position.y = pose_msg->position.y;
  quadPose.pose.position.z = pose_msg->position.z;
  quadPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_msg->rpy.x, pose_msg->rpy.y, pose_msg->rpy.z);
  quadPose.header.stamp = ros::Time::now();
  quadPose_.set(quadPose);

  double dt = (quadPose.header.stamp - lastPos.header.stamp).toSec();

  geometry_msgs::TwistStamped quadVel;
  quadVel.header.stamp = lastPos.header.stamp;
  quadVel.twist.linear.x = (quadPose.pose.position.x - lastPos.pose.position.x)/dt;
  quadVel.twist.linear.y = (quadPose.pose.position.y - lastPos.pose.position.y)/dt;
  quadVel.twist.linear.z = (quadPose.pose.position.z - lastPos.pose.position.z)/dt;

  tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);
  double r, p, y; tf::Matrix3x3(q).getRPY(r,p,y);

  tf::Quaternion ql; tf::quaternionMsgToTF(lastPos.pose.orientation, ql);
  double rl, pl, yl; tf::Matrix3x3(ql).getRPY(rl,pl,yl);

  tf::Vector3 w = tf::Transform(
        tf::createQuaternionFromRPY(0.0, 0.0, 0.0), 
        tf::Vector3(0,0,0))*tf::Vector3((r-rl)/dt, (p-pl)/dt, (y-yl)/dt);

  quadVel.twist.angular.x = w.x();
  quadVel.twist.angular.y = w.y();
  quadVel.twist.angular.z = w.z();
  quadVel_.set(quadVel);

  geometry_msgs::Vector3 quadAcc;
  dt = (quadVel.header.stamp - lastVel.header.stamp).toSec();
  quadAcc.x = (quadVel.twist.linear.x - lastVel.twist.linear.x)/dt;
  quadAcc.y = (quadVel.twist.linear.y - lastVel.twist.linear.y)/dt;
  quadAcc.z = (quadVel.twist.linear.z - lastVel.twist.linear.z)/dt;
  quadAcc_.set(quadAcc);
}

void MPController::velocityControllerTimer(const ros::TimerEvent& event) {
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  double ex = goal.x - quadVel.twist.linear.x;
  double ey = goal.y - quadVel.twist.linear.y;
  double ez = goal.z - quadVel.twist.linear.z;

  if(abs(ex) < tolerance && abs(ey) < tolerance && abs(ez) < tolerance) {
    return;
  }

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
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
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

void MPController::setGoal(double vx, double vy, double vz, double yaw) {
  hoverTimer.stop();
  goal.x = vx;
  goal.y = vy;
  goal.z = vz;
  goal_yaw = yaw;
  controllerTimer.start();
}

void MPController::setPosGoal(double px, double py, double pz, double yaw) {
  hoverTimer.stop();
  goal_pos.x = px; goal_pos.y = py; goal_pos.z = pz;
  goal_yaw = yaw;
  posControllerTimer.start();
}

void MPController::setAccGoal(double ax, double ay, double az, double y) {
  ROS_INFO("Commanded : %f %f %f", ax, ay, az);
  ros::Time ts = ros::Time::now();
  geometry_msgs::Vector3 quadAcc = quadAcc_.get();
  while(abs(ax - quadAcc.x) > tolerance && 
          abs(ay - quadAcc.y) > tolerance &&
          abs(az - quadAcc.z) > tolerance) {
    if((ros::Time::now() - ts).toSec() > 0.2) return;
    ROS_INFO("Current : %f %f %f", quadAcc.x, quadAcc.y, quadAcc.z);
    gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

    Eigen::Vector3d world_acc;
    world_acc(0) = ax;
    world_acc(1) = ay;
    world_acc(2) = az;

    // Compensate for gravity after limiting the residual
    world_acc(2) += 9.81;
    geometry_msgs::PoseStamped quadPose = quadPose_.get();
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
    rpyt_msg.yaw = y;

    rpyPub.publish(rpyt_msg);
    ros::Duration(0.02).sleep();
    quadAcc = quadAcc_.get();
  }
}

void MPController::mpcTimerCallback(const ros::TimerEvent& event) {
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  geometry_msgs::Vector3 quadAcc = quadAcc_.get();
  // ROS_INFO("Current : %f %f %f", quadAcc.x, quadAcc.y, quadAcc.z);

  VariablesGrid reference_grid(22, timeGrid);
  VariablesGrid init_states(16, timeGrid);
  VariablesGrid init_controls(6, timeGrid);
  for(int t=0; t < numSteps; t++) {
    int rt = ((t+to)/5 < totalSteps) ? (t+to)/5 : (totalSteps-1);
    for(int x=0; x < 16; x++) {
      reference_grid(t,x) = ref_states[rt][x];
      // if(x > 8 && x < 12)
      //   reference_grid(t,x) = 0.0;
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

  // ocp.subjectTo(AT_START, x3 == output_states(5,9));
  // ocp.subjectTo(AT_START, y3 == output_states(5,10));
  // ocp.subjectTo(AT_START, z3 == output_states(5,11));

  ocp.subjectTo(AT_START, ga0 == tf::getYaw(quadPose.pose.orientation));
  ocp.subjectTo(AT_START, ga1 == quadVel.twist.angular.z);

  ocp.subjectTo(AT_START, q1 == output_states(prop_steps,14));
  ocp.subjectTo(AT_START, q2 == output_states(prop_steps,15));

  ocp.subjectTo(AT_START, x4 == output_controls(prop_steps,0));
  ocp.subjectTo(AT_START, y4 == output_controls(prop_steps,1));
  ocp.subjectTo(AT_START, z4 == output_controls(prop_steps,2));
  ocp.subjectTo(AT_START, ga2 == output_controls(prop_steps,3));
  ocp.subjectTo(AT_START, qd1 == output_controls(prop_steps,4));
  ocp.subjectTo(AT_START, qd2 == output_controls(prop_steps,5));

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
  if(!algorithm->solve()) return;

  algorithm->getDifferentialStates(output_states);
  algorithm->getControls(output_controls);

  // setGoal(output_states(5,3), output_states(5,4), 
  //             output_states(5,5), output_states(5,13), output_states(5,12));

  gazebo_aerial_manipulation_plugin::JointCommand joint_command;
  joint_command.header.stamp = ros::Time::now();
  joint_command.desired_joint_angles.push_back(-output_states(prop_steps,14));
  joint_command.desired_joint_angles.push_back(-output_states(prop_steps,15));
  jointPub.publish(joint_command);

  setAccGoal(output_states(prop_steps,6), output_states(prop_steps,7), output_states(prop_steps,8), output_states(prop_steps,12));
  // ROS_INFO("Commanded : %f %f %f", output_states(prop_steps,6), output_states(prop_steps,7), output_states(prop_steps,8));

  to += prop_steps;
  if(to == 5*numSteps) {
    hoverTimer.start();
    controllerTimer.start();
    mpcTimer.stop();
  }

}

void MPController::posControllerTimerCallback(const ros::TimerEvent& event) {
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  double ex = goal_pos.x - quadPose.pose.position.x;
  double ey = goal_pos.y - quadPose.pose.position.y;
  double ez = goal_pos.z - quadPose.pose.position.z;

  if(abs(ex) < pos_tolerance && abs(ey) < pos_tolerance && abs(ez) < pos_tolerance) {
    ROS_INFO("Positon controller converged");
    posControllerTimer.stop();
    gazebo_aerial_manipulation_plugin::JointCommand joint_command;
    joint_command.header.stamp = ros::Time::now();
    joint_command.desired_joint_angles.push_back(-ref_states[0][14]);
    joint_command.desired_joint_angles.push_back(-ref_states[0][15]);
    jointPub.publish(joint_command);
    hoverTimer.start();
    return;
  }

  cex_p += ex; cey_p += ey; cez_p += ez;

  setGoal(kp_p*ex + ki_p*cex_p*0.05, kp_p*ey + ki_p*cey_p*0.05, kp_p*ez + ki_p*cez_p*0.05, goal_yaw);
}

void MPController::hoverCallback(const ros::TimerEvent& event) {
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  setGoal(0.0, 0.0, 0.0, tf::getYaw(quadPose.pose.orientation));
}

void MPController::rpyCmdTimerCallback(const ros::TimerEvent& event) {
  geometry_msgs::Vector3 quadAcc = quadAcc_.get();
  if(abs(quadAcc.x - goal_acc.x) < tolerance && 
        abs(quadAcc.y - goal_acc.y) < tolerance &&
        abs(quadAcc.z - goal_acc.z) < tolerance) {
    rpyCmdTimer.stop();
    hoverTimer.stop();
    return;
  }

  gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

  Eigen::Vector3d world_acc;
  world_acc(0) = goal_acc.x;
  world_acc(1) = goal_acc.y;
  world_acc(2) = goal_acc.z;

  // Limit acceleration magnitude
  double acc_norm = world_acc.norm();

  // Compensate for gravity after limiting the residual
  world_acc(2) += 9.81;
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
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
  if((abs(rot_acc[1]) - 1.0) < tolerance)
    rpyt_msg.pitch = atan2(rot_acc[0], rot_acc[2]);
  else
    rpyt_msg.pitch = 0.0;
  rpyt_msg.yaw = goal_yaw;

  rpyPub.publish(rpyt_msg);
}

void MPController::reconfigureCallback(quad_arm_trajectory_tracking::MpcConfig &config, uint32_t level) {
  if(config.takeoff) {
    config.takeoff = false;
    setPosGoal(ref_states[0][0], ref_states[0][1], ref_states[0][2], ref_states[0][12]);
    gazebo_aerial_manipulation_plugin::JointCommand joint_command;
    joint_command.header.stamp = ros::Time::now();
    joint_command.desired_joint_angles.push_back(-ref_states[0][14]);
    joint_command.desired_joint_angles.push_back(-ref_states[0][15]);
    jointPub.publish(joint_command);
  }

  if(config.start_mpc) {
    config.start_mpc = false;
    hoverTimer.stop();
    mpcTimer.start();
  }

  if(config.reset) {
    config.reset = false;
    reset();
  }
}

void MPController::mpcTimerToggleCallback(const std_msgs::Bool::ConstPtr& msg) {
  if(msg->data) {
    posControllerTimer.stop();
    hoverTimer.stop();
    controllerTimer.stop();
    mpcTimer.start();
    // rpyCmdTimer.start();
  } else {
    mpcTimer.stop();
    hoverTimer.start();
    controllerTimer.start();
  }
}

void MPController::reset() {
  hoverTimer.stop();
  controllerTimer.stop();
  mpcTimer.stop();
  geometry_msgs::PoseStamped quadPose;
  quadPose.pose.position.x = 0.0;
  quadPose.pose.position.y = 0.0;
  quadPose.pose.position.z = 0.0;
  quadPose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  quadPose_.set(quadPose);
  ros::Duration(0.5).sleep();
  posPub.publish(quadPose.pose);
  to = 0;
  cex = 0; cey = 0; cez = 0;
  cex_p = 0; cey_p = 0; cez_p = 0;
}
