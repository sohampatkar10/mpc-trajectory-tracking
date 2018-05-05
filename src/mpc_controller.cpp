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

  std::string output_dir = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/output_states.txt";
  std::string achieved_dir = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/achieved_states.txt";
  output_file = std::ofstream(output_dir);
  achieved_file = std::ofstream(achieved_dir);
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

  prop_steps = int(double(numSteps)/te)/(totalSteps/tfinal);

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

  controllerTimer = nh.createTimer(ros::Duration(0.02), &MPController::velocityControllerTimer, this);
  posControllerTimer = nh.createTimer(ros::Duration(0.05), &MPController::posControllerTimerCallback,this);
  mpcTimer = nh.createTimer(ros::Duration(0.05), &MPController::mpcTimerCallback, this);
  hoverTimer = nh.createTimer(ros::Duration(0.02), &MPController::hoverCallback, this);
  rpyCmdTimer = nh.createTimer(ros::Duration(0.01), &MPController::rpyCmdTimerCallback, this);
  refPub = nh.advertise<nav_msgs::Path>("/ref_path",1);
  pathPub = nh.advertise<nav_msgs::Path>("/path",10);
  jointPub = nh.advertise<gazebo_aerial_manipulation_plugin::JointCommand>("/joint_command",1);
  rpyPub = nh.advertise<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>("/rpyt_command",1);
  posPub = nh.advertise<geometry_msgs::Pose>("/set_model_pose", 1);
  toggleSub = nh.subscribe("/toggle_mpc", 1, &MPController::mpcTimerToggleCallback, this);

  // pose_so = ros::SubscribeOptions::create<quad_arm_trajectory_tracking::FlatState>("/flat_state",1,
  //     boost::bind(&MPController::posSubCallback, this, _1), ros::VoidPtr(), &pose_callback_queue);
  // posSub = nh.subscribe(pose_so);

  pose_so = ros::SubscribeOptions::create<gazebo_aerial_manipulation_plugin::RPYPose>("/base_pose",1,
      boost::bind(&MPController::posSubCallback, this, _1), ros::VoidPtr(), &pose_callback_queue);
  posSub = nh.subscribe(pose_so);

  controllerTimer.stop();
  mpcTimer.stop();
  rpyCmdTimer.stop();
  hoverTimer.stop();
  buffer_ready_.set(false);

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
}

// void MPController::posSubCallback(const quad_arm_trajectory_tracking::FlatState::ConstPtr& state_msg) {
//   geometry_msgs::PoseStamped lastPos = quadPose_.get();
//   geometry_msgs::PoseStamped quadPose = state_msg->pose;

//   tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);
//   double r, p, y; tf::Matrix3x3(q).getRPY(r,p,y);

//   tf::Quaternion ql; tf::quaternionMsgToTF(lastPos.pose.orientation, ql);
//   double rl, pl, yl; tf::Matrix3x3(ql).getRPY(rl,pl,yl);

//   double dt = (state_msg->header.stamp - lastPos.header.stamp).toSec();
//   geometry_msgs::TwistStamped quadVel = state_msg->velocity;
//   quadVel.twist.angular.z = (y-yl)/dt;

//   quadPose_.set(quadPose);
//   quadVel_.set(quadVel);
//   quadAcc_.set(state_msg->acceleration);
//   quadJerk_.set(state_msg->jerk);
// }

void MPController::posSubCallback(const gazebo_aerial_manipulation_plugin::RPYPose::ConstPtr& pose_msg) {

  geometry_msgs::PoseStamped lastPos = quadPose_.get();
  geometry_msgs::TwistStamped lastVel = quadVel_.get();
  geometry_msgs::Vector3 lastAcc = quadAcc_.get();

  geometry_msgs::PoseStamped quadPose;
  quadPose.pose.position.x = pose_msg->position.x;
  quadPose.pose.position.y = pose_msg->position.y;
  quadPose.pose.position.z = pose_msg->position.z;
  quadPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_msg->rpy.x, pose_msg->rpy.y, pose_msg->rpy.z);
  quadPose.header.stamp = pose_msg->header.stamp;
  quadPose_.set(quadPose);

  geometry_msgs::TwistStamped quadVel;

  double dt = (pose_msg->header.stamp - lastPos.header.stamp).toSec();
  quadVel.header.stamp = pose_msg->header.stamp;
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

  geometry_msgs::Vector3 a;
  dt = (quadVel.header.stamp - lastVel.header.stamp).toSec();
  a.x = (quadVel.twist.linear.x - lastVel.twist.linear.x)/dt;
  a.y = (quadVel.twist.linear.y - lastVel.twist.linear.y)/dt;
  a.z = (quadVel.twist.linear.z - lastVel.twist.linear.z)/dt;

  double T = sqrt(a.x*a.x + a.y*a.y + (a.z + 9.81)*(a.z + 9.81));

  tf::Transform bodyTf = tf::Transform(q, tf::Vector3(0,0,0));
  tf::Vector3 bodyAcc = bodyTf.inverse()*tf::Vector3(0,0,T);

  geometry_msgs::Vector3 quadAcc;
  quadAcc.x = bodyAcc.x(); quadAcc.y = bodyAcc.y(); quadAcc.z = bodyAcc.z()-9.81;
  quadAcc_.set(quadAcc);

  geometry_msgs::Vector3 quadJerk;
  quadJerk.x = (quadAcc.x - lastAcc.x)/dt;
  quadJerk.y = (quadAcc.y - lastAcc.y)/dt;
  quadJerk.z = (quadAcc.z - lastAcc.z)/dt;
  quadJerk_.set(quadJerk);
}

void MPController::velocityControllerTimer(const ros::TimerEvent& event) {
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  double ex = goal.x - quadVel.twist.linear.x;
  double ey = goal.y - quadVel.twist.linear.y;
  double ez = goal.z - quadVel.twist.linear.z;

  if(abs(ex) < tolerance && abs(ey) < tolerance && abs(ez) < tolerance) {
    cex = 0; cey = 0; cez = 0;
    return;
  }

  gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

  cex += ex; cey += ey; cez += ez;

  // Acceleration in world frame
  Eigen::Vector3d world_acc;
  world_acc(0) = kp * ex + ki*cex*0.02;
  world_acc(1) = kp * ey + ki*cey*0.02;
  world_acc(2) = kp * ez + ki*cez*0.02;

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

void MPController::setAccGoal(double ax, double ay, double az, double yd) {
  ros::Time t0 = ros::Time::now();

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
  rpyt_msg.yaw = yd;

  rpyPub.publish(rpyt_msg);

  tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);
  double r, p, y; tf::Matrix3x3(q).getRPY(r,p,y);
}

void MPController::mpcTimerCallback(const ros::TimerEvent& event) {
  to = int((ros::Time::now()-mpc_start_time).toSec()*double(totalSteps)/double(tfinal));
  if(to > totalSteps) {
    hoverTimer.start();
    controllerTimer.start();
    mpcTimer.stop();
    return;
  }

  // ROS_INFO("Current : %f %f %f", quadPose.pose.position.x, 
  //   quadPose.pose.position.y, quadPose.pose.position.z);

  int R = int(double(numSteps)/te)/(totalSteps/tfinal);
  VariablesGrid reference_grid(22, timeGrid);
  VariablesGrid init_states(16, timeGrid);
  VariablesGrid init_controls(6, timeGrid);

  for(int t=0; t < numSteps; t++) {
    // int rt = ((t+to)/R < totalSteps) ? (t+to)/R : (totalSteps-1);
    // double k = double((t+to)%R)/double(R);
    int rt = (t/R+to) < totalSteps ? (t/R+to) : (totalSteps-1);
    for(int x=0; x < 16; x++)
      reference_grid(t,x) = ref_states[rt][x];
    for(int u = 0; u < 6; u++)
      reference_grid(t,16+u) = ref_controls[rt][u];
      // reference_grid(t, 16+u) = 0.0;
    // reference_grid(t,9) = 0.0;
    // reference_grid(t,10) = 0.0;
    // reference_grid(t,11) = 0.0;
    for(int x = 0; x < 16; x++)
      init_states(t,x) = ref_states[rt][x];
    for(int u = 0; u < 6; u++)
      init_controls(t,u) = ref_controls[rt][u];
  }

  DMatrix Q(22,22); Q.setIdentity();

  ACADO::Function eta;
  eta << x0 << y0 << z0
      << x1 << y1 << z1
      << x2 << y2 << z2
      << x3 << y3 << z3
      << ga0 << ga1
      << q1 << q2
      << x4 << y4 << z4
      << ga2 << qd1 << qd2;

  DVector final_state(16);
  for(int x=0; x < 16; x++) 
    final_state(x) = ref_states[totalSteps-1][x];

  ACADO::Function phi;
  phi << x0 << y0 << z0
      << x1 << y1 << z1
      << x2 << y2 << z2
      << x3 << y3 << z3
      << ga0 << ga1
      << q1 << q2;

  DMatrix W(16, 16); W.setIdentity();

  IntermediateState r,p,T;
  T = sqrt(x2*x2 + y2*y2 + (z2+9.81)*(z2+9.81));
  r = asin((-x2*sin(ga0) + y2*cos(ga0))/T);
  p = atan((x2*cos(ga0) + y2*sin(ga0))/(z2+9.81));

  IntermediateState wX, wY;
  wX = y3*(cos(r)*cos(ga0) + sin(p)*sin(r)*sin(ga0)) - x3*(cos(r)*sin(ga0) - cos(ga0)*sin(p)*sin(r)) + z3*cos(p)*sin(r);
  wY = x3*cos(p)*cos(ga0) - z3*sin(p) + y3*cos(p)*sin(ga0);

  OCP ocp(timeGrid);
  ocp.subjectTo(f);
  ocp.minimizeLSQ(Q, eta, reference_grid);
  // ocp.minimizeLSQEndTerm(W, phi, final_state);

  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  geometry_msgs::Vector3 quadAcc = quadAcc_.get();
  geometry_msgs::Vector3 quadJerk = quadJerk_.get();

  ROS_INFO("From sensor : %f %f %f, From Opti : %f %f %f", 
    quadVel.twist.linear.x, quadVel.twist.linear.y, quadVel.twist.linear.z, 
    output_states(prop_steps,6), output_states(prop_steps,7), output_states(prop_steps,8));

  // ocp.subjectTo(AT_START, x0 == quadPose.pose.position.x);
  // ocp.subjectTo(AT_START, y0 == quadPose.pose.position.y);
  // ocp.subjectTo(AT_START, z0 == quadPose.pose.position.z);

  ocp.subjectTo(AT_START, x1 == quadVel.twist.linear.x);
  ocp.subjectTo(AT_START, y1 == quadVel.twist.linear.y);
  ocp.subjectTo(AT_START, z1 == quadVel.twist.linear.z);

  ocp.subjectTo(AT_START, x0 == quadPose.pose.position.x);
  ocp.subjectTo(AT_START, y0 == quadPose.pose.position.y);
  ocp.subjectTo(AT_START, z0 == quadPose.pose.position.z);

  // ocp.subjectTo(AT_START, x1 == output_states(prop_steps,3));
  // ocp.subjectTo(AT_START, y1 == output_states(prop_steps,4));
  // ocp.subjectTo(AT_START, z1 == output_states(prop_steps,5));

  ocp.subjectTo(AT_START, x2 == output_states(prop_steps,6));
  ocp.subjectTo(AT_START, y2 == output_states(prop_steps,7));
  ocp.subjectTo(AT_START, z2 == output_states(prop_steps, 8));

  ocp.subjectTo(AT_START, x3 == output_states(prop_steps, 9));
  ocp.subjectTo(AT_START, y3 == output_states(prop_steps, 10));
  ocp.subjectTo(AT_START, z3 == output_states(prop_steps, 11));

  ocp.subjectTo(AT_START, ga0 == tf::getYaw(quadPose.pose.orientation));
  ocp.subjectTo(AT_START, ga1 == output_states(prop_steps, 13));

  ocp.subjectTo(AT_START, q1 == output_states(prop_steps,14));
  ocp.subjectTo(AT_START, q2 == output_states(prop_steps,15));

  ocp.subjectTo(AT_START, x4 == output_controls(prop_steps, 0));
  ocp.subjectTo(AT_START, y4 == output_controls(prop_steps, 1));
  ocp.subjectTo(AT_START, z4 == output_controls(prop_steps, 2));
  ocp.subjectTo(AT_START, ga2 == output_controls(prop_steps, 3));
  ocp.subjectTo(AT_START, qd1 == output_controls(prop_steps, 4));
  ocp.subjectTo(AT_START, qd2 == output_controls(prop_steps, 5));

  ocp.subjectTo(-1.57 <= q1 <= 0.0); // joint limits
  ocp.subjectTo(-1.57 <= q2 <= 1.57); // joint limits
  ocp.subjectTo(-1.57 <= ga0 <= 1.57); // joint limits

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  ocp.subjectTo(-1.0 <= x1 <= 1.0);
  ocp.subjectTo(-1.0 <= y1 <= 1.0);
  ocp.subjectTo(-1.0 <= z1 <= 1.0);
  ocp.subjectTo(-1.0 <= ga1 <= 1.0);

  ocp.subjectTo(T <= 12);
  // ocp.subjectTo(-3.14 <= wX <= 3.14);
  // ocp.subjectTo(-3.14 <= wY <= 3.14);

  std::unique_ptr<OptimizationAlgorithm> algorithm;
  algorithm.reset(new OptimizationAlgorithm(ocp));
  algorithm->set(MAX_NUM_QP_ITERATIONS, 50);
  algorithm->set(MAX_NUM_ITERATIONS, 50);
  algorithm->set(INFEASIBLE_QP_HANDLING, IQH_STOP);
  algorithm->set( INTEGRATOR_TYPE, INT_RK45);
  algorithm->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  algorithm->set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  algorithm->set(KKT_TOLERANCE, 1e-8);

  algorithm->initializeDifferentialStates(init_states);
  algorithm->initializeControls(init_controls);
  if(!algorithm->solve()) {
    return;
  }

  algorithm->getDifferentialStates(output_states);
  algorithm->getControls(output_controls);

  // ROS_INFO("Commanded : %f %f %f", reference_grid(prop_steps,3), 
  //   reference_grid(prop_steps,4), reference_grid(prop_steps,5));
  // ROS_INFO("Output : %f %f %f", output_states(prop_steps,3), 
  //   output_states(prop_steps,4), output_states(prop_steps,5));

  // ROS_INFO("Velocity:");
  // for(int t=0; t < numSteps; t++)
  //   ROS_INFO("Reference : %f %f %f", reference_grid(t,3), reference_grid(t,4), reference_grid(t,5));
  // for(int t=0; t < numSteps; t++) 
  //   ROS_INFO("Output %f %f %f", output_states(t,3), output_states(t,4), output_states(t,5));

  achieved_file << quadPose.pose.position.x <<" "
                << quadPose.pose.position.y <<" "
                << quadPose.pose.position.z <<" "
                << tf::getYaw(quadPose.pose.orientation) <<" "
                << quadVel.twist.linear.x <<" "
                << quadVel.twist.linear.y <<" "
                << quadVel.twist.linear.z <<"\n";

  output_file << output_states(prop_steps,0) <<" "<<
                 output_states(prop_steps,1) <<" "<<
                 output_states(prop_steps,2) <<" "<<
                 output_states(prop_steps,12) <<" "<<
                 output_states(prop_steps,3) <<" "<< 
                 output_states(prop_steps,4) <<" "<<
                 output_states(prop_steps,5) <<"\n";

  // setAccGoal(reference_grid(prop_steps,6), reference_grid(prop_steps,7), reference_grid(prop_steps,8), reference_grid(prop_steps,12));
  setAccGoal(output_states(prop_steps,6), output_states(prop_steps,7), output_states(prop_steps,8), output_states(prop_steps,12));
  // setGoal(output_states(prop_steps,3), output_states(prop_steps,4), output_states(prop_steps,5), output_states(prop_steps,12));
  // setGoal(reference_grid(prop_steps,3), reference_grid(prop_steps,4), reference_grid(prop_steps,5), reference_grid(prop_steps,12));
  // setPosGoal(reference_grid(prop_steps,0), reference_grid(prop_steps,1), reference_grid(prop_steps,2), reference_grid(prop_steps,12));
  gazebo_aerial_manipulation_plugin::JointCommand joint_command;
  joint_command.header.stamp = ros::Time::now();
  joint_command.desired_joint_angles.push_back(-output_states(prop_steps,14));
  joint_command.desired_joint_angles.push_back(-output_states(prop_steps,15));
  jointPub.publish(joint_command);
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
  ROS_INFO("Hover Callback");
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
    controllerTimer.stop();
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
    ROS_INFO("Starting MPC");
    posControllerTimer.stop();
    hoverTimer.stop();
    controllerTimer.stop();
    mpc_start_time = ros::Time::now();
    mpcTimer.start();
    // rpyCmdTimer.start();
  } else {
    ROS_INFO("Stopping MPC");
    mpcTimer.stop();
    hoverTimer.start();
    controllerTimer.start();
    to = 0;
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
