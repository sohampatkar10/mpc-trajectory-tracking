#include <mpc_controller.h>

USING_NAMESPACE_ACADO

MPController::MPController() {


  /* Dynamics */
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

  /* Populate reference states from file */
  std::string states_file = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/reference_states_new_arm.txt";
  std::string controls_file = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/reference_controls_new_arm.txt";
  std::ifstream xref(states_file);
  std::ifstream uref(controls_file);

  /* Files to write output states to */
  std::string output_dir = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/output_states.txt";
  std::string achieved_dir = "/home/soham/ascol_ws/src/quad_arm_trajectory_tracking/data/achieved_states.txt";
  output_file = std::ofstream(output_dir);
  achieved_file = std::ofstream(achieved_dir);
  ref_states = std::vector<std::vector<double>>(totalSteps, std::vector<double>(16));
  ref_controls = std::vector<std::vector<double>>(totalSteps, std::vector<double>(6));


  /* Populate reference vectors from file*/
  for (int i = 0; i < totalSteps; ++i) {
    std::string line;
    std::getline(uref, line);
    std::istringstream iss(line);
    for(int u=0; u < 6; u++) {
      iss >> ref_controls[i][u];
    }
  }

    for (int i = 0; i < totalSteps; ++i) {
    std::string line;
    std::getline(xref, line);
    std::istringstream iss(line);
    for(int x=0; x < 16; x++)
      iss >> ref_states[i][x];
  }

  prop_steps = int(double(numSteps)/te)/(totalSteps/tfinal);

  timeGrid = Grid(ts, te, numSteps);
  output_states = VariablesGrid(16, timeGrid); ///< 16 stats variables
  output_controls = VariablesGrid(6, timeGrid); ///< 6 control

  /* Initialize output variables */
  for(int x=0; x <16; x++)
    output_states(prop_steps,x) = ref_states[0][x];

  for(int u=0; u < 6; u++)
    output_controls(prop_steps,u) = ref_controls[0][u];

  /* create timers */
  controllerTimer = nh.createTimer(ros::Duration(controller_timer_duration), &MPController::velocityControllerTimer, this);
  posControllerTimer = nh.createTimer(ros::Duration(mpc_timer_duration), &MPController::posControllerTimerCallback,this);
  mpcTimer = nh.createTimer(ros::Duration(mpc_timer_duration), &MPController::mpcTimerCallback, this);
  hoverTimer = nh.createTimer(ros::Duration(0.2), &MPController::hoverCallback, this);

  /* Initialize publishers */
  jointPub = nh.advertise<gazebo_aerial_manipulation_plugin::JointCommand>("/joint_command",1);
  rpyPub = nh.advertise<gazebo_aerial_manipulation_plugin::RollPitchYawThrust>("/rpyt_command",1);

  toggleSub = nh.subscribe("/toggle_mpc", 1, &MPController::mpcTimerToggleCallback, this);

  /* Initialize subscribers to sensor callback thread */
  ros::SubscribeOptions pose_so = ros::SubscribeOptions::create<gazebo_aerial_manipulation_plugin::RPYPose>("/base_pose",1,
      boost::bind(&MPController::posSubCallback, this, _1), ros::VoidPtr(), &sensor_callback_queue);
  posSub = nh.subscribe(pose_so);

  ros::SubscribeOptions vel_so = ros::SubscribeOptions::create<geometry_msgs::TwistStamped>("/base_twist",1,
      boost::bind(&MPController::velSubCallback, this, _1), ros::VoidPtr(), &sensor_callback_queue);
  velSub = nh.subscribe(vel_so);

  ros::SubscribeOptions joint_so = ros::SubscribeOptions::create<sensor_msgs::JointState>("/joint_state",1,
      boost::bind(&MPController::jointSubCallback, this, _1), ros::VoidPtr(), &sensor_callback_queue);
  jointSub = nh.subscribe(joint_so);

  ros::SubscribeOptions accn_so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/base_accn",1,
      boost::bind(&MPController::accnSubCallback, this, _1), ros::VoidPtr(), &sensor_callback_queue);
  accnSub = nh.subscribe(accn_so);

  ros::SubscribeOptions yr_so = ros::SubscribeOptions::create<std_msgs::Float64>("/yaw_rate",1,
      boost::bind(&MPController::yawRateSubCallback, this, _1), ros::VoidPtr(), &sensor_callback_queue);
  yawRateSub = nh.subscribe(yr_so);

  /* Stop all timers initially */
  controllerTimer.stop();
  mpcTimer.stop();
  rpyCmdTimer.stop();
  hoverTimer.stop();

  /* Go to starting state */
  geometry_msgs::PoseStamped quadPose;
  quadPose.pose.position.x = 0.0;
  quadPose.pose.position.y = 0.0;
  quadPose.pose.position.z = 0.0;
  double init_yaw = 0.0;
  quadPose.pose.orientation = tf::createQuaternionMsgFromYaw(init_yaw);
  quadPose_.set(quadPose);

  setPosGoal(ref_states[0][0], ref_states[0][1], ref_states[0][2], ref_states[0][12]);
}

void MPController::yawRateSubCallback(const std_msgs::Float64::ConstPtr& yr_msg) {
  yaw_rate_.set(yr_msg->data);
}

void MPController::accnSubCallback(const geometry_msgs::Vector3::ConstPtr& accn_msg) {
  quadAcc_.set(*accn_msg);
}

void MPController::jointSubCallback(const sensor_msgs::JointState::ConstPtr& joint_msg) {
  jointState_.set(*joint_msg);
}

void MPController::velSubCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_msg) {
  quadVel_.set(*vel_msg);
}

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
}

void MPController::velocityControllerTimer(const ros::TimerEvent& event) {
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  double ex = goal_vel.x - quadVel.twist.linear.x;
  double ey = goal_vel.y - quadVel.twist.linear.y;
  double ez = goal_vel.z - quadVel.twist.linear.z;

  if(abs(ex) < tolerance && abs(ey) < tolerance && abs(ez) < tolerance) {
    cex = 0; cey = 0; cez = 0;
    return;
  }

  gazebo_aerial_manipulation_plugin::RollPitchYawThrust rpyt_msg;

  cex += ex; cey += ey; cez += ez;

  // Acceleration in world frame
  Eigen::Vector3d world_acc;
  world_acc(0) = kp_x * ex + ki*cex*controller_timer_duration;
  world_acc(1) = kp_y * ey + ki*cey*controller_timer_duration;
  world_acc(2) = kp_z * ez + ki*cez*controller_timer_duration;

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

void MPController::setVelGoal(double vx, double vy, double vz, double yr) {
  double yaw = tf::getYaw(quadPose_.get().pose.orientation);
  hoverTimer.stop();
  goal_vel.x = vx;
  goal_vel.y = vy;
  goal_vel.z = vz;
  goal_yaw = yr;
  controllerTimer.start();
}

void MPController::setPosGoal(double px, double py, double pz, double yaw) {
    hoverTimer.stop();
    goal_pos.x = px; goal_pos.y = py; goal_pos.z = pz;
    goal_yaw = yaw;
    posControllerTimer.start();
}

void MPController::setAccGoal(double ax, double ay, double az, double yr) {
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
  rpyt_msg.yaw = yr;

  rpyPub.publish(rpyt_msg);
}

void MPController::mpcTimerCallback(const ros::TimerEvent& event) {

  /* Get all sensor data */
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  geometry_msgs::TwistStamped quadVel = quadVel_.get();
  geometry_msgs::Vector3 quadAcc = quadAcc_.get();
  sensor_msgs::JointState jointState = jointState_.get();

  tf::Quaternion q; tf::quaternionMsgToTF(quadPose.pose.orientation, q);

  tf::Transform rot = tf::Transform(q, tf::Vector3(0,0,0));
  double rr,pp,yy; tf::Matrix3x3(q).getRPY(rr,pp,yy);

  /* Write current state to file */
  achieved_file << quadPose.pose.position.x <<" "
                << quadPose.pose.position.y <<" "
                << quadPose.pose.position.z <<" "
                << tf::getYaw(quadPose.pose.orientation) <<" "
                << quadVel.twist.linear.x <<" "
                << quadVel.twist.linear.y <<" "
                << quadVel.twist.linear.z <<" "
                << yaw_rate_.get() <<" "
                << quadAcc.x <<" "
                << quadAcc.y <<" "
                << quadAcc.z <<" "
                << -jointState.position[0] <<" "
                << -jointState.position[1] <<" "
                << rr <<" "<< pp <<" "<< yy <<"\n";

  /* If current step is larger than totalSteps, stop timer */
  to = int( ((ros::Time::now()-mpc_start_time).toSec()-mpc_timer_duration) * double(totalSteps)/double(tfinal));
  if(to > totalSteps) {
    hoverTimer.start();
    controllerTimer.start();
    mpcTimer.stop();
    return;
  }

  // // ROS_INFO("Current : %f %f %f", quadPose.pose.position.x, 
  // //   quadPose.pose.position.y, quadPose.pose.position.z);

  int R = double(numSteps)*double(tfinal)/(te*totalSteps);
  double tnow = (ros::Time::now()-mpc_start_time).toSec()-mpc_timer_duration;
  VariablesGrid reference_grid(22, timeGrid);
  VariablesGrid init_states(16, timeGrid);
  VariablesGrid init_controls(6, timeGrid);

  for(int t=0; t < numSteps; t++) {

    double future_time = tnow + double(t)*te/double(numSteps);
    int step_reference = int(future_time*double(totalSteps)/double(tfinal));
    step_reference = step_reference < totalSteps ? step_reference : totalSteps - 1;
    // ROS_INFO("future_time : %f, step : %i", future_time, step_reference);
    int next_step = step_reference + 1 < totalSteps ? step_reference + 1 : totalSteps-1;
    int nr = (t+to*R)%R; double k = double(nr)/double(numSteps)*te;

    for(int x=0; x < 16; x++)
      reference_grid(t,x) = (1.0 - k)*ref_states[step_reference][x] + k*ref_states[next_step][x];
    for(int u = 0; u < 6; u++)
      reference_grid(t,16+u) = (1.0-k)*ref_controls[step_reference][u] + k*ref_controls[next_step][u];

    for(int x = 0; x < 16; x++)
      init_states(t,x) = reference_grid(t,x);
    for(int u = 0; u < 6; u++)
      init_controls(t,u) = reference_grid(t,16+u);
  }

  /* Cost function */
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

  /* Thrust as ACADO variable */
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

  /* 
  * Use acceleration and jerk from previous state 
  * Acceleration data is obtained from a moving average filter
  * and the optimization is sensitive to it
  */
  ocp.subjectTo(AT_START, x2 == output_states(prop_steps,6));
  ocp.subjectTo(AT_START, y2 == output_states(prop_steps,7));
  ocp.subjectTo(AT_START, z2 == output_states(prop_steps, 8));

  ocp.subjectTo(AT_START, x3 == output_states(prop_steps, 9));
  ocp.subjectTo(AT_START, y3 == output_states(prop_steps, 10));
  ocp.subjectTo(AT_START, z3 == output_states(prop_steps, 11));

  ocp.subjectTo(AT_START, ga0 == tf::getYaw(quadPose.pose.orientation));
  ocp.subjectTo(AT_START, ga1 == output_states(prop_steps, 13));

  ocp.subjectTo(AT_START, q1 == -jointState.position[0]);
  ocp.subjectTo(AT_START, q2 == -jointState.position[1]);

  ocp.subjectTo(AT_START, x4 == output_controls(prop_steps, 0));
  ocp.subjectTo(AT_START, y4 == output_controls(prop_steps, 1));
  ocp.subjectTo(AT_START, z4 == output_controls(prop_steps, 2));
  ocp.subjectTo(AT_START, ga2 == output_controls(prop_steps, 3));
  ocp.subjectTo(AT_START, qd1 == output_controls(prop_steps, 4));
  ocp.subjectTo(AT_START, qd2 == output_controls(prop_steps, 5));

  ocp.subjectTo(-1.3 <= q1 <= 0.0); // joint limits
  ocp.subjectTo(-1.57 <= q2 <= 1.57); // joint limits
  ocp.subjectTo(-1.57 <= ga0 <= 1.57); // joint limits

  ocp.subjectTo(-0.78 <= qd1 <= 0.78); // joint velocity limits
  ocp.subjectTo(-0.78 <= qd2 <= 0.78);

  ocp.subjectTo(-1.0 <= x1 <= 1.0);
  ocp.subjectTo(-1.0 <= y1 <= 1.0);
  ocp.subjectTo(-1.0 <= z1 <= 1.0);
  ocp.subjectTo(-1.0 <= ga1 <= 1.0);

  ocp.subjectTo(T <= 12); // Bounds on thrust

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

  /* Write output state to file */
  output_file << output_states(prop_steps,0) <<" "<<
                 output_states(prop_steps,1) <<" "<<
                 output_states(prop_steps,2) <<" "<<
                 output_states(prop_steps,12) <<" "<<
                 output_states(prop_steps,3) <<" "<< 
                 output_states(prop_steps,4) <<" "<<
                 output_states(prop_steps,5) <<" "<<
                 output_states(prop_steps, 13) <<" "<<
                 output_states(prop_steps, 6) <<" "<<
                 output_states(prop_steps, 7) <<" "<<
                 output_states(prop_steps, 8) <<" "<<
                 output_states(prop_steps, 14) <<" "<<
                 output_states(prop_steps, 15) <<"\n";

  setVelGoal(output_states(prop_steps,3), output_states(prop_steps,4), output_states(prop_steps,5), output_states(prop_steps,12));
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
    hoverTimer.start();
    return;
  }

  double eyaw = goal_yaw - tf::getYaw(quadPose.pose.orientation);
  cex_p += ex; cey_p += ey; cez_p += ez;

  /* Send command to velocity controller */
  setVelGoal(kp_p*ex + ki_p*cex_p*0.05, kp_p*ey + ki_p*cey_p*0.05, kp_p*ez + ki_p*cez_p*0.05, goal_yaw);
}

void MPController::hoverCallback(const ros::TimerEvent& event) {
  ROS_INFO("Hover Callback");
  geometry_msgs::PoseStamped quadPose = quadPose_.get();
  setVelGoal(0.0, 0.0, 0.0, tf::getYaw(quadPose.pose.orientation));
}

void MPController::mpcTimerToggleCallback(const std_msgs::Bool::ConstPtr& msg) {
  if(msg->data) {
    ROS_INFO("Starting MPC");
    posControllerTimer.stop();
    hoverTimer.stop();
    controllerTimer.stop();
    gazebo_aerial_manipulation_plugin::JointCommand joint_command;
    joint_command.header.stamp = ros::Time::now();
    joint_command.desired_joint_angles.push_back(-ref_states[0][14]);
    joint_command.desired_joint_angles.push_back(-ref_states[0][15]);
    jointPub.publish(joint_command);
    sensor_msgs::JointState jointState = jointState_.get();

    ROS_INFO("Commanded : %f %f, Current : %f %f", 
      joint_command.desired_joint_angles[0], joint_command.desired_joint_angles[1],
      jointState.position[0], jointState.position[1]);
    int steps = 0;
    while(abs(joint_command.desired_joint_angles[0] - jointState.position[0]) > pos_tolerance ||
          abs(joint_command.desired_joint_angles[1] - jointState.position[1]) > pos_tolerance) {
      ROS_INFO("Joint state : %f %f", jointState.position[0], jointState.position[1]);
      if(steps > 100) break;
      ros::Duration(0.005).sleep();
      jointState = jointState_.get();
      steps++;

    }
    mpc_start_time = ros::Time::now();
    mpcTimer.start();
  } else {
    ROS_INFO("Stopping MPC");
    mpcTimer.stop();
    hoverTimer.start();
    controllerTimer.start();
    to = 0;
  }
}
