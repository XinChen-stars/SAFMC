//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), fail_detec_(false), ctrl_enable_(true),
      landing_commanded_(false), feedthrough_enable_(false),
      node_state(WAITING_FOR_HOME_POSE) {
  
  referenceSub_     = nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  rviztargetposeSub_   = nh_.subscribe("move_base_simple/goal", 1, &geometricCtrl::rviztargetposeCallback, this, ros::TransportHints().tcpNoDelay());
  targetexistSub_   = nh_.subscribe("planning/target_exist", 1, &geometricCtrl::targetexistCallback, this, ros::TransportHints().tcpNoDelay());
  quadcmdSub_       = nh_.subscribe("/planning/pos_cmd", 1, &geometricCtrl::quadmsgCallback, this, ros::TransportHints().tcpNoDelay());
  yawreferenceSub_  = nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  mavstateSub_      = nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_       = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
  mavtwistSub_      = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());
  
  // 动态参数服务器，用于设置控制器的参数
  ctrltriggerServ_ = nh_.advertiseService("tigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  // 定时器，用于控制控制器的频率
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,this); // Define timer for constant loop rate
  // 定时器，用于发布系统状态
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,this); // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  target_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  last_request_ = ros::Time::now();
  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.07); // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1); // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 1.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_; // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}

//饱和限幅函数
float geometricCtrl::satfunc(float data, float Max) {

  if (abs(data) > Max)
    return (data > 0) ? Max : -Max;
  else
    return data;
}

//目标回调函数
void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ =
      (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

// EGO-Planner & Fast-Planner 目标回调函数
void geometricCtrl::quadmsgCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {
  node_state = MISSION_EXECUTION;
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  cmdpos_time_last_ = reference_request_now_;

  reference_request_dt_ =
      (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ =
      Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  targetVel_ =
      Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  targetAcc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                               cmd->acceleration.z);
  targetJerk_ = Eigen::Vector3d::Zero();
  // targetJerk_ = Eigen::Vector3d(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);//only for EGO-Planner-V2
  targetSnap_ = Eigen::Vector3d::Zero();
  mavYaw_ = double(cmd->yaw);
  // cmdBodyRate_[2] = cmd->yaw_dot;
}

//flattarget回调函数
void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  node_state = MISSION_EXECUTION;
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ =
      (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);

  // if (mavVel_.norm() > 1)
  // velocity_yaw_ = true;
  if (msg.type_mask == 1) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();
  } else if (msg.type_mask == 2) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  } else if (msg.type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  } else {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

//yaw角度回调函数
void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_)
    mavYaw_ = double(msg.data);
}

//rviz目标位置姿态回调函数
void geometricCtrl::rviztargetposeCallback(const geometry_msgs::PoseStamped &msg) {
  target_mavPos_ = toEigen(msg.pose.position);
}

//暂时没有用到
void geometricCtrl::targetexistCallback(const std_msgs::Bool &msg) {
  if(msg.data == 0){
    node_state = MISSION_HOLD;
  }
}

//无人机位置姿态回调函数
void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

//无人机速度、角速度回调函数
void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

//降落服务回调函数
bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request,
                                 std_srvs::SetBool::Response &response) {
  node_state = LANDING;
  return true;
}

//定时器回调函数，状态机
void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
  case WAITING_FOR_HOME_POSE:
    waitForPredicate(&received_home_pose, "Waiting for home pose...");
    ROS_INFO("Got pose! Drone Ready to be armed.");
    node_state = MISSION_EXECUTION;
    // node_state = TAKING_OFF;
    break;

  case TAKING_OFF: {
    geometry_msgs::PoseStamped takingoff_msg;
    takingoff_msg.header.stamp = ros::Time::now();
    takingoff_msg.pose.position.x = initTargetPos_x_;
    takingoff_msg.pose.position.y = initTargetPos_y_;
    takingoff_msg.pose.position.z = initTargetPos_z_;
    target_pose_pub_.publish(takingoff_msg);
    ros::spinOnce();
    break;
  }

  case MISSION_EXECUTION:
  {
    std::cout << "                                   missionexection  " << dtttt << std::endl;
    computeBodyRateCmd(cmdBodyRate_, targetPos_, targetVel_, targetAcc_);
    pubReferencePose(targetPos_, q_des);
    pubRateCommands(cmdBodyRate_);
    appendPoseHistory();
    pubPoseHistory();
    ros::spinOnce();
    break;
  }

  case MISSION_HOLD: {
    std::cout << "                                   HOLDHOLDHOLD  " << dtttt << std::endl;
    geometry_msgs::PoseStamped takingoff_msg;

    holdTargetPos_x_ = mavPos_(0);
    holdTargetPos_y_ = mavPos_(1);
    holdTargetPos_z_ = mavPos_(2);

    takingoff_msg.header.stamp = ros::Time::now();
    takingoff_msg.pose.position.x = holdTargetPos_x_;
    takingoff_msg.pose.position.y = holdTargetPos_y_;
    takingoff_msg.pose.position.z = holdTargetPos_z_;
    // takingoff_msg.pose.orientation.w = mavAtt_(0);
    // takingoff_msg.pose.orientation.x = mavAtt_(1);
    // takingoff_msg.pose.orientation.y = mavAtt_(2);
    // takingoff_msg.pose.orientation.z = mavAtt_(3);
    target_pose_pub_.publish(takingoff_msg);
    ros::spinOnce();
    break;
  }

  case LANDING: {
    if (autoland())
      node_state = LANDED;
    ros::spinOnce();
    break;
  }
  
  case LANDED: 
    ROS_INFO("Landed. Please set to position control and disarm.");
    cmdloop_timer_.stop();
    break;
  }
}

//判断是否降落
bool geometricCtrl::autoland(){
  geometry_msgs::PoseStamped landingmsg;
  if (mavPos_(2) <= 0.3) {
    if (current_state_.mode != "AUTO.LAND") {
      offb_set_mode_.request.custom_mode = "AUTO.LAND";
      if (set_mode_client_.call(offb_set_mode_) &&
          offb_set_mode_.response.mode_sent) {
        ROS_INFO("AUTO.LAN enabled");
        return true;
      }
    }
  } else {
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose.position.z = 0.15;
    landingmsg.pose.position.x = mavPos_(0);
    landingmsg.pose.position.y = mavPos_(1);
    landingmsg.pose.orientation.w = mavAtt_(0);
    landingmsg.pose.orientation.x = mavAtt_(1);
    landingmsg.pose.orientation.y = mavAtt_(2);
    landingmsg.pose.orientation.z = mavAtt_(3);
    target_pose_pub_.publish(landingmsg);
  }
  return false;
}

//无人机连接状态回调函数
void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
}

//定时器，用于发布系统状态
void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
      if (set_mode_client_.call(offb_set_mode_) &&
          offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed &&
          (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

//发布参考位置姿态
void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position,
                                     const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

//发布角速度控制指令
void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd) {
  mavros_msgs::AttitudeTarget msg;
  double maxVelocity = 6.0;
  geometry_msgs::TwistStamped velocity_msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128; // Ignore orientation messages
  msg.thrust = cmd(3);

  // Publish velocity cmd (PI controller)
  const Eigen::Vector3d pos_error = targetPos_ - mavPos_;
  const Eigen::Vector3d vel_error = targetVel_ - mavVel_;
  Eigen::Vector3d v_fb = 0.1 * Kpos_.asDiagonal() * pos_error +
                         0.1 * Kvel_.asDiagonal() *
                             vel_error; // feedback term for trajectory error
  velocity_msg.twist.linear.x =
      satfunc(velocity_msg.twist.linear.x, maxVelocity);
  velocity_msg.twist.linear.y =
      satfunc(velocity_msg.twist.linear.y, maxVelocity);
  velocity_msg.twist.linear.z = satfunc(velocity_msg.twist.linear.z, 0.7);
  angularVelPub_.publish(msg);
//  target_velocity_pub_.publish(velocity_msg);
}

//发布历史位置信息
void geometricCtrl::pubPoseHistory() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

//发布系统状态
void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

//添加位置历史信息
void geometricCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(),
                             vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

//转化位置姿态信息
geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                       Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "world";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

//计算控制指令
void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd,
                                       const Eigen::Vector3d &target_pos,
                                       const Eigen::Vector3d &target_vel,
                                       const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;
  std::cout << "the position error is: " << pos_error(2) << std::endl;

  // 根据位置误差和速度误差计算加速度前馈项
  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; // Clip acceleration if reference is too large

  // Rotor Drag compensation 旋翼阻力补偿
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; // Rotor drag

  // Reference acceleration 期望加速度
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

  // Reference quaternion 期望四元数
  q_des = acc2quaternion(a_des, mavYaw_);
 
  // 根据ctrl_mode_选择控制器
  if (ctrl_mode_ == ERROR_GEOMETRIC) {
    bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_); // Calculate rotate matrix error based BodyRate
  } 
  else if(ctrl_mode_ == ERROR_QUATERNION){
    bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_); // Calculate Quaternion error based BodyRate
  }
  else if (ctrl_mode_ == ERROR_JERKTRACK)
  {
    bodyrate_cmd = jerk_trackingcontroller(q_des, a_des, targetJerk_, mavAtt_);
  }
  else
  {
    ROS_ERROR_STREAM("Invalid control mode: " << ctrl_mode_);
  }
  
}

//四元数乘法
Eigen::Vector4d geometricCtrl::quatMultiplication(const Eigen::Vector4d &q,
                                                  const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
      p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
      p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

//四元数转换为旋转矩阵
Eigen::Matrix3d geometricCtrl::quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
      2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2),
      q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

//旋转矩阵转换为四元数
Eigen::Vector4d geometricCtrl::rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

//将加速度与偏航转换为四元数
Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc,
                                              const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1),
      xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

//获取偏航速度
double geometricCtrl::getVelocityYaw(const Eigen::Vector3d velocity) {
  return atan2(velocity(1), velocity(0));
}

//ctrl_mode_ == ERROR_QUATERNION 四元数计算姿态的误差
Eigen::Vector4d geometricCtrl::attcontroller(const Eigen::Vector4d &ref_att,
                                             const Eigen::Vector3d &ref_acc,
                                             Eigen::Vector4d &curr_att) {
  // Geometric attitude controller
  // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and
  // Raffaello D'Andrea. Nonlinear quadrocopter attitude control: Technical
  // report. ETH Zurich, 2013.

  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;

  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;

  qe = quatMultiplication(q_inv, ref_att);

  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (1.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust

  return ratecmd;
}

//ctrl_mode_ == ERROR_GEOMETRIC 旋转矩阵计算姿态的误差
Eigen::Vector4d geometricCtrl::geometric_attcontroller(const Eigen::Vector4d &ref_att,
                                       const Eigen::Vector3d &ref_acc,
                                       Eigen::Vector4d &curr_att) {
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris
  // McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th
  // IEEE conference on decision and control (CDC). IEEE, 2010. The original
  // paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;   // Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; // Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat -
                                   rotmat.transpose() * rotmat);
  ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust

  return ratecmd;
}

//ctrl_mode_ == ERROR_JERKTRACK
Eigen::Vector4d geometricCtrl::jerk_trackingcontroller(const Eigen::Vector4d &ref_att,
                                      const Eigen::Vector3d &ref_acc,
                                      const Eigen::Vector3d &ref_jerk,
                                      Eigen::Vector4d &curr_att){
  // Jerk feedforward control
  // Based on: Lopez, Brett Thomas. Low-latency trajectory planning for high-speed navigation in unknown environments.
  // Diss. Massachusetts Institute of Technology, 2016.
  // Feedforward control from Lopez(2016)

  Eigen::Vector4d ratecmd;
  double dt_ = 0.01;
  // Numerical differentiation to calculate jerk_fb
  const Eigen::Vector3d jerk_fb = (ref_acc - last_ref_acc_) / dt_;
  const Eigen::Vector3d jerk_des = ref_jerk + jerk_fb;
  const Eigen::Matrix3d R = quat2RotMatrix(curr_att);
  const Eigen::Vector3d zb = R.col(2);

  const Eigen::Vector3d jerk_vector =
      jerk_des / ref_acc.norm() - ref_acc * ref_acc.dot(jerk_des) / std::pow(ref_acc.norm(), 3);
  const Eigen::Vector4d jerk_vector4d(0.0, jerk_vector(0), jerk_vector(1), jerk_vector(2));

  Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
  const Eigen::Vector4d qd = quatMultiplication(q_inv, ref_att);

  const Eigen::Vector4d qd_star(qd(0), -qd(1), -qd(2), -qd(3));

  const Eigen::Vector4d ratecmd_pre = quatMultiplication(quatMultiplication(qd_star, jerk_vector4d), qd);

  
  ratecmd(0) = ratecmd_pre(2);  // TODO: Are the coordinate systems consistent?
  ratecmd(1) = (-1.0) * ratecmd_pre(1);
  ratecmd(2) = 0.0;
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); // Calculate thrust
  last_ref_acc_ = ref_acc;

  return ratecmd;                          
}

// Returns the skew symmetric matrix of a vector
Eigen::Matrix3d geometricCtrl::matrix_hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

// Returns the inverse of the skew symmetric matrix
Eigen::Vector3d geometricCtrl::matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

//返回无人机的位置、姿态、速度、角速度
void geometricCtrl::getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att,
                              Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
  pos = mavPos_;
  att = mavAtt_;
  vel = mavVel_;
  angvel = mavRate_;
}

//返回无人机的位置和速度误差
void geometricCtrl::getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
  pos = mavPos_ - targetPos_;
  vel = mavVel_ - targetVel_;
}

//动态参数服务器设置控制器模式
bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}

//设置控制指令 no use
void geometricCtrl::setBodyRateCommand(Eigen::Vector4d bodyrate_command) {
  cmdBodyRate_ = bodyrate_command;
}

//不知道这个函数是干嘛的 no use
void geometricCtrl::setFeedthrough(bool feed_through) {
  feedthrough_enable_ = feed_through;
}

//配置动态参数服务器，geometric_controller/cfg/GeometricController.cfg，动态改变控制器参数Kpos_、Kvel_、max_fb_acc_
void geometricCtrl::dynamicReconfigureCallback(
    geometric_controller::GeometricControllerConfig &config, uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
}
