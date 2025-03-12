#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <cmath>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/DroneTagDetection.h>
#include <apriltag_ros/DroneTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/ColorRGBA.h>
#include <kf/kalmanFilter.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Empty.h>
#include <unordered_map>
#include <unordered_set>

ros::Publisher vis_tag_pub;
ros::Publisher uav_pose_pub;
ros::Publisher uav_model_pub;
ros::Publisher uav_land_pose_pub;
ros::Publisher drone_detect_pub;
ros::Subscriber uav_state_sub;
ros::Subscriber wp_finished_sub;
ros::Subscriber other_drone_detect_sub;
geometry_msgs::PoseStamped uav_pose;
visualization_msgs::MarkerArray vis_uav_model;
ros::Publisher fov_pub;
std::vector<ros::Publisher> VICTIM_pose_pub;
std::vector<ros::Publisher> DANGER_pose_pub;
std::vector<geometry_msgs::PoseStamped> VICTIM_pose_list;
std::vector<geometry_msgs::PoseStamped> DANGER_pose_list;
ros::ServiceClient px4_set_mode_client;//设置飞行模式
std::string node_name;
std::string prename;
std::string uav_name;
bool use_prename;
bool finished_wp_flag = false;
bool init_land_flag = false;
bool final_land_flag = false;
int drone_id;
int drone_num;
int VICTIM_num;
int DANGER_num;
bool received_odom = false;

Eigen::Matrix4d body2Cam; // from body frame to camera frame
Eigen::Matrix4d local2Cam; // from local frame to camera frame
Eigen::Matrix3d local2Cam_rotate;
double offset_x, offset_y, offset_z;

Eigen::Vector3d odom_p, odom_v;
Eigen::Vector3d init_land_p, final_land_p;
int land_p_target_id;
int other_already_land_id;
std::vector<int> other_already_land_ids;
std::unordered_set<int> other_already_land_ids_set;
std::unordered_set<int> other_already_land_ids_set_second;
bool is_other_drone_land = false;
bool is_other_drone_land_second = false;
int land_way_mode;//0:own land 1:other land
Eigen::Quaterniond odom_q;

apriltag_ros::AprilTagDetection detect_msg;
apriltag_ros::AprilTagDetectionArray detect_msgs;
apriltag_ros::DroneTagDetection other_detect_msg;
apriltag_ros::DroneTagDetectionArray other_detect_msgs;
std::unordered_set<int32_t> known_drone_ids;
std::unordered_map<int32_t, ros::Time> known_drone_timestamps;
std::unordered_map<int32_t, apriltag_ros::DroneTagDetectionArray> other_drone_detect_msgs;
std::map<int32_t, ros::Subscriber> drone_subscribers;
geometry_msgs::PoseStamped tag0_pose;
geometry_msgs::PoseStamped tag1_pose;
geometry_msgs::PoseStamped tag2_pose;

struct Tag_Sptial_Temporal
{
  Eigen::Vector3d tag_position;
  Eigen::Vector3d sum_position;
  kalman_filter kf;
  Eigen::Vector3d init_tag_position;//初始位置
  Eigen::Vector3d observation_tag_position;//观测到的位置
  Eigen::Vector3d estimated_tag_position;//估计位置
  int record_num;
  int tag_id;
  int target_id;
};

/* Apriltag  Detection Callback function */
std::vector<Tag_Sptial_Temporal> detected_VICTIM_tags_;// id: 0 ~ 7 VICTIM
std::vector<Tag_Sptial_Temporal> detected_DANGER_tags_;// id: 8 ~ 11 DANGER
std::vector<Eigen::Vector3d> other_detected_VICTIM_tag_;
std::vector<Eigen::Vector3d> other_already_land_pose_;
bool is_the_same_land = false;
std::vector<bool> VICTIM_init_flag;
std::vector<bool> DANGER_init_flag;
int VICTIM_tags_num = 0;
int wait_time = 0;

std::vector<double> colorR = {150, 0, 0, 255, 255, 0, 243, 143, 112, 255, 241};
std::vector<double> colorG = {150, 229, 0, 0, 44, 212, 110, 255, 0, 165, 84};
std::vector<double> colorB = {150, 238, 255, 255, 44, 112, 50, 0, 255, 0, 76};
std::vector<std_msgs::ColorRGBA> COLOR_LISTS;

mavros_msgs::State uav_state;//无人机状态
mavros_msgs::State last_uav_state;//上一时刻无人机状态

void DeleteSameOtherDroneDetectResult()
{
  if (other_detected_VICTIM_tag_.size() < 2)
  {
    return;
  }
  
 for (size_t i = 0; i < other_detected_VICTIM_tag_.size(); ++i) {
    for (size_t j = i + 1; j < other_detected_VICTIM_tag_.size(); ) {
      Eigen::Vector3d diff = other_detected_VICTIM_tag_[i] - other_detected_VICTIM_tag_[j];
      double dx = diff.x();
      double dy = diff.y();
      double horizontal_dist = sqrt(dx*dx + dy*dy);
      if (horizontal_dist < 0.5) 
      {
        other_detected_VICTIM_tag_.erase(other_detected_VICTIM_tag_.begin() + j);
      } 
      else {
        ++j;
      }
    }
  }
}

bool CheckSameLand(const int type)
{
  other_already_land_ids_set.clear();
  if (other_already_land_pose_.empty())
  {
    return false;
  }
  for (int i = 0; i < other_already_land_pose_.size(); i++)
  {
    if (type == 0)
    {
      for (int j = 0; j < detected_VICTIM_tags_.size(); j++)
      {
        Eigen::Vector3d diff = other_already_land_pose_[i] - detected_VICTIM_tags_[j].tag_position;
        double dx = diff.x();
        double dy = diff.y();
        double horizontal_dist = sqrt(dx*dx + dy*dy);
        if (horizontal_dist < 0.5)
        {
          other_already_land_ids_set.insert(j);
          ROS_ERROR_STREAM(prename << " Type " << type << " Same land with other drone " << j);
        }
      }
    }
    else
    {
      for (int j = 0; j < other_detected_VICTIM_tag_.size(); j++)
      {
        Eigen::Vector3d diff = other_already_land_pose_[i] - other_detected_VICTIM_tag_[j];
        double dx = diff.x();
        double dy = diff.y();
        double horizontal_dist = sqrt(dx*dx + dy*dy);
        if (horizontal_dist < 0.5)
        {
          other_already_land_ids_set.insert(j);
          ROS_ERROR_STREAM(prename << " Type " << type <<" Same land with other drone " << j);
        }
      }
    }
  }
  if (!other_already_land_ids_set.empty())
  {
    is_other_drone_land = true;
    return true;
  }
  return false;
}

void CheckSameLandSecond()
{
  other_already_land_ids_set_second.clear();
  if (other_already_land_pose_.empty())
  {
    return;
  }
  for (int i = 0; i < other_already_land_pose_.size(); i++)
  {
    
    for (int j = 0; j < detected_VICTIM_tags_.size(); j++)
    {
      Eigen::Vector3d diff = other_already_land_pose_[i] - detected_VICTIM_tags_[j].tag_position;
      double dx = diff.x();
      double dy = diff.y();
      double horizontal_dist = sqrt(dx*dx + dy*dy);
      if (horizontal_dist < 0.5)
      {
        other_already_land_ids_set_second.insert(j);
        ROS_ERROR_STREAM(prename << " Second Check "  << " Same land with other drone " << j);
      }
    }
  }
  if (!other_already_land_ids_set_second.empty())
  {
    is_other_drone_land_second = true;
    return;
  }
  return;
}

void GetOtherDroneDetectResult()
{
  other_detected_VICTIM_tag_.clear();
  other_already_land_pose_.clear();

  for (auto& drone_id : known_drone_ids)
  {
    other_detect_msgs = other_drone_detect_msgs[drone_id];
    for (int i = 0; i < other_detect_msgs.detections.size(); i++)
    {
      other_detect_msg = other_detect_msgs.detections[i];
      if (other_detect_msg.is_claimed)
      {
        Eigen::Vector3d already_land_pose;
        already_land_pose << other_detect_msg.position.x, other_detect_msg.position.y, other_detect_msg.position.z;
        ROS_WARN_STREAM(prename << " Other drone already land pose: " << already_land_pose.transpose() << " from drone_id: " << drone_id);
        other_already_land_pose_.push_back(already_land_pose);
      }
      else
      {
        Eigen::Vector3d other_VICTIM_pose;
        other_VICTIM_pose << other_detect_msg.position.x, other_detect_msg.position.y, other_detect_msg.position.z;
        Eigen::Vector3d diff = other_VICTIM_pose - odom_p;
        double dx = diff.x();
        double dy = diff.y();
        double horizontal_dist = sqrt(dx*dx + dy*dy);
        if (horizontal_dist > 10.0)
        {
          ROS_ERROR_STREAM(prename << " Other drone detect VICTIM pose: " << other_VICTIM_pose.transpose() << " from drone_id: " << drone_id << " is not a good land pose horizontal_dist: " << horizontal_dist);
          continue;
        }
        other_detected_VICTIM_tag_.push_back(other_VICTIM_pose);
      }
    }
  }
}

void PubDroneDetectResult()
{
  apriltag_ros::DroneTagDetectionArray drone_detect_msg;
  drone_detect_msg.header.stamp = ros::Time::now();
  drone_detect_msg.drone_id = drone_id;

  if (!detected_VICTIM_tags_.empty())
  {
    // 本机无人机检测结果
    for (int i = 0; i < detected_VICTIM_tags_.size(); ++i) {
      if (other_already_land_ids_set.count(i) && is_other_drone_land)
      {
        // 跳过已经降落过无人机的victim
        continue;
      }
      
      apriltag_ros::DroneTagDetection own_detect_msg;
      own_detect_msg.target_id = detected_VICTIM_tags_[i].target_id;
      own_detect_msg.position.x = detected_VICTIM_tags_[i].tag_position(0);
      own_detect_msg.position.y = detected_VICTIM_tags_[i].tag_position(1);
      own_detect_msg.position.z = detected_VICTIM_tags_[i].tag_position(2);
      if (i == land_p_target_id)
      {
        own_detect_msg.is_claimed = true;
      }
      else
      {
        own_detect_msg.is_claimed = false;
      }
      drone_detect_msg.detections.push_back(own_detect_msg);
    }
  }
  else
  {
    // 发布其他无人机检测结果（本机筛选降落点后的）
    for (int i = 0; i < other_detected_VICTIM_tag_.size(); ++i) {
      if (other_already_land_ids_set.count(i) && is_other_drone_land)
      {
        // 跳过已经降落过无人机的victim
        continue;
      }
      apriltag_ros::DroneTagDetection own_detect_msg;
      own_detect_msg.target_id = i;
      own_detect_msg.position.x = other_detected_VICTIM_tag_[i](0);
      own_detect_msg.position.y = other_detected_VICTIM_tag_[i](1);
      own_detect_msg.position.z = other_detected_VICTIM_tag_[i](2);
      if (i == land_p_target_id)
      {
        own_detect_msg.is_claimed = true;
      }
      else
      {
        own_detect_msg.is_claimed = false;
      }
      drone_detect_msg.detections.push_back(own_detect_msg);
    }
  }

  drone_detect_pub.publish(drone_detect_msg);
  ROS_ERROR_STREAM(prename << "---------  Publish drone detect result" << "size: " << drone_detect_msg.detections.size());
}

void PubDroneDetectResultSecond()
{
  apriltag_ros::DroneTagDetectionArray drone_detect_msg;
  drone_detect_msg.header.stamp = ros::Time::now();
  drone_detect_msg.drone_id = drone_id;

  if (!detected_VICTIM_tags_.empty())
  {
    // 本机无人机检测结果
    for (int i = 0; i < detected_VICTIM_tags_.size(); ++i) {
      if (other_already_land_ids_set_second.count(i) && is_other_drone_land_second)
      {
        // 跳过已经降落过无人机的victim
        continue;
      }
      if ((other_detected_VICTIM_tag_[land_p_target_id] - detected_VICTIM_tags_[i].tag_position).head(2).norm() < 0.5)
      {
        // 跳过本机降落过无人机的victim
        continue;
      }
      
      apriltag_ros::DroneTagDetection own_detect_msg;
      own_detect_msg.target_id = detected_VICTIM_tags_[i].target_id;
      own_detect_msg.position.x = detected_VICTIM_tags_[i].tag_position(0);
      own_detect_msg.position.y = detected_VICTIM_tags_[i].tag_position(1);
      own_detect_msg.position.z = detected_VICTIM_tags_[i].tag_position(2);
      own_detect_msg.is_claimed = false;
      drone_detect_msg.detections.push_back(own_detect_msg);
    }
  }
  
  // 发布其他无人机检测结果（本机筛选降落点后的）
  for (int i = 0; i < other_detected_VICTIM_tag_.size(); ++i) {
    if (other_already_land_ids_set.count(i) && is_other_drone_land)
    {
      // 跳过已经降落过无人机的victim
      continue;
    }
    apriltag_ros::DroneTagDetection own_detect_msg;
    own_detect_msg.target_id = i;
    own_detect_msg.position.x = other_detected_VICTIM_tag_[i](0);
    own_detect_msg.position.y = other_detected_VICTIM_tag_[i](1);
    own_detect_msg.position.z = other_detected_VICTIM_tag_[i](2);
    if (i == land_p_target_id)
    {
      own_detect_msg.is_claimed = true;
    }
    else
    {
      own_detect_msg.is_claimed = false;
    }
    drone_detect_msg.detections.push_back(own_detect_msg);
  }
  

  drone_detect_pub.publish(drone_detect_msg);
  ROS_INFO_STREAM(prename << "--------- Publish drone detect result" << "size: " << drone_detect_msg.detections.size());
}

// select a new land position
Eigen::Vector3d SelectNewLandPosition(const Eigen::Vector3d& Closest_VICTIM_position, const Eigen::Vector3d& Closest_Danger_position) {
  Eigen::Vector3d new_land_position;
  Eigen::Vector3d VICTIM_position;
  Eigen::Vector3d Danger_position;
  Eigen::Vector3d mid_position;
  VICTIM_position = Closest_VICTIM_position;
  Danger_position = Closest_Danger_position;
  VICTIM_position(2) = 0;//ignore the z axis
  Danger_position(2) = 0;//ignore the z axis

  // calculate the mid point between the closest VICTIM and DANGER
  mid_position = (VICTIM_position + Danger_position) / 2;
  mid_position(2) = 0;//ignore the z axis

  // calculate the vector from the closest DANGER to the closest VICTIM
  double distance = (VICTIM_position - Danger_position).norm();
  Eigen::Vector3d direction = (VICTIM_position - Danger_position).normalized();
  direction(2) = 0;//ignore the z axis

  Eigen::Vector3d Danger_bound_position;
  Danger_bound_position = Danger_position + direction * 1.0;
  Danger_bound_position(2) = 0;//ignore the z axis

  Eigen::Vector3d VICTIM_bound_position;
  VICTIM_bound_position = VICTIM_position + direction * 1.0;
  VICTIM_bound_position(2) = 0;//ignore the z axis

  new_land_position = (VICTIM_bound_position + Danger_bound_position) / 2;
  new_land_position(2) = 0;//ignore the z axis

  // select the new land position as the mid point plus the vector from the mid point to the closest VICTIM
  // ROS_ERROR_STREAM(prename << " Original land position: " << VICTIM_position.transpose());
  // ROS_ERROR_STREAM(prename << " Select New land position: " << new_land_position.transpose());
  return new_land_position;
}

// find the closest point in the detected VICTIM tags
int findClosestVICTIM(const Eigen::Vector3d& pos , const int type, bool same_land) {
  double min_dist = std::numeric_limits<double>::max();
  int closest_tag = -1;

  if (type == 0)
  {
    for (int i = 0; i < detected_VICTIM_tags_.size(); ++i) {
      if (same_land && other_already_land_ids_set.count(i))
      {
        continue;
      }
      
      Eigen::Vector3d diff = detected_VICTIM_tags_[i].tag_position - pos;
      double dx = diff.x();
      double dy = diff.y();
      double horizontal_dist = sqrt(dx*dx + dy*dy);
      // only horizontal distance
      // double horizontal_dist = (detected_VICTIM_tags_[i].tag_position - pos).head(2).norm();
      // ROS_ERROR_STREAM(prename << " VICTIM ID: " << i << "  horizontal_dist: " << horizontal_dist);
      // ROS_ERROR_STREAM(prename << " VICTIM ID: " << i 
      // << "\n tag_pos: " << detected_VICTIM_tags_[i].tag_position.transpose()
      // << "\n current_pos: " << pos.transpose()
      // << "\n dx: " << dx << " dy: " << dy
      // << "\n horizontal_dist: " << horizontal_dist);
      if (horizontal_dist < min_dist) {
        min_dist = horizontal_dist;
        closest_tag = i;
      }
    }
  }
  else if (type == 1)
  {
    for (int i = 0; i < other_detected_VICTIM_tag_.size(); ++i) {
      if (same_land && other_already_land_ids_set.count(i))
      {
        continue;
      }
      Eigen::Vector3d diff = other_detected_VICTIM_tag_[i] - pos;
      double dx = diff.x();
      double dy = diff.y();
      double horizontal_dist = sqrt(dx*dx + dy*dy);
      // only horizontal distance
      // double horizontal_dist = (other_detected_VICTIM_tag_[i] - pos).head(2).norm();
      ROS_INFO_STREAM(prename << " VICTIM ID: " << i 
      << "\n tag_pos: " << other_detected_VICTIM_tag_[i].transpose()
      << "\n current_pos: " << pos.transpose()
      << "\n dx: " << dx << " dy: " << dy
      << "\n horizontal_dist: " << horizontal_dist);
      if (horizontal_dist < min_dist) {
        min_dist = horizontal_dist;
        closest_tag = i;
      }
    }
  }
  
  return closest_tag;
}

int findClosestDANGER(const Eigen::Vector3d& pos) {
  double min_dist = std::numeric_limits<double>::max();
  int closest_tag = -1;

  for (int i = 0; i < detected_DANGER_tags_.size(); ++i) {
    // only horizontal distance
    Eigen::Vector3d diff = detected_DANGER_tags_[i].tag_position - pos;
    double dx = diff.x();
    double dy = diff.y();
    double horizontal_dist = sqrt(dx*dx + dy*dy);
    // double horizontal_dist = (detected_DANGER_tags_[i].tag_position - pos).head(2).norm();
    if (horizontal_dist < min_dist) {
      min_dist = horizontal_dist;
      closest_tag = i;
    }
  }
  
  return closest_tag;
}

void UavStateCallback(const mavros_msgs::StateConstPtr& msg) {
  last_uav_state = uav_state;
  uav_state = *msg;
}

void SetPX4Mode(const std::string& mode) {
  mavros_msgs::SetMode set_mode;
  set_mode.request.custom_mode = mode;
  if (px4_set_mode_client.call(set_mode)) {
    // ROS_WARN_STREAM("Set mode to " << mode);
  } else {
    ROS_ERROR_STREAM("Failed to set mode to " << mode);
  }
}

std::vector<geometry_msgs::Point> generateCirclePoints(double radius, int num_points) {
  std::vector<geometry_msgs::Point> points;
  for (int i = 0; i < num_points; ++i) {
    double angle = 2 * M_PI * i / num_points;
    geometry_msgs::Point p;
    p.x = radius * cos(angle);
    p.y = radius * sin(angle);
    p.z = 0;
    points.push_back(p);
  }
  // 连接最后一个点到第一个点
  points.push_back(points.front());
  return points;
}

std_msgs::ColorRGBA Id2Color(int idx, double a){
  std_msgs::ColorRGBA color;
  idx = idx % int(COLOR_LISTS.size());
  color = COLOR_LISTS[idx];
  color.a = a;
  return color;
}

void Kalman_Filter_Init(double init_x, double init_y, double init_z, kalman_filter &kf) {
  // 状态向量维度和观测向量维度
  int state_dim = 3;  // [x, y, z]
  int measurement_dim = 3; // [观测到的位置维度]

  // 状态向量初始化
  Eigen::MatrixXd states(state_dim, 1);
  double state1 = init_x;
  double state2 = init_y;
  double state3 = init_z;// 初始位置均为 0
  states << state1, state2, state3;  

  // 状态转移矩阵 A
  Eigen::MatrixXd A(state_dim, state_dim);
  A << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
  // 控制矩阵 B
  Eigen::MatrixXd B(state_dim, 1);
  B << 0,
        0,
        0;

  // 状态协方差矩阵 P
  Eigen::MatrixXd P(state_dim, state_dim);
  P << 0.3, 0, 0,
        0, 0.3, 0,
        0, 0, 0.3;

  // 过程噪声协方差矩阵 Q
  Eigen::MatrixXd Q(state_dim, state_dim);
  Q << 0.00001, 0, 0,
        0, 0.00001, 0,
        0, 0, 0.00001;

  // 观测噪声协方差矩阵 R
  Eigen::MatrixXd R(measurement_dim, measurement_dim);
  R << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 0.1;
  // 观测矩阵 H
  Eigen::MatrixXd H(measurement_dim, state_dim);
  H << 1, 0, 0,
        0, 1, 0,
        0, 0, 1; // 观测值为位置坐标

  // 初始化卡尔曼滤波器
  kf.setup(states, A, B, H, P, Q, R);    

}

Eigen::Vector3d Kalman_Filter_Estimate(double obs_x, double obs_y, double obs_z, double est_x, double est_y, double est_z, kalman_filter &kf) {
  // 控制输入为0
  Eigen::MatrixXd control_input(1, 1);
  control_input << 0; 

  // 模拟观测值 (位置)
  Eigen::MatrixXd measurement(3, 1);
  measurement << obs_x, obs_y, obs_z;  // 观测到的位置


  int state_dim = 3;  // [x, y, z]
  int measurement_dim = 3; // [观测到的位置维度]

  // 状态向量初始化
  Eigen::MatrixXd states(3, 1);
  states << est_x, est_y, est_z;  
  kf.set_state(states);

  // 执行卡尔曼滤波进行更新
  kf.estimate(measurement, control_input);

  // 输出估计值
  //std::cout << "Estimated position: " << kf.output(0) << std::endl;//x坐标
  //std::cout << "Estimated position: " << kf.output(1) << std::endl;//y坐标
  //std::cout << "Estimated position: " << kf.output(2) << std::endl;//z坐标
  double state1_output;
  double state2_output;
  double state3_output;

  state1_output =kf.output(0);
  state2_output =kf.output(1);
  state3_output =kf.output(2);

  Eigen::Vector3d estimated_position;
  estimated_position << state1_output, state2_output, state3_output;
  return estimated_position;

}


// Create the visualization model of the UAV
void CreateVisModels(){
  vis_uav_model.markers.resize(5);
  vis_uav_model.markers[0].header.frame_id = "world";
  vis_uav_model.markers[0].header.stamp = ros::Time::now();
  vis_uav_model.markers[0].id = 0;
  vis_uav_model.markers[0].action = visualization_msgs::Marker::ADD;
  vis_uav_model.markers[0].type = visualization_msgs::Marker::SPHERE;
  vis_uav_model.markers[0].scale.x = 0.15;
  vis_uav_model.markers[0].scale.y = 0.15;
  vis_uav_model.markers[0].scale.z = 0.04;
  vis_uav_model.markers[0].color = Id2Color(1, 1.0);

  vis_uav_model.markers[1] = vis_uav_model.markers[0];
  vis_uav_model.markers[1].id = 1;
  vis_uav_model.markers[2] = vis_uav_model.markers[0];
  vis_uav_model.markers[2].id = 2;
  vis_uav_model.markers[3] = vis_uav_model.markers[0];
  vis_uav_model.markers[3].id = 3;
  vis_uav_model.markers[4] = vis_uav_model.markers[0];
  vis_uav_model.markers[4].id = 4;
  vis_uav_model.markers[4].type = visualization_msgs::Marker::LINE_LIST;
  vis_uav_model.markers[4].scale.x = 0.025;
  vis_uav_model.markers[4].scale.y = 0.025;
  vis_uav_model.markers[4].scale.z = 0.025;

  geometry_msgs::Point pt;

  pt.x = 0.1;
  pt.y = 0.1;
  pt.z = -0.02;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.y = -0.1;
  pt.x = -0.1;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.x = -0.1;
  pt.y = 0.1;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.x = 0.1;
  pt.y = -0.1;
  vis_uav_model.markers[4].points.emplace_back(pt);
}

// Load the visualization model of the UAV
void LoadVisModels(){
  vis_uav_model.markers[0].header.stamp = ros::Time::now();
  vis_uav_model.markers[1].header.stamp = ros::Time::now();
  vis_uav_model.markers[2].header.stamp = ros::Time::now();
  vis_uav_model.markers[3].header.stamp = ros::Time::now();
  vis_uav_model.markers[4].header.stamp = ros::Time::now();

  vis_uav_model.markers[0].pose.orientation = uav_pose.pose.orientation;
  vis_uav_model.markers[1].pose.orientation = uav_pose.pose.orientation;
  vis_uav_model.markers[2].pose.orientation = uav_pose.pose.orientation;
  vis_uav_model.markers[3].pose.orientation = uav_pose.pose.orientation;
  vis_uav_model.markers[4].pose = uav_pose.pose;

  Eigen::Quaterniond rot;
  rot.x() = uav_pose.pose.orientation.x;
  rot.y() = uav_pose.pose.orientation.y;
  rot.z() = uav_pose.pose.orientation.z;
  rot.w() = uav_pose.pose.orientation.w;
  Eigen::Vector3d pos;
  pos(0) = uav_pose.pose.position.x;
  pos(1) = uav_pose.pose.position.y;
  pos(2) = uav_pose.pose.position.z;

  Eigen::Vector3d p(0.1, 0.1, -0.02);
  std::vector<Eigen::Vector3d> pl;
  pl.emplace_back(p);
  p(0) = -p(0);
  pl.emplace_back(p);
  p(1) = -p(1);
  pl.emplace_back(p);
  p(0) = -p(0);
  pl.emplace_back(p);

  for(int j = 0; j < 4; j++){
      p = rot.toRotationMatrix() * pl[j] + pos;
      vis_uav_model.markers[j].pose.position.x = p(0);
      vis_uav_model.markers[j].pose.position.y = p(1);
      vis_uav_model.markers[j].pose.position.z = p(2);
  }
    
}

// check if the position is valid
bool isValidPosition(const Eigen::Vector3d& pos) {
    return std::isfinite(pos(0)) && std::isfinite(pos(1)) && std::isfinite(pos(2));
          //  pos.norm() > 0.1 && // 确保位置不在原点附近
          //  pos(2) >= 0;  // 确保高度非负
}

// filter the position of the detected tag
void FilterPosition(const Eigen::Vector3d& new_position,const int type, const int apriltag_id){
  
  if (!isValidPosition(new_position)) {
    ROS_WARN("Invalid position detected, not adding new target");
    return;  // invalid position will not add to KF
  }

  if (type == 0)// VICTIM
  {
    // 1. check if match with existed target    
    for (auto& target : detected_VICTIM_tags_) {               
        // 2. select the same apriltag id
        if (target.tag_id == apriltag_id) {
            target.record_num++;
            target.observation_tag_position = new_position;
            target.estimated_tag_position = Kalman_Filter_Estimate(new_position(0),new_position(1),new_position(2),target.estimated_tag_position(0),target.estimated_tag_position(1),target.estimated_tag_position(2),target.kf);
            target.sum_position += target.estimated_tag_position;
            
            target.tag_position = target.sum_position * (1.0 / target.record_num);
            
            return;
        }
    }
  }
  else if (type == 1)// DANGER ZONE
  {
    // 1. check if match with existed target      
    for (auto& target : detected_DANGER_tags_) {               
        // 2. select the same apriltag id
        if (target.tag_id == apriltag_id) {
            target.record_num++;
            target.observation_tag_position = new_position;
            target.estimated_tag_position = Kalman_Filter_Estimate(new_position(0),new_position(1),new_position(2),target.estimated_tag_position(0),target.estimated_tag_position(1),target.estimated_tag_position(2),target.kf);
            target.sum_position += target.estimated_tag_position;
            
            target.tag_position = target.sum_position * (1.0 / target.record_num);
            
            return;
        }
    }
  }
}

// transform the tag position from camera frame to world frame
void transform_tag_position()
{
  for (int i = 0; i < detect_msgs.detections.size() - 1; i++)
  {
    detect_msg = detect_msgs.detections[i];
    
    if (detect_msg.id[0]<=7 && detect_msg.id[0]>=0)// VICTIM
    {
      int apriltag_id = detect_msg.id[0];
      Eigen::Vector3d RDF_pose;
      RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
      Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

      if (!VICTIM_init_flag[apriltag_id])
      {
        Tag_Sptial_Temporal detected_tag_VICTIM;
        detected_tag_VICTIM.tag_position = ENU_pose;
        detected_tag_VICTIM.sum_position = ENU_pose;
        detected_tag_VICTIM.record_num = 1;
        detected_tag_VICTIM.tag_id = apriltag_id;
        detected_tag_VICTIM.init_tag_position = ENU_pose;
        detected_tag_VICTIM.observation_tag_position = ENU_pose;
        Kalman_Filter_Init(ENU_pose(0),ENU_pose(1),ENU_pose(2),detected_tag_VICTIM.kf);
        detected_tag_VICTIM.estimated_tag_position = ENU_pose;
        detected_VICTIM_tags_.push_back(detected_tag_VICTIM);
        VICTIM_init_flag[apriltag_id] = true;
      }
      else
      {
        // Use Kalman Filter to estimate the position
        FilterPosition(ENU_pose,0,apriltag_id);
      }
    }
    else if (detect_msg.id[0]<=11 && detect_msg.id[0]>=8)// DANGER ZONE
    {
      int apriltag_id = detect_msg.id[0];
      Eigen::Vector3d RDF_pose;
      RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
      Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

      if (!DANGER_init_flag[apriltag_id - 8])
      {
        Tag_Sptial_Temporal detected_tag_DANGER;
        detected_tag_DANGER.tag_position = ENU_pose;
        detected_tag_DANGER.sum_position = ENU_pose;
        detected_tag_DANGER.record_num = 1;
        detected_tag_DANGER.tag_id = apriltag_id;
        detected_tag_DANGER.init_tag_position = ENU_pose;
        detected_tag_DANGER.observation_tag_position = ENU_pose;
        Kalman_Filter_Init(ENU_pose(0),ENU_pose(1),ENU_pose(2),detected_tag_DANGER.kf);
        detected_tag_DANGER.estimated_tag_position = ENU_pose;
        detected_DANGER_tags_.push_back(detected_tag_DANGER);
        DANGER_init_flag[apriltag_id -8] = true;
      }
      else
      {
        // Use Kalman Filter to estimate the position
        FilterPosition(ENU_pose,1,apriltag_id);
      }
    }
    
  }

}

visualization_msgs::Marker VisCUBETag(const geometry_msgs::PoseStamped tag_pose,const int id, const int target_id)
{
  // 可视化方块
  visualization_msgs::Marker tag_marker;
  tag_marker.header.frame_id = "world";
  tag_marker.header.stamp = ros::Time::now();
  tag_marker.ns = "tagCube" + std::to_string(id);
  tag_marker.id = target_id;
  tag_marker.type = visualization_msgs::Marker::CUBE;
  tag_marker.action = visualization_msgs::Marker::ADD;
  tag_marker.pose = tag_pose.pose;
  tag_marker.pose.orientation.w = 1.0;
  tag_marker.scale.x = 0.3;
  tag_marker.scale.y = 0.3;
  tag_marker.scale.z = 0.1;
  tag_marker.color.a = 0.5;
  if (id == 0)
  {
    tag_marker.color.r = 0.0;
    tag_marker.color.g = 1.0;
    tag_marker.color.b = 0.0;
  }
  else if (id == 1)
  {
    tag_marker.color.r = 1.0;
    tag_marker.color.g = 0.0;
    tag_marker.color.b = 0.0;
  }
  else if (id == 2)
  {
    tag_marker.color.r = 0.0;
    tag_marker.color.g = 0.0;
    tag_marker.color.b = 1.0;
  }
  return tag_marker;
}

visualization_msgs::Marker VisTextTag(const geometry_msgs::PoseStamped tag_pose,const int id, const int target_id)
{
  // 可视化标签
  visualization_msgs::Marker tag_text;
  tag_text.header.frame_id = "world";
  tag_text.header.stamp = ros::Time::now();
  tag_text.ns = "tagText" + std::to_string(id);
  tag_text.id = target_id;
  tag_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tag_text.action = visualization_msgs::Marker::ADD;
  tag_text.pose.position.x = tag_pose.pose.position.x;
  tag_text.pose.position.y = tag_pose.pose.position.y;
  tag_text.pose.position.z = tag_pose.pose.position.z + 1.0;
  tag_text.pose.orientation.w = 1.0;
  tag_text.scale.z = 0.2;
  tag_text.color.a = 1.0;
  if (id == 0)
  {
    tag_text.text = "VICTIM " + std::to_string(target_id);
    // tag_text.text = "Tag36h11_0";
    tag_text.color.r = 0.0;
    tag_text.color.g = 1.0;
    tag_text.color.b = 0.0;
  }
  else if (id == 1)
  {
    tag_text.text = "Danger Zone " + std::to_string(target_id);
    // tag_text.text = "Tag36h11_1";
    tag_text.color.r = 1.0;
    tag_text.color.g = 0.0;
    tag_text.color.b = 0.0;
  }
  else if (id == 2)
  {
    tag_text.text = "Tag36h11_2";
    tag_text.color.r = 0.0;
    tag_text.color.g = 0.0;
    tag_text.color.b = 1.0;
  }
  
  return tag_text;
}

visualization_msgs::Marker VisPosTag(const geometry_msgs::PoseStamped tag_pose,const int id, const int target_id)
{
  // 可视化位置
  visualization_msgs::Marker tag_position;
  tag_position.header.frame_id = "world";
  tag_position.header.stamp = ros::Time::now();
  tag_position.ns = "tagpos" + std::to_string(id);
  tag_position.id = target_id;
  tag_position.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  tag_position.action = visualization_msgs::Marker::ADD;
  tag_position.pose.position.x = tag_pose.pose.position.x;
  tag_position.pose.position.y = tag_pose.pose.position.y;
  tag_position.pose.position.z = tag_pose.pose.position.z + 0.8;
  tag_position.pose.orientation.w = 1.0;
  tag_position.text = "(" + std::to_string(tag_pose.pose.position.x) + "," + std::to_string(tag_pose.pose.position.y) + "," + std::to_string(tag_pose.pose.position.z) + ")";
  tag_position.scale.z = 0.2;
  tag_position.color.a = 1.0;

  if (id == 0)
  {
    tag_position.color.r = 0.0;
    tag_position.color.g = 1.0;
    tag_position.color.b = 0.0;
  }
  else if (id == 1)
  {
    tag_position.color.r = 1.0;
    tag_position.color.g = 0.0;
    tag_position.color.b = 0.0;
  }
  else if (id == 2)
  {
    tag_position.color.r = 0.0;
    tag_position.color.g = 0.0;
    tag_position.color.b = 1.0;
  }

  return tag_position;
}

visualization_msgs::Marker VisCircleTag(const geometry_msgs::PoseStamped tag_pose,const int id, const int target_id)
{
  // 可视化圆圈
  visualization_msgs::Marker tag_circle;
  tag_circle.header.frame_id = "world";
  tag_circle.header.stamp = ros::Time::now();
  tag_circle.ns = "tagcircle" + std::to_string(id);
  tag_circle.id = target_id;
  tag_circle.type = visualization_msgs::Marker::LINE_STRIP;
  tag_circle.action = visualization_msgs::Marker::ADD;
  tag_circle.pose.position = tag_pose.pose.position;
  tag_circle.pose.orientation.w = 1.0;
  tag_circle.points = generateCirclePoints(1.0, 100);
  tag_circle.scale.x = 0.04;//线宽
  tag_circle.color.a = 1.0;
  
  if (id == 0)
  {
    tag_circle.color.r = 0.0;
    tag_circle.color.g = 1.0;
    tag_circle.color.b = 0.0;
  }
  else if (id == 1)
  {
    tag_circle.color.r = 1.0;
    tag_circle.color.g = 0.0;
    tag_circle.color.b = 0.0;
  }
  else if (id == 2)
  {
    tag_circle.color.r = 0.0;
    tag_circle.color.g = 0.0;
    tag_circle.color.b = 1.0;
  }
  return tag_circle;
}

// Visualize the apriltag detection result
void VisualizeTag()
{
  // 发布无人机位置
  uav_pose.header.stamp = ros::Time::now();
  uav_pose_pub.publish(uav_pose);
  // 发布facker无人机
  LoadVisModels();
  uav_model_pub.publish(vis_uav_model);

  visualization_msgs::MarkerArray tag_markers;

  visualization_msgs::Marker clear_marker;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  tag_markers.markers.push_back(clear_marker);

  // check if the detected tags is empty
  if (detected_VICTIM_tags_.empty() && detected_DANGER_tags_.empty())
  {
    return;
  }

  // publish the VICTIM detected tags
  for ( auto& target : detected_VICTIM_tags_)
  {
    if (target.record_num <= 1)
    {
      continue;
    }
    
    int apriltag_id = target.tag_id;
    // publish the position
    geometry_msgs::PoseStamped VICTIM_pose;
    VICTIM_pose.header.stamp = ros::Time::now();
    VICTIM_pose.header.frame_id = "world";
    VICTIM_pose.pose.position.x = target.tag_position(0);
    VICTIM_pose.pose.position.y = target.tag_position(1);
    VICTIM_pose.pose.position.z = target.tag_position(2);
    VICTIM_pose_list[apriltag_id] = VICTIM_pose;
    VICTIM_pose_pub[apriltag_id].publish(VICTIM_pose);

    // visualization the cube
    visualization_msgs::Marker VICTIM_marker;
    VICTIM_marker = VisCUBETag(VICTIM_pose,0,apriltag_id);
    tag_markers.markers.push_back(VICTIM_marker);

    // visualization the tag
    visualization_msgs::Marker VICTIM_text;
    VICTIM_text = VisTextTag(VICTIM_pose,0,apriltag_id);
    tag_markers.markers.push_back(VICTIM_text);

    // visualization the position
    visualization_msgs::Marker VICTIM_position;
    VICTIM_position = VisPosTag(VICTIM_pose,0,apriltag_id);
    tag_markers.markers.push_back(VICTIM_position);

    // visualization the circle
    visualization_msgs::Marker VICTIM_circle;
    VICTIM_circle = VisCircleTag(VICTIM_pose,0,apriltag_id);
    tag_markers.markers.push_back(VICTIM_circle);
  }

  // publish the DANGER detected tags
  for ( auto& target : detected_DANGER_tags_)
  {
    if (target.record_num <= 1)
    {
      continue;
    }

    int apriltag_id = target.tag_id;
    // publish the position
    geometry_msgs::PoseStamped DANGER_pose;
    DANGER_pose.header.stamp = ros::Time::now();
    DANGER_pose.header.frame_id = "world";
    DANGER_pose.pose.position.x = target.tag_position(0);
    DANGER_pose.pose.position.y = target.tag_position(1);
    DANGER_pose.pose.position.z = target.tag_position(2);
    DANGER_pose_list[apriltag_id - 8] = DANGER_pose;
    DANGER_pose_pub[apriltag_id - 8].publish(DANGER_pose);

    // vis the cube
    visualization_msgs::Marker DANGER_marker;
    DANGER_marker = VisCUBETag(DANGER_pose,1,apriltag_id);
    tag_markers.markers.push_back(DANGER_marker);

    // vis the tag
    visualization_msgs::Marker DANGER_text;
    DANGER_text = VisTextTag(DANGER_pose,1,apriltag_id);
    tag_markers.markers.push_back(DANGER_text);

    // vis the position
    visualization_msgs::Marker DANGER_position;
    DANGER_position = VisPosTag(DANGER_pose,1,apriltag_id);
    tag_markers.markers.push_back(DANGER_position);

    // vis the circle
    visualization_msgs::Marker DANGER_circle;
    DANGER_circle = VisCircleTag(DANGER_pose,1,apriltag_id);
    tag_markers.markers.push_back(DANGER_circle);
  }

  vis_tag_pub.publish(tag_markers);

}

// Apriltag px4 land
void PX4_TAG_LAND()
{
  if (finished_wp_flag && !init_land_flag)
  {
    // 获取其他无人机的检测结果
    GetOtherDroneDetectResult();

    // Check detceted VICTIM tags is empty
    if (detected_VICTIM_tags_.empty() && other_detected_VICTIM_tag_.empty())
    {
      // ROS_INFO_STREAM(prename << ": No VICTIM tag detected !");
      // 直接降落，注意别在Danger区域降落就行
      return;
    }
    else
    {
      if (!detected_VICTIM_tags_.empty() && land_way_mode == 0)
      {
        //0. Check if have the same VICTIM tag in the already land VICTIM tags
        bool same_land_flag = CheckSameLand(0);
        if (same_land_flag && detected_VICTIM_tags_.size() == 1)
        {
          // ROS_ERROR_STREAM(prename << ": Already land the same VICTIM tag , Waitting for other land place !");
          // 清空本机检测到的VICTIM标签
          detected_VICTIM_tags_.clear();
          land_way_mode = 1;
          return;
        }
      
        //1. find the closest VICTIM tag
        int closest_tag = findClosestVICTIM(odom_p , 0, same_land_flag);
        if (closest_tag == -1)
        {
          // ROS_ERROR_STREAM(prename << ": Already land the same VICTIM tag , Waitting for other land place !");
          // 清空本机检测到的VICTIM标签
          detected_VICTIM_tags_.clear();
          land_way_mode = 1;
          return;
        }
        
        Eigen::Vector3d closest_tag_position = detected_VICTIM_tags_[closest_tag].tag_position;
        land_p_target_id = closest_tag;
        init_land_p = closest_tag_position;

        //2. select the land position around the VICTIM tag within the range of 1m & not in the danger zone with the range of 1m
        geometry_msgs::PoseStamped uav_land_pose;
        uav_land_pose.header.stamp = ros::Time::now();
        uav_land_pose.header.frame_id = "world";
        uav_land_pose.pose.position.x = closest_tag_position(0);
        uav_land_pose.pose.position.y = closest_tag_position(1);
        uav_land_pose.pose.position.z = 1.0;
        uav_land_pose_pub.publish(uav_land_pose);
        PubDroneDetectResult();
        init_land_flag = true;
        land_way_mode = 0;
        VICTIM_tags_num = detected_VICTIM_tags_.size();
      }
      else
      { 
        //0.0 Delete the same VICTIM tag in the other_detected_VICTIM_tag_
        DeleteSameOtherDroneDetectResult();

        //0.1 Check if have the same VICTIM tag in the already land VICTIM tags
        bool same_land_flag = CheckSameLand(1);
        if (same_land_flag && other_detected_VICTIM_tag_.size() == 1)
        {
          // ROS_ERROR_STREAM(prename << ": Already land the same VICTIM tag , Waitting for other land place !");
          // 清空其他飞机检测到的VICTIM标签
          other_detected_VICTIM_tag_.clear();
          return;
        }
        //1. find the closest VICTIM tag
        int closest_tag = findClosestVICTIM(odom_p , 1, same_land_flag);
        if (closest_tag == -1)
        {
          // ROS_ERROR_STREAM(prename << ": Already land the same VICTIM tag , Waitting for other land place !");
          // 清空其他飞机检测到的VICTIM标签
          other_detected_VICTIM_tag_.clear();
          return;
        }

        Eigen::Vector3d diff = other_detected_VICTIM_tag_[closest_tag] - odom_p;
        double dx = diff.x();
        double dy = diff.y();
        double horizontal_dist = sqrt(dx*dx + dy*dy);

        // wait for drone_id * 10 times
        if(wait_time < horizontal_dist)
        {
          wait_time ++;
          return;
        }

        // if(wait_time < 10*(drone_id + 1))
        // {
        //   wait_time ++;
        //   return;
        // }

        // ROS_ERROR_STREAM(prename << ": odom_p : " << odom_p.transpose());
        Eigen::Vector3d closest_tag_position = other_detected_VICTIM_tag_[closest_tag];
        land_p_target_id = closest_tag;
        init_land_p = closest_tag_position;

        //2. select the land position around the VICTIM tag within the range of 1m & not in the danger zone with the range of 1m
        geometry_msgs::PoseStamped uav_land_pose;
        uav_land_pose.header.stamp = ros::Time::now();
        uav_land_pose.header.frame_id = "world";
        uav_land_pose.pose.position.x = closest_tag_position(0);
        uav_land_pose.pose.position.y = closest_tag_position(1);
        uav_land_pose.pose.position.z = 1.0;
        uav_land_pose_pub.publish(uav_land_pose);
        PubDroneDetectResult();
        init_land_flag = true;
        land_way_mode = 1;
        VICTIM_tags_num = detected_VICTIM_tags_.size();
      }
    }
    
  }
  else
  {
    if (init_land_flag && !final_land_flag)
    {
      // check if near the land position
      double distance = (odom_p - init_land_p).head(2).norm();
      if (distance < 0.3)
      {
        //1. find the closest VICTIM tag
        if (land_way_mode == 0)
        {
          final_land_p = detected_VICTIM_tags_[land_p_target_id].tag_position;
        }
        else if (land_way_mode == 1)
        {
          final_land_p = other_detected_VICTIM_tag_[land_p_target_id];
          // CheckSameLandSecond();
          // PubDroneDetectResultSecond();
        }
        final_land_flag = true;
        ROS_WARN_STREAM(prename << ": From init land to Final land !");
        geometry_msgs::PoseStamped uav_land_pose;
        uav_land_pose.header.stamp = ros::Time::now();
        uav_land_pose.header.frame_id = "world";
        uav_land_pose.pose.position.x = final_land_p(0);
        uav_land_pose.pose.position.y = final_land_p(1);
        uav_land_pose.pose.position.z = 1.0;
        uav_land_pose_pub.publish(uav_land_pose);

      }
      else
      {
        // 如果本机识别数量发生了变化，再次发布检测结果
        if (VICTIM_tags_num != detected_VICTIM_tags_.size() && land_way_mode == 1)
        {
          CheckSameLandSecond();
          PubDroneDetectResultSecond();
          VICTIM_tags_num = detected_VICTIM_tags_.size();
        }
        else if (VICTIM_tags_num != detected_VICTIM_tags_.size() && land_way_mode == 0)
        {
          PubDroneDetectResult();
          VICTIM_tags_num = detected_VICTIM_tags_.size();
        }
        
        // CheckSameLandSecond();
        // PubDroneDetectResultSecond();
        // geometry_msgs::PoseStamped uav_land_pose;
        // uav_land_pose.header.stamp = ros::Time::now();
        // uav_land_pose.header.frame_id = "world";
        // uav_land_pose.pose.position.x = init_land_p(0);
        // uav_land_pose.pose.position.y = init_land_p(1);
        // uav_land_pose.pose.position.z = 1.0;
        // uav_land_pose_pub.publish(uav_land_pose);
        // ROS_INFO_STREAM(prename << ": Waitting for near the init land position !");
      }
    }
    else if (init_land_flag && final_land_flag)
    {
      // check if near the land position
      double distance = (odom_p - final_land_p).head(2).norm();
      if (distance < 0.3)
      {
        // check if in the DANGER zone, if have detected DANGER tags
        if (!detected_DANGER_tags_.empty())
        {
          int closest_danger = findClosestDANGER(odom_p);
          Eigen::Vector3d closest_danger_position = detected_DANGER_tags_[closest_danger].tag_position;
          double danger_distance = (odom_p - closest_danger_position).head(2).norm();
          if (danger_distance > 1.0)
          {
            // 切换到Land模式
            if (uav_state.mode != "AUTO.LAND"){
                // ROS_INFO_STREAM(prename << ": NO in the Danger Zone , Start to land !");
                SetPX4Mode("AUTO.LAND");
            }
          }
          else
          {
            // select a new final_land_p avoid the danger zone
            //1. find the closest VICTIM tag
            Eigen::Vector3d closest_victim_position;
            if (land_way_mode == 0)
            {
              closest_victim_position = detected_VICTIM_tags_[land_p_target_id].tag_position;
            }
            else if (land_way_mode == 1)
            {
              closest_victim_position = other_detected_VICTIM_tag_[land_p_target_id];
            }
            final_land_p = SelectNewLandPosition(closest_victim_position, closest_danger_position);
            geometry_msgs::PoseStamped uav_land_pose;
            uav_land_pose.header.stamp = ros::Time::now();
            uav_land_pose.header.frame_id = "world";
            uav_land_pose.pose.position.x = final_land_p(0);
            uav_land_pose.pose.position.y = final_land_p(1);
            uav_land_pose.pose.position.z = 1.0;
            uav_land_pose_pub.publish(uav_land_pose);
          }
        }
        else
        {
          // 切换到Land模式
          if (uav_state.mode != "AUTO.LAND"){
              // ROS_INFO_STREAM(prename << ": Without Danger Zone , Start to land !");
              SetPX4Mode("AUTO.LAND");
          }
        }
      }
      else
      {
        // geometry_msgs::PoseStamped uav_land_pose;
        // uav_land_pose.header.stamp = ros::Time::now();
        // uav_land_pose.header.frame_id = "world";
        // uav_land_pose.pose.position.x = final_land_p(0);
        // uav_land_pose.pose.position.y = final_land_p(1);
        // uav_land_pose.pose.position.z = 1.0;
        // uav_land_pose_pub.publish(uav_land_pose);
        // ROS_INFO_STREAM(prename << ": Waitting for near the final land position !");
      } 
    }
    else
    {
      // ROS_INFO_STREAM(prename << ": No waypoint finished !");
      return;
    }
  }

  
  //  if no, continue to the next step
  //    1. find the closest VICTIM tag
  //    2. select the land position around the VICTIM tag within the range of 1m & not in the danger zone with the range of 1m
  //    3. fly to the land position & recheck the land position is in the VICTIM tag range
  //     if yes, land
  //     if no, select a new land position
  //    4. percise landing
  //  if yes, waitting for the other drone to detect the VICTIM tags more than 1.
  //    follow the steps above
}

// the callback function of the apriltag detection result
void Tag_Odom_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& tag_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
  detect_msgs = *tag_msg;
  if (odom_msg->pose.pose.position.x == 0 && odom_msg->pose.pose.position.y == 0 && odom_msg->pose.pose.position.z == 0)
  {
    ROS_WARN_STREAM(node_name << ": No Odom detected !");
    return;
  }

  uav_pose.pose = odom_msg->pose.pose;
  uav_pose.header.frame_id = "world";

  odom_p << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
  // ROS_INFO_STREAM(prename << ": odom_p : " << odom_p.transpose());
  odom_v << odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z; 
  odom_q = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
                                odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  Eigen::Matrix3d rot = odom_q.toRotationMatrix();
  // convert body pose to camera pose
  Eigen::Matrix4d local2body; local2body.setZero();
  local2body.block<3, 3>(0, 0) = rot;
  local2body(0, 3) = odom_msg->pose.pose.position.x; 
  local2body(1, 3) = odom_msg->pose.pose.position.y;
  local2body(2, 3) = odom_msg->pose.pose.position.z;
  local2body(3, 3) = 1.0;

  local2Cam = local2body * body2Cam;

  local2Cam_rotate = local2Cam.block<3, 3>(0, 0);

  if (detect_msgs.detections.size() == 0)
  {
    return;
  }

  // transform the tag position to the world frame
  transform_tag_position();
}

// the callback function of the ego-planner waypoint finished
void WpFinishedCallback(const std_msgs::EmptyConstPtr& wp_finished_msg)
{
  finished_wp_flag = true;
  ROS_WARN_STREAM(prename << ": Waypoint finished !");
}

// the callback function of the other drone detection result
void OtherDroneDetectCallback(const apriltag_ros::DroneTagDetectionArrayConstPtr& other_drone_detect_msg, int32_t source_drone_id)
{
  int32_t received_drone_id = other_drone_detect_msg->drone_id;
  ros::Time received_time = other_drone_detect_msg->header.stamp;

  // 判断drone_id是否与接收到的消息的drone_id相同
  if (received_drone_id != source_drone_id)
  {
    ROS_ERROR_STREAM(prename << ": Received message from drone[" << source_drone_id << "] with mismatched drone_id " << received_drone_id);
    return;
  }
  
  // 如果该无人机的 ID 已经接收过消息，检查时间戳
  if (known_drone_ids.find(source_drone_id) != known_drone_ids.end()) {
    // 如果新消息的时间戳更晚，更新存储
    if (received_time > known_drone_timestamps[source_drone_id]) {
      ROS_INFO_STREAM(prename << ": Updated detection from drone[" << source_drone_id << "]" << " with " << other_drone_detect_msg->detections.size() << " detections");

      // 删除旧的消息（替换为新消息）
      other_drone_detect_msgs.erase(source_drone_id);

      // 更新时间戳并更新消息
      known_drone_timestamps[source_drone_id] = received_time;
      other_drone_detect_msgs[source_drone_id] = *other_drone_detect_msg;
    } 
    else {
      ROS_WARN_STREAM(prename << ": Discarded outdated message from drone[" << source_drone_id << "]");
    }
  } 
  else {
    // 新的无人机 ID，存储其消息并记录时间戳
    known_drone_ids.insert(source_drone_id);
    known_drone_timestamps[source_drone_id] = received_time;
    other_drone_detect_msgs[source_drone_id] = *other_drone_detect_msg;
    ROS_INFO_STREAM(prename << ": Received detection result from drone[" << source_drone_id << "]" << " with " << other_drone_detect_msg->detections.size() << " detections");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_result_pub");
  ros::NodeHandle nh("~");
  node_name = ros::this_node::getName();
  nh.param("uav_name",uav_name,std::string("iris"));
  nh.param("use_prename", use_prename, false);
  nh.param("VICTIM_num", VICTIM_num, 8);
  nh.param("DANGER_num", DANGER_num, 4);
  nh.param("offset_x", offset_x, 0.0);
  nh.param("offset_y", offset_y, 0.0);
  nh.param("offset_z", offset_z, 0.0);
  nh.param("drone_id", drone_id, 0);
  nh.param("drone_num", drone_num, 1);

  body2Cam << 0,  -1,   0,  offset_x,
             -1,   0,   0,  offset_y,
              0,   0,  -1,  offset_z,
              0,   0,   0,         1;

  std::string tag_topic_name;
  std::string topic_robot_odom;
  std::string topic_uav_state;
  std::string topic_uav_mode;
  std::string topic_wp_finished;
  std::string topic_drone_detect;

  odom_p.setZero();
  land_way_mode = 0;

  // init the VICTIM and DANGER tags flag
  VICTIM_init_flag.resize(VICTIM_num);
  for (int i = 0; i < VICTIM_num; i++)
  {
    VICTIM_init_flag[i] = false;
  }
  DANGER_init_flag.resize(DANGER_num);
  for (int i = 0; i < DANGER_num; i++)
  {
    DANGER_init_flag[i] = false;
  }

  detected_VICTIM_tags_.reserve(VICTIM_num);
  detected_DANGER_tags_.reserve(DANGER_num);


  topic_drone_detect = "/drone_detect_" + std::to_string(drone_id);

  prename = uav_name + "_" + std::to_string(drone_id);

  if (use_prename)
  {
    tag_topic_name = "/" + prename + "/tag_detections";
    topic_robot_odom = "/" + prename + "/mavros/local_position/odom";
    topic_uav_state = "/" + prename + "/mavros/state";
    topic_uav_mode = "/" + prename + "/mavros/set_mode";
    topic_wp_finished = "/drone_" + std::to_string(drone_id) + "/waypoint_finished";
  }
  else
  {
    tag_topic_name = "/tag_detections";
    topic_robot_odom = "/iris_0/mavros/local_position/odom";
    topic_uav_state = "/iris_0/mavros/state";
    topic_uav_mode = "/iris_0/mavros/set_mode";
    topic_wp_finished = "/drone_" + std::to_string(drone_id) + "/waypoint_finished";
  }
  ROS_WARN_STREAM(node_name << ": Tag topic_name: " << tag_topic_name);
  ROS_WARN_STREAM(node_name << ": Odom topic_name: " << topic_robot_odom);
  ROS_WARN_STREAM(node_name << ": VICTIM_num: " << VICTIM_num);
  ROS_WARN_STREAM(node_name << ": DANGER_num: " << DANGER_num);
  
  for (int id = 0; id < drone_num; ++id) {
    if (id == drone_id) {
      continue;
    }
    std::string topic_drone_detect = "/drone_detect_" + std::to_string(id);
    // 使用boost::bind来传递drone_id到回调函数
    drone_subscribers[id] = nh.subscribe<apriltag_ros::DroneTagDetectionArray>(topic_drone_detect, 1, boost::bind(&OtherDroneDetectCallback, _1, id));
    ROS_WARN_STREAM(prename << ": Subscribed to drone " << id << " on topic " << topic_drone_detect);
  }


  // other_drone_detect_sub = nh.subscribe<apriltag_ros::DroneTagDetectionArray>("/drone_detect", 1, OtherDroneDetectCallback);
  uav_state_sub = nh.subscribe<mavros_msgs::State>(topic_uav_state, 10, UavStateCallback);
  wp_finished_sub = nh.subscribe<std_msgs::Empty>(topic_wp_finished, 10, WpFinishedCallback);
  px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(topic_uav_mode);
  // ros::Subscriber detect_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(tag_topic_name, 1, detect_callback);
  // ros::Subscriber uav_odom_ = nh.subscribe<nav_msgs::Odometry>(topic_robot_odom, 10, Odomcallback);

  // 时间软同步 time synchronization
  typedef message_filters::sync_policies::ApproximateTime<apriltag_ros::AprilTagDetectionArray, nav_msgs::Odometry> SyncPolicyTagOdom;
  typedef message_filters::Synchronizer<SyncPolicyTagOdom> SyncTagOdom;

  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> tag_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

  tag_sub.subscribe(nh, tag_topic_name, 10);
  odom_sub.subscribe(nh, topic_robot_odom, 10);
  SyncTagOdom sync_tag_odom(SyncPolicyTagOdom(10), tag_sub, odom_sub);
  sync_tag_odom.connectInput(tag_sub, odom_sub);
  sync_tag_odom.registerCallback(boost::bind(&Tag_Odom_callback, _1, _2));

  std::string uav_land_pose_topic = "/Apriltag/land_goal_" + std::to_string(drone_id);

  // drone detect result publisher
  drone_detect_pub = nh.advertise<apriltag_ros::DroneTagDetectionArray>(topic_drone_detect, 1);
  vis_tag_pub = nh.advertise<visualization_msgs::MarkerArray>("tag_result/vis", 1);
  uav_land_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_land_pose_topic, 1);
  uav_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("uav_pose", 1);
  uav_model_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_model", 1);
  fov_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_down_fov",5);


  std_msgs::ColorRGBA color;
  for(int i = 0; i < colorR.size(); i++){
    color.a = 1.0;
    color.r = colorR[i] / 255.0; // 确保颜色值在0到1的范围内
    color.g = colorG[i] / 255.0;
    color.b = colorB[i] / 255.0;
    COLOR_LISTS.push_back(color);
  }
  // 打印颜色列表的所有颜色

  CreateVisModels();

  // init the VICITM_pose_pub and DANGER_pose_pub
  VICTIM_pose_pub.resize(VICTIM_num);
  DANGER_pose_pub.resize(DANGER_num);
  for (int i = 0; i < VICTIM_num; i++)
  {
    VICTIM_pose_pub[i] = nh.advertise<geometry_msgs::PoseStamped>("VICTIM" + std::to_string(i) + "_pose", 1);
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    VICTIM_pose_list.push_back(target_pose);
  }
  for (int i = 0; i < DANGER_num; i++)
  {
    DANGER_pose_pub[i] = nh.advertise<geometry_msgs::PoseStamped>("DANGER" + std::to_string(i + 8) + "_pose", 1);
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    DANGER_pose_list.push_back(target_pose);
  }

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // ROS_WARN_STREAM(prename << "VICTIM_num: " << detected_VICTIM_tags_.size() << " DANGER_num: " << detected_DANGER_tags_.size());
    VisualizeTag();
    PX4_TAG_LAND();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}