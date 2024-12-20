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
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/ColorRGBA.h>

ros::Publisher vis_tag_pub;
ros::Publisher uav_pose_pub;
ros::Publisher uav_model_pub;
geometry_msgs::PoseStamped uav_pose;
visualization_msgs::MarkerArray vis_uav_model;
ros::Publisher fov_pub;
std::vector<ros::Publisher> VICTIM_pose_pub;
std::vector<ros::Publisher> DANGER_pose_pub;
std::vector<geometry_msgs::PoseStamped> VICTIM_pose_list;
std::vector<geometry_msgs::PoseStamped> DANGER_pose_list;
std::string node_name;
std::string prename;
bool use_prename;
int VICTIM_num;
int DANGER_num;
bool received_odom = false;

Eigen::Matrix4d body2Cam; // from body frame to camera frame
Eigen::Matrix4d local2Cam; // from local frame to camera frame
Eigen::Matrix3d local2Cam_rotate;
double offset_x, offset_y, offset_z;

Eigen::Vector3d odom_p, odom_v;
Eigen::Quaterniond odom_q;

apriltag_ros::AprilTagDetection detect_msg;
apriltag_ros::AprilTagDetectionArray detect_msgs;
geometry_msgs::PoseStamped tag0_pose;
geometry_msgs::PoseStamped tag1_pose;
geometry_msgs::PoseStamped tag2_pose;

struct Tag_Sptial_Temporal
{
  Eigen::Vector3d tag_position;
  Eigen::Vector3d sum_position;
  int record_num;
  int tag_id;
  int target_id;
};

std::vector<Tag_Sptial_Temporal> detected_VICTIM_tags_;
std::vector<Tag_Sptial_Temporal> detected_DANGER_tags_;
double POSITION_THRESHOLD = 1.0;// 位置阈值(米)
int next_VICTIM_id = 1;  // VICTIM ID计数器
int next_DANGER_id = 1;  // DANGER ID计数器
bool init_VICTIM_tags_ = false;
bool init_DANGER_tags_ = false;

std::vector<double> colorR = {150, 0, 0, 255, 255, 0, 243, 143, 112, 255, 241};
std::vector<double> colorG = {150, 229, 0, 0, 44, 212, 110, 255, 0, 165, 84};
std::vector<double> colorB = {150, 238, 255, 255, 44, 112, 50, 0, 255, 0, 76};
std::vector<std_msgs::ColorRGBA> COLOR_LISTS;

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

//创建无人机可视化模型
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

bool isValidPosition(const Eigen::Vector3d& pos) {
    return std::isfinite(pos(0)) && std::isfinite(pos(1)) && std::isfinite(pos(2));
          //  pos.norm() > 0.1 && // 确保位置不在原点附近
          //  pos(2) >= 0;  // 确保高度非负
}

bool isNewTarget(const Eigen::Vector3d& new_position,const int id){
    ros::Time current_time = ros::Time::now();
    
    if (!isValidPosition(new_position)) {
      ROS_WARN("Invalid position detected, not adding new target");
      return false;  // 无效位置不添加新目标
    }

    if (id == 0)
    {
      // 1. 检查是否与已有目标匹配      
      for (auto& target : detected_VICTIM_tags_) {       

          double distance = (new_position - target.tag_position).norm();
          
          // 尝试重新识别历史目标
          if (distance < POSITION_THRESHOLD) {
              target.record_num++;
              target.sum_position += new_position;
              // target.position = new_position;
              target.tag_position = target.sum_position * (1.0 / target.record_num);
              return false;
          }
      }

      // 添加新目标
      Tag_Sptial_Temporal new_target;
      new_target.tag_position = new_position;
      new_target.sum_position = new_position;
      new_target.record_num = 1;
      new_target.tag_id = 0;
      new_target.target_id = next_VICTIM_id;
      detected_VICTIM_tags_.push_back(new_target);
      next_VICTIM_id ++;
      return true;
    }
    else if (id == 1)
    {
      // 1. 检查是否与已有目标匹配      
      for (auto& target : detected_DANGER_tags_) {       

          double distance = (new_position - target.tag_position).norm();
          
          // 尝试重新识别历史目标
          if (distance < POSITION_THRESHOLD) {
              target.record_num++;
              target.sum_position += new_position;
              // target.position = new_position;
              target.tag_position = target.sum_position * (1.0 / target.record_num);
              return false;
          }
      }

      // 添加新目标
      Tag_Sptial_Temporal new_target;
      new_target.tag_position = new_position;
      new_target.sum_position = new_position;
      new_target.record_num = 1;
      new_target.tag_id = 1;
      new_target.target_id = next_DANGER_id;
      detected_DANGER_tags_.push_back(new_target);
      next_DANGER_id ++;
      return true;
    }
    return true;
}

void transform_tag_position()
{

  for (int i = 0; i < detect_msgs.detections.size() - 1; i++)
  {
    detect_msg = detect_msgs.detections[i];
    if (detect_msg.id[0] == 0)
    {
      Eigen::Vector3d RDF_pose;
      RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
      Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

      if (!init_VICTIM_tags_)
      {
        Tag_Sptial_Temporal detected_tag0;
        detected_tag0.tag_position = ENU_pose;
        detected_tag0.sum_position = ENU_pose;
        detected_tag0.record_num = 1;
        detected_tag0.tag_id = 0;
        detected_tag0.target_id = next_VICTIM_id;
        detected_VICTIM_tags_.push_back(detected_tag0);
        next_VICTIM_id ++;
        init_VICTIM_tags_ = true;
      }
      else
      {
        if (isNewTarget(ENU_pose,0))
        {
          ROS_INFO("New VICTIM detected");
        }
      }
    }
    else if (detect_msg.id[0] == 1)
    {
      Eigen::Vector3d RDF_pose;
      RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
      Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

      if (!init_DANGER_tags_)
      {
        Tag_Sptial_Temporal detected_tag1;
        detected_tag1.tag_position = ENU_pose;
        detected_tag1.sum_position = ENU_pose;
        detected_tag1.record_num = 1;
        detected_tag1.tag_id = 1;
        detected_tag1.target_id = next_DANGER_id;
        detected_DANGER_tags_.push_back(detected_tag1);
        next_DANGER_id ++;
        init_DANGER_tags_ = true;
      }
      else
      {
        if (isNewTarget(ENU_pose,1))
        {
          ROS_INFO("New DANGER ZONE detected");
        }
      }
    }
    
  }


  // if (!init_VICTIM_tags_)
  // {
  //   for (int i = 0; i < detect_msgs.detections.size() - 1; i++)
  //   {
  //     detect_msg = detect_msgs.detections[i];
  //     if (detect_msg.id[0] == 0)
  //     {
  //       Eigen::Vector3d RDF_pose;
  //       RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
  //       Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

  //       Tag_Sptial_Temporal detected_tag0;
  //       detected_tag0.tag_position = ENU_pose;
  //       detected_tag0.sum_position = ENU_pose;
  //       detected_tag0.record_num = 1;
  //       detected_tag0.tag_id = 0;
  //       detected_tag0.target_id = next_VICTIM_id;
  //       detected_VICTIM_tags_.push_back(detected_tag0);
  //       next_VICTIM_id ++;

  //     }
  //     else if (detect_msg.id[0] == 1)
  //     {
  //       Eigen::Vector3d RDF_pose;
  //       RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
  //       Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);

  //       Tag_Sptial_Temporal detected_tag1;
  //       detected_tag1.tag_position = ENU_pose;
  //       detected_tag1.sum_position = ENU_pose;
  //       detected_tag1.record_num = 1;
  //       detected_tag1.tag_id = 0;
  //       detected_tag1.target_id = next_DANGER_id;
  //       detected_DANGER_tags_.push_back(detected_tag1);
  //       next_DANGER_id ++;
  //     }
      
  //   }
  //   init_VICTIM_tags_ = true;
  // }
  // else
  // {
  //   for (int i = 0; i < detect_msgs.detections.size() - 1; i++)
  //   {
  //     detect_msg = detect_msgs.detections[i];
  //     if (detect_msg.id[0] == 0)
  //     {
  //       Eigen::Vector3d RDF_pose;
  //       RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
  //       Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);
  //       if (isNewTarget(ENU_pose,0))
  //       {
  //         ROS_INFO("New VICTIM detected");
  //       }
        
  //     }
  //     else if (detect_msg.id[0] == 1)
  //     {
  //       Eigen::Vector3d RDF_pose;
  //       RDF_pose << detect_msg.pose.pose.pose.position.x, detect_msg.pose.pose.pose.position.y, detect_msg.pose.pose.pose.position.z;
  //       Eigen::Vector3d ENU_pose = local2Cam.block<3, 3>(0, 0) * RDF_pose + local2Cam.block<3, 1>(0, 3);
  //       if (isNewTarget(ENU_pose,1))
  //       {
  //         ROS_INFO("New DANGER ZONE detected");
  //       }
  //     }
  //   }
  // }

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
  tag_circle.scale.x = 0.02;//线宽
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

void VisualizeTag()
{
  // 发布无人机位置
  // uav_pose.header.stamp = ros::Time::now();
  // uav_pose_pub.publish(uav_pose);
  // 发布facker无人机
  // LoadVisModels();
  // uav_model_pub.publish(vis_uav_model);

  visualization_msgs::MarkerArray tag_markers;

  visualization_msgs::Marker clear_marker;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  tag_markers.markers.push_back(clear_marker);

  int VICTIM_target_id = 0;
  int DANGER_target_id = 0;

  // 检查是否有目标
  if (detected_VICTIM_tags_.empty() && detected_DANGER_tags_.empty())
  {
    return;
  }

  // 发布VICTIM中的目标
  for ( auto& target : detected_VICTIM_tags_)
  {
    if (target.record_num <= 1)
    {
      continue;
    }

    ROS_INFO_STREAM( "VICTIM Target id: " << VICTIM_target_id + 1 << 
            " Target position: " << target.tag_position.transpose() 
            << " Target record num: " << target.record_num);

    // 发布位置
    geometry_msgs::PoseStamped VICTIM_pose;
    VICTIM_pose.header.stamp = ros::Time::now();
    VICTIM_pose.header.frame_id = "world";
    VICTIM_pose.pose.position.x = target.tag_position(0);
    VICTIM_pose.pose.position.y = target.tag_position(1);
    VICTIM_pose.pose.position.z = target.tag_position(2);
    VICTIM_pose_list[VICTIM_target_id] = VICTIM_pose;
    VICTIM_pose_pub[VICTIM_target_id].publish(VICTIM_pose);

    // 可视化方块
    visualization_msgs::Marker VICTIM_marker;
    VICTIM_marker = VisCUBETag(VICTIM_pose,0,target.target_id);
    tag_markers.markers.push_back(VICTIM_marker);

    // 可视化标签
    visualization_msgs::Marker VICTIM_text;
    VICTIM_text = VisTextTag(VICTIM_pose,0,target.target_id);
    tag_markers.markers.push_back(VICTIM_text);

    // 可视化位置
    visualization_msgs::Marker VICTIM_position;
    VICTIM_position = VisPosTag(VICTIM_pose,0,target.target_id);
    tag_markers.markers.push_back(VICTIM_position);

    // 可视化圆圈
    visualization_msgs::Marker VICTIM_circle;
    VICTIM_circle = VisCircleTag(VICTIM_pose,0,target.target_id);
    tag_markers.markers.push_back(VICTIM_circle);

    VICTIM_target_id ++;
  }

  // 发布DANGER中的目标
  for ( auto& target : detected_DANGER_tags_)
  {
    if (target.record_num <= 1)
    {
      continue;
    }

    ROS_INFO_STREAM( "DANGER Target id: " << DANGER_target_id + 1 << 
            " Target position: " << target.tag_position.transpose() 
            << " Target record num: " << target.record_num);

    // 发布位置
    geometry_msgs::PoseStamped DANGER_pose;
    DANGER_pose.header.stamp = ros::Time::now();
    DANGER_pose.header.frame_id = "world";
    DANGER_pose.pose.position.x = target.tag_position(0);
    DANGER_pose.pose.position.y = target.tag_position(1);
    DANGER_pose.pose.position.z = target.tag_position(2);
    DANGER_pose_list[DANGER_target_id] = DANGER_pose;
    DANGER_pose_pub[DANGER_target_id].publish(DANGER_pose);

    // 可视化方块
    visualization_msgs::Marker DANGER_marker;
    DANGER_marker = VisCUBETag(DANGER_pose,1,target.target_id);
    tag_markers.markers.push_back(DANGER_marker);

    // 可视化标签
    visualization_msgs::Marker DANGER_text;
    DANGER_text = VisTextTag(DANGER_pose,1,target.target_id);
    tag_markers.markers.push_back(DANGER_text);

    // 可视化位置
    visualization_msgs::Marker DANGER_position;
    DANGER_position = VisPosTag(DANGER_pose,1,target.target_id);
    tag_markers.markers.push_back(DANGER_position);

    // 可视化圆圈
    visualization_msgs::Marker DANGER_circle;
    DANGER_circle = VisCircleTag(DANGER_pose,1,target.target_id);
    tag_markers.markers.push_back(DANGER_circle);

    DANGER_target_id ++;
  }

  vis_tag_pub.publish(tag_markers);

}

void Tag_Odom_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& tag_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
  detect_msgs = *tag_msg;
  if (detect_msgs.detections.size() == 0)
  {
    ROS_WARN_STREAM(node_name << ": No Tag detected !");
    return;
  }
  if (odom_msg->pose.pose.position.x == 0 && odom_msg->pose.pose.position.y == 0 && odom_msg->pose.pose.position.z == 0)
  {
    ROS_WARN_STREAM(node_name << ": No Odom detected !");
    return;
  }
  // ROS_WARN_STREAM(node_name << ": Tag detected NUMBER: " << detect_msgs.detections.size() - 1);

  uav_pose.pose = odom_msg->pose.pose;
  uav_pose.header.frame_id = "world";

  odom_p << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
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

  transform_tag_position();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_result_pub");
  ros::NodeHandle nh("~");
  node_name = ros::this_node::getName();
  nh.param("prename",prename,std::string("iris"));
  nh.param("use_prename", use_prename, false);
  nh.param("VICTIM_num", VICTIM_num, 8);
  nh.param("DANGER_num", DANGER_num, 4);
  nh.param("POSITION_THRESHOLD", POSITION_THRESHOLD, 1.0);
  nh.param("offset_x", offset_x, 0.0);
  nh.param("offset_y", offset_y, 0.0);
  nh.param("offset_z", offset_z, 0.0);

  body2Cam << 0,  -1,   0,  offset_x,
             -1,   0,   0,  offset_y,
              0,   0,  -1,  offset_z,
              0,   0,   0,         1;

  std::string tag_topic_name;
  std::string topic_robot_odom;

  if (use_prename)
  {
    tag_topic_name = "/" + prename + "/tag_detections";
    topic_robot_odom = "/" + prename + "/mavros/local_position/odom";
  }
  else
  {
    tag_topic_name = "/tag_detections";
    topic_robot_odom = "/iris_0/mavros/local_position/odom";
  }
  ROS_WARN_STREAM(node_name << ": Tag topic_name: " << tag_topic_name);
  ROS_WARN_STREAM(node_name << ": Odom topic_name: " << topic_robot_odom);
  ROS_WARN_STREAM(node_name << ": VICTIM_num: " << VICTIM_num);
  ROS_WARN_STREAM(node_name << ": DANGER_num: " << DANGER_num);
  ROS_WARN_STREAM(node_name << ": POSITION_THRESHOLD: " << POSITION_THRESHOLD);
  
  // ros::Subscriber detect_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(tag_topic_name, 1, detect_callback);
  // ros::Subscriber uav_odom_ = nh.subscribe<nav_msgs::Odometry>(topic_robot_odom, 10, Odomcallback);

  // 时间软同步
  typedef message_filters::sync_policies::ApproximateTime<apriltag_ros::AprilTagDetectionArray, nav_msgs::Odometry> SyncPolicyTagOdom;
  typedef message_filters::Synchronizer<SyncPolicyTagOdom> SyncTagOdom;

  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> tag_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

  tag_sub.subscribe(nh, tag_topic_name, 10);
  odom_sub.subscribe(nh, topic_robot_odom, 10);
  SyncTagOdom sync_tag_odom(SyncPolicyTagOdom(10), tag_sub, odom_sub);
  sync_tag_odom.connectInput(tag_sub, odom_sub);
  sync_tag_odom.registerCallback(boost::bind(&Tag_Odom_callback, _1, _2));

  vis_tag_pub = nh.advertise<visualization_msgs::MarkerArray>("tag_result/vis", 1);
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


  VICTIM_pose_pub.resize(VICTIM_num);
  DANGER_pose_pub.resize(DANGER_num);
  for (int i = 0; i < VICTIM_num; i++)
  {
    VICTIM_pose_pub[i] = nh.advertise<geometry_msgs::PoseStamped>("VICTIM" + std::to_string(i+1) + "_pose", 1);
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    VICTIM_pose_list.push_back(target_pose);
  }
  for (int i = 0; i < DANGER_num; i++)
  {
    DANGER_pose_pub[i] = nh.advertise<geometry_msgs::PoseStamped>("DANGER" + std::to_string(i+1) + "_pose", 1);
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    DANGER_pose_list.push_back(target_pose);
  }

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    VisualizeTag();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

// no used
void detect_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  detect_msgs = *msg;
  if (detect_msgs.detections.size() == 0)
  {
    ROS_WARN_STREAM(node_name << ": No Tag detected !");
    return;
  }
  
  // ROS_WARN_STREAM(node_name << ": Tag detected NUMBER: " << detect_msgs.detections.size() - 1);
  if (received_odom)
  {
    transform_tag_position();
  }
  else
  {
    ROS_WARN_STREAM(node_name << ": Waitting for Odom !");
  }
}

// no used
void Odomcallback(const nav_msgs::OdometryConstPtr& msg)
{
  received_odom = true;
  odom_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  odom_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
  odom_q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix3d rot = odom_q.toRotationMatrix();
  // convert body pose to camera pose
  Eigen::Matrix4d local2body; local2body.setZero();
  local2body.block<3, 3>(0, 0) = rot;
  local2body(0, 3) = msg->pose.pose.position.x; 
  local2body(1, 3) = msg->pose.pose.position.y;
  local2body(2, 3) = msg->pose.pose.position.z;
  local2body(3, 3) = 1.0;

  local2Cam = local2body * body2Cam;

  local2Cam_rotate = local2Cam.block<3, 3>(0, 0);
}