#include<ros/ros.h>
#include<tf2_msgs/TFMessage.h>
#include<mavros_msgs/PositionTarget.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Quaternion.h>
#include<sensor_msgs/CameraInfo.h>
#include<cmath>
#include<iostream>
#include<sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

mavros_msgs::PositionTarget pos_setpoint;
ros::Publisher target_pub;
ros::Publisher landmark_pub;
#define CX_RAD2DEG 57.2957795
double fx, fy, cx, cy;
int width, height;
double center_x, center_y;
double target_yaw;
double target_pitch;
std::string node_name;
std::string group_name;

apriltag_ros::AprilTagDetection detect0_msg;
apriltag_ros::AprilTagDetection detect1_msg;
apriltag_ros::AprilTagDetection detect2_msg;
apriltag_ros::AprilTagDetectionArray detect_msg;
geometry_msgs::PoseStamped tag0_pose;
geometry_msgs::PoseStamped tag1_pose;
geometry_msgs::PoseStamped tag2_pose;


void calculate_center(const double x, const double y, const double z)
{
  center_x = (x * fx / z + cx);
  center_y = (y * fy / z + cy);
  target_yaw = atan((center_x - width / 2.) / fx )* CX_RAD2DEG;
  target_pitch = atan((center_y - height / 2.) / fy) * CX_RAD2DEG;
  // std::cout << "center_x: " << center_x << " center_y: " << center_y << " target_yaw" << target_yaw <<" target_pitch"<< target_pitch <<std::endl;
}

void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  // Get the camera intrinsic parameters.
  fx = msg->P[0];
  fy = msg->P[5];
  cx = msg->P[2];
  cy = msg->P[6];
  width = msg->width;
  height = msg->height;
  // std::cout << "width: " << width << " height: " << height <<" fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy << std::endl;
}

void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  // Get the position of the camera in the world frame.[RDF]
  geometry_msgs::TransformStamped tf = msg->transforms[0];
  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;
  double z = tf.transform.translation.z;
  calculate_center(x,y,z);
  geometry_msgs::Quaternion q = tf.transform.rotation;
  double qx = q.x;
  double qy = q.y;
  double qz = q.z;
  double qw = q.w;

  //两个解算roll pitch yaw的方法
  //1. 通过四元数解算
  double roll, pitch, yaw;
  // quaternionToEulerFLU(q, roll, pitch, yaw);
  // std::cout << "roll: " << roll * CX_RAD2DEG << " pitch: " << pitch * CX_RAD2DEG << " yaw: " << yaw * CX_RAD2DEG << std::endl;
  
  pos_setpoint.header.stamp = ros::Time::now();
  pos_setpoint.header.frame_id = "world";
  pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  pos_setpoint.position.x = z;
  pos_setpoint.position.y = -x;
  pos_setpoint.position.z = -y;
  pos_setpoint.yaw = target_yaw;
  pos_setpoint.yaw_rate = 0.0;
  target_pub.publish(pos_setpoint);
  // std::cout << "x: " << z << " y: " << -x << " z: " << -y << " yaw: " << target_yaw << " pitch: " << target_pitch << std::endl;

}

void read_detect_result()
{
  if(detect_msg.detections.size() == 2)
  {
    detect0_msg = detect_msg.detections[0];
    // 检查id是否为0
    if(detect0_msg.id[0] == 0)
    {
      tag0_pose.header.frame_id = "landmark0";
      tag0_pose.header.seq = detect0_msg.pose.header.seq;
      tag0_pose.header.stamp = detect0_msg.pose.header.stamp;
      tag0_pose.pose.position.x = - detect0_msg.pose.pose.pose.position.y;
      tag0_pose.pose.position.y = - detect0_msg.pose.pose.pose.position.x;
      tag0_pose.pose.position.z = - detect0_msg.pose.pose.pose.position.z;
      landmark_pub.publish(tag0_pose);
      std::cout << node_name << ": Tag 0 detected :(FLU) "<<tag0_pose.pose.position.x << " " << tag0_pose.pose.position.y << " " << tag0_pose.pose.position.z << std::endl;
    }
    else if(detect0_msg.id[0] == 1)
    {
      tag1_pose.header.frame_id = "landmark1";
      tag1_pose.header.seq = detect0_msg.pose.header.seq;
      tag1_pose.header.stamp = detect0_msg.pose.header.stamp;
      tag1_pose.pose.position.x = - detect0_msg.pose.pose.pose.position.y;
      tag1_pose.pose.position.y = - detect0_msg.pose.pose.pose.position.x;
      tag1_pose.pose.position.z = - detect0_msg.pose.pose.pose.position.z;
      landmark_pub.publish(tag1_pose);
      std::cout << node_name << ": Tag 1 detected :(FLU) "<<tag1_pose.pose.position.x << " " << tag1_pose.pose.position.y << " " << tag1_pose.pose.position.z << std::endl;
    }
    else if(detect0_msg.id[0] == 2)
    {
      tag2_pose.header.frame_id = "landmark2";
      tag2_pose.header.seq = detect0_msg.pose.header.seq;
      tag2_pose.header.stamp = detect0_msg.pose.header.stamp;
      tag2_pose.pose.position.x = - detect0_msg.pose.pose.pose.position.y;
      tag2_pose.pose.position.y = - detect0_msg.pose.pose.pose.position.x;
      tag2_pose.pose.position.z = - detect0_msg.pose.pose.pose.position.z;
      landmark_pub.publish(tag2_pose);
      std::cout << node_name << ": Tag 2 detected :(FLU) "<<tag2_pose.pose.position.x << " " << tag2_pose.pose.position.y << " " << tag2_pose.pose.position.z << std::endl;
    }
  }
}

void detect_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
  detect_msg = *msg;
  // ROS_WARN_STREAM(node_name << ": Tag detected NUMBER: " << detect_msg.detections.size());
  read_detect_result();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tagdetect");
  ros::NodeHandle nh("~");
  node_name = ros::this_node::getName();
  nh.param("group_name",group_name,std::string("group"));

  ROS_WARN_STREAM( ": group_name: " << group_name);
  std::string topic_name = "/" + group_name + "/tag_detections";
  ROS_WARN_STREAM( ": topic_name: " << topic_name);
  std::string landmark_name = "/" + group_name + "/landmark_body_pose";
  // target_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
  // ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, tf_callback);
  // ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera_info", 1, camera_info_callback);
  ros::Subscriber detect_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(topic_name, 1, detect_callback);
  landmark_pub = nh.advertise<geometry_msgs::PoseStamped>(landmark_name, 1);


  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}