#include <iostream>
#include <string>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "armadillo"
#include "pose_utils.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "CameraPoseVisualization.h"
#include <std_msgs/ColorRGBA.h>


using namespace arma;
using namespace std;

std::vector<double> colorR = {150, 0, 0, 255, 255, 0, 243, 143, 112, 255, 241};
std::vector<double> colorG = {150, 229, 0, 0, 44, 212, 110, 255, 0, 165, 84};
std::vector<double> colorB = {150, 238, 255, 255, 44, 112, 50, 0, 255, 0, 76};
std::vector<std_msgs::ColorRGBA> COLOR_LISTS;

static string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale, rotate_yaw;
static double camera_r, camera_g, camera_b;
double search_height = 0.0;
bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);
ros::Publisher posePub;
ros::Publisher pathPub;
ros::Publisher velPub;
ros::Publisher covPub;
ros::Publisher covVelPub;
ros::Publisher trajPub;
ros::Publisher sensorPub;
ros::Publisher meshPub;
ros::Publisher uav_model_pub;
ros::Publisher heightPub;
ros::Publisher pub_camera_pose_visual;// 相机位姿可视化
tf::TransformBroadcaster *broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path pathROS;
visualization_msgs::Marker velROS;
visualization_msgs::Marker covROS;
visualization_msgs::Marker covVelROS;
visualization_msgs::Marker trajROS;
visualization_msgs::Marker sensorROS;
visualization_msgs::Marker meshROS;
visualization_msgs::MarkerArray vis_uav_model;
sensor_msgs::Range heightROS;
string _frame_id;
int _drone_id;
CameraPoseVisualization cameraposevisual(135, 74, 32, 1);// 相机位姿可视化，更改颜色

std_msgs::ColorRGBA Id2Color(int idx, double a){
  std_msgs::ColorRGBA color;
  idx = idx % int(COLOR_LISTS.size());
  color = COLOR_LISTS[idx];
  color.a = a;
  return color;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

  if (msg->header.frame_id == string("null"))
    return;

  vis_uav_model.markers[0].header.stamp = ros::Time::now();
  vis_uav_model.markers[1].header.stamp = ros::Time::now();
  vis_uav_model.markers[2].header.stamp = ros::Time::now();
  vis_uav_model.markers[3].header.stamp = ros::Time::now();
  vis_uav_model.markers[4].header.stamp = ros::Time::now();

  vis_uav_model.markers[0].pose.orientation = msg->pose.pose.orientation;
  vis_uav_model.markers[1].pose.orientation = msg->pose.pose.orientation;
  vis_uav_model.markers[2].pose.orientation = msg->pose.pose.orientation;
  vis_uav_model.markers[3].pose.orientation = msg->pose.pose.orientation;
  vis_uav_model.markers[4].pose = msg->pose.pose;

  Eigen::Quaterniond rot;
  rot.x() = msg->pose.pose.orientation.x;
  rot.y() = msg->pose.pose.orientation.y;
  rot.z() = msg->pose.pose.orientation.z;
  rot.w() = msg->pose.pose.orientation.w;
  Eigen::Vector3d pos;
  pos(0) = msg->pose.pose.position.x;
  pos(1) = msg->pose.pose.position.y;
  pos(2) = msg->pose.pose.position.z;

  Eigen::Vector3d p(0.08, 0.08, -0.01);
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
  uav_model_pub.publish(vis_uav_model);

  colvec pose(6);
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;

  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin = pose;
  }
  if (origin)
  {
    vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel = ypr_to_R(pose.rows(3, 5)) * vel;
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);
  posePub.publish(poseROS);

  // Velocity
  colvec yprVel(3);
  yprVel(0) = atan2(vel(1), vel(0));
  yprVel(1) = -atan2(vel(2), norm(vel.rows(0, 1), 2));
  yprVel(2) = 0;
  q = R_to_quaternion(ypr_to_R(yprVel));
  velROS.header.frame_id = string("world");
  velROS.header.stamp = msg->header.stamp;
  velROS.ns = string("velocity");
  velROS.id = _drone_id;
  velROS.type = visualization_msgs::Marker::ARROW;
  velROS.action = visualization_msgs::Marker::ADD;
  velROS.pose.position.x = pose(0);
  velROS.pose.position.y = pose(1);
  velROS.pose.position.z = pose(2);
  velROS.pose.orientation.w = q(0);
  velROS.pose.orientation.x = q(1);
  velROS.pose.orientation.y = q(2);
  velROS.pose.orientation.z = q(3);
  velROS.scale.x = norm(vel, 2);
  velROS.scale.y = 0.05;
  velROS.scale.z = 0.05;
  velROS.color.a = 1.0;
  velROS.color.r = color_r;
  velROS.color.g = color_g;
  velROS.color.b = color_b;
  velPub.publish(velROS);

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);
  }

  // Covariance color
  double r = 1;
  double g = 1;
  double b = 1;
  bool G = msg->twist.covariance[33];
  bool V = msg->twist.covariance[34];
  bool L = msg->twist.covariance[35];
  if (cov_color)
  {
    r = G;
    g = V;
    b = L;
  }

  // Covariance Position
  if (cov_pos)
  {
    mat P(6, 6);
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        P(i, j) = msg->pose.covariance[i + j * 6];
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P.submat(0, 0, 2, 2));
    if (det(eigVec) < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covROS.header.frame_id = string("world");
    covROS.header.stamp = msg->header.stamp;
    covROS.ns = string("covariance");
    covROS.id = _drone_id;
    covROS.type = visualization_msgs::Marker::SPHERE;
    covROS.action = visualization_msgs::Marker::ADD;
    covROS.pose.position.x = pose(0);
    covROS.pose.position.y = pose(1);
    covROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covROS.pose.orientation.w = q(0);
    covROS.pose.orientation.x = q(1);
    covROS.pose.orientation.y = q(2);
    covROS.pose.orientation.z = q(3);
    covROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covROS.color.a = 0.4;
    covROS.color.r = r * 0.5;
    covROS.color.g = g * 0.5;
    covROS.color.b = b * 0.5;
    covPub.publish(covROS);
  }

  // Covariance Velocity
  if (cov_vel)
  {
    mat P(3, 3);
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++)
        P(i, j) = msg->twist.covariance[i + j * 6];
    mat R = ypr_to_R(pose.rows(3, 5));
    P = R * P * trans(R);
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P);
    if (det(eigVec) < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covVelROS.header.frame_id = string("world");
    covVelROS.header.stamp = msg->header.stamp;
    covVelROS.ns = string("covariance_velocity");
    covVelROS.id = _drone_id;
    covVelROS.type = visualization_msgs::Marker::SPHERE;
    covVelROS.action = visualization_msgs::Marker::ADD;
    covVelROS.pose.position.x = pose(0);
    covVelROS.pose.position.y = pose(1);
    covVelROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covVelROS.pose.orientation.w = q(0);
    covVelROS.pose.orientation.x = q(1);
    covVelROS.pose.orientation.y = q(2);
    covVelROS.pose.orientation.z = q(3);
    covVelROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covVelROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covVelROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covVelROS.color.a = 0.4;
    covVelROS.color.r = r;
    covVelROS.color.g = g;
    covVelROS.color.b = b;
    covVelPub.publish(covVelROS);
  }

  // Color Coded Trajectory
  static colvec ppose = pose;
  static ros::Time pt = msg->header.stamp;
  ros::Time t = msg->header.stamp;
  if ((t - pt).toSec() > 0.5)
  {
    trajROS.header.frame_id = string("world");
    trajROS.header.stamp = ros::Time::now();
    trajROS.ns = string("world");
    trajROS.type = visualization_msgs::Marker::LINE_LIST;
    trajROS.action = visualization_msgs::Marker::ADD;
    trajROS.pose.position.x = 0;
    trajROS.pose.position.y = 0;
    trajROS.pose.position.z = 0;
    trajROS.pose.orientation.w = 1;
    trajROS.pose.orientation.x = 0;
    trajROS.pose.orientation.y = 0;
    trajROS.pose.orientation.z = 0;
    trajROS.scale.x = 0.02;
    trajROS.scale.y = 0;
    trajROS.scale.z = 0;
    trajROS.color.r = 4.0;
    trajROS.color.g = 243.0;
    trajROS.color.b = 253.0;
    trajROS.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = ppose(0);
    p.y = ppose(1);
    p.z = ppose(2);
    trajROS.points.push_back(p);
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS.points.push_back(p);
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 253.0;
    color.g = 97.0;
    color.b = 157.0;
    trajROS.colors.push_back(color);
    trajROS.colors.push_back(color);
    ppose = pose;
    pt = t;
    trajPub.publish(trajROS);
  }

  // Sensor availability
  sensorROS.header.frame_id = string("world");
  sensorROS.header.stamp = msg->header.stamp;
  sensorROS.ns = string("sensor");
  sensorROS.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  sensorROS.action = visualization_msgs::Marker::ADD;
  sensorROS.pose.position.x = pose(0);
  sensorROS.pose.position.y = pose(1);
  sensorROS.pose.position.z = pose(2) + 1.0;
  sensorROS.pose.orientation.w = q(0);
  sensorROS.pose.orientation.x = q(1);
  sensorROS.pose.orientation.y = q(2);
  sensorROS.pose.orientation.z = q(3);
  string strG = G ? string(" GPS ") : string("");
  string strV = V ? string(" Vision ") : string("");
  string strL = L ? string(" Laser ") : string("");
  sensorROS.text = "| " + strG + strV + strL + " |";
  sensorROS.color.a = 1.0;
  sensorROS.color.r = 1.0;
  sensorROS.color.g = 1.0;
  sensorROS.color.b = 1.0;
  sensorROS.scale.z = 0.5;
  sensorPub.publish(sensorROS);

  // Laser height measurement
  double H = msg->twist.covariance[32];
  heightROS.header.frame_id = string("height");
  heightROS.header.stamp = msg->header.stamp;
  heightROS.radiation_type = sensor_msgs::Range::ULTRASOUND;
  heightROS.field_of_view = 5.0 * M_PI / 180.0;
  heightROS.min_range = -100;
  heightROS.max_range = 100;
  heightROS.range = H;
  heightPub.publish(heightROS);

  // Mesh model
  meshROS.header.frame_id = "world";
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "world";
  meshROS.id = _drone_id;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x;
  meshROS.pose.position.y = msg->pose.pose.position.y;
  meshROS.pose.position.z = msg->pose.pose.position.z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  colvec ypr = R_to_ypr(quaternion_to_R(q));
  ypr(0) += rotate_yaw * PI / 180.0;
  q = R_to_quaternion(ypr_to_R(ypr));
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);

  // 相机位姿可视化
  Eigen::Vector3d pose_camera(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond q_camera(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  // 相机的姿态始终是固定的，朝下看，偏航角度为90度
  // 姿态绕x轴旋转180度
  // 再绕z轴旋转90度
  double angle_x = M_PI; // 180 degrees
  double angle_z = M_PI / 2; // 180 degrees

  Eigen::AngleAxisd rotation_x(angle_x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotation_z(angle_z, Eigen::Vector3d::UnitZ());
  q_camera = q_camera * rotation_x;
  q_camera = q_camera * rotation_z;
  // Eigen::Quaterniond q_camera = Eigen::Quaterniond(0.7071, 0, 0, 0.7071);
  // q_camera = q_camera * Eigen::Quaterniond(0.7071, 0, 0, 0.7071);

  cameraposevisual.reset();
  cameraposevisual.add_pose(pose_camera, q_camera);
  cameraposevisual.publish_by(pub_camera_pose_visual, msg->header);

  // TF for raw sensor visualization
  if (tf45)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    transform.setRotation(tf::Quaternion(q(1), q(2), q(3), q(0)));

    tf::Transform transform45;
    transform45.setOrigin(tf::Vector3(0, 0, 0));
    colvec y45 = zeros<colvec>(3);
    y45(0) = 45.0 * M_PI / 180;
    colvec q45 = R_to_quaternion(ypr_to_R(y45));
    transform45.setRotation(tf::Quaternion(q45(1), q45(2), q45(3), q45(0)));

    tf::Transform transform90;
    transform90.setOrigin(tf::Vector3(0, 0, 0));
    colvec p90 = zeros<colvec>(3);
    p90(1) = 90.0 * M_PI / 180;
    colvec q90 = R_to_quaternion(ypr_to_R(p90));
    transform90.setRotation(tf::Quaternion(q90(1), q90(2), q90(3), q90(0)));

    string base_s = _drone_id == -1 ? string("base") : string("base") + std::to_string(_drone_id);
    string laser_s = _drone_id == -1 ? string("laser") : string("laser") + std::to_string(_drone_id);
    string vision_s = _drone_id == -1 ? string("vision") : string("vision") + std::to_string(_drone_id);
    string height_s = _drone_id == -1 ? string("height") : string("height") + std::to_string(_drone_id);

    broadcaster->sendTransform(tf::StampedTransform(transform, msg->header.stamp, string("world"), base_s));
    broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, base_s, laser_s));
    broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, base_s, vision_s));
    broadcaster->sendTransform(tf::StampedTransform(transform90, msg->header.stamp, base_s, height_s));
  }
}

void cmd_callback(const quadrotor_msgs::PositionCommand cmd)
{
  if (cmd.header.frame_id == string("null"))
    return;

  colvec pose(6);
  pose(0) = cmd.position.x;
  pose(1) = cmd.position.y;
  pose(2) = cmd.position.z;
  colvec q(4);
  q(0) = 1.0;
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

  // Mesh model
  meshROS.header.frame_id = "world";
  meshROS.header.stamp = cmd.header.stamp;
  meshROS.ns = "world";
  meshROS.id = _drone_id;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = cmd.position.x;
  meshROS.pose.position.y = cmd.position.y;
  meshROS.pose.position.z = cmd.position.z;

  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = 2.0;
  meshROS.scale.y = 2.0;
  meshROS.scale.z = 2.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);
}

//创建无人机可视化模型
void CreateVisModels(){
  vis_uav_model.markers.resize(5);
  vis_uav_model.markers[0].header.frame_id = "world";
  vis_uav_model.markers[0].header.stamp = ros::Time::now();
  vis_uav_model.markers[0].id = 0;
  vis_uav_model.markers[0].action = visualization_msgs::Marker::ADD;
  vis_uav_model.markers[0].type = visualization_msgs::Marker::SPHERE;
  vis_uav_model.markers[0].scale.x = 0.10;
  vis_uav_model.markers[0].scale.y = 0.10;
  vis_uav_model.markers[0].scale.z = 0.02;
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
  vis_uav_model.markers[4].scale.x = 0.01;
  vis_uav_model.markers[4].scale.y = 0.01;
  vis_uav_model.markers[4].scale.z = 0.01;

  geometry_msgs::Point pt;

  pt.x = 0.08;
  pt.y = 0.08;
  pt.z = -0.01;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.y = -0.08;
  pt.x = -0.08;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.x = -0.08;
  pt.y = 0.08;
  vis_uav_model.markers[4].points.emplace_back(pt);
  pt.x = 0.08;
  pt.y = -0.08;
  vis_uav_model.markers[4].points.emplace_back(pt);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/fake_drone.dae"));
  n.param("color/r", color_r, 1.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 1.0);
  n.param("camera/r",camera_r, 1.0);
  n.param("camera/g",camera_g, 1.0);
  n.param("camera/b",camera_b, 1.0);
  n.param("origin", origin, false);
  n.param("robot_scale", scale, 2.0);
  n.param("frame_id", _frame_id, string("world"));
  n.param("rotate_yaw_deg", rotate_yaw, 0.0);
  n.param("search_height", search_height, 5.0);

  n.param("cross_config", cross_config, false);
  n.param("tf45", tf45, false);
  n.param("covariance_scale", cov_scale, 100.0);
  n.param("covariance_position", cov_pos, false);
  n.param("covariance_velocity", cov_vel, false);
  n.param("covariance_color", cov_color, false);
  n.param("drone_id", _drone_id, -1);

  ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback);
  ros::Subscriber sub_cmd = n.subscribe("cmd", 100, cmd_callback);
  posePub = n.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
  pathPub = n.advertise<nav_msgs::Path>("path", 100, true);
  velPub = n.advertise<visualization_msgs::Marker>("velocity", 100, true);
  covPub = n.advertise<visualization_msgs::Marker>("covariance", 100, true);
  covVelPub = n.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
  trajPub = n.advertise<visualization_msgs::Marker>("trajectory", 100, true);
  sensorPub = n.advertise<visualization_msgs::Marker>("sensor", 100, true);
  meshPub = n.advertise<visualization_msgs::Marker>("robot", 100, true);
  uav_model_pub = n.advertise<visualization_msgs::MarkerArray>("uav_model", 1);
  heightPub = n.advertise<sensor_msgs::Range>("height", 100, true);
  pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 10,true);// 相机位姿可视化

  std_msgs::ColorRGBA color;
  for(int i = 0; i < colorR.size(); i++){
    color.a = 1.0;
    color.r = colorR[i] / 255.0; // 确保颜色值在0到1的范围内
    color.g = colorG[i] / 255.0;
    color.b = colorB[i] / 255.0;
    COLOR_LISTS.push_back(color);
  }
  std_msgs::ColorRGBA camera_color;
  camera_color = Id2Color(1, 1.0);
  cameraposevisual.setImageBoundaryColor( camera_color.r, camera_color.g, camera_color.b);// 相机位姿可视化，设置图像边界颜色
  cameraposevisual.setOpticalCenterConnectorColor( camera_color.r, camera_color.g, camera_color.b);// 相机位姿可视化，设置光学中心连接器颜色
  cameraposevisual.setScale(search_height);// 相机位姿可视化，设置尺度
  cameraposevisual.setLineWidth(0.03);// 相机位姿可视化，设置线宽

  CreateVisModels();

  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::spin();

  return 0;
}
