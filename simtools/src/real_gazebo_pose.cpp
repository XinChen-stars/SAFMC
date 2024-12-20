#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <mutex>
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ParamSet.h>

geometry_msgs::PoseStamped camera_pose;
geometry_msgs::PoseStamped lidar_pose;
std::string uav_name;
int uav_num;
bool init_ = false;
double init_x = 0, init_y = 0, init_z = 0;
tf2::Quaternion init_q(1, 0, 0, 0);
std::vector<ros::Publisher> multi_pose_pub;
std::vector<ros::Publisher> multi_camera_pub;
std::vector<ros::Publisher> multi_lidar_pub;
std::vector<geometry_msgs::PoseStamped> multi_local_pose;




void GazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (int i = 0; i < uav_num; ++i) {
        std::string target_name = uav_name + "_" + std::to_string(i);
        auto it = std::find(msg->name.begin(), msg->name.end(), target_name);
        if (it != msg->name.end())
        {
            int id = std::distance(msg->name.begin(), it);
            multi_local_pose[i].header.stamp = ros::Time::now();
            multi_local_pose[i].header.frame_id = "world";
            multi_local_pose[i].pose = msg->pose[id];
            
            if (!init_)
            {
                init_x = msg->pose[id].position.x;
                init_y = msg->pose[id].position.y;
                init_z = msg->pose[id].position.z;
                init_q = tf2::Quaternion(msg->pose[id].orientation.x, msg->pose[id].orientation.y, msg->pose[id].orientation.z, msg->pose[id].orientation.w);
                init_ = true;
            }  
        }
    }
}

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "real_gazebo_pose");
    ros::NodeHandle nh("~");
    std::string node_name;
    node_name = ros::this_node::getName();

    nh.param<std::string>("uav_name", uav_name, "iris");
    nh.param<int>("uav_num", uav_num, 1);
    ROS_WARN_STREAM("uav_name: " << uav_name << " uav_num: " << uav_num);

    multi_pose_pub.resize(uav_num);
    multi_local_pose.resize(uav_num);
    multi_camera_pub.resize(uav_num);
    multi_lidar_pub.resize(uav_num);

    for (int i = 0; i < uav_num; ++i) {
        std::string topic_name = "/" + uav_name + "_" + std::to_string(i) + "/mavros/vision_pose/pose";
        std::string camera_topic_name = "/" + uav_name + "_" + std::to_string(i) + "/camera_pose";
        std::string lidar_topic_name = "/" + uav_name + "_" + std::to_string(i) + "/lidar_pose";
        multi_pose_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
        multi_camera_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(camera_topic_name, 10);
        multi_lidar_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(lidar_topic_name, 10);
        ROS_WARN("Publishing %s groundtruth pose", topic_name.c_str());

        ros::ServiceClient px4_set_freq;//设置相关消息频率
        px4_set_freq = nh.serviceClient<mavros_msgs::MessageInterval>("/" + uav_name + "_" +std::to_string(i) + "/mavros/set_message_interval");
        mavros_msgs::MessageInterval pose_frq;
        mavros_msgs::MessageInterval vel_frq;
        pose_frq.request.message_id = 31;
        pose_frq.request.message_rate = 100;
        if (px4_set_freq.call(pose_frq) && pose_frq.response.success) 
        {
            ROS_WARN_STREAM("/" + uav_name + "_" + std::to_string(i) + "/mavros/local_position/pose frequency up to 100Hz successed !");
        }
        vel_frq.request.message_id = 32;
        vel_frq.request.message_rate = 100;
        if (px4_set_freq.call(vel_frq) && vel_frq.response.success) 
        {
            ROS_WARN_STREAM("/" + uav_name + "_" + std::to_string(i) + "/mavros/local_position/velocity_local frequency up to 100Hz successed !");
        }
    }

    ros::Subscriber sub_gazebo_pose = nh.subscribe("/gazebo/model_states", 10, GazeboCallback);


    ros::Rate rate(100);
    while (ros::ok())
    {
        
        for (int i = 0; i < uav_num; i++)
        {
            geometry_msgs::PoseStamped pose;
            tf2::Quaternion current_q(
                multi_local_pose[i].pose.orientation.x,
                multi_local_pose[i].pose.orientation.y,
                multi_local_pose[i].pose.orientation.z,
                multi_local_pose[i].pose.orientation.w
            );

            tf2::Quaternion q_relative = current_q * init_q.inverse();
            pose.pose.orientation.x = q_relative.x();
            pose.pose.orientation.y = q_relative.y();
            pose.pose.orientation.z = q_relative.z();
            pose.pose.orientation.w = q_relative.w();

            tf2::Vector3 relative_position(
                multi_local_pose[i].pose.position.x - init_x,
                multi_local_pose[i].pose.position.y - init_y,
                multi_local_pose[i].pose.position.z - init_z
            );
            tf2::Vector3 transformed_position = tf2::quatRotate(init_q.inverse(), relative_position);

            pose.pose.position.x = transformed_position.x();
            pose.pose.position.y = transformed_position.y();
            pose.pose.position.z = transformed_position.z();

            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "world";

            camera_pose.header.stamp = ros::Time::now();
            camera_pose.header.frame_id = "world";
            camera_pose.pose.position.x = pose.pose.position.x + 0.1;//根据相机在无人机上的安装位置调整
            camera_pose.pose.position.y = pose.pose.position.y;
            camera_pose.pose.position.z = pose.pose.position.z;
            camera_pose.pose.orientation = pose.pose.orientation;

            lidar_pose.header.stamp = ros::Time::now();
            lidar_pose.header.frame_id = "world";
            lidar_pose.pose.position.x = pose.pose.position.x + 0.2;
            lidar_pose.pose.position.y = pose.pose.position.y;
            lidar_pose.pose.position.z = pose.pose.position.z + 0.1;//根据雷达在无人机上的安装位置调整
            lidar_pose.pose.orientation = pose.pose.orientation;


            multi_pose_pub[i].publish(pose);
            multi_camera_pub[i].publish(camera_pose);
            multi_lidar_pub[i].publish(lidar_pose);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
