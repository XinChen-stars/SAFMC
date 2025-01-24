#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <iostream>
#include <quadrotor_msgs/PositionCommand.h>

#define VELOCITY2D_CONTROL 0b101111000111 //mask，from right to left [PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE]
// if you want to use , set 0, otherwise set 1
// if you want to use velocity control, you should set the mask to 0b101111000111
// if you want to use position control, you should set the mask to 0b100111011000

std::string uav_name;
int uav_id;
double takeoff_height;
std::string pre_name;

mavros_msgs::State current_state;
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
int flag = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
double position_x_begin, position_y_begin, position_z_begin, yaw_begin;
bool get_first_pos = false;
double position_x, position_y, position_z, current_yaw;
double rviz_pos_x,rviz_pos_y;

// position velocity acceleration yaw yaw_dot
double cmd_pos_x, cmd_pos_y, cmd_pos_z, cmd_vel_x, cmd_vel_y, cmd_vel_z, cmd_a_x, cmd_a_y, cmd_a_z, cmd_yaw, cmd_yaw_rate; 
double pi = 3.14159265;

bool cmd_receive = false;//pos_cmd_cb receive flag

ros::ServiceClient px4_set_mode_client;
ros::ServiceClient px4_arming_client;
ros::ServiceClient px4_set_param;

quadrotor_msgs::PositionCommand cmd;

// set px4 mode
void Set_PX4_Mode(const std::string & mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    if (px4_set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent)
    {
        // ROS_INFO_STREAM(pre_name + " : " << mode << "call enabled !");
    }
    else
    {
        ROS_ERROR_STREAM(pre_name + " : " << mode << "call failed !");
    }
}

// uav state callback
void uav_state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

//uav odometry callback
void uav_odom_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	if (!get_first_pos) {
		position_x_begin = position_msg.pose.pose.position.x;
		position_y_begin = position_msg.pose.pose.position.y;
		position_z_begin = position_msg.pose.pose.position.z;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
		yaw_begin = yaw;
		get_first_pos = true;
	}
    position_x = position_msg.pose.pose.position.x - position_x_begin;
    position_y = position_msg.pose.pose.position.y - position_y_begin;
    position_z = position_msg.pose.pose.position.z - position_z_begin;
	current_yaw = yaw;

}

// rviz target callback
void rviz_target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
  target_pos = *msg;
  rviz_pos_x = target_pos.pose.position.x;
  rviz_pos_y = target_pos.pose.position.y;
}

// pos command callback
void pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
    cmd_receive = true;
	cmd = *msg;
    cmd_pos_x = cmd.position.x;
	cmd_pos_y = cmd.position.y;
	cmd_pos_z = cmd.position.z;
	cmd_vel_x = cmd.velocity.x;
	cmd_vel_y = cmd.velocity.y;
	cmd_vel_z = cmd.velocity.z;
	cmd_a_x = cmd.acceleration.x;
	cmd_a_y = cmd.acceleration.y;
	cmd_a_y = cmd.acceleration.y;
	cmd_yaw = cmd.yaw + yaw_begin;
	cmd_yaw_rate = cmd.yaw_dot;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "px4_basic_ctrl");
	ros::NodeHandle nh("~");
	std::string node_name;
    node_name = ros::this_node::getName();

    nh.param<std::string>("uav_name", uav_name, "iris");
    nh.param<int>("uav_id", uav_id, 0);
    nh.param<double>("takeoff_height", takeoff_height, 1.0);
	pre_name = "/" + uav_name + "_" + std::to_string(uav_id);

	// subscribe the uav state
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(pre_name + "/mavros/state", 10, uav_state_cb);//uav state
	ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry> (pre_name + "/mavros/local_position/odom",10,uav_odom_cb);//uav odometry
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, rviz_target_cb);//rviz target
	ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, pos_cmd_cb);//position command

	// publish the local position target
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(pre_name + "/mavros/setpoint_raw/local", 1); //vel control

	// the service clients
	px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(pre_name + "/mavros/set_mode");//set uav mode
	px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(pre_name + "/mavros/cmd/arming");//arm or disarm uav
	px4_set_param = nh.serviceClient<mavros_msgs::ParamSet>(pre_name + "/mavros/param/set");//set uav param
	
	
    ros::Rate rate(50.0); //control rate, faster than 30Hz
	
	// wait for FCU connection
	while(ros::ok() && !current_state.connected)
	{
		ROS_WARN_STREAM(pre_name << " : Waiting for FCU connection...");
		ros::spinOnce();
		rate.sleep();
	}

	// Wait for the service to become available
    if (!px4_set_param.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR_STREAM(pre_name << " : Service /mavros/param/set is not available.");
        return -1;
    }

    // Prepare the service request
    mavros_msgs::ParamSet param_set_srv;
    param_set_srv.request.param_id = "COM_RCL_EXCEPT";

    // Set the desired value (integer: 4, real: 0.0)
    param_set_srv.request.value.integer = 4;
    param_set_srv.request.value.real = 0.0;

    // Call the service
    if (px4_set_param.call(param_set_srv)) {
        if (param_set_srv.response.success) {
            ROS_INFO_STREAM(pre_name << ": Parameter set successfully: " << param_set_srv.request.param_id.c_str());
        } else {
            ROS_ERROR_STREAM(pre_name <<  " : Failed to set parameter: " << param_set_srv.request.param_id.c_str());
        }
    } else {
        ROS_ERROR_STREAM(pre_name <<  " : Service call failed.");
    }



	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i)
	{
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	while(ros::ok())
	{
		if( current_state.mode != "OFFBOARD"){
			if( px4_set_mode_client.call(offb_set_mode) &&	offb_set_mode.response.mode_sent){
				ROS_WARN_STREAM(pre_name << " : Offboard enabled");
			}
			last_request = ros::Time::now();
		}
		else {
			if( !current_state.armed)
			{
				if( px4_arming_client.call(arm_cmd) &&arm_cmd.response.success)
				{
					ROS_WARN_STREAM(pre_name << " : Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}
       
		// auto take off and hover at takeoff_height until receive the first target
		if(!cmd_receive)
		{
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.header.stamp = ros::Time::now();
			current_goal.type_mask = velocity_mask;
			current_goal.velocity.x = (0 - position_x) * 1;
			current_goal.velocity.y = (0 - position_y) * 1;
			current_goal.velocity.z = (takeoff_height - position_z) * 1;
			current_goal.yaw = current_yaw;
			ROS_INFO_STREAM(pre_name << " : Hover! Waiting for the cmd target... ");
			local_pos_pub.publish(current_goal);
			ros::spinOnce();
			rate.sleep();
		}

		// track the target position from pos_cmd_cb
		if(cmd_receive)
		{
			current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			current_goal.header.stamp = ros::Time::now();
			current_goal.type_mask = velocity_mask;
			current_goal.velocity.x =  (cmd_pos_x - position_x - position_x_begin) * 2;
			current_goal.velocity.y =  (cmd_pos_y - position_y - position_y_begin) * 2;
			current_goal.velocity.z =  (cmd_pos_z - position_z - position_z_begin) * 2;
			current_goal.yaw = cmd_yaw;
			double vel = sqrt(pow(current_goal.velocity.x, 2) + pow(current_goal.velocity.y, 2));
			ROS_INFO_STREAM(pre_name << " : vel = " << vel);
		}
		
		local_pos_pub.publish(current_goal);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
