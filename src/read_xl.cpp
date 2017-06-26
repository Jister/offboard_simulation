#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "libxl/libxl.h"
#include "Eigen/Dense"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define NEAR_DISTANCE 		0.2

using namespace libxl;
using namespace Eigen;
using namespace std;

Vector3f local_pos(0.0,0.0,0.0);
Vector3f goal_pos(0.0,0.0,2.0);

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg){
	local_pos(0) = msg.pose.position.x;
	local_pos(1) = msg.pose.position.y;
	local_pos(2) = msg.pose.position.z;
}

vector<double> vx;
vector<double> vy;

void data_init()
{
	Book* book = xlCreateBook();
	book->setKey("chenjie", "linux-2c232e010dcbe60168b76d6da0f5hfga"); 
	if(book)
	{
		if(book->load("/home/ubuntu/catkin_ws/src/offboard_simulation/data/100Hz.xls"))
		{
			Sheet* sheet = book->getSheet(0);
			if(sheet)
			{
				for(int row = sheet->firstRow(); row < sheet->lastRow(); ++row)
				{	
					vx.push_back(sheet->readNum(row-1, 0));
					vy.push_back(sheet->readNum(row-1, 1));	
				}
			}
		}
	}
	book->release();
}

bool isArrived(Vector3f& local, Vector3f& goal)
{
	if((local - goal).norm() < NEAR_DISTANCE)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prp");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
			("mavros/state", 10, state_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, pose_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local", 10);
	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
			("mavros/setpoint_velocity/cmd_vel", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(100);

	data_init();
	bool arrived = false;

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success){
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		arrived = isArrived(local_pos, goal_pos);
		if(arrived)
		{
			for(int i = 0 ; i < vx.size(); i++)
			{
				geometry_msgs::TwistStamped cmd;
				cmd.twist.linear.x = vx[i];
				cmd.twist.linear.y = vy[i];
				cmd.twist.linear.z = 0;
				cmd.twist.angular.x = 0;
				cmd.twist.angular.y = 0;
				cmd.twist.angular.z = 0;
				local_vel_pub.publish(cmd);
				ros::spinOnce();
				rate.sleep();
			}

			while(ros::ok())
			{
				geometry_msgs::TwistStamped cmd;
				cmd.twist.linear.x = 0;
				cmd.twist.linear.y = 0;
				cmd.twist.linear.z = 0;
				cmd.twist.angular.x = 0;
				cmd.twist.angular.y = 0;
				cmd.twist.angular.z = 0;
				local_vel_pub.publish(cmd);
				ros::spinOnce();
				rate.sleep();
			}

		}else
		{
			local_pos_pub.publish(pose);
		}
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}