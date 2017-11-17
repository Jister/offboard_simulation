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
Vector3f local_vel(0.0,0.0,0.0);
Vector3f goal_pos(1.0,1.0,2.0);

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg){
	local_pos(0) = msg.pose.position.x;
	local_pos(1) = msg.pose.position.y;
	local_pos(2) = msg.pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped msg){
	local_vel(0) = msg.twist.linear.x;
	local_vel(1) = msg.twist.linear.y;
	local_vel(2) = msg.twist.linear.z;
}

vector<double> vx;
vector<double> vy;
vector<double> pos_x;
vector<double> pos_y;

void data_init()
{
	Book* book = xlCreateBook();
	book->setKey("chenjie", "linux-2c232e010dcbe60168b76d6da0f5hfga"); 
	if(book)
	{
		if(book->load("/home/ubuntu/catkin_ws/src/offboard_simulation/data/data20Hz_Z.xls"))
		//if(book->load("/home/ubuntu/catkin_ws/src/offboard_simulation/data/data20Hz_insert.xls"))
		{
			Sheet* sheet = book->getSheet(0);
			if(sheet)
			{
				for(int row = sheet->firstRow(); row < sheet->lastRow(); ++row)
				{	
					pos_x.push_back(sheet->readNum(row-1, 0));
					pos_y.push_back(sheet->readNum(row-1, 1));
					vx.push_back(sheet->readNum(row-1, 2));
					vy.push_back(sheet->readNum(row-1, 3));
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
	ros::Subscriber velcocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
			("mavros/local_position/velocity", 10, vel_cb);
			
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local", 10);

	ros::Publisher set_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
			("position_sp", 10);
	ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
			("velocity_sp", 10);
	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
			("mavros/setpoint_velocity/cmd_vel", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20);

	data_init();
	bool arrived = false;
	bool int_init = false;

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = goal_pos(0);
	pose.pose.position.y = goal_pos(1);
	pose.pose.position.z = goal_pos(2);

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
			for(int i = 0 ; i < pos_x.size(); i++)
			{
				double kp = 1.5, ki = 0.1;
				Vector2f err;
				static Vector2f err_int;
				if(!int_init)
				{
					err_int(0) = 0;
					err_int(1) = 0;
					int_init = true;
				}
				
				err(0) = pos_x[i] - local_pos(0);
				err(1) = pos_y[i] - local_pos(1);

				double vx_pos = kp * err(0) ;//+ ki * err_int(0);
				double vy_pos = kp * err(1) ;//+ ki * err_int(1);
				geometry_msgs::TwistStamped cmd;
				cmd.header.stamp = ros::Time::now();
				cmd.twist.linear.x =  vx_pos; //+vx[i]+0.1*(vx[i]-local_vel(0)) 
				cmd.twist.linear.y =  vy_pos; //+vy[i]+0.1*(vy[i]-local_vel(1)) ;
				cmd.twist.linear.z = 0;
				cmd.twist.angular.x = 0;
				cmd.twist.angular.y = 0;
				cmd.twist.angular.z = 0;
				local_vel_pub.publish(cmd);

				err_int += err/20;

				// geometry_msgs::PoseStamped  pos_sp;
				// pos_sp.pose.position.x = pos_x[i];
				// pos_sp.pose.position.y = pos_y[i];
				// pos_sp.pose.position.z = 2;
				// local_pos_pub.publish(pos_sp);
				geometry_msgs::PoseStamped pose_sp;
				pose_sp.header.stamp = ros::Time::now();
				pose_sp.pose.position.x = pos_x[i];
				pose_sp.pose.position.y = pos_y[i];
				pose_sp.pose.position.z = 2;
				set_pos_pub.publish(pose_sp);

				geometry_msgs::TwistStamped vel_sp;
				vel_sp.header.stamp = ros::Time::now();
				vel_sp.twist.linear.x = vx[i];
				vel_sp.twist.linear.y = vy[i];
				vel_sp.twist.linear.z = 0;
				vel_sp.twist.angular.x = 0;
				vel_sp.twist.angular.y = 0;
				vel_sp.twist.angular.z = 0;
				set_vel_pub.publish(vel_sp);

				ros::spinOnce();
				rate.sleep();
			}

			while(ros::ok())
			{
				// geometry_msgs::TwistStamped cmd;
				// cmd.twist.linear.x = 0;
				// cmd.twist.linear.y = 0;
				// cmd.twist.linear.z = 0;
				// cmd.twist.angular.x = 0;
				// cmd.twist.angular.y = 0;
				// cmd.twist.angular.z = 0;
				// local_vel_pub.publish(cmd);
				geometry_msgs::PoseStamped  pos_sp;
				pos_sp.pose.position.x = pos_x[pos_x.size()-1];
				pos_sp.pose.position.y = pos_y[pos_x.size()-1];
				pos_sp.pose.position.z = 2;
				local_pos_pub.publish(pos_sp);
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