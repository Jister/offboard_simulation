#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "Eigen/Dense"


#define Pi 					3.141592653
#define NEAR_DISTANCE 		0.3
#define OBSTACLE_REGION 	6.0

#define STATE_IDLE 			0
#define STATE_HOLDING 		1
#define STATE_TAKEOFF 		2
#define STATE_MOVING 		3
#define STATE_LANDING 		4

#define MOVING_SPEED_UP 	1
#define MOVING_SPEED_DOWN 	2
#define MOVING_NORMAL		0

using namespace Eigen;

Vector3f local_pos(0.0,0.0,0.0);
Vector3f goal_pos(0.0,0.0,0.0);
MatrixXf goal(5,3);

int current_stage = 0;
bool arrived = false;
bool takeoff_ready = false;

int flight_state = 0;
int moving_state = 0;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg){
	local_pos(0) = msg.pose.position.x;
	local_pos(1) = msg.pose.position.y;
	local_pos(2) = msg.pose.position.z;
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
	ros::init(argc, argv, "offboard_node");
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
	ros::Rate rate(20.0);

	goal << 0.0,  0.0,  3.0,
			10.0, 0.0,  3.0,
	        10.0, 10.0, 3.0,
	        5.0,  10.0, 3.0,
	        0.0,  0.0,  3.0;
	        

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	//send a few setpoints before starting
	for(int i = 10; ros::ok() && i > 0; --i){
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
					flight_state = STATE_TAKEOFF;
				}
				last_request = ros::Time::now();
			}
		}

		if(current_stage < 5)
		{
			goal_pos(0) = goal(current_stage, 0);
			goal_pos(1) = goal(current_stage, 1);
			goal_pos(2) = goal(current_stage, 2);
			arrived = isArrived(local_pos,goal_pos);	
		}else
		{
			flight_state = STATE_IDLE;
		}

		switch(flight_state)
		{
			case STATE_IDLE:
			{
				geometry_msgs::TwistStamped cmd;
				cmd.twist.linear.x = 0;
				cmd.twist.linear.y = 0;
				cmd.twist.linear.z = 0;
				cmd.twist.angular.x = 0;
				cmd.twist.angular.y = 0;
				cmd.twist.angular.z = 0;
				local_vel_pub.publish(cmd);

				break;
			}
			case STATE_HOLDING:
			{
				ROS_INFO("Holding");
				for(int i = 10; ros::ok() && i > 0; --i){
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
				flight_state = STATE_MOVING;
				moving_state = MOVING_SPEED_UP;

				break;
			}
			case STATE_TAKEOFF:
			{
				if(!takeoff_ready)
				{
					if(arrived){
						takeoff_ready = true;
						ROS_INFO("Takeoff ready");
					}else
					{
						geometry_msgs::PoseStamped pose;
						pose.pose.position.x = 0;
					    pose.pose.position.y = 0;
					    pose.pose.position.z = 3;
					    local_pos_pub.publish(pose);
					}		
				}else
				{
					flight_state = STATE_HOLDING;
					current_stage ++;
				}
				break;
			}
			case STATE_MOVING:
			{
				geometry_msgs::TwistStamped cmd;

				if(arrived)
				{
					cmd.twist.linear.x = 0;
					cmd.twist.linear.y = 0;
					cmd.twist.linear.z = 0;
					cmd.twist.angular.x = 0;
					cmd.twist.angular.y = 0;
					cmd.twist.angular.z = 0;
					local_vel_pub.publish(cmd);

					flight_state = STATE_HOLDING;
					current_stage++;
				}else
				{
					switch(moving_state)
					{
						case MOVING_SPEED_UP:
						{
							for(int i = 0; ros::ok() && i < 20; i++){
								ROS_INFO("Speed up");
								Vector3f dirction = (goal_pos - local_pos).normalized();
								float vel = (float)i/20;
								
								cmd.twist.linear.x = vel * dirction(0);
								cmd.twist.linear.y = vel * dirction(1);
								cmd.twist.linear.z = vel * dirction(2);
								cmd.twist.angular.x = 0;
								cmd.twist.angular.y = 0;
								cmd.twist.angular.z = 0;
								local_vel_pub.publish(cmd);
								ros::spinOnce();
								rate.sleep();
							}
							moving_state = MOVING_NORMAL;
							ROS_INFO("Normal fly");
							break;
						}
						case MOVING_SPEED_DOWN:
						{
							ROS_INFO("Speed down");
							Vector3f dirction = (goal_pos - local_pos).normalized();
							float vel = ((goal_pos - local_pos).norm())/2.0;
							
							cmd.twist.linear.x = vel * dirction(0);
							cmd.twist.linear.y = vel * dirction(1);
							cmd.twist.linear.z = vel * dirction(2);
							cmd.twist.angular.x = 0;
							cmd.twist.angular.y = 0;
							cmd.twist.angular.z = 0;
							local_vel_pub.publish(cmd);
							ros::spinOnce();
							rate.sleep();

							break;
						}
						case MOVING_NORMAL:
						{
							if((goal_pos - local_pos).norm() < 2)
							{
								moving_state = MOVING_SPEED_DOWN;
							}else
							{
								Vector3f dirction = (goal_pos - local_pos).normalized();
								float vel = 1;
								
								cmd.twist.linear.x = vel * dirction(0);
								cmd.twist.linear.y = vel * dirction(1);
								cmd.twist.linear.z = vel * dirction(2);
								cmd.twist.angular.x = 0;
								cmd.twist.angular.y = 0;
								cmd.twist.angular.z = 0;
								local_vel_pub.publish(cmd);
							}

							break;
						}
					}
							
				}

				break;
			}
			
			case STATE_LANDING:
			{
				break;
			}
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
