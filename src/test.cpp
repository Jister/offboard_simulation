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
MatrixXf goal(4,3);

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
			5.0,  0.0,  3.0,
	        5.0,  5.0,  3.0,
	        0.0,  0.0,  3.0;
	        

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;

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

	int count = 0;
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

		goal_pos(0) = goal(count,0);
		goal_pos(1) = goal(count,1);
		goal_pos(2) = goal(count,2);

		arrived = isArrived(local_pos, goal_pos);
		if(arrived)
		{		
			count++;
			if(count > 3)
			{
				while(ros::ok())
				{
				    pose.pose.position.x = goal_pos(0);
				    pose.pose.position.y = goal_pos(1);
				    pose.pose.position.z = goal_pos(2);
					local_pos_pub.publish(pose);
					ros::spinOnce();
					rate.sleep();
				}
			}
		}
	    pose.pose.position.x = goal_pos(0);
	    pose.pose.position.y = goal_pos(1);
	    pose.pose.position.z = goal_pos(2);
		local_pos_pub.publish(pose);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}