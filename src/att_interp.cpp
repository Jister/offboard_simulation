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
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#define NEAR_DISTANCE 		0.2

using namespace libxl;
using namespace Eigen;
using namespace std;

Vector3f local_pos(0.0,0.0,0.0);
// Vector3f local_angv(0.0,0.0,0.0);
Vector4f local_att(0.0,0.0,0.0,0.0);

Vector3f goal_pos(0.0,0.0,2.0);
// Vector3f goal_angv(0.3,0.2,0.1);
// // Vector4f goal_att(0.98929,0.039527,0.10418,0.094246);
// Vector4f goal_att(0.78607,0.16752,0.57049,0.16752);

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg){
	local_pos(0) = msg.pose.position.x;
	local_pos(1) = msg.pose.position.y;
	local_pos(2) = msg.pose.position.z;
}

void att_cb(const sensor_msgs::Imu msg){
	local_att(0)=msg.orientation.x;
	local_att(1)=msg.orientation.y;
	local_att(2)=msg.orientation.z;
	local_att(3)=msg.orientation.w;
}

// void angv_cb(const geometry_msgs::TwistStamped msg)
// {
// 	local_angv(0)=msg.twist.angular.x;
// 	local_angv(1)=msg.twist.angular.y;
// 	local_angv(2)=msg.twist.angular.z;
// }
vector<double> vx;
vector<double> vy;
vector<double> vz;
vector<double> vw;
vector<double> measure_x;
vector<double> measure_y;
vector<double> measure_z;
vector<double> measure_w;

bool isArrived3(Vector3f& local, Vector3f& goal)
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

// bool isArrived4(Vector4f& local, Vector4f& goal)
// {
// 	if((local - goal).norm() < NEAR_DISTANCE)
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		cout<<"difference:"<<(local - goal).norm()<<endl;
// 		return false;
// 	}
// }

void data_init()
{
	Book* book = xlCreateBook();
	book->setKey("chenjie", "linux-2c232e010dcbe60168b76d6da0f5hfga"); 
	if(book)
	{
		if(book->load("/home/ubuntu/catkin_ws/src/offboard_simulation/data/1000Hz_overMVC_02.xls")) //1000Hz_belowMVC.xls 1000Hz_overMVC.xls 
		{
			Sheet* sheet = book->getSheet(0);
			if(sheet)
			{
				sheet->insertRow(0,999);
				for (int row =0;row<=999;++row)
				{
					sheet->writeNum(row,0,sheet->readNum(1000,0));
        			sheet->writeNum(row,1,sheet->readNum(1000,1));
        			sheet->writeNum(row,2,sheet->readNum(1000,2));
        			sheet->writeNum(row,3,sheet->readNum(1000,3)); //copy 1st row 1000 times and insert at the beginning
				}
				int mark;
				mark=sheet->lastRow();
				sheet->insertRow(mark+1,mark+1000);
				for (int row =mark;row<=(mark+999);++row)
				{
					sheet->writeNum(row,0,sheet->readNum(mark-1,0));
        			sheet->writeNum(row,1,sheet->readNum(mark-1,1));
        			sheet->writeNum(row,2,sheet->readNum(mark-1,2));
        			sheet->writeNum(row,3,sheet->readNum(mark-1,3));
				}

				for(int row = sheet->firstRow(); row < sheet->lastRow(); ++row)
				{	
					vw.push_back(sheet->readNum(row-1, 0));
					vx.push_back(sheet->readNum(row-1, 1));
					vy.push_back(sheet->readNum(row-1, 2));
					vz.push_back(sheet->readNum(row-1, 3)); //copy the last row 1000 times at the end;	
				}
				sheet->removeRow(0,999);
				sheet->removeRow(mark,sheet->lastRow());
			}
			book->save("/home/ubuntu/catkin_ws/src/offboard_simulation/data/1000Hz_overMVC_02.xls");//1000Hz_belowMVC.xls 1000Hz_overMVC.xls
		}
	}
	book->release();
}

void data_write()
{
	Book* book = xlCreateBook(); // xlCreateXMLBook() for xlsx
	book->setKey("chenjie", "linux-2c232e010dcbe60168b76d6da0f5hfga");
    if(book)
    {
        Sheet* sheet = book->addSheet("sognsong");
        if(sheet)
        {
        	for (int row=1;row<=measure_w.size();row++)
        	{
        		sheet->writeNum(row-1,0,measure_w[row-1]);
        		sheet->writeNum(row-1,1,measure_x[row-1]);
        		sheet->writeNum(row-1,2,measure_y[row-1]);
        		sheet->writeNum(row-1,3,measure_z[row-1]);
        	}
        }
        book->save("/home/ubuntu/catkin_ws/src/offboard_simulation/data/1000Hz_wr_overMVC_02.xls"); //1000Hz_wr_belowMVC.xls 1000Hz_wr_overMVC.xls
        book->release();
    } 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "songsong");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
			("mavros/state", 10, state_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, pose_cb);
	ros::Subscriber att_sub = nh.subscribe<sensor_msgs::Imu>
			("mavros/imu/data", 10, att_cb);
	// ros::Subscriber angv_sub = nh.subscribe<geometry_msgs::TwistStamped>
	// 		("mavros/setpoint/attitude/cmd_vel", 10, angv_cb);
	// ros::Publisher local_angv_pub = nh.advertise<geometry_msgs::TwistStamped>
	// 		("mavros/setpoint_attitude/cmd_vel", 10);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_position/local", 10);
	ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
			("mavros/setpoint_attitude/attitude", 10);
	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
			("mavros/setpoint_velocity/cmd_vel", 10);
	ros::Publisher local_throttle_pub = nh.advertise<std_msgs::Float64>
			("mavros/setpoint_attitude/att_throttle", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(1000);

	data_init();
	bool arrived = false;

	// wait for FCU connection
	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped init_pose;
	init_pose.pose.position.x = 0;
	init_pose.pose.position.y = 0;
	init_pose.pose.position.z = 2;

	// geometry_msgs::TwistStamped init_angv;
	// init_angv.twist.angular.x=0.3;
	// init_angv.twist.angular.y=0.2;
	// init_angv.twist.angular.z=0.1;

	// geometry_msgs::PoseStamped init_att;
	// // init_att.pose.orientation.w=0.98929;
	// // init_att.pose.orientation.x=0.039527;
	// // init_att.pose.orientation.y=0.10418;
	// // init_att.pose.orientation.z=0.094246;

	// init_att.pose.orientation.w=0.78607;
	// init_att.pose.orientation.x=0.16752;
	// init_att.pose.orientation.y=0.57094;
	// init_att.pose.orientation.z=0.16752;

	// send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(init_pose);
		// local_angv_pub.publish(init_angv);
		// local_att_pub.publish(init_att);
		ros::spinOnce();
		rate.sleep();
	}
	// 	for(int i = 100; ros::ok() && i > 0; --i){
	// 	// local_pos_pub.publish(init_pose);
	// 	// local_angv_pub.publish(init_angv);
	// 	local_att_pub.publish(init_att);
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success)
			{
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} 
		else {
			if( !current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		arrived = isArrived3(local_pos, goal_pos);
		// isArrived4(local_att,goal_att)&&isArrived3(local_angv,goal_angv)&&
		cout<<"arrived:"<<arrived<<endl;

		if(arrived)
		{
			for(int i = 0 ; i < vx.size(); i++)
			{
/*				geometry_msgs::TwistStamped steady;
				steady.twist.linear.x=0;
				steady.twist.linear.y=0;
				steady.twist.linear.z=0;
				local_att_vel_pub.publish(steady);*/
				std_msgs::Float64 thrott;
				thrott.data=0.575;
				local_throttle_pub.publish(thrott);

				geometry_msgs::PoseStamped cmd;
				cmd.pose.orientation.x=vx[i];
				cmd.pose.orientation.y=vy[i];
				cmd.pose.orientation.z=vz[i];
				cmd.pose.orientation.w=vw[i];
				local_att_pub.publish(cmd);

				// if (i==0)
				// {
				// 	ros::Time current_time=ros::Time::now();
				// 	while (1)
				// 	{
				// 		if ((ros::Time::now()-current_time)>=ros::Duration(4.0))
				// 		{
				// 			break;
				// 		}
				// 		else
				// 		{

				// 			cout<<"waiting for the initial attitude\t";
				// 		}
				// 	}
					
				// } //delay to achieve the initial attitude

				measure_x.push_back(local_att(0));
				measure_y.push_back(local_att(1));
				measure_z.push_back(local_att(2));
				measure_w.push_back(local_att(3));
				ros::spinOnce();
				rate.sleep();
			}

			ros::Time last_time = ros::Time::now();
			bool flag=true;

			while(ros::ok())
			{
				if (ros::Time::now() - last_time > ros::Duration(5.0)&& flag)
				{
					data_write();
					flag=false;
				}
				geometry_msgs::TwistStamped cmd;
				cmd.twist.linear.x = 0;
				cmd.twist.linear.y = 0;
				cmd.twist.linear.z = 0;
				cmd.twist.angular.x = 0;
				cmd.twist.angular.y = 0;
				cmd.twist.angular.z = 0;
				local_vel_pub.publish(cmd);
				measure_x.push_back(local_att(0));
				measure_y.push_back(local_att(1));
				measure_z.push_back(local_att(2));
				measure_w.push_back(local_att(3));
				ros::spinOnce();
				rate.sleep();
			}

		}else
		{
			local_pos_pub.publish(init_pose);
		}
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}