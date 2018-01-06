#include "ros/ros.h"
#include <iostream>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <mavros_msgs/State.h>

#include "Eigen/Dense"
#include "libxl/libxl.h"

#define STATE_WAITING 	  				0
#define STATE_TAKEOFF 	  				1
#define STATE_FLY 						2
#define STATE_HOVERING 					3
#define STATE_LANDING 					4

#define VEL_XY							0.5
#define VEL_UP							1.0
#define VEL_DOWN						-0.8
#define VEL_Z 							0.8

#define PI 								3.14

using namespace libxl;
using namespace std;
using namespace Eigen;

int vehicle_status = 0;
int px4_status = 0;
int counter = 0;
mavros_msgs::State previous_state;
mavros_msgs::State current_state;

px4_autonomy::Position pos_sp_dt; 
float yaw_sp = PI/2.0;
px4_autonomy::Position current_pos;
px4_autonomy::Position pos_stamp;
bool _reset_pos_sp_xy = false;
bool _reset_pos_sp_z = false;

vector<double> vx;
vector<double> vy;
vector<double> pos_x;
vector<double> pos_y;

void reset_pos_sp_xy()
{
	if(_reset_pos_sp_xy)
	{
		pos_sp_dt.header.stamp = ros::Time::now();
		pos_sp_dt.x = current_pos.x;
		pos_sp_dt.y = current_pos.y;
		ROS_INFO("Reset pos xy: %f  %f",pos_sp_dt.x, pos_sp_dt.y);
		_reset_pos_sp_xy = false;
	}
}

void reset_pos_sp_z()
{
	if(_reset_pos_sp_z)
	{
		pos_sp_dt.header.stamp = ros::Time::now();
		pos_sp_dt.z = current_pos.z;
		ROS_INFO("Reset pos z: %f",pos_sp_dt.z);
		_reset_pos_sp_z = false;
	}
}

bool isArrived_xy(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(sqrt((pos_sp.x - pos.x)*(pos_sp.x - pos.x) + (pos_sp.y - pos.y)*(pos_sp.y - pos.y))< 0.2) 
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool isArrived_z(px4_autonomy::Position &pos, px4_autonomy::Position &pos_sp)
{
	if(fabs(pos_sp.z - pos.z) < 0.1) 
		return true;
	else
		return false;
}

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

void state_callback(const mavros_msgs::State &msg){
	previous_state = current_state;
    current_state = msg;
}

void px4_status_callback(const std_msgs::UInt8 &msg)
{
	px4_status =  msg.data;
}

void pose_callback(const px4_autonomy::Position &msg)
{
	current_pos = msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle n;
	ros::Subscriber state_sub = n.subscribe("/mavros/state", 1, state_callback);
	ros::Subscriber status_sub = n.subscribe("/px4/status", 1, px4_status_callback);
	ros::Subscriber pose_sub = n.subscribe("/px4/pose", 1, pose_callback );
	ros::Publisher takeoff_pub = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1); 
	ros::Publisher pose_pub = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1); 
	ros::Publisher vel_pub = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1); 
	ros::Rate loop_rate(20);

	while(ros::ok())
	{
		if(current_state.mode != "OFFBOARD")
		{
			vehicle_status = STATE_WAITING;
			loop_rate.sleep();
			ros::spinOnce();
			continue;
		}else
		{
			if(previous_state.mode != "OFFBOARD")
			{
				vehicle_status = STATE_WAITING;
				ROS_INFO("OFFBOARD");
			}

			switch(vehicle_status)
			{
				case STATE_WAITING:
				{
					ROS_INFO("Waiting......");
					if(counter < 50)
					{
						counter ++;
					}else
					{
						counter = 0;
						vehicle_status = STATE_TAKEOFF;
					}
					break;
				}

				case STATE_TAKEOFF:
				{
					if(px4_status == 1 || px4_status == 2)
					{
						px4_autonomy::Takeoff takeOff;
						takeOff.take_off = 1; 
						takeoff_pub.publish(takeOff);
						_reset_pos_sp_xy = true;
						_reset_pos_sp_z = true;
					}else
					{
						//fly to set height
						px4_autonomy::Position pos_sp;
						pos_sp.x = current_pos.x;
						pos_sp.y = current_pos.y;
						pos_sp.z = 3.0;
						if(isArrived_z(current_pos, pos_sp))
						{
							_reset_pos_sp_xy = true;
							reset_pos_sp_xy();
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;	
							pose_pub.publish(pos_sp_dt);

							pos_stamp = current_pos;

							vehicle_status = STATE_FLY;
							ROS_INFO("Takeoff ready...");

						}else
						{
							ROS_INFO("TAKEOFF RISING");
							//rise the height
							reset_pos_sp_xy();
							reset_pos_sp_z();
							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.x = pos_sp.x;
							pos_sp_dt.y = pos_sp.y;
							if(pos_sp_dt.z < pos_sp.z)
							{
								pos_sp_dt.z += VEL_UP * 0.05;
							}else
							{
								pos_sp_dt.z = pos_sp.z;
							}
							pos_sp_dt.yaw = yaw_sp;	
							ROS_INFO("%f", pos_sp_dt.z);
							pose_pub.publish(pos_sp_dt);
						}
					}
					break;
				}

				case STATE_FLY:
				{
					ROS_INFO("Flying...");

					for(int i = 0 ; i < pos_x.size(); i++)
					{
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.x = pos_x[i] - pos_x[0] + pos_stamp.x;
						pos_sp_dt.y = pos_y[i] - pos_y[0] + pos_stamp.y;
                        pos_sp_dt.z = 3.0;
                        pos_sp_dt.yaw = yaw_sp;
                        pose_pub.publish(pos_sp_dt);

						ros::spinOnce();
						loop_rate.sleep();
					}
					vehicle_status = STATE_HOVERING;					
					break;
				}

				case STATE_HOVERING:
				{
					ROS_INFO("Hovering......");
					if(counter < 20)
					{
						counter ++;
						reset_pos_sp_xy();
						pos_sp_dt.header.stamp = ros::Time::now();
						pos_sp_dt.yaw = yaw_sp;	
						pose_pub.publish(pos_sp_dt);
					}else
					{
						counter = 0;
						vehicle_status = STATE_HOVERING;
					}
					break;
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}