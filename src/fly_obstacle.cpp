#include "ros/ros.h"
#include <iostream>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
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
#define OBSTACLE_DIST 					5.0

using namespace libxl;
using namespace std;
using namespace Eigen;

int obstacle[20][20]={0};
int obstacle_count[20][20]={0};
double k_att = 10.0;
double k_rep = 10.0;
double k_add = 2.0;

Vector2f v_dir;
Vector2f last_v_dir;
bool first = true;

int vehicle_status = 0;
int px4_status = 0;
int counter = 0;
mavros_msgs::State previous_state;
mavros_msgs::State current_state;

px4_autonomy::Position pos_sp_dt; 
// float yaw_sp = PI/2.0;
float yaw_sp = 0.0;
px4_autonomy::Position current_pos;
px4_autonomy::Position pos_stamp;

bool _reset_pos_sp_xy = false;
bool _reset_pos_sp_z = false;

void rotate_2D(double yaw,  const Vector2f& input,  Vector2f& output)
{
	double sy = sinf(yaw);
	double cy = cosf(yaw);

	Matrix2f R;
	R(0,0) = cy;
	R(0,1) = -sy;
	R(1,0) = sy;
	R(1,1) = cy;

	output = R * input;
}

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

void scanCallback(const sensor_msgs::LaserScan scan)
{
	memset(obstacle,0,sizeof(obstacle));
	memset(obstacle_count,0,sizeof(obstacle_count));
	for (unsigned int i = 0; i < scan.ranges.size(); ++i)
	{
		double angle = scan.angle_min + i * scan.angle_increment;
		Vector2f point;
		point(0) = scan.ranges[i] * cos(angle);
		point(1) = scan.ranges[i] * sin(angle);

		if(point(0) >= -5 && point(0) <= 5 && point(1) >= -5 && point(1) <= 5)
		{
			int o_i = floor((point(0) + 5)*20/10);
			int o_j = floor((point(1) + 5)*20/10);
			if(o_i>19)o_i=19; 
			if(o_j>19)o_j=19;
			obstacle[o_i][o_j] = 1;
			obstacle_count[o_i][o_j] ++;

		}else
		{
			continue;
		}
	}

	for(int i = 0; i < 20; i ++)
	{
		for(int j = 0; j < 20; j ++)
		{
			if(obstacle_count[i][j] < 2)
			{
				obstacle[i][j] = 0;
			}
			printf("%2d",obstacle[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle n;
	ros::Subscriber state_sub = n.subscribe("/mavros/state", 1, state_callback);
	ros::Subscriber status_sub = n.subscribe("/px4/status", 1, px4_status_callback);
	ros::Subscriber pose_sub = n.subscribe("/px4/pose", 1, pose_callback );
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_projected", 1, scanCallback);

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
					px4_autonomy::Position pos_sp;
	                pos_sp.x = 20.0;
	                pos_sp.y = 0;
	                pos_sp.z = 3.0;

					if(isArrived_xy(current_pos, pos_sp))
					{
						while(ros::ok())
						{
							pos_sp_dt.header.stamp = ros::Time::now();
							pos_sp_dt.x = pos_sp.x;
							pos_sp_dt.y = pos_sp.y;
							pos_sp_dt.z = pos_sp.z;
							pos_sp_dt.yaw = yaw_sp;
							pose_pub.publish(pos_sp_dt);

							ros::spinOnce();
							loop_rate.sleep();
						}
					}else
					{
						Vector2f F_rep;
						F_rep(0) = 0;
						F_rep(1) = 0;

						Vector2f F_att;
						F_att(0) = 0;
						F_att(1) = 0;

						Vector2f pos;
						pos(0) = current_pos.x;
						pos(1) = current_pos.y;

						Vector2f target;
						target(0) = pos_sp.x;
						target(1) = pos_sp.y;

						F_att = k_att * (target - pos);

						for(int i = 0; i < 20; i ++)
						{
							for(int j = 0; j < 20; j ++)
							{
								if(obstacle[i][j] == 1)
								{
									Vector2f ob;
									ob(0) = (i+1)*10/20 - 5 - 10/20/2;
									ob(1) = (j+1)*10/20 - 5 - 10/20/2;

									Vector2f ob_w;
									rotate_2D(0, ob, ob_w);

									ob_w = ob_w + pos;

									Vector2f vec1 = ob_w - pos;
									Vector2f vec2 = target - ob_w;

									if(vec1.dot(vec2)/(vec1.norm()*vec2.norm()) > 0.95)
									{
										// ROS_INFO("TEST");
										Vector2f goal_dir = target - pos;
										Vector2f new_dir;
										new_dir(1) = goal_dir(0);
										new_dir(0) = -goal_dir(1);
										if(new_dir.dot(ob_w-pos)>0)
										{
											new_dir(1) = -goal_dir(0);
											new_dir(0) = goal_dir(1);   
										}
										F_rep = F_rep + k_add*new_dir + k_rep*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)/((ob_w-pos).norm()*(ob_w-pos).norm())*(pos-ob_w)*((target-pos).norm()) + 1/2*k_rep*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)*(target-pos)/((target-pos).norm());
									}else
									{

										F_rep = F_rep + k_rep*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)/((ob_w-pos).norm()*(ob_w-pos).norm())*(pos-ob_w)*((target-pos).norm()) + 1/2*k_rep*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)*(1/((ob_w-pos).norm())-1/OBSTACLE_DIST)*(target-pos)/((target-pos).norm());
									}
								}
							}
						}

						Vector2f F;
						F = F_att + F_rep;

						if(first)
						{
							first = false;
							v_dir(0) =  (F/F.norm())(0);
							v_dir(1) =  (F/F.norm())(1);

							double speed = 0.8;
							pos_sp_dt.x = pos_sp_dt.x + v_dir(0)*speed*0.05;
							pos_sp_dt.y = pos_sp_dt.y + v_dir(1)*speed*0.05;
							pos_sp_dt.header.stamp = ros::Time::now();

							if(isArrived_xy(pos_sp_dt,pos_sp))
							{
								pos_sp_dt = pos_sp;
							}
							pose_pub.publish(pos_sp_dt);
						}else
						{
							last_v_dir = v_dir;
							v_dir(0) =  (F/F.norm())(0);
							v_dir(1) =  (F/F.norm())(1);

							double theta = atan2(v_dir(1),v_dir(0));
							double last_theta = atan2(last_v_dir(1),last_v_dir(0));
							double d_theta = theta - last_theta;
							if(d_theta > PI) d_theta = -(2*PI - d_theta);
							if(d_theta < -PI) d_theta = 2*PI + d_theta;
							if(fabs(d_theta > PI/180)) theta = last_theta + d_theta/fabs(d_theta)*PI/180;
							v_dir(0) =  cos(theta);
							v_dir(1) =  sin(theta);

							double speed = 0.8;
							pos_sp_dt.x = pos_sp_dt.x + v_dir(0)*speed*0.05;
							pos_sp_dt.y = pos_sp_dt.y + v_dir(1)*speed*0.05;
							pos_sp_dt.header.stamp = ros::Time::now();

							if(isArrived_xy(pos_sp_dt,pos_sp))
							{
								pos_sp_dt = pos_sp;
							}
							pose_pub.publish(pos_sp_dt);
						}


					}
										
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