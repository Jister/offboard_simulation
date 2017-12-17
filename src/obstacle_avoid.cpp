#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"

#include <sensor_msgs/LaserScan.h>

#include "Eigen/Dense"

#define OBSTACLE_DIST 5.0

using namespace Eigen;

mavros_msgs::State current_state;
int obstacle[20][20]={0};
int obstacle_count[20][20]={0};
double k_att = 10.0;
double k_rep = 10.0;
double k_add = 1.0;

Vector3f local_pos(0.0,0.0,0.0);
Vector3f pos_sp(0.0,0.0,0.0);
Vector3f goal(30.0,0.0,2.0);
double yaw = 0;

bool takeoff_ready = false;
bool _reset_pos = true;


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

void reset_sp(const Vector3f& pos,  Vector3f& setpoint)
{
	setpoint(0) = pos(0);
	setpoint(1) = pos(1);
}


void pose_cb(const geometry_msgs::PoseStamped msg){
    local_pos(0) = msg.pose.position.x;
    local_pos(1) = msg.pose.position.y;
    local_pos(2) = msg.pose.position.z;

    yaw = tf::getYaw(msg.pose.orientation);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
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

bool isArrived(Vector3f& local, Vector3f& goal)
{
    if((local - goal).norm() < 0.2)
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
    ros::init(argc, argv, "off_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_projected", 1, scanCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped takeoff;
    Vector3f takeoff_v;
    takeoff.pose.position.x = 0;
    takeoff.pose.position.y = 0;
    takeoff.pose.position.z = 2;
    takeoff_v(0) = 0;
    takeoff_v(1) = 0;
    takeoff_v(2) = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(takeoff);
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


        if(!takeoff_ready)
        {
            local_pos_pub.publish(takeoff);
            if(isArrived(local_pos, takeoff_v))
            {
                takeoff_ready = true;
            }
        }else
        {
        	if(_reset_pos)
        	{
        		reset_sp(local_pos,pos_sp);
        		pos_sp(2) = 2;
        		_reset_pos = false;
        	}else
        	{

        	}
            if(isArrived(pos_sp, goal))
            {
            	while(ros::ok())
            	{
	                geometry_msgs::PoseStamped stop;
	                stop.pose.position.x = goal(0);
	                stop.pose.position.y = goal(1);
	                stop.pose.position.z = goal(2);
	                local_pos_pub.publish(stop);

	                ros::spinOnce();
        			rate.sleep();
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
                pos(0) = local_pos(0);
                pos(1) = local_pos(1);

                Vector2f target;
                target(0) = goal(0);
                target(1) = goal(1);

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


						    //printf("   %2.1f %2.1f",ob(0),ob(1));   

                            Vector2f ob_w;
                            rotate_2D(0, ob, ob_w);

                            ob_w = ob_w + pos;

                            Vector2f vec1 = ob_w - pos;
                            Vector2f vec2 = target - ob_w;

                            if(vec1.dot(vec2)/(vec1.norm()*vec2.norm()) > 0.90)
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
                        }else
                        {
                        	//printf("   *   *  ");
                        }
                    }
                    //printf("\n");
                }
                //printf("\n");

                Vector2f F;
                F = F_att + F_rep;

                // geometry_msgs::TwistStamped cmd;
                // double speed = 0.5;

                // cmd.twist.linear.x = (F/F.norm())(0)*speed;
                // cmd.twist.linear.y = (F/F.norm())(1)*speed;
                // cmd.twist.linear.z = 0;
                // cmd.twist.angular.x = 0;
                // cmd.twist.angular.y = 0;
                // cmd.twist.angular.z = 0;
                // local_vel_pub.publish(cmd);
                double speed = 0.8;
                pos_sp(0) = pos_sp(0) + (F/F.norm())(0)*speed*0.05;
                pos_sp(1) = pos_sp(1) + (F/F.norm())(1)*speed*0.05;

                geometry_msgs::PoseStamped pos_set;
                pos_set.pose.position.x = pos_sp(0);
                pos_set.pose.position.y = pos_sp(1);
                pos_set.pose.position.z = pos_sp(2);
                local_pos_pub.publish(pos_set);
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
