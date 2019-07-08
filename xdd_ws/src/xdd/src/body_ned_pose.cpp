/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

 //只是发布机体坐标下无人机的位置
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

#include <xdd/PID.h>


#define pi 3.1415926
using namespace std;


std_msgs::Float64 cur_yaw;
geometry_msgs::Pose2D curPoseXY;
geometry_msgs::Pose2D xdd_xy_pub;
PID x, y, z;
geometry_msgs::Twist vel;
mavros_msgs::State current_state;
int flag_getbegin_pos = 0;
int flag_start = 0;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
   
        //四元数转欧拉角
        float64 ww = msg->pose.orientation.w;
        float64 xx = msg->pose.orientation.x;
        float64 yy = msg->pose.orientation.y;
        float64 zz = msg->pose.orientation.z;
        cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));


        //初始化
        if(flag_getbegin_pos == 0)
        {    
            ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
            x.set_beginpose(msg->pose.position.x);
            y.set_beginpose(msg->pose.position.y);
           
                
            flag_getbegin_pos = 1;
        }

        //在机体坐标系下的位置
        else
        {
            x.set_curpose(x.read_beginpose() + (msg->pose.position.x - x.read_beginpose()) * cos(cur_yaw.data) +
                            (msg->pose.position.y - y.read_beginpose()) * sin(cur_yaw.data));
            y.set_curpose(y.read_beginpose() + (-1) * (msg->pose.position.x - x.read_beginpose()) * sin(cur_yaw.data) +
                            (msg->pose.position.y - y.read_beginpose()) * cos(cur_yaw.data));
        }

            xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();           //机体坐标系下，发布当前的XY位置，相对初始点而言
            xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
           
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;
    
    // wait for FCU connection
   while(ros::ok() && !current_state.connected)
    {   
        cmd_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting

    for(int i = 75; ros::ok() && i > 0; --i)
    {
        cmd_vel_pub.publish(vel);
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
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                     flag_start = 1;
                }
                last_request = ros::Time::now();
            }
        }
	
    cmd_vel_pub.publish(vel);

    xdd_poseXY_pub.publish(xdd_xy_pub);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

