/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

using namespace std;
//PID 参数设置，X Y Z与目标位置的距离
double kp =2.0 , ki = 0, kd = 0.0;
double lastDistanceX = 0, lastDistanceY = 0, lastDistanceZ = 0;
double distanceSumX = 0, distanceSumY = 0, distanceSumZ = 0;


geometry_msgs::PoseStamped curPos;
geometry_msgs::PoseStamped aimPos;
geometry_msgs::PoseStamped next_aim_pos;
mavros_msgs::State current_state;
geometry_msgs::TwistStamped vel;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


//回调函数， 返回无人机现在的位置
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    curPos.pose.position.x = msg->pose.position.x;
    curPos.pose.position.y = msg->pose.position.y;
    curPos.pose.position.z = msg->pose.position.z;
}

void poin2i_bridge_cb 


/*
void set_aim_pos(geometry_msgs::PoseStamped next_aim_pos)
{
    aimPos.pose.position.x =next_aim_pos.pose.position.x;
    aimPos.pose.position.y =next_aim_pos.pose.position.y;
    aimPos.pose.position.z =next_aim_pos.pose.position.z;
}
*/

double get_distance(geometry_msgs::PoseStamped aimPos,geometry_msgs::PoseStamped curPos)
{
    double dz = aimPos.pose.position.z - curPos.pose.position.z;
    return dz;
}


void set_vel(geometry_msgs::PoseStamped aimPos,
             geometry_msgs::PoseStamped curPos)
{  
    double distanceZ = aimPos.pose.position.z - curPos.pose.position.z ;
  
    vel.twist.linear.z =kp *distanceZ + kd * (lastDistanceZ - distanceZ) + ki * distanceSumZ;


    lastDistanceZ = distanceZ;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~my_namespace");
    pn.param<double>("kp",kp, 2.0);

    pn.setParam("kp",1.0);
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::Subscriber local_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber point2i_sub = nh.subscribe<xdd::point2i>
            ("/detect_balck/Detect_Color/black_point",poin2i_bridge_cb);
    

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {   
        cmd_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();

       
    }

    
    aimPos.pose.position.x = 0;
    aimPos.pose.position.y = 0;
    aimPos.pose.position.z = 2;

    // aimPos.pose.position.x = -5;
    // aimPos.pose.position.y = -5;
    // aimPos.pose.position.z = -5;
    // vel.twist.linear.z = 1;
    // vel.twist.linear.x = 1;
    // vel.twist.linear.y = 1;
    // next_aim_pos.pose.position.x = -2;
    // next_aim_pos.pose.position.y = -2;
    // next_aim_pos.pose.position.z = -2;
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
                }
                last_request = ros::Time::now();
            }
        }
	

    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
    cout<<"get_distance:"<<get_distance(aimPos, curPos)<<std::endl;
    if(get_distance(aimPos, curPos) <0.2)
    {
        // aimPos.pose.position.z = 2;
        // vel.twist.linear.x = 0;
        //  vel.twist.linear.z = 0.1;
        aimPos.pose.position.x += 0.0;
        aimPos.pose.position.y += 1.0;

        set_vel(aimPos, curPos);
        cout<<" aimPos.pose.position.y_1:"<< aimPos.pose.position.y<<std::endl;
        //next_aim_pos = next_two_aim_pos;
    }
    else
    {
         set_vel(aimPos, curPos);
    }
     pn.getParam("kp",kp);
    std::cout<<"kp_value:"<<kp<<std::endl;
    cmd_vel_pub.publish(vel);
    ROS_INFO("aimPos.pose.position.z:[%lf]", aimPos.pose.position.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}
