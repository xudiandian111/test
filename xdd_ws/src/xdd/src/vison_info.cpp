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
#include <geometry_msgs/Pose2D.h>
#include <cmath>

using namespace std;

mavros_msgs::State current_state;
// geometry_msgs::TwistStamped vel;

geometry_msgs::Pose2D vison_xy_pub;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



int main(int argc, char **argv)
{   
    vison_xy_pub.x = 500;
    vison_xy_pub.y = 500;


    ros::init(argc, argv, "vison_info");
    ros::NodeHandle nh;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    // ros::Subscriber local_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 1000, cur_pos_cb);
    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Publisher vision_pose_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_vison_pose",1);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    // while(ros::ok() && !current_state.connected)
    // {   
    //     vision_pose_pub.publish(vison_xy_pub);
    //     ros::spinOnce();
    //     rate.sleep();

       
    // }

    


    // for(int i = 75; ros::ok() && i > 0; --i)
    // {
    //     // cmd_vel_pub.publish(vel);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();

    // while(ros::ok())
    // {
    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0)))
    //     {
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent)
    //         {
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     }
    //     else
    //     {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(2.0)))
    //         {
    //             if( arming_client.call(arm_cmd) && arm_cmd.response.success)
    //             {
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }
	

    // //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
   
    // // cmd_vel_pub.publish(vel);
    while(ros::ok())
    {
        
    vision_pose_pub.publish(vison_xy_pub);
    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}
