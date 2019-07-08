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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <mavros_msgs/PositionTarget.h>

#include <xdd/PID.h>


//PID 参数设置，X Y Z与目标位置的距离
double kp =2.0 , ki = 0, kd = 0.0;
double lastDistanceX = 0, lastDistanceY = 0, lastDistanceZ = 0;
double distanceSumX = 0, distanceSumY = 0, distanceSumZ = 0;

int count =0;
geometry_msgs::PoseStamped curPos;
geometry_msgs::PoseStamped aimPos;
geometry_msgs::PoseStamped next_aim_pos;
mavros_msgs::State current_state;
// geometry_msgs::TwistStamped vel;
geometry_msgs::Twist vel;

int flag_getbegin_yaw = 0;
PID yaw;

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

void set_yaw_rate(geometry_msgs::Twist &_vel, float64 yaw)
{
    _vel.angular.z = yaw;
}

void local_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) 
{   
   if(flag_getbegin_yaw == 0)
   {    
        ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(msg->yaw);
          
        yaw.init(yaw.read_beginpose(), 2.0, 0, 0);// 暂时设定yaw的目标角度0度
       
        flag_getbegin_yaw = 1; 

   }
   else
   {     
        yaw.set_curpose(msg->yaw);
        count++;
        if(count > 200)
            {
                yaw.set_aimpose(yaw.read_curpose()+ 3.14/6);
                 count = 0;
            }

        if(count > 200)
        {

        }

        yaw.update();
    
        set_yaw_rate(vel, yaw.out());

        // ROS_INFO("yaw.aimpose:[%lf]", yaw.read_aimpose());
        // ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
        // ROS_INFO("yaw_rate:   [%lf]", yaw.out());
   }
}
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
    double dx = aimPos.pose.position.x - curPos.pose.position.x;
    double dy = aimPos.pose.position.y - curPos.pose.position.y;
    return sqrt(dx*dx+dy*dy+dz*dz);

    //  ROS_INFO("aim_pos.z[%lf]",aimPos.pose.position.z);
    //     ROS_INFO("aim_pos.x[%lf]",aimPos.pose.position.x);
}


void set_vel(geometry_msgs::PoseStamped aimPos,
             geometry_msgs::PoseStamped curPos)
{  
    double distanceZ = aimPos.pose.position.z - curPos.pose.position.z ;
  
    vel.linear.z =kp *distanceZ + kd * (lastDistanceZ - distanceZ) + ki * distanceSumZ;


    lastDistanceZ = distanceZ;
}


   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::Subscriber local_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_pos_cb);
    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
        ros::Subscriber yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 10, local_yaw_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    aimPos.pose.position.x = 0;
    aimPos.pose.position.y = 0;
    aimPos.pose.position.z = 2.0;
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {   
        // cmd_vel_pub.publish(vel);
         local_pos_pub.publish(aimPos);
        ros::spinOnce();
        rate.sleep();
    }

    
  

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
        // cmd_vel_pub.publish(vel);
         local_pos_pub.publish(aimPos);
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
    // set_vel(aimPos, curPos);


    if(get_distance(aimPos, curPos) <0.4)
    {
        aimPos.pose.position.z +=0.0;
        aimPos.pose.position.x =1.0;
        aimPos.pose.position.y +=0.0;
        // vel.twist.linear.x = 0;
        // vel.twist.linear.y = 0.1;
        //next_aim_pos = next_two_aim_pos;
        
    }

    
     local_pos_pub.publish(aimPos);
    
   

    
    //ROS_INFO("aimPos.pose.position.z:[%lf]", aimPos.pose.position.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}
