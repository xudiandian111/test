/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <iostream>
using namespace std;

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

//#include <mavros_msgs/AttitudeTarget.h>


#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <cmath>

#include <xdd/PID.h>
#include <xdd/EulerAngle.h>

#include <queue>

std_msgs::Float64 cur_Z;
PID x, y, z;



PID yaw;
geometry_msgs::Twist yaw_rate;

geometry_msgs::PoseStamped cur_pos;
//geometry_msgs::PoseStamped aim_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

//mavros_msgs::AttitudeTarget thrust_value;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_yaw = 0;
std::queue<float64> PXFLOW_Pose_Z_data;
int PXFLOW_cnt = 0;
float64 Z_sum = 0;

//无人机状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


//赋XYZ速度
void set_LinearVel_XYZ(geometry_msgs::Twist &vel, float64 lx, float64 ly, float64 lz)
{

    vel.linear.x = lx;
    vel.linear.y = ly;
    vel.linear.z = lz;
    
}


//赋yaw速度
void set_yaw_rate(geometry_msgs::Twist &local_yaw, float64 yaw_rate)
{
    local_yaw.angular.x = yaw_rate;  
}



//获得无人机Z坐标值
//限幅+算数平均值滤波
void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
{   
    PXFLOW_cnt++;
    if(msg->range >= msg->min_range && msg->range <= 1.0)
    {     
       if(PXFLOW_cnt <= 10)     
        {
            cur_Z.data = msg->range - msg->min_range;
            Z_sum += msg->range - msg->min_range;

            //ROS_INFO("get_Z:[%lf]", cur_Z.data);

        }
        else
        {
             PXFLOW_Pose_Z_data.push(msg->range - msg->min_range);
             Z_sum -= PXFLOW_Pose_Z_data.front();        
             PXFLOW_Pose_Z_data.pop();
             Z_sum += msg->range - msg->min_range;
                         
             cur_Z.data = Z_sum / PXFLOW_Pose_Z_data.size();

             //ROS_INFO("get_Z:[%lf]", cur_Z.data);
        }
    }
}  

/*
void thrust_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    msg->thrust = 0.8;
}
*/


//回调函数,计算yaw速度
void local_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) 
{   
   if(flag_getbegin_yaw == 0)
   {    
        ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(msg->yaw);
        cout<< "yaw.read_beginpose():"<<yaw.read_beginpose();
          
        yaw.init(yaw.read_beginpose()+0, 2.0, 0, 0);// 暂时设定yaw的目标角度0度
       
        flag_getbegin_yaw = 1; 

   }
   else
   {     
        yaw.set_curpose(msg->yaw);

        yaw.update();
    
        set_yaw_rate(yaw_rate, yaw.out());

        ROS_INFO("yaw.aimpose:[%lf]", yaw.read_aimpose());
        ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
        ROS_INFO("yaw_rate:   [%lf]", yaw.out());
   }
}





//pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   

   if(flag_getbegin_pos == 0)
   {    
        ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        z.set_beginpose(msg->pose.position.z);
   
        x.init(0.0, 2.0, 0, 0);
        y.init(0.0, 2.0, 0, 0);
        z.init(5.0, 2.0, 0, 0);

        ROS_INFO("z.read_aimpose:[%lf]", z.read_aimpose());
		//ROS_INFO("PXFLOW_Pose_Z_data:[%lf]", PXFLOW_Pose_Z_data.back());
        flag_getbegin_pos = 1; 

   }
   else
   {     
        x.set_curpose(msg->pose.position.x);
        y.set_curpose(msg->pose.position.y);
        z.set_curpose(msg->pose.position.z);

        x.update();
        y.update();
        z.update();

        set_LinearVel_XYZ(vel, x.out(), y.out(),z.out());

       //ROS_INFO("z.aimpose[%lf]", z.read_aimpose());
       //ROS_INFO("z.curpose:[%lf]", z.read_curpose());
        //ROS_INFO("z.out:[%lf]",z.out());

   }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_pos_cb);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
  	ros::Subscriber local_z_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/px4flow/ground_distance", 1000, cur_z_cb);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 1000);  
    ros::Subscriber yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 10, local_yaw_cb);
    ros::Publisher yaw_rate_pub = nh.advertise<geometry_msgs::Twist> 
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    //是否发送到该话题？？？？？？？？？（mavros_msgs / GlobalPositionTarget）
    // ros::Publisher yaw_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>
    //        ("mavros/setpoint_raw/attitude", 100, thrust_cb);
    
            
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);

    vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;

    // wait for FCU connection
   while(ros::ok() && !current_state.connected)
    {   
        //cmd_vel_pub.publish(vel);
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

                }
                last_request = ros::Time::now();
            }
        }
	
	

    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);

    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(cur_Z);
    yaw_rate_pub.publish(yaw_rate);
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

