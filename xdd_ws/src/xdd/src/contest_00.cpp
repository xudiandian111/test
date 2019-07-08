/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <cmath>
//#include <vector>
#include <deque>
#include <algorithm>

#include <xdd/PID.h>
#include "xdd/get_aim.h"


#define pi 3.1415926
using namespace std;

std_msgs::Float64 cur_yaw;


PID x, y, z;
PID yaw;

geometry_msgs::Twist vel;
mavros_msgs::State current_state;
bool g_flag_finished = false;

int g_back_aim = 0;
int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_yaw = 0;
int aim_num = 0;
int back_aim_num = 0;
int hold_time = 0;
int z_hold_time = 0;
int px4flow_hold_time = 0;
int pixl_min = 0;

bool mode_change_flag = false;
bool flag_game_is_over = false;
bool flag_is_back_home = true;

std_msgs::Float64  flag_can_start_detect;


//获得飞机当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


//赋XY方向的速度
void set_LinearVel_XY(geometry_msgs::Twist &_vel, float64 lx, float64 ly)
{
    _vel.linear.x = lx;
    _vel.linear.y = ly;
}



//赋角速度
void set_yaw_rate(geometry_msgs::Twist &_vel, float64 yaw)
{
    _vel.angular.z = yaw;
}



//赋Z方向速度
void set_LinearVel_Z(geometry_msgs::Twist &_vel, float64 lz)
{
    _vel.linear.z = lz;
}



//偏航角回调函数
void local_yaw_cb(const std_msgs::Float64::ConstPtr& msg) 
{       
    if(flag_getbegin_yaw == 0)
    {    
        ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(cur_yaw.data);
        
        // yaw.init(0, 2.0, 0, 0);// 暂时不变
        yaw.init(0.0 - cur_yaw.data, 2.0, 0, 0);//暂时设定0
       
        flag_getbegin_yaw = 1; 
    }
    if(flag_getbegin_yaw == 1 && flag_start == 1)
    {     
        yaw.set_curpose(cur_yaw.data);
        yaw.update();
        set_yaw_rate(vel, yaw.out());
        
    }
}


//pos回调函数,通过位置计算并且赋速度，机体坐标系
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
    // *******************四元数转偏航角************************
    {         
        float64 ww = msg->pose.orientation.w;
        float64 xx = msg->pose.orientation.x;
        float64 yy = msg->pose.orientation.y;
        float64 zz = msg->pose.orientation.z;
        cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));
    }

    if(flag_getbegin_pos == 0)
    {    
        ROS_INFO("flag_getbegin_pxflow_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        z.set_beginpose(msg->pose.position.z);
    
        x.init(0.0, 0.5, 0, 0.1);
        y.init(0.0, 0.5, 0, 0.1);
        z.init(0.6, 4.0, 0.0, 0.5);
    
        flag_getbegin_pos = 1;
            
    }



    if(flag_start == 1 && flag_getbegin_pos == 1)
    {   
        //BODY_NED坐标系的区别，将LOCAL_NED坐标转换到BODY_NED
        x.set_curpose(x.read_beginpose() + (msg->pose.position.x - x.read_beginpose()) * cos(  cur_yaw.data)
                        + (msg->pose.position.y - y.read_beginpose()) * sin(  cur_yaw.data));
        y.set_curpose(y.read_beginpose() + (-1)*(msg->pose.position.x - x.read_beginpose()) * sin(  cur_yaw.data)
                        + (msg->pose.position.y - y.read_beginpose()) * cos(  cur_yaw.data));             
        z.set_curpose(msg->pose.position.z);

        if(z.read_curpose() - z.read_beginpose() > 0.3)
        {
            mode_change_flag =true;
        }
        

        if(z.read_curpose() - z.read_beginpose() > 1.5)
        {
            vel.linear.z = 0;
            ROS_INFO("warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!z_curpose:[%lf]",z.read_curpose());
        }

        if(fabs(z.read_err()) < 0.15)
        {
            z_hold_time++;
        }
                
        double AllSetPositionX[] = {0, 1, 1};               //目标0,1,2
        double AllSetPositionY[] = {0, 0, 1};

        double BackPositionX[] = {1, 1, 0};               //目标2 1 0 
        double BackPositionY[] = {1, 0, 0};

        if(z_hold_time > 150)                           //稳定之后，才设定xy目标点位置
        {     
            if(!g_flag_finished)                     
            {

                cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!gooooooo"<<endl;
                x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 3]);
                y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 3]);
                
                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    hold_time++;                                 //到达目标点的稳定时间                  
                }
                if(aim_num > 2)
                {
                    g_flag_finished = true;
                    // g_back_aim = aim_num - 1;
                    hold_time = 0;
                    // aim_num = 0;
                }
                if(hold_time > 50 && aim_num <=2)
                {   
                   aim_num++;
                   hold_time = 0;
                }
            }
            else
            {
                aim_num = 0;
                cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!backkkkkkkk"<<endl;
                x.set_aimpose(x.read_beginpose() + BackPositionX[g_back_aim % 3]);
                y.set_aimpose(y.read_beginpose() + BackPositionY[g_back_aim % 3]);
                
                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    hold_time++;                                 //到达目标点的稳定时间
                }
                if(g_back_aim > 2)
                {
                    flag_game_is_over = true;
                    // g_back_aim = 2;
                }
                if(hold_time > 50 && g_back_aim <= 2)
                {   
                   g_back_aim++;
                   hold_time = 0;
                }
            }
            
        }  
    }   

    x.update();
    y.update();
    z.update();
     
    set_LinearVel_Z(vel, z.out());
    set_LinearVel_XY(vel, x.out(), y.out());

    cout <<"flag_game_is_over:"<< flag_game_is_over <<endl;
    cout << "hold_time:"<<hold_time << endl<<"g_back_aim:"<<g_back_aim<<endl;

    cout<<"mode_change_flag:"<<mode_change_flag<<endl;
    cout << "aim_pose_num:" <<aim_num << endl;
    cout << "back_aim_pose_num:" <<back_aim_num << endl;
    ROS_INFO("px4flow_x:[%lf]", x.read_curpose() - x.read_beginpose());
    ROS_INFO("px4flow_x_aimpose:[%lf]", x.read_aimpose() - x.read_beginpose());
    ROS_INFO("px4_set_x_vel:[%lf]", x.out());
    ROS_INFO("px4flow_y:[%lf]", y.read_curpose() - y.read_beginpose());
    ROS_INFO("px4flow_y_aimpose:[%lf]", y.read_aimpose() - y.read_beginpose());
    ROS_INFO("px4_set_y_vel:[%lf]", y.out());  
    ROS_INFO("z.read_aimpose:[%lf]", z.read_aimpose() - z.read_beginpose());
    ROS_INFO("z.read_curpose:[%lf]", z.read_curpose() - z.read_beginpose());
    ROS_INFO("px4_set_z_vel:[%lf]", z.out());  
    cout << endl;
    cout << endl;
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
    ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float64>
            ("xdd_cur_yaw", 10, local_yaw_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw",10);
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
 
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

    mavros_msgs::SetMode offb_set_mode_two;
    offb_set_mode_two.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( !flag_game_is_over )                           
        {
            if( !mode_change_flag)
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
                            // mode_change_flag = true;            //todo
                        }
                        last_request = ros::Time::now();
                    }
                }
            }
        }

        if(flag_game_is_over)
        {
            if( current_state.mode == "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
            {   
                offb_set_mode.request.custom_mode = "AUTO.LAND";                 //修改为AUTO.LAND模式
                if( set_mode_client.call(offb_set_mode_two) &&
                    offb_set_mode_two.response.mode_sent)
                {   
                    ROS_INFO("flight_has_shut_down");
                }
                last_request = ros::Time::now();
            }
        }
	    
    cmd_vel_pub.publish(vel);  
    cur_yaw_pub.publish(cur_yaw);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;

}

