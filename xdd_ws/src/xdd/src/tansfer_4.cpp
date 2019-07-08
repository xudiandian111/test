/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

// 用std_msgs::Float64  flag_can_start_detect 来publish无人机能否开始检测气球，1.0:能，0.0不能

//  这种方法并没有在飞机回到起点，将res.is_back_home改为1,在下次调用时，才会知道飞机已经回到初始点
//  可以发布一个话题，每次接收到目标点，is_back_home为false,回到目标点之后,is_back_home为true

//  黑圆坐标系下
//  飞往以黑圆为参照的四个坐标（1,1）（1,-1）（-1,-1）（-1,1）
//  添加了用服务发布目标点，//降落功能
//  从图像接受的信息，现在是一个目标点num，和黑圆的位置（像素点，用于定位）

// z的位置这里用中值滤波
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>

#include <cmath>
//#include <vector>
#include <deque>
#include <algorithm>

#include <xdd/PID.h>
#include "xdd/get_aim.h"
#include "xdd/TF.h"

#include <std_srvs/Trigger.h>

#define pi 3.1415926
using namespace std;
// int my_test_server_count = 0;

std_msgs::Float64 cur_Z;
std::deque<float64> deque_z_data;
std_msgs::Float64 z_to_pub;
std_msgs::Float64 cur_yaw;

#define PICTURE  0
#define PXFLOW   1

int flag_use_data_from = 0;

// std_msgs::Float64 total_vel;

// geometry_msgs::Pose2D pose_2d;

geometry_msgs::Pose2D xdd_xy_pub;
geometry_msgs::Pose2D error_xy;

// geometry_msgs::PoseStamped pose_info;

PID x, y, z;
PID px4_x, px4_y;
PID yaw;

// geometry_msgs::PoseStamped cur_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

double z_first_aim = 0.6;
double z_second_aim = 0.6;
int flag_getbegin_picture = 0;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
int flag_getbegin_yaw = 0;

int back_home_time = 0;
int aim_num = 2;
int hold_time = 0;
int z_hold_time = 0;
int px4flow_hold_time = 0;

int pixl_min = 0;

bool flag_have_got_aim = false;

bool mode_change_flag = false;
bool flag_game_is_over = false;

bool flag_is_back_home = true;
bool first_detect = true;


//回到初始点检测能否开启相机，设置的标志位
bool  flag_can_start_detect = false;
int high_stable_time = 0;
int pose_hold_time = 0;
std_msgs::Bool msg_bool;

xdd::TF tf_call;

//定义的全局图像接口
ros::Publisher can_start_detect_pub;
ros::ServiceClient call_detect_client;

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
        
        yaw.init(0, 2.0, 0, 0);// 暂时不变
        // yaw.init(0.0 - cur_yaw.data, 2.0, 0, 0);//暂时设定0
    
        flag_getbegin_yaw = 1; 
    }
    if(flag_getbegin_yaw == 1 && flag_start == 1)
    {     
        yaw.set_curpose(cur_yaw.data);
        yaw.update();
        set_yaw_rate(vel, yaw.out());
    }
}



// server接口，获得图像中目标点number的回调函数
bool is_back_home_cb(xdd::TF::Request &req, xdd::TF::Response &res)
{      
    cout << "进入server!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    aim_num = int(req.tf);       
    flag_have_got_aim = true;
    cout << "!!!!!!!!!!!!!!!!!!!aim_pose_num:" << req.tf << endl;
    flag_is_back_home =false;
    // flag_can_start_detect = false;	    //获得目标点之后将次标志位置0
    // flag_use_data_from = PICTURE;
    return true;
}


//图像调用的server
void call_detect_cb(xdd::TF::Request &req, xdd::TF::Response & res)
{
    // aim_num = int(req.tf);
}



//Z坐标回调函数，控制飞高度为0.7+0.15米，飞机当前高度
void  cur_z_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(flag_getbegin_z == 0)
    {
        z.set_beginpose(msg->pose.position.z);
        z.init(0.70, 5.0, 0.0, 0.1);
        flag_getbegin_z = 1;
    }

    //开始第一次检测
    if(first_detect)
    {
        if(z.read_err() < 0.10)
        {
            z_hold_time++;
        }

        if(z_hold_time > 50)
        {
            flag_can_start_detect = true;
            first_detect = false;

            //第一次发布开始检测true
            if(flag_can_start_detect)
            {                           
                call_detect_client.call(tf_call);           // call 服务端,    trigger

                msg_bool.data = flag_can_start_detect;
                can_start_detect_pub.publish(msg_bool);     //  话题发布能否开始检测的命令
                flag_can_start_detect = false;
            }
        }
    }

    if(flag_start == 1)
    {
        z.set_curpose(msg->pose.position.z);
        z.update();
        set_LinearVel_Z(vel, z.out());
    }

   // *********************中值滤波,发布当前的高度*****************
    cur_Z.data = msg->pose.position.z;
    if(deque_z_data.size() <= 20)
    {
        deque_z_data.push_back(cur_Z.data);
    }
    else
    {
        deque_z_data.pop_front();
        deque_z_data.push_back(cur_Z.data);
    }

    std::sort(deque_z_data.begin(),deque_z_data.end());
    int mid = deque_z_data.size() / 2;

    z_to_pub.data = deque_z_data.at(mid);                  


    // *******************四元数转偏航角************************      
    float64 ww = msg->pose.orientation.w;
    float64 xx = msg->pose.orientation.x;
    float64 yy = msg->pose.orientation.y;
    float64 zz = msg->pose.orientation.z;
    cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));


    // **************************防止Z高度大于1.5米*************
    if(z.read_curpose() - z.read_beginpose() > 1.5)
    {
        vel.linear.z = 0;
        ROS_INFO("warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!z_curpose:[%lf]",z.read_curpose());
    }
}


void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
//***********************只在看不到图像的时候用光流的信息**************
    if(msg->x < pixl_min || msg->y < pixl_min)
    {
        flag_use_data_from = PXFLOW;
    }
   
    if(flag_use_data_from == PICTURE)
    {   
        if(flag_getbegin_picture == 0)          //开始时获取当前坐标信息
        {
            x.set_beginpose((msg->x)/200 );
            y.set_beginpose((msg->y)/200 );

            x.init(0.0, 0.5, 0, 0.1);            //三个参数分别为aim，Kp和Kd
            y.init(0.0, 0.5, 0, 0.1);             //开始目标点为初始位置

            flag_getbegin_picture = 1;

            ROS_INFO("Using Picture!!!!!!!!!!!!!!!");
	    }
		
        if(flag_start == 1 && flag_getbegin_picture == 1)
        {   
            // ROS_INFO("Using Picture!!!!!!!!!!!!!!!");
            x.set_curpose((msg->x)/200);
        	y.set_curpose((msg->y)/200);

            double AllSetPositionX[] = {0, 1, 0, -1, 0};
            double AllSetPositionY[] = {0, 0, 1, 0, -1};

     
            if(flag_have_got_aim)                          //首先需要已经获得xy目标点位置aim_num
            {   
                x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 5]);
                y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 5]);
                
                // ×××××××××××××××××××××××××××××××××××××××××××××××××××××××
                // 飞一个轮回
                // 用于判断飞机是否到达目标点
                if(fabs(x.read_err()) <= 0.3 && fabs(y.read_err()) <= 0.3)
                {
                    hold_time++;                                 //到达目标点的稳定时间
                }
                if(hold_time > 50)
                {
                    x.set_aimpose(x.read_beginpose());          //设定新的目标点为初始位置，返回起点
                    y.set_aimpose(y.read_beginpose()); 
                
                    if(fabs(x.read_err()) <= 0.30 && fabs(y.read_err()) <= 0.30)
                    {
                        back_home_time++;                       //判断返回了起点的时间
                    }
                }
                if(back_home_time > 50)                        //reset重置标志位，重置稳定时间
                {   
                    hold_time = 0;
                    back_home_time =0;

                    flag_have_got_aim = false;
                    flag_can_start_detect = true;

                    if(flag_can_start_detect)
                    {                           
                        call_detect_client.call(tf_call);           // call 服务端

                        msg_bool.data = flag_can_start_detect;
                        can_start_detect_pub.publish(msg_bool);     //  话题发布能否开始检测的命令
                        flag_can_start_detect = false;
                    }
                    
                        // flag_is_back_home = true;
                        // flag_getbegin_picture = 0;//每次重回黑圆下方之后，重置初始坐标点
                }
                    
            }  

			x.update();
			y.update();

            set_LinearVel_XY(vel, x.out(), y.out());  
            

            //其他情况下开始检测的话题一直发布false
            if(!flag_can_start_detect)
            {
                msg_bool.data = false;
                can_start_detect_pub.publish(msg_bool);
            }
            
		}
	}
}



//pos回调函数,通过位置计算并且赋速度，机体坐标系
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
//无论如何先初始化
    if(flag_getbegin_pos == 0)
    {    
        ROS_INFO("flag_getbegin_pxflow_pos:OK!!!!!!!!");
        px4_x.set_beginpose(msg->pose.position.x);
        px4_y.set_beginpose(msg->pose.position.y);
    
        px4_x.init(0.0, 0.5, 0, 0.1);
        px4_y.init(0.0, 0.5, 0, 0.1);            
        flag_getbegin_pos = 1;                
    }


    if(flag_use_data_from == PXFLOW)
    {
        if(flag_getbegin_pos == 1 && flag_start == 1)
        {   
            //BODY_NED坐标系的区别，将LOCAL_NED坐标转换到BODY_NED
            px4_x.set_curpose(px4_x.read_beginpose() + (msg->pose.position.x - px4_x.read_beginpose()) * cos( cur_yaw.data)
                            + (msg->pose.position.y - px4_y.read_beginpose()) * sin( cur_yaw.data));
            px4_y.set_curpose(px4_y.read_beginpose() + (-1)*(msg->pose.position.x - px4_x .read_beginpose()) * sin( cur_yaw.data)
                            + (msg->pose.position.y - px4_y.read_beginpose()) * cos( cur_yaw.data));             


            if(fabs(z.read_err()) < 0.15  && fabs(px4_x.read_err()) < 0.15 && fabs(px4_y.read_err()) < 0.15)
            {
                px4flow_hold_time++;
            }

            if(px4flow_hold_time > 20)
            {
                //给图像发布可以检测的命令
                flag_can_start_detect = true;
                flag_use_data_from = PICTURE;
                px4flow_hold_time = 0;
            }
                            
            px4_x.update();
            px4_y.update();
            set_LinearVel_XY(vel, px4_x.out(), y.out());

 	    }
    }
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
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
    // ros::Publisher error_XY_pub = nh.advertise<geometry_msgs::Pose2D>
	// 		("error_XY", 1000);
            
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw",10);
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("/msg_bridge/black_point", 1, vision_pose_cb);
    ros::Subscriber cur_z_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_z_pos_cb);

    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
    
    //新加入的
    ros::ServiceServer get_aim_num_service = nh.advertiseService
            ("get_aimpose_num", is_back_home_cb);
    //全局变量

    can_start_detect_pub = nh.advertise<std_msgs::Bool>
			("xdd/can_start_detect", 1000);
    call_detect_client = nh.serviceClient<xdd::TF>
            ("xdd/TF_call");  

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

    xdd::TF TF_call;
    TF_call.request.tf = 1;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode offb_set_mode_two;
    offb_set_mode_two.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {

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
                        mode_change_flag = true;            //todo
                    }
                    last_request = ros::Time::now();
                }
            }
        }
     
	    
    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(z_to_pub);
    // xdd_poseXY_pub.publish(xdd_xy_pub);
    // error_XY_pub.publish(error_xy);
    cur_yaw_pub.publish(cur_yaw);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

