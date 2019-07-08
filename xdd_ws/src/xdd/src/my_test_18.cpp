/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

//

 //这种方法并没有在飞机回到起点，将res.is_back_home改为1,在下次调用时，才会知道飞机已经回到初始点
 //可以发布一个话题，每次接收到目标点，is_back_home为false,回到目标点之后,is_back_home为true
 //2019-05-07,10修改
 //BODY_NED坐标系下
 //飞往以机体为参照的四个坐标（1,1）（1,-1）（-1,-1）（-1,1）
 //添加了用服务发布目标点，降落功能
 //从图像接受的信息，现在只是一个目标点num
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
int my_test_server_count = 0;

std_msgs::Float64 cur_Z;
std::deque<float64> deque_z_data;
std_msgs::Float64 z_to_pub;
std_msgs::Float64 cur_yaw;

// std_msgs::Float64 total_vel;

geometry_msgs::Pose2D xdd_xy_pub;

geometry_msgs::Pose2D error_xy;

geometry_msgs::PoseStamped pose_info;

PID x, y, z;
PID yaw;

geometry_msgs::PoseStamped cur_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
int flag_getbegin_yaw = 0;
// int flag_go_back_home = 0;
int back_home_time = 0;
int aim_num = 0;
int hold_time = 0;
int z_hold_time = 0;
bool flag_can_get_aim = true;
int flag_have_got_aim = 0;
bool client_call = false;
bool mode_chang_flag = false;
bool flag_game_is_over = false;

bool flag_is_back_home = true;

int count_for_down = 0;





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
    // cur_yaw.data = msg->data;
    // ROS_INFO("cur_yaw[%lf]", cur_yaw.data);
   
    // ROS_INFO("cur_yaw:[%lf]", yaw.read_aimpose());
    if(flag_getbegin_yaw == 0)
    {    
        ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(cur_yaw.data);
        
        // yaw.init(0, 2.0, 0, 0);// 暂时不变
        yaw.init(0.0 - cur_yaw.data, 2.0, 0, 0);//暂时设定0
        // yaw.init(0.0 - cur_yaw.data +2*pi/4.0 , 2.0, 0, 0);//暂时设定90
    
        flag_getbegin_yaw = 1; 
    }
    else
    {     
        //cout<<cur_yaw.data<<endl;
        yaw.set_curpose(cur_yaw.data);
        yaw.update();
        set_yaw_rate(vel, yaw.out());
        // ROS_INFO("yaw.aimpose:[%lf]", yaw.read_aimpose());
        // ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
        // ROS_INFO("yaw_rate:   [%lf]", yaw.out());
    }
}


//server其实是为了进行姿态解算，或者目标识别，这种一次性但是需要较长时间处理而产生的，
//server接口，获得图像中目标点number的回调函数
//可以有多个client,但只有一个server
bool is_back_home(xdd::get_aim::Request &req, xdd::get_aim::Response &res)
{   
    // my_test_server_count++;
    // cout <<"my_test_server_count_num:" << my_test_server_count << endl;

    cout << "aim_pose_num:" << req.aim_num << endl;
    cout << "response:" << res.is_back_home << endl;
    // cout<<"flag_can_get_aim:"<<flag_can_get_aim<<endl;
    if(flag_can_get_aim)
    {   
        // client_call = true;
        cout << "进去了" << endl;
        aim_num = req.aim_num;
        flag_have_got_aim = 1;
        flag_can_get_aim = false;

        flag_is_back_home =false;
        res.is_back_home = 1;                         
        return true;
    }
    return false; 
}



//pos回调函数,通过位置计算并且赋速度，机体坐标系 
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
    // 四元数转欧拉角
    float64 ww = msg->pose.orientation.w;
    float64 xx = msg->pose.orientation.x;
    float64 yy = msg->pose.orientation.y;
    float64 zz = msg->pose.orientation.z;
    cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));


   if(flag_getbegin_z == 0)
   {    
        ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        z.set_beginpose(msg->pose.position.z);
        	
        x.init(0.0, 0.5, 0, 0.1);
        y.init(0.0, 0.5, 0, 0.1);
        z.init(2.0, 2.0, 0, 0.1);
        flag_getbegin_z = 1;
        flag_getbegin_pos = 1;
        flag_can_get_aim = true;
    }

    else
    {
        x.set_curpose(x.read_beginpose() + (msg->pose.position.x - x.read_beginpose())*cos(0.000+ cur_yaw.data)
                        +(msg->pose.position.y - y.read_beginpose())*sin(0.000+ cur_yaw.data));
        y.set_curpose(y.read_beginpose() + (-1)*(msg->pose.position.x - x.read_beginpose())*sin(0.000+ cur_yaw.data)
                        +(msg->pose.position.y - y.read_beginpose())*cos(0.000+ cur_yaw.data));
        z.set_curpose(z.read_beginpose() + msg->pose.position.z);    
    }


    //逆时针方向的四个点,0,1,2,3
    double AllSetPositionX[] = {1,0,-1,0};
    double AllSetPositionY[] = {0,1,0,-1};
    

    if(z.read_err() < 0.10)
    {
        z_hold_time++;
    }

    if(z_hold_time > 150) 
    {     
        // cout<<"x.read_curpose()"<<x.read_curpose()-x.read_beginpose()<<endl;
        // cout<<"y.read_curpose()"<<y.read_curpose()-x.read_beginpose()<<endl;
        // cout<<aim_num % 4 + 1 <<endl;
       
        if(flag_have_got_aim == 1)
        {
            x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4]);
            y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 4]);
            

            if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
            {
                hold_time++;
            }

            if(hold_time > 150)
            {
                

                x.set_aimpose(x.read_beginpose());
                y.set_aimpose(y.read_beginpose()); 
                // 回来
                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    back_home_time++;
                }
            }
                
            if(back_home_time > 150)
            {   
                hold_time = 0;
                back_home_time =0;

                flag_can_get_aim = true;
                flag_have_got_aim = 0;

                flag_is_back_home = true;
            }
        }
   
        xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();
        xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
        error_xy.x = x.read_err();
        error_xy.y = y.read_err(); 
    }
        z.update();
        set_LinearVel_Z(vel,z.out());
                
        x.update();
        y.update();

        set_LinearVel_XY(vel, x.out(), y.out());
}

// void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
// {

// }

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


    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
 
  	// ros::Subscriber local_z_sub = nh.subscribe<sensor_msgs::Range>
    //         ("mavros/px4flow/ground_distance", 2, cur_z_cb);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
    ros::Publisher error_XY_pub = nh.advertise<geometry_msgs::Pose2D>
			("error_XY", 1000);
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw",10);

    ros::ServiceServer get_aim_num_client = nh.advertiseService
            ("get_aimpose_num", is_back_home);  
 
	// ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
	// 		("vision_pose", 1, vision_pose_cb);
	// ros::Subscriber aim_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
	// 		("aim_pose", 1, aim_pose_cb);

    
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

    bool mode_chang_flag = false;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( !flag_game_is_over )                            //可以用相机检测，如果气球的数目为零，flag_game_is_over = true
        {
            if( !mode_chang_flag)
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
            }
        }

        else
        {
            if( current_state.mode == "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
            {   
                // offb_set_mode.request.custom_mode = "AUTO.LAND";                 //修改为AUTO.LAND模式
                if( set_mode_client.call(offb_set_mode_two) &&
                    offb_set_mode_two.response.mode_sent)
                {   
                    mode_chang_flag = true;
                    ROS_INFO("flight_has_shut_down");
                }
                last_request = ros::Time::now();
            }
        }
	

    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
    if(z.read_curpose() - z.read_beginpose() > 5.0)
    {
        vel.linear.z = 0;
        // cout<<"警告！！！！z_curpose:"<<z.read_curpose()<<endl;
        ROS_INFO("warning!!!!!!!!!!z_curpose:[%lf]",z.read_curpose());
    }
    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(z_to_pub);
    xdd_poseXY_pub.publish(xdd_xy_pub);
    error_XY_pub.publish(error_xy);
    cur_yaw_pub.publish(cur_yaw);
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

