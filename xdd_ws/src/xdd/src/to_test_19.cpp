/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

//测试版，把发布速度写在server

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

#define PICTURE  0
#define PXFLOW   1


long long int mytime = 0;




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
// bool flag_can_get_aim = true;
int flag_have_got_aim = 0;
bool client_call = false;
bool mode_chang_flag = false;
bool flag_game_is_over = false;

// bool flag_is_back_home = true;

int count_for_down = 0;


int flag_getbegin_picture = 0;


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
        
        yaw.init(0, 2.0, 0, 0);// 暂时不变
    
        flag_getbegin_yaw = 1; 
    }
    else
    {     
        //cout<<cur_yaw.data<<endl;
        yaw.set_curpose(cur_yaw.data);
        yaw.update();
        set_yaw_rate(vel, yaw.out());
    }
}



//Z坐标回调函数，控制飞高度为0.7米，飞机当前高度
void  cur_z_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(flag_getbegin_z == 0)
    {
        z.set_beginpose(msg->pose.position.z);
        z.init(0.7, 4.0, 0.0, 0.5);
        flag_getbegin_z = 1;
    }
    if(flag_start == 1)
    {
         // 四元数转欧拉角
        float64 ww = msg->pose.orientation.w;
        float64 xx = msg->pose.orientation.x;
        float64 yy = msg->pose.orientation.y;
        float64 zz = msg->pose.orientation.z;
        cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));

        z.set_curpose(msg->pose.position.z);
        z_to_pub.data = z.read_curpose();                       //发布当前Z的位置

        z.update();
        set_LinearVel_Z(vel, z.out());
    }
}

//pos回调函数,通过位置计算并且赋速度，机体坐标系
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
    // if(first_time)
    // { 
    //     can_dectect_ballon.data = 1.0;                          //初始化can_dectect_ballon，最开始是能够开始检测
    //     first_time = false;
    // }
    if(flag_use_data_from == PXFLOW)
    {
    if(flag_getbegin_pos == 0)
    {    
            ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
            x.set_beginpose(msg->pose.position.x);
            y.set_beginpose(msg->pose.position.y);
        
            x.init(0.0, 0.5, 0, 0.1);
            y.init(0.0, 0.5, 0, 0.1);
        
            flag_getbegin_pos = 1;
            
        }
        else
        {   

            //BODY_NED坐标系的区别，将LOCAL_NED坐标转换到BODY_NED
            x.set_curpose(x.read_beginpose() + (msg->pose.position.x - x.read_beginpose()) * cos(0.000 + cur_yaw.data)
                            + (msg->pose.position.y - y.read_beginpose()) * sin(0.000 + cur_yaw.data));
            y.set_curpose(y.read_beginpose() + (-1)*(msg->pose.position.x - x.read_beginpose()) * sin(0.000 + cur_yaw.data)
                            + (msg->pose.position.y - y.read_beginpose()) * cos(0.000 + cur_yaw.data));
            // z.set_curpose(msg->pose.position.z);    
        }


        //逆时针方向的四个点,0,1,2,3
        double AllSetPositionX[] = {1, 0, -1, 0};
        double AllSetPositionY[] = {0, 1, 0, -1};
        // double AllSetPositionX[] = {1.0,-1.0, -1.0,1.0};
        // double AllSetPositionY[] = {1.0, 1.0,-1.0,-1.0};
        

        if(z.read_err() < 0.10)
        {
            z_hold_time++;
        }

        if(z_hold_time > 150) 
        {     
            
            if(flag_have_got_aim == 1)
            {
                // cout<<"x.read_curpose()"<<x.read_curpose()-x.read_beginpose()<<endl;
                // cout<<"y.read_curpose()"<<y.read_curpose()-x.read_beginpose()<<endl;
                // cout<<aim_num % 4 + 1 <<endl;
    
                x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4]);
                y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 4]);
            

                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    hold_time++;
                }

                if(hold_time > 150)
                {
                    x.set_aimpose(0.0);
                    y.set_aimpose(0.0); 
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
                    can_dectect_ballon.data = 1.0;          //要给相机传送的消息，回到初始位置后，can_dectect_ballon置1.0
                    
                    flag_can_get_aim = true;
                    flag_have_got_aim = 0;
                }
            }
    
            xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();
            xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
            error_xy.x = x.read_err();
            error_xy.y = y.read_err(); 
        }
            // z.update();
            // z.set_curpose(msg->pose.position.z);
            // set_LinearVel_Z(vel,z.out());
                    
            x.update();
            y.update();
            // float64 vx = x.out()* std::cos(cur_yaw.data) + y.out()*std::sin(cur_yaw.data);
            // float64 vy = -x.out()* std::sin(cur_yaw.data) + y.out()*std::cos(cur_yaw.data);

            // set_LinearVel_XY(vel, vx ,vy);
            set_LinearVel_XY(vel, x.out(), y.out());
    }

}



//vision_pose回调函数,通过位置计算并且赋速度，机体坐标系 
void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // 初始化
   if(flag_getbegin_picture == 0)
   {    
        ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose((msg->x)/100.0);
        y.set_beginpose((msg->x)/100.0);
        	
        x.init(0.0, 0.5, 0, 0.1);
        y.init(0.0, 0.5, 0, 0.1);
        
        flag_getbegin_picture = 1;
    }
    else
    {
        x.set_curpose((msg->x)/100.0);                  //设定当前位置
        y.set_curpose((msg->x)/100.0);
    }

    //逆时针方向的四个点,0,1,2,3
    double AllSetPositionX[] = {1,0,-1,0};              //设置四个目标点的位置
    double AllSetPositionY[] = {0,1,0,-1};
    
    if(z.read_err() < 0.10)
    {
        z_hold_time++;                      //Z稳定了的时间
    }

    if(z_hold_time > 150) 
    {     
        aim_num = 1;                        //设定目标点的象限
        cout<<"aim_num:" <<aim_num % 4 <<endl;
       
        x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4]);       //设定目标点的位置
        y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 4]);
    
        if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
        {
            hold_time++;                                    //到达目标点后，稳定的时间
        }

        if(hold_time > 150)
        {
            x.set_aimpose(x.read_beginpose());              //设定新的目标点，返回初始位置
            y.set_aimpose(x.read_beginpose()); 
            
            if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
            {
                back_home_time++;               //回到初始位置，稳定的时间
            }
        }
            
        if(back_home_time > 150)
        {   
            hold_time = 0;
            back_home_time =0;
        }
        
        xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();
        xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
        error_xy.x = x.read_err();
        error_xy.y = y.read_err(); 
    }  

        x.update();
        y.update();

        set_LinearVel_XY(vel, x.out(), y.out());


        ROS_INFO("pic_curpose_x:[%lf]", x.read_curpose());
        ROS_INFO("pic_curpose_y:[%lf]", y.read_curpose());
        ROS_INFO("pic_aimpose_x:[%lf]", x.read_aimpose());
        ROS_INFO("pic_aimpose_y:[%lf]", y.read_aimpose());
        ROS_INFO("pic_set_X_vel:[%lf]", x.out());
        ROS_INFO("pic_set_Y_vel:[%lf]", y.out());
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
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
    ros::Subscriber cur_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_z_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
    ros::Publisher error_XY_pub = nh.advertise<geometry_msgs::Pose2D>
			("error_XY", 1000);
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw",10);

    // ros::ServiceServer get_aim_num_client = nh.advertiseService
    //         ("get_aimpose_num", is_back_home);  

    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);                                     //接口名称需要修改
 
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

