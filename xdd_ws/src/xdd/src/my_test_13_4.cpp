/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


 //BODY_NED坐标系下
 //
 //接收目标点的在图像中的二维坐标信息，无人机当前在图像中的二维坐标
 //用flag_use_data_from 来区分，用图像中的位置，还是pxflow中的位置
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
#include <deque>
#include <algorithm>

#include <xdd/PID.h>
#include <xdd/get_aim.h>

#define pi 3.1415926
using namespace std;


#define PICTURE  0
#define PXFLOW   1

int flag_use_data_from = 0;
int accel_time = 0;

std_msgs::Float64 cur_Z;
std::deque<float64> deque_z_data;
std_msgs::Float64 z_to_pub;
std_msgs::Float64 cur_yaw;

geometry_msgs::Pose2D cur_pic_PoseXY;
// geometry_msgs::Pose2D pic_to_body_PoseXY;
// geometry_msgs::PoseStamped local_to_body_pose;
geometry_msgs::Pose2D xdd_xy_pub;

geometry_msgs::Pose2D error_xy;

PID x, y, z;
// PID x_body, y_body;
// PID yaw;

// geometry_msgs::PoseStamped cur_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

int flag_getbegin_picture = 0;
// int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
// int flag_get_body_pose = 0;
// int flag_getbegin_yaw = 0;
int hold_time = 0;
// int px4flow_hold_time = 0;
int z_hold_time = 0;
bool flag_mode_change = false;

int flag_have_got_aim = 0;
int back_home_time = 0;


int aim_num = 0;
// std_msgs::Float64 flag_picture_can_get_aim;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



void set_LinearVel_XY(geometry_msgs::Twist &_vel, float64 lx, float64 ly)
{
    _vel.linear.x = lx;
    _vel.linear.y = ly;
}



void set_LinearVel_Z(geometry_msgs::Twist &_vel, float64 lz)
{
    _vel.linear.z = lz;
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
        z.set_curpose(msg->pose.position.z);
        z_to_pub.data = z.read_curpose();                       //发布当前Z的位置

        z.update();
        set_LinearVel_Z(vel, z.out());
    }
}



//获得无人机在图像中的位置，并且需要将比例修改
void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("Set cur x:[%lf]",(msg->x)/100.0);
    ROS_INFO("Set cur y:[%lf]",(msg->y)/100.0);

	if(flag_use_data_from == PICTURE)	
	{				
		if(flag_getbegin_picture == 0)          //开始时获取当前坐标信息
		{
    	    x.set_beginpose((msg->x)/100.0 );
    	    y.set_beginpose((msg->y)/100.0 );

    	    x.init(0.0, 0.2, 0, 0);            //三个参数分别为aim，Kp和Kd
    	    y.init(0.0, 0.2, 0, 0);             //开始目标点为初始位置

			flag_getbegin_picture = 1;
			ROS_INFO("Using Picture!!!!!!!!!!!!!!!");
		}
		
		if(flag_start == 1 && flag_getbegin_picture == 1)
		{   
            x.set_curpose((msg->x)/100.0);
			y.set_curpose((msg->y)/100.0);

            double AllSetPositionX[] = {1, 0, -1, 0};
            double AllSetPositionY[] = {0, 1, 0, -1};

            if(z.read_err() < 0.10)
            {
                z_hold_time++;
            }

            if(z_hold_time > 150)                           //稳定之后，才设定目标点位置
            {     
                aim_num = 1;            
                
                x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4]);
                y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 4]);
                

                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    hold_time++;
                }

                if(hold_time > 150)
                {
                    x.set_aimpose(x.read_beginpose());       //返回起点
                    y.set_aimpose(y.read_beginpose()); 
                   
                    if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                    {
                        back_home_time++;           //判断返回了起点的时间
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
			// cur_pic_PoseXY.x = x.read_curpose();             //在图像的坐标系下无人机的位置
			// cur_pic_PoseXY.y = y.read_curpose();

            ROS_INFO("pic_curpose_x:[%lf]", x.read_curpose());
    	   	ROS_INFO("pic_curpose_y:[%lf]", y.read_curpose());
    	   	ROS_INFO("pic_aimpose_x:[%lf]", x.read_aimpose());
    	   	ROS_INFO("pic_aimpose_y:[%lf]", y.read_aimpose());
			ROS_INFO("pic_set_X_vel:[%lf]", x.out());
			ROS_INFO("pic_set_Y_vel:[%lf]", y.out());
    	   
			x.update();
			y.update();

            set_LinearVel_XY(vel, x.out(), y.out());          
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
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw", 10);
    ros::Subscriber cur_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_z_pos_cb);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);

    // ros::Publisher local_to_body_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
	// 		("local_to_body_pose", 1000);
    ros::Publisher error_XY_pub = nh.advertise<geometry_msgs::Pose2D>
			("error_XY", 1000);

    ros::Publisher can_get_picture_aim_pub = nh.advertise<std_msgs::Float64>
            ("can_get_picture_aim",1);
    

    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);


    
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
            (ros::Time::now() - last_request > ros::Duration(5.0)) &&
            !flag_mode_change)
            
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                flag_mode_change = true;
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
	
	

    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
    if(z.read_curpose() - z.read_beginpose() > 2.0)
    {
        vel.linear.z = 0;
        // cout<<"警告！！！！z_curpose:"<<z.read_curpose()<<endl;
        ROS_INFO("警告！！！！z_curpose:[%lf]",z.read_curpose());
    }

    // local_to_body_pose_pub.publish(local_to_body_pose);
    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(z_to_pub);
    xdd_poseXY_pub.publish(xdd_xy_pub);
    error_XY_pub.publish(error_xy);
    cur_yaw_pub.publish(cur_yaw);
    // can_get_picture_aim_pub.publish(flag_picture_can_get_aim);
   

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

