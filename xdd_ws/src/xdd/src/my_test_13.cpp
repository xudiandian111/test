/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

 //2019-05-01晚上修改
 //BODY_NED坐标系下
 //飞往以机体为参照的四个坐标（1,1）（1,-1）（-1,-1）（-1,1）
 //接收目标点的在图像中的二维坐标信息，无人机当前在图像中的二维坐标
 //用flag_use_data_from 判断用图像中的位置，还是pxflow中的位置
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
#include <xdd/get_aim.h>

#define pi 3.1415926
using namespace std;


#define PICTURE  0
#define PXFLOW   1

int flag_use_data_from = 1;
int accel_time = 0;

std_msgs::Float64 cur_Z;
std::deque<float64> deque_z_data;
std_msgs::Float64 z_to_pub;
std_msgs::Float64 cur_yaw;

geometry_msgs::Pose2D curPoseXY;

// std_msgs::Float64 total_vel;

geometry_msgs::Pose2D xdd_xy_pub;

geometry_msgs::Pose2D error_xy;

PID x, y, z;
PID yaw;

geometry_msgs::PoseStamped cur_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

int flag_getbegin_picture = 0;
int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
int flag_getbegin_yaw = 0;
// int flag_go_back_home = 0;
int back_home_time = 0;
int aim_num = 0;
int hold_time = 0;
int z_hold_time = 0;
bool flag_get_aim_pose = false;
bool client_call = false;

std_msgs::Float64 flag_picture_can_get_aim;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



void set_LinearVel_XY(geometry_msgs::Twist &_vel, float64 lx, float64 ly)
{
    _vel.linear.x = lx;
    _vel.linear.y = ly;
}



void set_yaw_rate(geometry_msgs::Twist &_vel, float64 yaw)
{
    _vel.angular.z = yaw;
}



void set_LinearVel_Z(geometry_msgs::Twist &_vel, float64 lz)
{
    _vel.linear.z = lz;
}



void local_yaw_cb(const std_msgs::Float64::ConstPtr& msg) 
{   
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
        // ROS_INFO("yaw.aimpose:[%lf]", yaw.read_aimpose());
        // ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
        // ROS_INFO("yaw_rate:   [%lf]", yaw.out());
    }
}

 


//获得无人机在图像中的位置
void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	if(flag_use_data_from == PICTURE)	
	{			
		if(flag_getbegin_picture == 0)          //开始时获取当前坐标信息
		{
    	    x.set_beginpose(msg->x);
    	    y.set_beginpose(msg->y);

    	    x.init(0.0, 1.9, 0, 0);            //三个参数分别为aim，Kp和Kd
    	    y.init(0.0, 1.9, 0, 0);

			flag_getbegin_picture = 1;
			ROS_INFO("Using Picture!!!!!!!!!!!!!!!");
		}
		
		if(flag_start == 1 && flag_getbegin_picture == 1)
		{
			x.set_curpose(msg->x);
			y.set_curpose(msg->y);
			
			curPoseXY.x = x.read_curpose();             //此方法：在图像的坐标系下
			curPoseXY.y = y.read_curpose();
    	   
			x.update();
			y.update();

		}
	}

}


//获得目标点在图像中的位置，并给无人机速度，到达该目标点
void aim_pose_cb(const geometry_msgs::Pose2D::ConstPtr& _msg)
{
                                                        //只读取目标点的位置
	ROS_ERROR("set x aim:[%lf]", _msg->x);
	ROS_ERROR("set y aim:[%lf]", _msg->y);

   	if(flag_start == 1 && flag_getbegin_picture == 1)
   	{

		x.set_aimpose(_msg->x);
		y.set_aimpose(_msg->y);
        flag_picture_can_get_aim.data = 0;                  //，已经获得了一个目标，这时候图像不能再获得目标点位置
        
        if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
        {
             hold_time++;                                   //给目标点只需要是四个位置，如果跑到了目标点，
        }                                                   //然后就会返回原始初始坐标，准备接受下一个目标点

        if(hold_time > 150)
        {
            x.set_aimpose(x.read_beginpose());
            y.set_aimpose(y.read_beginpose());
    
            if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
            {
                back_home_time++;
            }
        }
              
        if(back_home_time > 20)
        {
            hold_time = 0;
            back_home_time =0;
            flag_picture_can_get_aim.data = 1;                  //已经回到了初始位置，图像可以继续获得目标点位置
            
        }
        
         //跑定点运动的策略，先不用PID,按照一个较大的速度跑，在运动一定时间之后再用PID控制            
        if(accel_time <= 15)
		{
					
			float64 ax_k;
			float64 ay_k;
			if(x.read_aimpose() == 0)
			{
				ax_k = 0.0;		                          //其实是黑圆中心在动，在图像中，

			}
			else
			{
				ax_k = x.read_aimpose() / fabs(x.read_aimpose());		
			}

			if(y.read_aimpose() == 0)
			{
				ay_k = 0.0;		
			}
			else
			{
				ay_k = y.read_aimpose() / fabs(y.read_aimpose());		
			}

			set_LinearVel_XY(vel, ax_k*6.0, ay_k*6.0);
					
			accel_time++;
		}
		else
		{				
			x.update();
			y.update();

            set_LinearVel_XY(vel, x.out(), y.out());                     //在这里赋速度
		}
	}
}



//Z坐标回调函数，控制飞高度为0.5米，并返回飞机当前高度
void  cur_z_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(flag_getbegin_z == 0)
    {
        z.set_beginpose(msg->pose.position.z);
        z.init(0.70, 2.0, 0.0, 0.0);
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




//  BODY_NED坐标系下
// pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
    if(flag_use_data_from == PXFLOW)
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
            // z.set_beginpose(msg->pose.position.z);
                
            x.init(0.0, 0.5, 0, 0.1);
            y.init(0.0, 0.5, 0, 0.1);
            // z.init(2.0, 2.0, 0, 0.1);                                    //Z必定是要分离出来，单独控制的
            flag_getbegin_pos = 1;
        }

        //在机体坐标系下的位置
        else
        {
            x.set_curpose((msg->pose.position.x - x.read_beginpose()) * cos(cur_yaw.data) +
                            (msg->pose.position.y - y.read_beginpose()) * sin(cur_yaw.data));
            y.set_curpose((-1) * (msg->pose.position.x - x.read_beginpose()) * sin(cur_yaw.data) +
                            (msg->pose.position.y - y.read_beginpose()) * cos(cur_yaw.data));
            // z.set_curpose(msg->pose.position.z);  
        }

        //逆时针方向的四个点,0,1,2,3
        double AllSetPositionX[] = {1,0,-1,0};
        double AllSetPositionY[] = {0,1,0,-1};
        // double AllSetPositionX[] = {1.0,-1.0, -1.0,1.0};
        // double AllSetPositionY[] = {1.0, 1.0,-1.0,-1.0};
        

        if(z.read_err() < 0.10)
        {
            z_hold_time++;
        }

        if(z_hold_time > 20) 
        {     
            cout<<"x.read_curpose()"<<x.read_curpose()-x.read_beginpose()<<endl;
            cout<<"y.read_curpose()"<<y.read_curpose()-x.read_beginpose()<<endl;
            cout<<aim_num % 4 + 1 <<endl;
            
            x.set_aimpose(AllSetPositionX[aim_num % 4]);
            y.set_aimpose(AllSetPositionY[aim_num % 4]);

        
            if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
            {
                hold_time++;
            }

            if(hold_time > 25)
            {
                //回到起始位置
                x.set_aimpose(0.0);
                y.set_aimpose(0.0); 
            
                if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)
                {
                    back_home_time++;
                }
            }
                    
            if(back_home_time > 20)
            {   
                hold_time = 0;
                back_home_time =0;
                aim_num++;
                // flag_use_data_from = PICTURE;
                flag_use_data_from = PXFLOW;
                flag_getbegin_picture = 0;
                flag_get_aim_pose = true;
            }

            xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();           //发布当前的XY位置，相对初始点而言
            xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
            error_xy.x = x.read_err();
            error_xy.y = y.read_err();                                      //发布离目标点的距离
        }
        
        x.update();
        y.update();
    
        set_LinearVel_XY(vel, x.out(), y.out());
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


    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);

    // ros::ServiceServer get_aim_num_client = nh.advertiseService
    //         ("get_aimpose_num", 2, aim_num_cb);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
 
  	// ros::Subscriber local_z_sub = nh.subscribe<sensor_msgs::Range>
    //         ("mavros/px4flow/ground_distance", 2, cur_z_cb);
    
    ros::Subscriber cur_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_z_pos_cb);

	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
    ros::Publisher error_XY_pub = nh.advertise<geometry_msgs::Pose2D>
			("error_XY", 1000);
    ros::Publisher cur_yaw_pub = nh.advertise<std_msgs::Float64>
            ("xdd_cur_yaw",10);
    ros::Publisher can_get_picture_aim_pub = nh.advertise<std_msgs::Float64>
            ("can_get_picture_aim",1);
    

    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);
	ros::Subscriber aim_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("aim_pose", 1, aim_pose_cb);
    // ros::ServiceServer get_aim_num_client = nh.advertiseService
    //         ("get_aimpose_num",is_back_home);
    
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
	
	

    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
    if(z.read_curpose() - z.read_beginpose() > 7.0)
    {
        vel.linear.z = 0;
        // cout<<"警告！！！！z_curpose:"<<z.read_curpose()<<endl;
        ROS_INFO("警告！！！！z_curpose:[%lf]",z.read_curpose());
    }
    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(z_to_pub);
    xdd_poseXY_pub.publish(xdd_xy_pub);
    error_XY_pub.publish(error_xy);
    cur_yaw_pub.publish(cur_yaw);
    can_get_picture_aim_pub.publish(flag_picture_can_get_aim);
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

