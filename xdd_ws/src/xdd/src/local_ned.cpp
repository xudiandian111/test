/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

 //2019-04-24晚上修改
 //LOCAL_NED坐标系下
 //飞往以机体为参照的四个坐标（1,1）（1,-1）（-1,-1）（-1,1）
 //OK的版本
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <cmath>
//#include <vector>
#include <deque>
#include <algorithm>

#include <xdd/PID.h>


using namespace std;

std_msgs::Float64 cur_Z;
std::deque<float64> deque_z_data;
std_msgs::Float64 z_to_pub;
std_msgs::Float64 cur_yaw;

// std_msgs::Float64 total_vel;

geometry_msgs::Pose2D xdd_xy_pub;
geometry_msgs::Pose2D error_xy;

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
double pi = 3.1415926;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



void set_LinearVel_XY(geometry_msgs::Twist &vel, float64 lx, float64 ly)
{
    vel.linear.x = lx;
    vel.linear.y = ly;
}



void set_yaw_rate(geometry_msgs::Twist &vel, float64 yaw)
{
    vel.angular.z = yaw;
}



void set_LinearVel_Z(geometry_msgs::Twist &vel, float64 lz)
{
    vel.linear.z = lz;
}


// void local_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) 
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
        // yaw.init(0.0 - cur_yaw.data, 2.0, 0, 0);//暂时设定90
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


//pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   
    //四元数转欧拉角
    float64 ww = msg->pose.orientation.w;
    float64 xx = msg->pose.orientation.x;
    float64 yy = msg->pose.orientation.y;
    float64 zz = msg->pose.orientation.z;
    cur_yaw.data = atan2(2 * (ww * zz + xx * yy) , 1 - 2 * (zz * zz + yy * yy));
    // cur_yaw.data += 3.1415926 / 4.0 ;

    //LOCAL_NED坐标系和BODY_NED的YAW方向的差别
    cout<<" cur_yaw.data:"<< cur_yaw.data<<endl;

   if(flag_getbegin_pos == 0)
   {    
        ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        z.set_beginpose(msg->pose.position.z);
        	
        x.init(0.0, 0.5, 0, 0.2);
        y.init(0.0, 0.5, 0, 0.2);
        z.init(2.0, 2.0, 0, 0.2);
        flag_getbegin_pos = 1;
    }

    else
    {
        x.set_curpose(msg->pose.position.x);
        y.set_curpose(msg->pose.position.y);
        z.set_curpose(msg->pose.position.z);

        cout<<"x.read_aimpose()"<<x.read_aimpose()<<endl;
        cout<<"y.read_aimpose()"<<y.read_aimpose()<<endl;
        cout<<aim_num % 4 + 1 <<endl;
    }


    //逆时针方向的四个点,0,1,2,3
    // double AllSetPositionX[] = {1,0,-1,0};
    // double AllSetPositionY[] = {0,1,0,-1};
    double AllSetPositionX[] = {1.0,-1.0, -1.0,1.0};
    double AllSetPositionY[] = {1.0, 1.0,-1.0,-1.0};
    

    if(z.read_err() < 0.10)
    {
        z_hold_time++;
    }

    if(z_hold_time > 150) 
    {     
        // cout<<cos(cur_yaw.data)<<endl;

        // x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4]);
        // y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 4]);

        x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 4] * cos(cur_yaw.data + 0.000) + 
                        AllSetPositionY[aim_num % 4] * sin(cur_yaw.data + 0.000));
        y.set_aimpose(y.read_beginpose() - AllSetPositionX[aim_num % 4] * sin(cur_yaw.data + 0.000) + 
                        AllSetPositionY[aim_num % 4] * cos(cur_yaw.data + 0.000));

        x.set_curpose(msg->pose.position.x);
        y.set_curpose(msg->pose.position.y);
       

        if(x.read_err() <= 0.15 && y.read_err() <= 0.15)
        {
             hold_time++;
        }

        if(hold_time > 150)
        {
            x.set_aimpose(x.read_beginpose());
            y.set_aimpose(y.read_beginpose());
                    
            if(x.read_err() <= 0.15 && y.read_err()<= 0.15)
            {
                back_home_time++;
            }
        }
              
        if(back_home_time > 150)
        {
            hold_time = 0;
            back_home_time =0;
            aim_num++;
        }
   
        xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();
        xdd_xy_pub.y = y.read_curpose() - y.read_beginpose(); 
        error_xy.x = x.read_err();
        error_xy.y = y.read_err(); 
    }
        z.update();
        z.set_curpose(msg->pose.position.z);
        set_LinearVel_Z(vel,z.out());
                
        x.update();
        y.update();
        // float64 vx = x.out()* std::cos(cur_yaw.data) + y.out()*std::sin(cur_yaw.data);
        // float64 vy = -x.out()* std::sin(cur_yaw.data) + y.out()*std::cos(cur_yaw.data);

        // set_LinearVel_XY(vel, vx ,vy);
        ROS_INFO("x_chazhi[%lf],y_chazhi[%lf]",x.read_curpose() - x.read_beginpose(),y.read_curpose() - y.read_beginpose());
        ROS_INFO("x.curpose:[%lf],y.curpose:[%lf]",x.read_curpose(),y.read_curpose());
        ROS_INFO("x.aimpose:[%lf],y.aimpose:[%lf]",x.read_aimpose(),y.read_aimpose());
        ROS_INFO("x.out:[%lf],y.out:[%lf]",x.out(), y.out());
        set_LinearVel_XY(vel, x.out(), y.out());
}




// void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
// {
//     if(msg->range >= msg->min_range && msg->range <= 1.0)
//     { 
//         cur_Z.data = msg->range - msg->min_range;
//         //中值滤波
//         if(deque_z_data.size() <= 10)
//         {
//             deque_z_data.push_back(cur_Z.data);
//         }
//         else
//         {
//             deque_z_data.pop_front();
//             deque_z_data.push_back(cur_Z.data);
//         }

//         std::sort(deque_z_data.begin(),deque_z_data.end());
//         int mid = deque_z_data.size()/2;

//         z_to_pub.data = deque_z_data.at(mid);
//     }
// //	cur_Z.data = msg->range - 0;
//     if(flag_getbegin_z == 0)
//     {
//     	if(msg->range >= msg->min_range && msg->range <= 1.0)
//     	{
//         	z.set_beginpose(cur_Z.data);
//         	z.init(z.read_beginpose()+5.0, 2, 0, 0);
// 			//z.init(0.150, 2, 0, 0);
// 			flag_getbegin_z = 1;
// 			ROS_INFO("set_Z_beginpose!!!!!!");
//     	}
//     }
//     if(flag_start == 1)
//     {
//         z.set_curpose(cur_Z.data);
        
//         z.update();

//         set_LinearVel_Z(vel, z.out());
//     }

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
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

