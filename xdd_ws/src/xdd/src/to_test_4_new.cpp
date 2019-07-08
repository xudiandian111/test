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
#include <mavros_msgs/PositionTarget.h>
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

geometry_msgs::Pose2D xdd_xy_pub;

geometry_msgs::Pose2D error_xy;

PID x, y, z;
PID yaw;

geometry_msgs::PoseStamped cur_pos;
//geometry_msgs::PoseStamped aim_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
int flag_getbegin_yaw = 0;
int aim_num = 0;
int hold_time = 0;
int z_hold_time = 0;
// int back_home_time = 0;
// bool flag_go = 1;

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


void local_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) 
{   
   if(flag_getbegin_yaw == 0)
   {    
        ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(msg->yaw);
          
        yaw.init(0.0 - yaw.read_beginpose(), 2.0, 0, 0);// 暂时设定yaw的目标角度0度
       
        flag_getbegin_yaw = 1; 

   }
   else
   {     
        yaw.set_curpose(msg->yaw);

        yaw.update();
    
        set_yaw_rate(vel, yaw.out());

        ROS_INFO("yaw.aimpose:[%lf]", yaw.read_aimpose());
        ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
        ROS_INFO("yaw_rate:   [%lf]", yaw.out());
   }
}

void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if(msg->range >= msg->min_range && msg->range <= 1.0)
    { 
        cur_Z.data = msg->range - msg->min_range;
        //中值滤波
        if(deque_z_data.size() <= 10)
        {
            deque_z_data.push_back(cur_Z.data);
        }
        else
        {
            deque_z_data.pop_front();
            deque_z_data.push_back(cur_Z.data);
        }

        std::sort(deque_z_data.begin(),deque_z_data.end());
        int mid = deque_z_data.size()/2;

        z_to_pub.data = deque_z_data.at(mid);
    }
//	cur_Z.data = msg->range - 0;
    if(flag_getbegin_z == 0)
    {
    	if(msg->range >= msg->min_range && msg->range <= 1.0)
    	{
        	z.set_beginpose(cur_Z.data);
        	z.init(z.read_beginpose()+0.50, 2, 0, 0);
			//z.init(0.50, 2, 0, 0);
			flag_getbegin_z = 1;
			ROS_INFO("set_Z_beginpose!!!!!!");
    	}
    }
    if(flag_start == 1)
    {
        z.set_curpose(cur_Z.data);
        
        z.update();

        set_LinearVel_Z(vel, z.out());
    }

}  


// double AllSetPositionX[] = {0, 0.5, 0, -0.5, 0, 0, 0, 0};
// double AllSetPositionY[] = {0, 0, 0,  0, 0, 0.5, 0, -0.5};


//pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   

   if(flag_getbegin_pos == 0)
   {    
        //ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        // z.set_beginpose(msg->pose.position.z);
        	
            
   
        x.init(x.read_beginpose() + 0.0, 1, 0, 0.3);
        y.init(y.read_beginpose() + 0.0, 1, 0, 0.3);
        // z.init(z.read_beginpose() + 5.0, 1, 0, 0.3);
        flag_getbegin_pos = 1;
    }


    x.set_curpose(msg->pose.position.x);
    y.set_curpose(msg->pose.position.y);
    // z.set_curpose(msg->pose.position.z);


    // double AllSetPositionX[] = {0, 0.5, 0, -0.5, 0, 0, 0, 0};
    // double AllSetPositionY[] = {0, 0, 0, 0, 0, 0.5, 0, -0.5};


    //逆时针方向的四个点,0,1,2,3
    double AllSetPositionX[] = {0.5, -0.5, -0.5, 0.5};
    double AllSetPositionY[] = {0.5, 0.5,  -0.5, -0.5};

    if(z.read_err() < 0.10)
    {
        z_hold_time++;
    }

    if(z_hold_time > 200) 
    {     
            //if(aim_num % 4 == 0)
            {
                x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num]);
                y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num]);

                x.set_curpose(msg->pose.position.x);
                y.set_curpose(msg->pose.position.y);
                

                if(x.read_err() < 0.10 && y.read_err() < 0.10)
                {
                    hold_time++;
                }
                if(hold_time > 150)
                {
                    hold_time = 0;
                    x.set_aimpose(x.read_beginpose());
                    y.set_aimpose(y.read_beginpose());
                }
            }
            xdd_xy_pub.x = x.read_curpose() - x.read_beginpose();
            xdd_xy_pub.y = y.read_curpose() - y.read_beginpose();  
    }

        // z.update();
        // z.set_curpose(msg->pose.position.z);
        // set_LinearVel_Z(vel,z.out());
                
        x.update();
        y.update();

        error_xy.x = x.read_err();
        error_xy.y = y.read_err();
       
        set_LinearVel_XY(vel, x.out(), y.out());
        // //测试
        // vel.angular.z = 10;

       //ROS_INFO("z.aimpose[%lf]", z.read_aimpose());
       //ROS_INFO("z.curpose:[%lf]", z.read_curpose());
       //ROS_INFO("z.out:[%lf]",z.out());

        // ROS_INFO("x.curpose[%lf],y.curpose[%lf]", x.read_curpose() - x.read_beginpose(), y.read_curpose() - y.read_beginpose());
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
    ros::Subscriber yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 10, local_yaw_cb);


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
            
	ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);
	ros::Subscriber aim_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("aim_pose", 1, aim_pose_cb);

    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;
    // vel.angular.z = 1;
    
    

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

    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(z_to_pub);
    xdd_poseXY_pub.publish(xdd_xy_pub);
    error_XY_pub.publish(error_xy);
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

