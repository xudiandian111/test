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

#include <xdd/PID.h>

using namespace std;

#define PICTURE  0
#define PXFLOW   1


int flag_getbegin_picture = 0;
int flag_getbegin_pxflow = 0;
int flag_use_data_from = 0;

std_msgs::Float64 cur_Z;
PID x, y, z;

geometry_msgs::PoseStamped cur_pos;
geometry_msgs::Pose2D curPoseXY;

geometry_msgs::TwistStamped vel;

mavros_msgs::State current_state;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;


std_msgs::Float64  can_start_detect;
int high_stable_time = 0;
int pose_hold_time = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



void set_LinearVel_XY(geometry_msgs::TwistStamped &_vel, float64 lx, float64 ly)
{
    _vel.twist.linear.x = lx;
    _vel.twist.linear.y = ly;
}



void set_LinearVel_Z(geometry_msgs::TwistStamped &_vel, float64 lz)
{
    _vel.twist.linear.z = lz;
}




// 从picture获得当前无人机的坐标
void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("Set cur x:[%lf]", (msg->x)/320);
    ROS_INFO("Set cur y:[%lf]", (msg->y)/320);

	if(flag_use_data_from == PICTURE)	
	{				
		if(flag_getbegin_picture == 0)                           //开始时获取一下当前坐标信息
		{
    	    x.set_beginpose((msg->x)/320);
    	    y.set_beginpose((msg->y)/320);
   
    	    x.init(0.0, 1.9, 0, 0);            //三个参数分别为aim，Kp和Kd
    	    y.init(0.0, 1.9, 0, 0);

			flag_getbegin_picture = 1;
			ROS_INFO("Using Picture!!!!!!!!!!!!!!!!!!!!");
		}
		
		if(flag_start == 1 && flag_getbegin_picture == 1)
		{
			x.set_curpose((msg->x)/320);
			y.set_curpose((msg->y)/320);
			
			curPoseXY.x = x.read_curpose();     //此方法：坐标系不再飞机身上，而在黑圆中心
			curPoseXY.y = y.read_curpose();

    	   	ROS_INFO("pic_curpose_x:[%lf]", x.read_curpose());
    	   	ROS_INFO("pic_curpose_y:[%lf]", y.read_curpose());
    	   	ROS_INFO("aimpose_x:[%lf]", x.read_aimpose());
    	   	ROS_INFO("aimpose_y:[%lf]", y.read_aimpose());
			ROS_INFO("set_X_vel:[%lf]", x.out());
			ROS_INFO("set_Y_vel:[%lf]", y.out());

			x.update();
			y.update();
            set_LinearVel_XY(vel, x.out(), y.out());
		}
	}

}

// 通过光流估计位置
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{  
    if(flag_use_data_from == PXFLOW)	
    {
        if(flag_getbegin_pos == 0)
        {    
            ROS_INFO("flag_getbegin_px4flow_pos:OK!!!!!!!!");
            x.set_beginpose(msg->pose.position.x);
            y.set_beginpose(msg->pose.position.y);
    
            x.init(0.0, 2.0, 0, 0);
            y.init(0.0, 2.0, 0, 0);
        
            //ROS_INFO("z.read_aimpose:[%lf]", z.read_aimpose());
            flag_getbegin_pos = 1; 

        }
        else
        {   
            x.set_curpose(msg->pose.position.x);
            y.set_curpose(msg->pose.position.y);

    
            x.update();
            y.update();
        
            // cout << "x.read_beginpose():" << x.read_beginpose() <<endl;
            cout << "x.read_curpose():" << x.read_curpose() <<endl;
            cout << "x.read_aimpose():" << x.read_aimpose() <<endl;
            cout << "x.out():" << x.out() << endl;

            // cout << "y.read_beginpose():" << y.read_beginpose() <<endl;
            cout << "y.read_curpose():" << y.read_curpose() <<endl;
            cout << "y.read_aimpose():" << y.read_aimpose() <<endl;
            cout << "y.out():" << y.out() << endl;

            cout << endl;

            set_LinearVel_XY(vel, x.out(), y.out());
           
        }
    }
}


void cur_z_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   

   if(flag_getbegin_z == 0)
   {    
        ROS_INFO("flag_getbegin_z_pose:OK!!!!!!!!");
      
        z.set_beginpose(msg->pose.position.z);
        z.init(0.7, 2.0, 0, 0);
        flag_getbegin_z = 1; 

   }
   else
   {     
        z.set_curpose(msg->pose.position.z);
        z.update();

        cout << "z.read_curpose():" << z.read_curpose() <<endl;
        cout << "z.read_aimpose():" << z.read_aimpose() <<endl;
        cout << "z.out():" << z.out() << endl;

        cout << endl;
        set_LinearVel_Z(vel, z.out());
   }
}


void judge_can_detect(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    if(fabs(z.read_err()) <= 0.10 )
    {
        high_stable_time++;
    }

    if(high_stable_time >= 20)
    {
        if(fabs(x.read_curpose()) < 0.15 && fabs(y.read_curpose()) < 0.15)
        {
            pose_hold_time++;
        }

        if(pose_hold_time >= 20)
        {
            can_start_detect.data = 1.0;
        }
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
    ros::Subscriber local_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_z_cb);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);
  	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 1000);  

    ros::Publisher can_start_detect_pub = nh.advertise<std_msgs::Float64>
			("xdd/can_start_detect", 1000);


    ros::Subscriber judge_can_detect_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, judge_can_detect);
    ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
	

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    vel.twist.linear.x = 0;
	vel.twist.linear.y = 0;
	vel.twist.linear.z = 0;

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
    xdd_z_pub.publish(cur_Z);
    xdd_poseXY_pub.publish(curPoseXY);
    can_start_detect_pub.publish(can_start_detect);     //发布能否开始检测的命令
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

