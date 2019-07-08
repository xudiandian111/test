#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <cmath>
#include <algorithm>
#include <vector>


#include "xdd/PID.h"


#define PICTURE  0
#define PXFLOW   1


long long int mytime = 0;

int flag_use_data_from = 0;

int flag_start = 0;
int flag_getbegin_picture = 0;
int flag_getbegin_z = 0;
int flag_getbegin_pxflow = 0;
int flag_getbegin_yaw = 0;



PID yaw;
mavros_msgs::GlobalPositionTarget yaw_rate;

geometry_msgs::TwistStamped vel;
geometry_msgs::Pose2D curPoseXY;
PID x, y, z;
mavros_msgs::State current_state;
std_msgs::Float64 cur_Z;
int accel_time = 0;




float64 aim_x_pxflow = 0.0;
float64 aim_y_pxflow = 0.0;
int hold_time = 0;
geometry_msgs::Pose2D curPXFlowXY;
int PXFLOW_cnt = 0;
std::vector<float64> PXFLOW_Pose_x_data;
std::vector<float64> PXFLOW_Pose_y_data;
float64 PXFLOW_beginpose_x = 0;
float64 PXFLOW_beginpose_y = 0;




//无人机状态回调
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


//给无人机赋XY方向的速度
void set_LinearVel_XY(geometry_msgs::TwistStamped &_vel, float64 lx, float64 ly)
{
    _vel.twist.linear.x = lx;
    _vel.twist.linear.y = ly;
}


//给无人机赋Z方向速度
void set_LinearVel_Z(geometry_msgs::TwistStamped &_vel, float64 lz)
{
    _vel.twist.linear.z = lz;
}

//赋yaw速度
void set_yaw_rate(mavros_msgs::GlobalPositionTarget &local_yaw_rate, float64 yaw_rate)
{
    local_yaw_rate.yaw_rate = yaw_rate;
}




//回调函数,计算yaw速度
void local_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg) 
{   
   if(flag_getbegin_yaw == 0)
   {    
        //ROS_INFO("flag_getbegin_yaw:OK!!!!!!!!");
        yaw.set_beginpose(msg->yaw);
          
        yaw.init(0.0, 2.0, 0, 0);// 暂时设定yaw的目标角度0度
       
        flag_getbegin_yaw = 1; 

   }
   else
   {     
        yaw.set_curpose(msg->yaw);

        yaw.update();
    
        set_yaw_rate(yaw_rate, yaw.out());

       ROS_INFO("yaw.aimpose[%lf]", yaw.read_aimpose());
       ROS_INFO("yaw.curpose:[%lf]", yaw.read_curpose());
       ROS_INFO("yaw.out:[%lf]",yaw.out());

   }
}



//通过传感器获得Z方向坐标，并计算Z方向速度
void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if(msg->range >= msg->min_range && msg->range <= 1.0)
    {
        cur_Z.data = msg->range - msg->min_range;
    }
//	cur_Z.data = msg->range - 0;
    if(flag_getbegin_z == 0)
    {
    	if(msg->range >= msg->min_range && msg->range <= 1.0)
    	{
        	z.set_beginpose(cur_Z.data);
        	z.init(z.read_beginpose()+0.50, 2, 0, 0);
			flag_getbegin_z = 1;
			//ROS_INFO("set_Z_beginpose!!!!!!");
    	}
    }
    if(flag_start == 1)
    {
        z.set_curpose(cur_Z.data);
        //ROS_INFO("curpose_z:[%lf]", z.read_curpose());
        //ROS_INFO("aimpose_z:[%lf]", z.read_aimpose());
		//ROS_INFO("beginpose_z:[%lf]", z.read_beginpose());
        z.update();

        set_LinearVel_Z(vel, z.out());
    }

}



//从图像中获得无人机在黑圆坐标系下无人机XY方向坐标
void vision_pose_cb(const geometry_msgs::Pose2D::ConstPtr& msg)
{

ROS_INFO("Set cur x:[%lf]", msg->x);
ROS_INFO("Set cur y:[%lf]", msg->y);
	if(flag_use_data_from == PICTURE)	
	{
				
		if(flag_getbegin_picture == 0)                           //开始时获取一下当前坐标信息
		{

    	    x.set_beginpose(msg->x);
    	    y.set_beginpose(msg->y);

    	    //x.init(msg->aimx, 2.2, 0, 0);            //三个参数分别为aim，Kp和Kd
    	    //y.init(msg->aimy, 2.2, 0, 0);
    	    x.init(0.0, 1.9, 0, 0);            //三个参数分别为aim，Kp和Kd
    	    y.init(0.0, 1.9, 0, 0);

			flag_getbegin_picture = 1;
			ROS_INFO("Using Picture!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		}
		
		if(flag_start == 1 && flag_getbegin_picture == 1)
		{
			x.set_curpose(msg->x);
			y.set_curpose(msg->y);
			
			curPoseXY.x = x.read_curpose();
			curPoseXY.y = y.read_curpose();
    	   	//ROS_INFO("pic_curpose_x:[%lf]", x.read_curpose());
    	   	//ROS_INFO("pic_curpose_y:[%lf]", y.read_curpose());
    	   	//ROS_INFO("aimpose_x:[%lf]", x.read_aimpose());
    	   	//ROS_INFO("aimpose_y:[%lf]", y.read_aimpose());
			//ROS_INFO("set_X_vel:[%lf]", x.out());
			//ROS_INFO("set_Y_vel:[%lf]", y.out());

			x.update();
			y.update();

		}
	}

}



/**
 * @brief aim_pose_cb get the aim pose from kinect.
 *
 * @param msg
 */
void aim_pose_cb(const geometry_msgs::Pose2D::ConstPtr& _msg)
{
	
	ROS_ERROR("set x aim:[%lf]", _msg->x);
	ROS_ERROR("set y aim:[%lf]", _msg->y);

   	if(flag_start == 1 && flag_getbegin_picture == 1)
   	{

		x.set_aimpose(_msg->x);
		y.set_aimpose(_msg->y);

		if(_msg->x==0 && _msg->y==0)
		{
			accel_time = 0;

			x.update();
			y.update();

			//ROS_INFO("set_X_vel:[%lf]", x.out());
			//ROS_INFO("set_Y_vel:[%lf]", y.out());
		
 			set_LinearVel_XY(vel, x.out(), y.out());
		}
		else if(accel_time <= 15) 
		{
					
			float64 ax_k;
			float64 ay_k;
			if(x.read_aimpose() == 0)
			{
				ax_k = 0.0;		
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

			
		
 			set_LinearVel_XY(vel, x.out(), y.out());
		}


	}


}



/**
 * @brief cur_pose_cb : get X, Y info from px4flow and update.
 *
 * @param:msg
 */
void pxflow_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(flag_use_data_from == PXFLOW)
	{
				
		if(flag_getbegin_pxflow == 0)
		{
			ROS_INFO("Using PX4FLOW!!!!!!!!!!!!!!!!!!!!!!!");
			
			x.set_beginpose(PXFLOW_beginpose_x);
			ROS_INFO("Use px4flow beginpose_x:[%lf]",x.read_beginpose()); 
			x.init(aim_x_pxflow, 0.3, 0, 0.9);

			y.set_beginpose(PXFLOW_beginpose_y);
			ROS_INFO("Use px4flow beginpose_y:[%lf]",y.read_beginpose()); 
			y.init(aim_y_pxflow, 0.10, 0, 1.6);

			ROS_INFO("aimpose_x:[%lf]", x.read_aimpose());
			ROS_INFO("aimpose_y:[%lf]", y.read_aimpose());

			flag_getbegin_pxflow = 1;
		}

		if(flag_start == 1)
		{
			x.set_curpose(msg->pose.position.x);
			y.set_curpose(msg->pose.position.y);
			
			curPoseXY.x = x.read_curpose();
			curPoseXY.y = y.read_curpose();
			//ROS_INFO("px4flow_curpose_x:[%lf]", x.read_curpose());
			//ROS_INFO("aimpose_x:[%lf]", x.read_aimpose());
			//ROS_INFO("px4flow_curpose_y:[%lf]", y.read_curpose());
			//ROS_INFO("aimpose_y:[%lf]", y.read_aimpose());
			
			x.update();
			y.update();
			
			//ROS_INFO("set_X_vel:[%lf]", x.out());
			//ROS_INFO("set_Y_vel:[%lf]", y.out());

			set_LinearVel_XY(vel, x.out(), y.out());
		}


		if(fabs(x.read_err()) <= 0.15 && fabs(y.read_err()) <= 0.15)	
		{
			hold_time++;	
		}

		if(hold_time >= 35)
		{
			x.set_aimpose(x.read_beginpose());
			y.set_aimpose(y.read_beginpose());
			ROS_INFO("aimpose_x:[%lf]", x.read_aimpose());
			ROS_INFO("aimpose_y:[%lf]", y.read_aimpose());
			hold_time++;
			//ROS_INFO("hold_time: 2s");
		}
		if(hold_time >= 55)
		{
			flag_use_data_from = PICTURE;		
			flag_getbegin_picture = 0;
			hold_time = 0;
		}
		//ROS_INFO("hold_time:[%d]", hold_time);

	}
}



/*********************************************************************************/
int main(int argc, char **argv)
{
	//ROS_INFO("using filter");
	//ball_ans.balloon_test();

    ros::init(argc, argv, "xdd");
	ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
	        ("mavros/setpoint_velocity/cmd_vel", 1);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber local_z_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/px4flow/ground_distance", 1000, cur_z_cb);
	ros::Subscriber vision_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("vision_pose", 1, vision_pose_cb);
	ros::Subscriber aim_pose_sub = nh.subscribe<geometry_msgs::Pose2D>
			("aim_pose", 1, aim_pose_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, pxflow_pose_cb);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 1000);
	ros::Publisher xdd_poseXY_pub = nh.advertise<geometry_msgs::Pose2D>
			("xdd_poseXY", 1000);
	ros::Subscriber yaw_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("setpoint_raw/target_local", 10, local_yaw_cb);
    ros::Publisher yaw_rate_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_position/global", 1000);



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
	
	vel.twist.linear.x = 0;
	vel.twist.linear.y = 0;
	vel.twist.linear.z = 0;


    while(ros::ok() && !current_state.connected)
    {

        ros::spinOnce();
        rate.sleep();
    }

	//send a few vels before starting
    for(int i = 150; ros::ok() && i > 0; --i)
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
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    flag_start = 1;
                }
                last_request = ros::Time::now();
            }
        }


		if(flag_start == 1)
		{
			mytime++;
			//ROS_INFO("TIME:[%lld]", mytime);
		}

		ros::spinOnce();

		xdd_z_pub.publish(cur_Z);			
		xdd_poseXY_pub.publish(curPoseXY);

		ROS_INFO("set_X_vel:[%lf]", vel.twist.linear.x);
		ROS_INFO("set_Y_vel:[%lf]", vel.twist.linear.y);
		
		//set_LinearVel_XY(vel,0.0, 0.0 );
	
		cmd_vel_pub.publish(vel);
		xdd_z_pub.publish(cur_Z);
   		yaw_rate_pub.publish(yaw_rate);
		//PXFlow_poseXY_pub.publish(curPXFlowXY);

        rate.sleep();

    }

    return 0;
}
