/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cmath>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

//#include <xdd/PID.h>


//PID x, y, z;
std_msgs::Float64 cur_Z;
int flag_getbegin_z = 0;
double dis_z;



//PID 参数设置，X Y Z与目标位置的距离
double kp =2.05 , ki = 0, kd = 0.0;
double lastDistanceX = 0, lastDistanceY = 0, lastDistanceZ = 0;
double distanceSumX = 0, distanceSumY = 0, distanceSumZ = 0;


geometry_msgs::PoseStamped curPos;
geometry_msgs::PoseStamped aimPos;
//geometry_msgs::PoseStamped next_aim_pos;

geometry_msgs::TwistStamped vel;

//geometry_msgs::PoseStamped cur_pos;
//geometry_msgs::PoseStamped aim_pos;
//geometry_msgs::TwistStamped vel;

mavros_msgs::State current_state;

//int flag_getbegin_pos = 0;
//int flag_start = 0;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



//回调函数， 返回无人机现在的位置
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
  
        curPos.pose.position.z = msg->pose.position.z;
        curPos.pose.position.x = msg->pose.position.x;
        curPos.pose.position.y = msg->pose.position.y; 
}


void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if(msg->range >= msg->min_range && msg->range <= 1.0)
    {
        cur_Z.data = msg->range - msg->min_range;
    }
}

void set_vel(geometry_msgs::PoseStamped aimPos,
             geometry_msgs::PoseStamped curPos)
{  
    //计算距离
    double distanceX = aimPos.pose.position.x - curPos.pose.position.x ;
    double distanceY = aimPos.pose.position.y - curPos.pose.position.y ;
    double distanceZ = aimPos.pose.position.z - curPos.pose.position.z ;
    //计算速度
    vel.twist.linear.x = kp *distanceX + kd * (lastDistanceX - distanceX) + ki * distanceSumX;
    vel.twist.linear.y = kp *distanceY + kd * (lastDistanceY - distanceY) + ki * distanceSumY;
    vel.twist.linear.z = kp *distanceZ + kd * (lastDistanceZ - distanceZ) + ki * distanceSumZ;

    distanceSumX += (lastDistanceX - distanceX);
    distanceSumY += (lastDistanceY - distanceY);
    distanceSumZ += (lastDistanceZ - distanceZ);

    lastDistanceX = distanceX;
    lastDistanceY = distanceY;
    lastDistanceZ = distanceZ;
}
/*赋XY速度
void set_LinearVel_XY(geometry_msgs::TwistStamped &vel, float64 lx, float64 ly)
{
    vel.twist.linear.x = lx;
    vel.twist.linear.y = ly;
}

//赋Z速度
void set_LinearVel_Z(geometry_msgs::TwistStamped &vel, float64 lz)
{
    vel.twist.linear.z = lz;
}
*/


/*
void cur_z_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if(msg->range >= msg->min_range && msg->range <= 1.0)
    {
        cur_Z.data = msg->range - msg->min_range;
    }

}
*/

/*pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   

   if(flag_getbegin_pos == 0)
   {    
       ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);
        z.set_beginpose(msg->pose.position.z);


        x.set_aimpose(x.read_beginpose() + 0.0);
        y.set_aimpose(y.read_beginpose() + 0.0);
        z.set_aimpose(z.read_beginpose() + 0.5);

    
        x.init(0.0, 2.0, 0, 0);
        y.init(0.0, 2.0, 0, 0);
        z.init(z.read_aimpose(), 2.0, 0, 0);

        ROS_INFO("z.read_aimpose:[%lf]", z.read_aimpose());
        flag_getbegin_pos = 1; 
   }

   if(flag_start == 1)
   {

        //ROS_INFO("z.aimpose[%lf]", z.read_aimpose());

        x.set_curpose(msg->pose.position.x);
        y.set_curpose(msg->pose.position.y);
        z.set_curpose(msg->pose.position.z);

        x.update();
        y.update();
        z.update();

        set_LinearVel_XY(vel, x.out(), y.out());
        set_LinearVel_Z(vel, z.out());

        ROS_INFO("z.set_curpose:[%lf]", z.read_curpose());
        ROS_INFO("z.out:[%lf]",z.out());

   }
}
*/




int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    /*
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
     */ 
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);      
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_z_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1000, cur_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Subscriber local_pos_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/px4flow/ground_distance", 1000, cur_z_cb);
    ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 1000);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    /*
    vel.twist.linear.x = 0;
	vel.twist.linear.y = 0;
	vel.twist.linear.z = 0;
    */

    aimPos.pose.position.x = 0;
    aimPos.pose.position.y = 0;
    aimPos.pose.position.z = 2;

    // wait for FCU connection
   while(ros::ok() && !current_state.connected)
    {   
        //cmd_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }


    //send a few setpoints before starting

    for(int i = 75; ros::ok() && i > 0; --i)
    {
        cmd_vel_pub.publish(vel);
        //local_pos_pub.publish(pose);
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
                   // flag_start = 1;

                }
                last_request = ros::Time::now();
            }
        }
	
    //ROS_INFO("set_z_vel:[%lf]", vel.twist.linear.z);
    //local_pos_pub.publish(pose);
    set_vel(aimPos, curPos);

    cmd_vel_pub.publish(vel);
    xdd_z_pub.publish(cur_Z);
    //ROS_INFO("cur_pos.x:[%lf]", cur_pos.pose.position.z);
    //ROS_INFO("cur_Z:[%f]", cur_Z.data);
    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

