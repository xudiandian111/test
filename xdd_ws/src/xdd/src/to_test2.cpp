/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <cmath>

#include <xdd/PID.h>

std_msgs::Float64 cur_Z;
PID x, y, z;

geometry_msgs::PoseStamped cur_pos;
//geometry_msgs::PoseStamped aim_pos;
geometry_msgs::Twist vel;

mavros_msgs::State current_state;

int flag_getbegin_pos = 0;
int flag_start = 0;
int flag_getbegin_z = 0;
int aim_num = 0;
int hold_time = 0;
int back_home_time = 0;
int z_hold_time = 0;


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
			//ROS_INFO("set_Z_beginpose!!!!!!");
    	}
    }
    if(flag_start == 1)
    {   
        if(z.read_err()<0.10)
        {
            z_hold_time++;
        }

        z.set_curpose(cur_Z.data);        
        z.update();
        set_LinearVel_Z(vel, z.out());
    }

}  
//  int set_position_num = 0;

//     double AllSetPositionX[] = {0, 1, 0, -1, 0};
//     double AllSetPositionY[] = {0, 1, 0, -1, 0};
//     double AllSetPositionZ[] = {1, 1, 1, 1, 0};


//pos回调函数,通过位置计算并且赋速度
void cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{   

   if(flag_getbegin_pos == 0)
   {    
        //ROS_INFO("flag_getbegin_pos:OK!!!!!!!!");
        x.set_beginpose(msg->pose.position.x);
        y.set_beginpose(msg->pose.position.y);       
       
        x.init(0.0, 2.0, 0, 0);
        y.init(0.0, 2.0, 0, 0);

        x.set_aimpose(x.read_beginpose() + 0.0);
        y.set_aimpose(y.read_beginpose() + 0.0);
       
        //ROS_INFO("z.read_aimpose:[%lf]", z.read_aimpose());
        flag_getbegin_pos = 1; 

   }
   else
   {    //if(flag_go == 1)
        //{ 
       //     x.set_curpose(msg->pose.position.x);
        //    y.set_curpose(msg->pose.position.y);
       // }

    if(z_hold_time > 100)
    {
        //飞向第一象限

      double AllSetPositionX[] = {0, 0.5, 0, -0.5, 0, 0, 0, 0};
      double AllSetPositionY[] = {0, 0, 0, 0, 0, 0.5, 0, -0.5};
        // double AllSetPositionZ[] = {1, 1, 1, 1, 0};
        // if(aim_num % 4 == 0)
        {
            x.set_aimpose(x.read_beginpose() + AllSetPositionX[aim_num % 8]);
            y.set_aimpose(y.read_beginpose() + AllSetPositionY[aim_num % 8]);

            x.set_curpose(msg->pose.position.x);
            y.set_curpose(msg->pose.position.y);

            if(x.read_err() < 0.10 && y.read_err() < 0.10)
            {
                hold_time++;
            }

            if(hold_time > 60)
            {
                x.set_aimpose(x.read_beginpose());
                y.set_aimpose(y.read_beginpose());//返回最开始的位置
                
                x.set_curpose(msg->pose.position.x);
                y.set_curpose(msg->pose.position.y);

                if(x.read_err() < 0.10 && y.read_err() < 0.10)
                {
                    back_home_time++;
                    if(back_home_time > 60)
                    {
                        aim_num++;
                        back_home_time = 0;
                        hold_time = 0;
                    }
                }
            } 
        }
// //飞向第二象限
//         if(aim_num % 4 == 1)
//         {
//             x.set_aimpose(x.read_beginpose() - 0.5);
//             y.set_aimpose(y.read_beginpose() + 0.5);
//             x.set_curpose(msg->pose.position.x);
//             y.set_curpose(msg->pose.position.y);

//             if(x.read_err() < 0.10 && y.read_err() < 0.10)
//             {
//                 hold_time++;
//             }

//             if(hold_time > 30)
//             {
//                 x.set_aimpose(x.read_beginpose());
//                 y.set_aimpose(y.read_beginpose());

//                 x.set_curpose(msg->pose.position.x);
//                 y.set_curpose(msg->pose.position.y);
                
//                 if(x.read_err() < 0.10 && y.read_err() < 0.10)
//                 {
//                     back_home_time++;
//                     if(back_home_time > 10)
//                     {
//                         aim_num++;
//                         back_home_time = 0;
//                         hold_time = 0;
//                     }
//                 }
//             } 
//         }
// //飞向第三象限
//         if(aim_num % 4 == 2)
//         {
//             x.set_aimpose(x.read_beginpose() - 0.5);
//             y.set_aimpose(y.read_beginpose() - 0.5);

//             x.set_curpose(msg->pose.position.x);
//             y.set_curpose(msg->pose.position.y);

//             if(x.read_err() < 0.10 && y.read_err() < 0.10)
//             {
//                 hold_time++;
//             }

//             if(hold_time > 30)
//             {
//                 x.set_aimpose(x.read_beginpose());
//                 y.set_aimpose(y.read_beginpose());

//                  x.set_curpose(msg->pose.position.x);
//                 y.set_curpose(msg->pose.position.y);
                
//                 if(x.read_err() < 0.10 && y.read_err() < 0.10)
//                 {
//                     back_home_time++;
//                     if(back_home_time > 10)
//                     {
//                         aim_num++;
//                         back_home_time = 0;
//                         hold_time = 0;
//                     }
//                 }
//             } 
//         }
// //飞向第四象限
//         if(aim_num % 4 == 3)
//         {
//             x.set_aimpose(x.read_beginpose() + 0.5);
//             y.set_aimpose(y.read_beginpose() - 0.5);

//             x.set_curpose(msg->pose.position.x);
//             y.set_curpose(msg->pose.position.y);

//             if(x.read_err() < 0.10 && y.read_err() < 0.10)
//             {
//                 hold_time++;
//             }

//             if(hold_time > 30)
//             {
//                 x.set_aimpose(x.read_beginpose());
//                 y.set_aimpose(y.read_beginpose());

//                 x.set_curpose(msg->pose.position.x);
//                 y.set_curpose(msg->pose.position.y);
                
                
//                 if(x.read_err() < 0.10 && y.read_err() < 0.10)
//                 {
//                     back_home_time++;
//                     if(back_home_time > 10)
//                     {
//                         aim_num++;
//                         back_home_time = 0;
//                         hold_time = 0;
//                     }
//                 }
//             } 
//         }

//    }     

   x.set_curpose(msg->pose.position.x);
   y.set_curpose(msg->pose.position.y);

        x.update();
        y.update();
        set_LinearVel_XY(vel, x.out(), y.out());

       //ROS_INFO("z.aimpose[%lf]", z.read_aimpose());
       //ROS_INFO("z.curpose:[%lf]", z.read_curpose());
       //ROS_INFO("z.out:[%lf]",z.out());

   }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "xdd");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1000, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe< geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 2, cur_pos_cb);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
  	ros::Subscriber local_z_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/px4flow/ground_distance", 2, cur_z_cb);
	ros::Publisher xdd_z_pub = nh.advertise<std_msgs::Float64>
	        ("xdd_z", 2);  
	

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

    cmd_vel_pub.publish(vel);
   // xdd_z_pub.publish(cur_Z);
    //ROS_INFO("vel.twist.linear.z:[%lf]", vel.twist.linear.z);

    ros::spinOnce();
    rate.sleep();

    }

    return 0;
}

