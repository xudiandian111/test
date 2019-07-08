/**
 * @file dck_offboard_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <math.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "xdd/uav.h"

template <class T> 
int getArrayLen(T & array)
{
	return (sizeof(array) / sizeof(array[0]));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dck_offboard_node");
    uav pixhawk_uav;

    enum ControlMode
    {
        ToTakeOff,
        GoToSetPsition,
        ThrowGoods
    };
    ControlMode control_mode = ToTakeOff;

    double AllSetPositionX[] = {0, 1, 0, -1, 0};
    double AllSetPositionY[] = {0, 1, 0, -1, 0};
    double AllSetPositionZ[] = {1, 1, 1, 1, 0};

    int set_position_num = 0;

    geometry_msgs::Point current_position;
    geometry_msgs::Point current_set_position;
    geometry_msgs::Point home_position;
    current_set_position.x =  AllSetPositionX[0];
    current_set_position.y =  AllSetPositionY[0];
    current_set_position.z =  AllSetPositionZ[0];
    home_position.x = 0.0;
    home_position.y = 0.0;
    home_position.z = 0.0;

    geometry_msgs::Point current_position_in_image;
    geometry_msgs::Point set_position_in_image;
    geometry_msgs::Vector3 set_vel;
    double image_vel_Kp = 0.1;

    int time_to_takeoff = 0;
    bool mode_chang_flag = false;
    bool return_flag = false;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while( ros::ok() && !pixhawk_uav.GetConnectedStatus() ){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 75; ros::ok() && i > 0; --i){
        pixhawk_uav.SetV(0, 0, 1);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        if (!mode_chang_flag){

            if( pixhawk_uav.GetCurrentMode() != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( pixhawk_uav.SetMode("OFFBOARD") ){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else {
                if( !pixhawk_uav.GetArmedStatus() &&
                    (ros::Time::now() - last_request > ros::Duration(1.5))){
                    if( pixhawk_uav.ChangeArmingStatus(true) ){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }            
        }
        
        switch (control_mode){
            case ToTakeOff:
                 pixhawk_uav.SetV(0, 0, 1);
                 if (pixhawk_uav.GetArmedStatus()){
                     time_to_takeoff++;
                 }

                 if (time_to_takeoff >= 6){
                     control_mode = GoToSetPsition;
                     pixhawk_uav.GetCurrentPosition(home_position);
                     time_to_takeoff = 0;
                     mode_chang_flag = true;
                     ROS_INFO("Take OFF");
                 }
                 
                 break;

            case GoToSetPsition:
                /* code for GoToSetPsition */
                 current_set_position.x =  home_position.x + AllSetPositionX[set_position_num];
                 current_set_position.y =  home_position.y + AllSetPositionY[set_position_num];
                 current_set_position.z =  home_position.z + AllSetPositionZ[set_position_num];

                 pixhawk_uav.SetPosition(current_set_position);

                 pixhawk_uav.GetCurrentPosition(current_position);

                 if ( fabs(current_position.x - current_set_position.x)<=0.2 && 
                      fabs(current_position.y - current_set_position.y)<=0.2 &&
                      fabs(current_position.z - current_set_position.z)<=0.2 ){
                     /* code for True */
                     if (return_flag){
                        set_position_num--;
                     }
                     else{
                        set_position_num++;
                     }

                     if(set_position_num >= getArrayLen(AllSetPositionX)) {
                         set_position_num = getArrayLen(AllSetPositionX) - 1;
                         return_flag = true;
                     }
                     if(set_position_num < 0) {
                         set_position_num = 0;
                     }
                 }
                 
                break;

            case ThrowGoods:
                /* code for ThrowGoods */
                set_vel.x = image_vel_Kp * (set_position_in_image.x - current_position_in_image.x);
                set_vel.y = image_vel_Kp * (set_position_in_image.y - current_position_in_image.y);
                set_vel.z = image_vel_Kp * (set_position_in_image.z - current_position_in_image.z);
                pixhawk_uav.SetV(set_vel.x, set_vel.y, set_vel.z);
                break;

            default:
                /* Default Code */
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}