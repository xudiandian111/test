#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/Point.h"

#include <string>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <math.h>

//#include "iostream"

class uav
{
public:
	uav();
	virtual ~uav();
	void SetPosition(geometry_msgs::Point _set_position);
	void SetV(double _set_linear_vx, double _set_linear_vy, double _set_linear_vz,
					  double _set_angular_vx = 0.0, double _set_angular_vy = 0.0, double _set_angular_vz = 0.0);
					  
	bool SetMode(std::string _set_mode);
	bool ChangeArmingStatus(bool _set_status);

	void GetCurrentPosition(geometry_msgs::Point & _current_position);
	void GetCurrentQuaternion(geometry_msgs::Quaternion & _current_quaternion);
	void GetCurrentEulerAngle(double & _roll, double & _pitch, double & _yaw);

	void GetCurrentLinearV(double & _current_linear_vx, double & _current_linear_vy, double & _current_linear_vz);
	void GetCurrentAngularV(double & _current_angular_vx, double & _current_angular_vy, double & _current_angular_vz);
	bool GetConnectedStatus();
	bool GetArmedStatus();
	std::string GetCurrentMode();

private:
	/* data */
	ros::NodeHandle pixhawk;

	ros::Publisher local_pos_pub;
	ros::Publisher velocity_pub;

	ros::Subscriber position_sub;
	ros::Subscriber velocity_sub;
	ros::Subscriber state_sub;

	ros::ServiceClient set_mode_client;
	ros::ServiceClient arming_client;

	geometry_msgs::PoseStamped set_position;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::TwistStamped current_velocity;
	geometry_msgs::Twist set_velocity;
	mavros_msgs::SetMode set_mode;
	mavros_msgs::CommandBool arm_cmd;
	mavros_msgs::State current_state;

	double theta;

	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _pose_msg);
	void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _velocity_msg);
	void StateCallback(const mavros_msgs::State::ConstPtr& _state_msg);
};