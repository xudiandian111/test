#include "xdd/uav.h"

uav::uav()
{
    local_pos_pub = pixhawk.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    velocity_pub = pixhawk.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);

    position_sub = pixhawk.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, &uav::PoseCallback, this);
    velocity_sub = pixhawk.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 1, &uav::VelocityCallback, this);
    state_sub = pixhawk.subscribe<mavros_msgs::State>
            ("mavros/state", 1, &uav::StateCallback, this);

    set_mode_client = pixhawk.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    arming_client = pixhawk.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    
    theta = 12.0 / 180.0 * 3.14159;
}

uav::~uav()
{

}

void uav::SetPosition(geometry_msgs::Point _set_position)
{
    set_position.pose.position.x = _set_position.y * cos(theta) - _set_position.x * sin(theta);
    set_position.pose.position.y = (-1.0) * _set_position.y * sin(theta) - _set_position.x * cos(theta);
    set_position.pose.position.z = _set_position.z;

    local_pos_pub.publish(set_position);
}

void uav::SetV(double _set_linear_vx, double _set_linear_vy, double _set_linear_vz,
                double _set_angular_vx, double _set_angular_vy, double _set_angular_vz)
{
    set_velocity.linear.x = _set_linear_vx;
    set_velocity.linear.y = _set_linear_vy;
    set_velocity.linear.z = _set_linear_vz; 

    set_velocity.angular.x = _set_angular_vx;
    set_velocity.angular.y = _set_angular_vy;
    set_velocity.angular.z = _set_angular_vz; 

    velocity_pub.publish(set_velocity);
}

bool uav::SetMode(std::string _set_mode)
{
    set_mode.request.custom_mode = _set_mode;
    return (set_mode_client.call(set_mode) && set_mode.response.mode_sent);
}

bool uav::ChangeArmingStatus(bool _set_status)
{
    arm_cmd.request.value = _set_status;
    return (arming_client.call(arm_cmd) && arm_cmd.response.success);
}

void uav::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _pose_msg)
{
    current_pose = *_pose_msg;
}

void uav::GetCurrentPosition(geometry_msgs::Point & _current_position)
{
    _current_position.x = (-1.0) * current_pose.pose.position.x * sin(theta) - current_pose.pose.position.y * cos(theta);
    _current_position.y = current_pose.pose.position.x * cos(theta) - current_pose.pose.position.y * sin(theta);
    _current_position.z = current_pose.pose.position.z;
}

void uav::GetCurrentQuaternion(geometry_msgs::Quaternion & _current_quaternion)
{
    _current_quaternion = current_pose.pose.orientation;
}

void uav::GetCurrentEulerAngle(double & _roll, double & _pitch, double & _yaw)
{
    tf::Quaternion tf_orientation;
    tf::quaternionMsgToTF(current_pose.pose.orientation, tf_orientation);
    tf::Matrix3x3(tf_orientation).getRPY(_roll, _pitch, _yaw);
}

void uav::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& _velocity_msg)
{
    current_velocity = *_velocity_msg;
}

void uav::GetCurrentLinearV(double & _current_linear_vx, double & _current_linear_vy, double & _current_linear_vz)
{
    _current_linear_vx = current_velocity.twist.linear.x;
    _current_linear_vy = current_velocity.twist.linear.y;
    _current_linear_vz = current_velocity.twist.linear.z;
}

void uav::GetCurrentAngularV(double & _current_angular_vx, double & _current_angular_vy, double & _current_angular_vz)
{
    _current_angular_vx = current_velocity.twist.angular.x;
    _current_angular_vy = current_velocity.twist.angular.y;
    _current_angular_vz = current_velocity.twist.angular.z;
}

void uav::StateCallback(const mavros_msgs::State::ConstPtr& _state_msg)
{
    current_state = *_state_msg;
}

bool uav::GetConnectedStatus()
{
    return current_state.connected;
}

bool uav::GetArmedStatus()
{
    return current_state.armed;
}

std::string uav::GetCurrentMode()
{
    return current_state.mode;
}