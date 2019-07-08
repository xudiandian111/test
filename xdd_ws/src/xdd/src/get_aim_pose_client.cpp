#include <ros/ros.h>
#include <xdd/get_aim.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "get_aim_pose_client");
  ros::NodeHandle n;
  
  // 创建一个client，
  ros::ServiceClient client = n.serviceClient<xdd::get_aim>("get_aimpose_num");
  
  // 创建
  xdd::get_aim srv;
  srv.request.aim_num = 1;
  
  // 发布service请求
  if (client.call(srv))
  {
    std::cout<<"back_home:"<<srv.response.is_back_home<<std::endl;
    cout << "srv.response.is_back_home: "<< srv.response.is_back_home << endl;
    // srv.request.aim_num += 1;
  }
  else
  {
    ROS_ERROR("Failed to call service get_aim_pose");
    cout <<"srv.response.is_back_home: "<< srv.response.is_back_home << endl;
    return 1;
  }

  return 0;
}