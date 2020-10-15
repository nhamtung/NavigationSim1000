#include "ros/ros.h"
#include <move_base/lift_up.h>


bool lift_up_srv(move_base::lift_up::Request  &req, move_base::lift_up::Response &res)
{
  ROS_INFO("agv_service.cpp-7-lift_up_srv()");

  res.sum = req.a + req.b;
  ROS_INFO("agv_service.cpp-10-request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("agv_service.cpp-11-sending back response: [%ld]", (long int)res.sum);
  return true;
}
bool lift_down_srv(move_base::lift_up::Request  &req, move_base::lift_up::Response &res)
{
  ROS_INFO("agv_service.cpp-16-lift_down_srv()");
  return true;
}
bool charging_in_srv(move_base::lift_up::Request  &req, move_base::lift_up::Response &res)
{
  ROS_INFO("agv_service.cpp-21-charging_in_srv()");
  return true;
}
bool charging_out_srv(move_base::lift_up::Request  &req, move_base::lift_up::Response &res)
{
  ROS_INFO("agv_service.cpp-26-charging_out_srv()");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agv_service");

  ros::NodeHandle n;
  ros::ServiceServer lift_up_service_ = n.advertiseService("lift_up_", lift_up_srv);
  ROS_INFO("agv_service.cpp-36-Advertise service: lift_up_");
  ros::ServiceServer lift_down_service_ = n.advertiseService("lift_down_", lift_down_srv);
  ROS_INFO("agv_service.cpp-38-Advertise service: lift_down_");
  ros::ServiceServer chaging_in_service = n.advertiseService("charging_in_", charging_in_srv);
  ROS_INFO("agv_service.cpp-40-Advertise service: chaging_in_");
  ros::ServiceServer charging_out_service = n.advertiseService("charging_out_", charging_out_srv);
  ROS_INFO("agv_service.cpp-42-Advertise service: chaging_out_");

  ros::spin();
  return 0;
}
