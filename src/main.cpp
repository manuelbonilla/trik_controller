#include <ros/ros.h>
#include "trik_controller/disable.h"
#include "trik_controller/enable.h"
#include "trik_controller/is_enabled.h"
#include <controller_manager_msgs/SwitchController.h>

bool is_enabled_flag;
ros::ServiceClient client;

bool cb_enable(trik_controller::enable::Request  &req,
               trik_controller::enable::Response &res)
{
  ROS_INFO("Enabling trik_controller");

  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.start_controllers.push_back("teleoperation_controller_mt_effort");
  switch_srv.request.stop_controllers.push_back("joint_trajectory_controller");
  switch_srv.request.stop_controllers.push_back("stiffness_trajectory_controller");
  switch_srv.request.stop_controllers.push_back("damping_trajectory_controller");
  switch_srv.request.stop_controllers.push_back("add_torque_trajectory_controller");
  switch_srv.request.strictness = 2;

  if (!client.call(switch_srv))
  {
    res.result =  2;
  }
  else
  {
    is_enabled_flag = true;
    ROS_INFO("Controller enabled");
    res.result =  1;
  }
  return true;
}

bool cb_disable(trik_controller::disable::Request  &req,
                trik_controller::disable::Response &res)
{
  ROS_INFO("Disabling trik_controller");
  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.stop_controllers.push_back("teleoperation_controller_mt_effort");
  switch_srv.request.start_controllers.push_back("joint_trajectory_controller");
  switch_srv.request.start_controllers.push_back("stiffness_trajectory_controller");
  switch_srv.request.start_controllers.push_back("damping_trajectory_controller");
  switch_srv.request.start_controllers.push_back("add_torque_trajectory_controller");
  switch_srv.request.strictness = 2;

  if (!client.call(switch_srv))
  {
    res.result =  2;
  }
  else
  {
    is_enabled_flag = false;
    res.result =  1;
  }
  return true;
}

bool cb_is_enable(trik_controller::is_enabled::Request  &req,
                  trik_controller::is_enabled::Response &res)
{
  ROS_INFO("Asking if trik_controller is enabled");
  res.result =  is_enabled_flag;
  return true;
}

int main(int argc, char **argv)
{
  is_enabled_flag = false;
  ros::init(argc, argv, "trik_controller");
  ros::NodeHandle n;

  client = n.serviceClient<controller_manager_msgs::SwitchController>("/right_arm/controller_manager/switch_controller");

  ros::ServiceServer srv_enable = n.advertiseService("enable", cb_enable);
  ros::ServiceServer srv_disable = n.advertiseService("disable", cb_disable);
  ros::ServiceServer srv_is_enable = n.advertiseService("is_enabled", cb_is_enable);

  ros::spin();

  return 0;
}