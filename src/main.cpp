#include <ros/ros.h>
#include "trik_controller/disable.h"
#include "trik_controller/enable.h"
#include "trik_controller/is_enabled.h"
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_msgs/Bool.h>

bool is_enabled_flag;
ros::ServiceClient srv_c_sw, srv_c_l;
ros::Publisher pub_activate_control;
std::string controller_to_use;

std::vector<std::string> lcld; // list of controllers loaded by default
std::vector<std::string> lcsd; // list of controllers stoped by default

bool cb_enable(trik_controller::enable::Request  &req,
               trik_controller::enable::Response &res)
{
  ROS_INFO("Enabling trik_controller");

  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.start_controllers.push_back(controller_to_use);
  switch_srv.request.stop_controllers.insert(switch_srv.request.stop_controllers.end(), lcld.begin(), lcld.end());
  switch_srv.request.strictness = 2;

  if (!srv_c_sw.call(switch_srv))
  {
    res.result =  2;
  }
  else
  {
    is_enabled_flag = true;
    ROS_INFO("Controller enabled");
    res.result =  1;
    std_msgs::Bool msg_ac;
    msg_ac.data = true;
    pub_activate_control.publish(msg_ac);
    ros::spinOnce();
  }
  return true;
}

bool cb_disable(trik_controller::disable::Request  &req,
                trik_controller::disable::Response &res)
{
  ROS_INFO("Disabling trik_controller");
  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.stop_controllers.push_back(controller_to_use);
  switch_srv.request.start_controllers.insert(switch_srv.request.start_controllers.end(), lcld.begin(), lcld.end());
  switch_srv.request.strictness = 2;

  if (!srv_c_sw.call(switch_srv))
  {
    res.result =  2;
  }
  else
  {
    is_enabled_flag = false;
    res.result =  1;
    std_msgs::Bool msg_ac;
    msg_ac.data = false;
    pub_activate_control.publish(msg_ac);
    ros::spinOnce();
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
  ROS_INFO("trik_controller started");
  ros::NodeHandle n;

  n.param<std::string>("controller", controller_to_use , "teleoperation_controller_mt_effort");

  std::string arm;
  n.param<std::string>("name_arm", arm , "right_arm");

  srv_c_sw = n.serviceClient<controller_manager_msgs::SwitchController>( arm + std::string("/controller_manager/switch_controller"));
  srv_c_l = n.serviceClient<controller_manager_msgs::ListControllers>( arm + std::string("/controller_manager/list_controllers"));
  pub_activate_control = n.advertise<std_msgs::Bool>( arm + controller_to_use + std::string("/start_controller"), 1);

  ros::ServiceServer srv_enable = n.advertiseService("enable", cb_enable);
  ros::ServiceServer srv_disable = n.advertiseService("disable", cb_disable);
  ros::ServiceServer srv_is_enable = n.advertiseService("is_enabled", cb_is_enable);
  ros::spinOnce();

  controller_manager_msgs::ListControllers list_c;
  srv_c_l.call(list_c.request, list_c.response);
  ros::spinOnce();
  std::vector<controller_manager_msgs::ControllerState>& controllers = list_c.response.controller;

  for (auto c : controllers)
  {
    if (c.state == std::string("stopped"))
    {
      lcsd.push_back(c.name);
    }
    else
    {
      if (c.state == std::string("running") && c.name != std::string("joint_state_controller"))
      {
        lcld.push_back(c.name);
      }
    }
  }

  ros::spin();

  return 0;
}