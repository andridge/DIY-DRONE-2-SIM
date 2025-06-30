#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <vector>
#include <iostream>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::string output = "Received joint positions: ";
  for (size_t i = 0; i < msg->name.size(); ++i) {
    output += msg->name[i] + "=" + std::to_string(msg->position[i]) + " ";
  }
  ROS_INFO("%s", output.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("drone_joint_commands", 10, jointStateCallback);

  ros::spin();
  return 0;
}