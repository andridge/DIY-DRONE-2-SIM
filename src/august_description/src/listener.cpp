#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <iostream>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::string output = "Received joint states: ";
  for (size_t i = 0; i < msg->name.size(); ++i) {
    output += msg->name[i] + "=" + std::to_string(msg->position[i]) + " ";
  }
  ROS_INFO("%s", output.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_listener");
  ros::NodeHandle nh;

  // Subscribe to the controller's published joint states
  ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointStateCallback);

  ros::spin();
  return 0;
}
