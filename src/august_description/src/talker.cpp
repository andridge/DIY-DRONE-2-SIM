#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <string>
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_talker");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("drone_joint_commands", 10);

  ros::Rate loop_rate(2); // 2 Hz
  //std::vector<std::string> joint_names = {"Revolute47", "Revolute48", "Revolute49", "Revolute50"};
  std::vector<std::string> joint_names;
  joint_names.push_back("Revolute 47");
  joint_names.push_back("Revolute 48");
  joint_names.push_back("Revolute 49");
  joint_names.push_back("Revolute 50");
  int count = 0;

  while (ros::ok())
  {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = joint_names;
    double t = count * 0.5;
   // msg.position = {sin(t), cos(t), sin(2*t), cos(2*t)};
  msg.position.clear();
  msg.position.push_back(sin(t));
  msg.position.push_back(cos(t));
  msg.position.push_back(sin(2*t));
  msg.position.push_back(cos(2*t));
    pub.publish(msg);
    ROS_INFO("Publishing joint positions: [%.2f, %.2f, %.2f, %.2f]",
      msg.position[0], msg.position[1], msg.position[2], msg.position[3]);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
}