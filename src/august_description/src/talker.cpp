#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_talker");
  ros::NodeHandle nh;

  // Publishers for each propeller effort controller
  ros::Publisher pub47 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_47_effort_controller/command", 10);
  ros::Publisher pub48 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_48_effort_controller/command", 10);
  ros::Publisher pub49 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_49_effort_controller/command", 10);
  ros::Publisher pub50 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_50_effort_controller/command", 10);

  ros::Rate loop_rate(20); // 20 Hz update rate

  std_msgs::Float64 cmd47, cmd48, cmd49, cmd50;
  cmd47.data = 0.5;   // CW
  cmd48.data = -0.5;  // CCW
  cmd49.data = 0.5;   // CW
  cmd50.data = -0.5;  // CCW

  while (ros::ok())
  {
    pub47.publish(cmd47);
    pub48.publish(cmd48);
    pub49.publish(cmd49);
    pub50.publish(cmd50);

    ROS_INFO("Publishing: [%.2f, %.2f, %.2f, %.2f]",
             cmd47.data, cmd48.data, cmd49.data, cmd50.data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
