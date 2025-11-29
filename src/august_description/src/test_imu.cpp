#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("IMU Data - Accel: [%.3f, %.3f, %.3f] | Gyro: [%.3f, %.3f, %.3f]",
             msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z,
             msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_imu");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/august/imu/data", 10, imuCallback);
    ROS_INFO("Listening to IMU data on /august/imu/data...");
    ros::spin();
    return 0;
}
