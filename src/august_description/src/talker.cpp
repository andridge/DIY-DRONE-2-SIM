#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <vector>
#include <cmath>

// Global variables to track joint positions (still optional if /joint_states is available)
std::vector<double> joint_positions(4, 0.0);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Store joint positions if they exist, but don't block logic on them
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "revolute_47_joint") joint_positions[0] = msg->position[i];
        else if (msg->name[i] == "revolute_48_joint") joint_positions[1] = msg->position[i];
        else if (msg->name[i] == "revolute_49_joint") joint_positions[2] = msg->position[i];
        else if (msg->name[i] == "revolute_50_joint") joint_positions[3] = msg->position[i];
    }

    ROS_INFO("Joint positions (optional): [%.2f, %.2f, %.2f, %.2f]",
             joint_positions[0], joint_positions[1],
             joint_positions[2], joint_positions[3]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_talker");
    ros::NodeHandle nh;

    // Publishers for each propeller effort controller
    ros::Publisher pub47 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_47_effort_controller/command", 10);
    ros::Publisher pub48 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_48_effort_controller/command", 10);
    ros::Publisher pub49 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_49_effort_controller/command", 10);
    ros::Publisher pub50 = nh.advertise<std_msgs::Float64>("/august/august_controller/revolute_50_effort_controller/command", 10);

    // Subscribe to joint states (optional)
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    ros::Rate loop_rate(20); // 20 Hz update rate
    std_msgs::Float64 cmd47, cmd48, cmd49, cmd50;

    // Initial commands (all zero)
    cmd47.data = 0.0;
    cmd48.data = 0.0;
    cmd49.data = 0.0;
    cmd50.data = 0.0;

    // State machine variables
    enum State { START, TEST_47, TEST_48, TEST_49, TEST_50, ALL_ACTIVE, LIFT_OFF };
    State current_state = START;

    double target_thrust = 0.1; // Target thrust for lift-off
    double current_thrust = 0.0;
    double thrust_increment = 0.001; // Gradual increase per second

    ROS_INFO("Starting rotor initialization sequence (clockwise only).");

    while (ros::ok())
    {
        ros::spinOnce(); // Process callbacks if /joint_states is publishing

        switch (current_state)
        {
            case START:
                ROS_INFO("Skipping wait — starting rotor 47 test...");
                current_state = TEST_47;
                break;

            case TEST_47:
                cmd47.data = 0.3; // Clockwise
                ROS_INFO("Rotor 47 spinning clockwise...");
                current_state = TEST_48;
                break;

            case TEST_48:
                cmd47.data = 0.0;
                cmd48.data = 0.3; // Clockwise (changed from -0.3)
                ROS_INFO("Rotor 48 spinning clockwise...");
                current_state = TEST_49;
                break;

            case TEST_49:
                cmd48.data = 0.0;
                cmd49.data = 0.3; // Clockwise
                ROS_INFO("Rotor 49 spinning clockwise...");
                current_state = TEST_50;
                break;

            case TEST_50:
                cmd49.data = 0.0;
                cmd50.data = 0.3; // Clockwise (changed from -0.3)
                ROS_INFO("Rotor 50 spinning clockwise...");
                current_state = ALL_ACTIVE;
                break;

            case ALL_ACTIVE:
                current_thrust += thrust_increment / 20.0; // Increment per loop (20Hz)
                if (current_thrust < target_thrust)
                {
                    // All motors spinning clockwise
                    cmd47.data = current_thrust;
                    cmd48.data = current_thrust; // Changed from -current_thrust
                    cmd49.data = current_thrust;
                    cmd50.data = current_thrust; // Changed from -current_thrust
                    ROS_INFO("Increasing thrust: %.2f/%.2f", current_thrust, target_thrust);
                }
                else
                {
                    current_state = LIFT_OFF;
                    ROS_INFO("Maximum thrust achieved! All motors spinning clockwise...");
                }
                break;

            case LIFT_OFF:
                // All motors spinning clockwise at target thrust
                cmd47.data = target_thrust;
                cmd48.data = target_thrust; // Changed from -target_thrust
                cmd49.data = target_thrust;
                cmd50.data = target_thrust; // Changed from -target_thrust
                break;
        }

        // Publish commands
        pub47.publish(cmd47);
        pub48.publish(cmd48);
        pub49.publish(cmd49);
        pub50.publish(cmd50);

        loop_rate.sleep();
    }

    return 0;
}