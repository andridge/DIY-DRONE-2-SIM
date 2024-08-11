import rospy
from std_msgs.msg import Float64

# Define the joint topics based on the naming conventions in the XML
joint_topics = {
    'Revolute 55_position_controller': '/august/Revolute_55_actr/command',
    'Revolute 56_position_controller': '/august/Revolute_56_actr/command',
    'Revolute 57_position_controller': '/august/Revolute_57_actr/command',
    'Revolute 58_position_controller': '/august/Revolute_58_actr/command'
}

def move_joint(joint_name, position):
    # Check if the joint name is in the joint_topics dictionary
    if joint_name in joint_topics:
        # Create a publisher for the specified joint topic
        pub = rospy.Publisher(joint_topics[joint_name], Float64, queue_size=10)
        # Wait for publisher to set up
        rospy.sleep(1)
        # Log the position to which the joint is moving
        rospy.loginfo(f"Moving {joint_name} to position: {position}")
        # Publish the position
        pub.publish(position)
    else:
        rospy.logerr(f"Joint name {joint_name} is not defined in the joint_topics dictionary.")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('joint_controller', anonymous=True)

    # Example: Move Revolute 55 to position 1.0
    move_joint('Revolute 55_position_controller', 1.0)

    # Example: Move Revolute 56 to position 0.5
    move_joint('Revolute 56_position_controller', 0.5)

    # Example: Move Revolute 57 to position -0.3
    move_joint('Revolute 57_position_controller', -0.3)

    # Example: Move Revolute 58 to position 0.8
    move_joint('Revolute 58_position_controller', 0.8)

    # Keep the node running
    rospy.spin()
