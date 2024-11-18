#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState

# Path to the .npy file with recorded joint states
file_path = "/home/caleb/ros_relaxed_ik_ws/src/relaxed_ik_ros1/scripts/shy.npy"

# Initialize the node
rospy.init_node('pilot_study')

# Publisher for the joint states
joint_state_pub = rospy.Publisher('/franka_state_controller/joint_states', JointState, queue_size=10)

# Load joint state data from .npy file
try:
    joint_states = np.load(file_path, allow_pickle=True)
    rospy.loginfo("Loaded joint state data from %s", file_path)
except Exception as e:
    rospy.logerr("Failed to load joint state data: %s", e)
    exit(1)

# Publish joint states at a fixed rate
rate = rospy.Rate(10)  # Publish at 10 Hz

# Main publishing loop
while not rospy.is_shutdown():
    for joint_state_data in joint_states:
        # Create and populate a JointState message
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.position = joint_state_data["positions"]
        joint_msg.velocity = joint_state_data["velocities"]
        
        # Publish the message
        joint_state_pub.publish(joint_msg)
        
        # Log for debugging
        rospy.loginfo("Published Joint State: %s", joint_msg)

        # Wait for the next cycle
        rate.sleep()
        
    # Once all joint states have been published, exit the loop
    rospy.loginfo("All joint states published.")
    break
