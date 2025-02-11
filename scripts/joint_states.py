#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
import readchar

rospy.init_node('joint_state_listener')

joint_states = []
recording = False 

file_path = "/home/caleb/ros_relaxed_ik_ws/src/relaxed_ik_ros1/scripts/hesitant_arc.npz"
global joint_positions
global joint_velocities
global times
joint_positions = []
joint_velocities = []
times = []
# Callback function for the 'joint_states' topic
def joint_state_callback(data):
    global joint_positions
    global joint_velocities
    global recording
    global times
    if recording:
        joint_positions.append(list(data.position[:7]))
        joint_velocities.append(list(data.velocity[:7]))
        #times.append(data.header.stamp.to_sec())
        #print("TIMESTEP", times[0], type(times[0]))
        #joint_state_data = {
        #    "positions": joint_positions,
        #    "velocities": joint_velocities,
        #}

        #joint_states.append(joint_state_data)

        #log_data = (
        #    f"Joint Positions: {joint_positions}\n"
        #    f"Joint Velocities: {joint_velocities}\n"
        #)
        #rospy.loginfo("Recording Joint State Data:")
        #rospy.loginfo(log_data)


def save_data():
    global joint_velocities
    global joint_positions
    global times
    initial_q = joint_positions[0]
    if True:  
        print("THE LGNTH OF JOINT POSITIONS", len(joint_positions))
        #np.save(file_path, joint_states)
        np.savez("shy.npz", q=joint_positions, qdot=joint_velocities, q_init = initial_q, num_steps = len(joint_positions), time_step = times)
        rospy.loginfo("Joint state data saved to %s", file_path)
    else:
        rospy.loginfo("No data to save.")

# Function to handle keypress for starting/stopping recording
def keypress_listener():
    global recording
    print("Press 'r' to start/stop recording joint states.")
    while not rospy.is_shutdown():
        key = readchar.readchar()
        if key == 'r':  # Start/stop recording on 'r' key
            recording = not recording
            if recording:
                start = time.time()
                rospy.loginfo("Recording started.")
            else:
                end = time.time()
                elapsed = end - start
                print("ELAPSED TIME", elapsed)
                rospy.loginfo("Recording stopped.")
        elif key == 'q':  # Quit the program on 'q' key
            rospy.signal_shutdown("User requested shutdown.")
            break

# Subscribe to the 'joint_states' topic
rospy.Subscriber("joint_states", JointState, joint_state_callback)

# Start the keypress listener in the background
import threading
keypress_thread = threading.Thread(target=keypress_listener)
keypress_thread.daemon = True
keypress_thread.start()

# Register the shutdown hook to save data on exit
rospy.on_shutdown(save_data)

# Keep the node running
rospy.spin()
