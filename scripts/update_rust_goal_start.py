#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import yaml
import os

class JointStateUpdater:
    def __init__(self, yaml_path):
        # Initialize the ROS node
        rospy.init_node('joint_state_yaml_updater', anonymous=True)
        
        # Store the yaml path
        self.yaml_path = yaml_path
        
        # Flag to ensure we only update once
        self.has_updated = False
        
        # Subscribe to joint states
        rospy.Subscriber("/joint_states", JointState, self.callback)
        
    def callback(self, msg):
        if not self.has_updated:
            # Get the positions from the message
            positions = list(msg.position)
            
            try:
                # Read the current YAML file
                with open(self.yaml_path, 'r') as file:
                    data = yaml.safe_load(file)
                
                # Update the starting_config
                data['starting_config'] = positions[:7]
                
                # Write back to the YAML file
                with open(self.yaml_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=None)
                
                # rospy.loginfo(f"Updated starting_config to: {positions}")
                if positions[0] != 0:
                    self.has_updated = True
                
            except Exception as e:
                rospy.logerr(f"Error updating YAML file: {e}")

def main():
    # Construct path to the YAML file (assuming it's in the same directory)
    yaml_path = "/home/caleb/ros_relaxed_ik_ws/src/relaxed_ik_ros1/relaxed_ik_core/configs/settings.yaml" # Change "info.yaml" to your actual filename
    
    # Create updater instance
    updater = JointStateUpdater(yaml_path)
    
    # Keep the node running
    rospy.loginfo("Waiting for joint states...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass