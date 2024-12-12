#! /usr/bin/env python3
import actionlib
import franka_gripper.msg
import rospy
import sys
from std_msgs.msg import Float32MultiArray


class hiro_grasp:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.__width = 0.1
        self.__inner = 0.2
        self.__outer = 0.2
        self.__speed = 0.1
        self.__force = 10

    def __set_open_grasp(self):
        self.__width = 0.1
        self.__inner = 0.1
        self.__outer = 0.1
        self.__speed = 0.1
        self.__force = 10

    def grasp(self):
        # self.client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.width = self.__width
        goal.epsilon.inner = self.__inner
        goal.epsilon.outer = self.__outer
        goal.speed = self.__speed
        goal.force = self.__force
        self.client.send_goal(goal)
        # self.client.wait_for_result()

    def open(self):
        self.__set_open_grasp()
        self.grasp()
    
    def set_grasp_width(self, width):
        self.__width = width

if __name__ == '__main__':
    hiro_g = hiro_grasp()
