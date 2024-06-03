from datetime import datetime, timedelta
import rospy
import rospkg
from tkinter import *
from xbox_input import XboxInput as RobotController
import time
import os
import hiro_grasp
import multiprocessing as mping
class hiro_gui_display:
    def __init__(self):
        mping.set_start_method('forkserver')
        p_display = mping.Process(name='p_display', target=self.create_display)
        p_ros = mping.Process(name='p_robo', target=self.init_robo)
        p_display.start()
        p_ros.start()
        p_ros.join()

    def init_robo(self):
        path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

        #Init and spin ros node
        flag = rospy.set_param('hiro_robochem/flag', 'list')
        xController = RobotController(flag=flag)


    def create_display(self):
        #Create Tk window and add buttons/text boxes
        self.root = Tk()
        self.root.title("RoboChem")
        self.root.geometry('350x200')

        self.lbl = Label(self.root, text="Enter number of hours")
        self.lbl.grid()

        self.txt = Entry(self.root, width=10)
        self.txt.grid(column=1, row=0)

        self.lbl = Label(self.root, text="Enter number of minutes")
        self.lbl.grid(column=0, row=1)

        self.txt2 = Entry(self.root, width=10)
        self.txt2.grid(column=1, row=1)

        self.lbl = Label(self.root, text="Enter number of seconds")
        self.lbl.grid(column=0, row=2)

        self.txt3 = Entry(self.root, width=10)
        self.txt3.grid(column=1, row=2)

        self.btn = Button(self.root, text = "Enter", fg ="coral", command=self.clicked)
        self.btn.grid(column=2, row=0)

        self.btn2 = Button(self.root, text="Execute transfers", command=self.closeGrip)
        self.btn2.grid(column=0, row=6)

        self.root.mainloop() # start loop on window

    def clicked(self):
        hours = self.txt.get()
        minutes = self.txt2.get()
        seconds = self.txt3.get()

        hours=int(hours)
        minutes=int(minutes)
        seconds=int(seconds)

        t = (hours*3600) + (minutes*60) + seconds
        self.display(hours, minutes, seconds, t)

    def display(self, hours, mins, secs, t):
        while t:
            cur_time = timedelta(seconds=t)
            timer = 'Time until next transfer: ' + str(cur_time)
            new_time = Label(self.root, text=timer)
            new_time.grid(column=0, row=4)
            self.root.update()
            self.root.update_idletasks()
            time.sleep(1)
            t -= 1
        new_time = Label(self.root, text="Executing next transfer,\n please stay clear of the robot")
        new_time.grid(column=0, row=4)

    def closeGrip(self):
        print("TYPE", type(self))
        self.hiro_g = hiro_grasp.hiro_grasp()
        print("HIRO_G", self.hiro_g)
        RobotController.grip_inc = RobotController.grip_cur
        self.hiro_g.grip_cur -= self.hiro_g.grip_inc
        self.move_gripper()

if __name__ == '__main__':   
    rospy.init_node('hiro_robochem')
    hiro_gui_display() 





