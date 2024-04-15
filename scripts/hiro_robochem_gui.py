from datetime import datetime, timedelta
import rospy
from tkinter import *
import time
from xbox_input import *

root = Tk()

root.title("RoboChem")

root.geometry('350x200')

lbl = Label(root, text="Enter number of hours")
lbl.grid()

txt = Entry(root, width=10)
txt.grid(column=1, row=0)

lbl = Label(root, text="Enter number of minutes")
lbl.grid(column=0, row=1)

txt2 = Entry(root, width=10)
txt2.grid(column=1, row=1)

lbl = Label(root, text="Enter number of seconds")
lbl.grid(column=0, row=2)

txt3 = Entry(root, width=10)
txt3.grid(column=1, row=2)

def clicked():
    hours = txt.get()
    minutes = txt2.get()
    seconds = txt3.get()
    
    msg = "Hours:" + hours + " Minutes:" + minutes + " Seconds:" + seconds 

    lbl = Label(root, text=msg)
    lbl.grid(column=0, row=3)
    
    hours=int(hours)
    minutes=int(minutes)
    seconds=int(seconds)

    t = (hours*3600) + (minutes*60) + seconds
    display(hours, minutes, seconds, t)

def display(hours, mins, secs, t):
    while t:
        cur_time = timedelta(seconds=t)
        timer = 'Time until next transfer: ' + str(cur_time)
        time.sleep(1)
        t -= 1
        new_time = Label(root, text=timer)
        new_time.grid(column=0, row=4)
        root.update_idletasks()
        root.update()

btn = Button(root, text = "Enter", fg ="coral", command=clicked)

btn.grid(column=2, row=0)

root.mainloop()

if __name__ == '__main__':
    flag = rospy.get_param("/xbox_input/flag")
    rospy.init_node('xbox_input')
    xController = XboxInput(flag=flag)
    rospy.spin()