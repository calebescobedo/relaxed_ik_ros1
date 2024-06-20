#!/usr/bin/env python3

import customtkinter
import tkinter.font
import os
import hiro_grasp
import rospy
import rospkg
import threading
import tkinter
from customtkinter import *
from datetime import datetime, timedelta
from std_msgs.msg import String
from tkinter import *
# from xbox_input_test import *

def validate_time_format(time_str):
    if time_str == "0": return False
    try:
        datetime.strptime(time_str, '%H')
        return True
    except ValueError:
        return False

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        self.gohead_pub = rospy.Publisher("/gui_pub", String, queue_size=1)
        self.xbox_sub = rospy.Subscriber("/gui_done", String, self.gui_done_cb)
        script_dir = os.path.dirname(__file__)
        theme_path = os.path.join(script_dir, 'Hades.json')
        customtkinter.deactivate_automatic_dpi_awareness()
        customtkinter.set_default_color_theme(theme_path)

        self.configure(fg_color=("darkblue", "#213844"))
        self.geometry("800x600")
        self.title("RoboDialysis GUI")
        self.font = customtkinter.CTkFont(family="Roboto", size=13)
        self.grid_columnconfigure(0, weight=1)

        self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
        self.textbox.tag_config("centered", justify="center")
        self.textbox.grid(row=0, column=0, padx="10", pady="20", sticky="ew")
        self.textbox.insert(INSERT, "Welcome to HIRO Robochem GUI, press the start button to begin\n", "centered")
        self.textbox.configure(state=DISABLED)

        self.button = customtkinter.CTkButton(self, text="Start", command=self.button_cb)
        self.button.grid(row=4, column=0, padx=20, pady=20, sticky="ew")
        
        self.error_label = None  # Initialize error_label variable
        self.transfer_times = []
        
        self.transfer_button = None  # Initialize transfer_button variable

        self.start_flag = ""

    def log_message(self, message):
        self.textbox.configure(state=NORMAL)
        self.textbox.insert(END, message + "\n")
        self.textbox.configure(state=DISABLED)
        self.textbox.yview(END)

    def button_cb(self):
        # Destroy error label if it exists
        if self.error_label and self.error_label.winfo_exists():
            self.error_label.destroy()
        
        self.prompt_transfer_count()

    def prompt_transfer_count(self):
        self.dialog = customtkinter.CTkInputDialog(text="Enter the number of buckets (default is 6):", title="Number of transfers between buckets")
        num_transfers = self.dialog.get_input()
        try:
            num_transfers = int(num_transfers)
            if num_transfers < 1:
                self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid number greater than 0")
                self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
            else:
                self.handle_transfers(num_transfers)
        except ValueError:
            self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid integer")
            self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

    def prompt_time_delay(self):
        self.dialog = customtkinter.CTkInputDialog(text="Enter the number of hours (1-23) before transfer 1:", title="Time Delay")
        time_input = self.dialog.get_input()
        while not validate_time_format(time_input):
            self.dialog = customtkinter.CTkInputDialog(text="Invalid format! Please enter a whole number of hours.", title="Invalid Input")
            time_input = self.dialog.get_input()
        self.log_message(f"Hours before transfer 1: {time_input}")
        self.transfer_times = []
        self.transfer_times.append(time_input)

    def handle_transfers(self, num_transfers):
        self.prompt_time_delay()
        self.prompt_next_transfer_time(1, num_transfers)

    def prompt_next_transfer_time(self, idx, total_transfers):
        if idx < total_transfers:
            self.dialog = customtkinter.CTkInputDialog(text=f"Enter the number of hours (1-23) between transfer {idx} and {idx + 1}:", title=f"Time betweeen transfer {idx} and transfer {idx + 1}")
            time_input = self.dialog.get_input()
            while not validate_time_format(time_input):
                self.dialog = customtkinter.CTkInputDialog(text="Invalid format! Please enter a whole number of hours between 1 and 23.", title="Invalid Input")
                time_input = self.dialog.get_input()
            self.log_message(f"Hours between transfer {idx} and {idx + 1}: {time_input}")
            self.transfer_times.append(time_input)
            self.prompt_next_transfer_time(idx + 1, total_transfers)
        else:
            self.create_execute_button()

    def create_execute_button(self):
        if not self.transfer_button:
            self.transfer_button = customtkinter.CTkButton(self, text="Execute Transfers", command=self.start_transfers)
            self.transfer_button.grid(row=5, column=0, padx=20, pady=20, sticky="ew")

    def start_transfers(self):
        # Clear the window
        for widget in self.winfo_children():
            widget.destroy()

        self.geometry("800x600")
        self.grid_columnconfigure(0, weight=1)
        self.font = customtkinter.CTkFont(family="Roboto", size=13)
        
        self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
        self.textbox.grid(row=0, column=0, padx="10", pady="20", sticky="ew")
        self.textbox.configure(state=DISABLED)

        for idx, time_str in enumerate(self.transfer_times):
            if idx < 1: self.log_message(f"Hours until transfer {idx + 1}: {time_str}")
            elif idx < 0: 
                self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid number greater than 0")
                self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
            else: self.log_message(f"Time between transfer {idx} and transfer {idx + 1}: {time_str}")

        self.current_transfer = 0
        self.execute_next_transfer()

    def execute_next_transfer(self):    
        if self.current_transfer <= len(self.transfer_times):
            time_str = self.transfer_times[self.current_transfer]
            remaining_hours = int(time_str)
            remaining_seconds = remaining_hours * 3600
            self.countdown(remaining_seconds, remaining_hours)
        else:
            self.log_message("Executing transfers, please stay clear of the robot")
            self.publisher("Go")

    def countdown(self, remaining_seconds, remaining_hours):
        remaining_hours = int(remaining_hours)
        if not remaining_seconds > 0:
            # self.log_message(f"Hours remaining until transfer {self.current_transfer + 1}: {remaining_hours}")
            self.after(1000, self.countdown, remaining_seconds - 1, remaining_hours - 0.00027777777)
        else:
            self.log_message("Executing transfers now, please stay clear of the robot")
            self.current_transfer += 1
            self.timer_cb("Go")
            self.execute_next_transfer()

    def publisher(self, pblsh_msg):
        go_msg = String(pblsh_msg)
        self.gohead_pub.publish(go_msg)

    def timer_cb(self, start_flag):
        self.update()
        self.start_flag = start_flag
        if self.start_flag == "Go": self.publisher("Go")

    def gui_done_cb(self, done_flag):
        self.done_flag = done_flag
        if done_flag:
            self.publisher("")

if __name__ == '__main__':
    rospy.init_node('gui_option1')
    app = App()
    while not rospy.is_shutdown():
        app.timer_cb("")
        # rospy.spin()
        rospy.sleep(0.05)
    
