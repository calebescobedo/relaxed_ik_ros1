import tkinter.font
import customtkinter
import hiro_grasp
import os
import rospy
import rospkg
import tkinter
import time
from customtkinter import *
from datetime import datetime, timedelta
from std_msgs.msg import String
from tkinter import *
from xbox_input_test import *

def validate_time_format(time_str):
    try:
        datetime.strptime(time_str, '%H:%M:%S')
        return True
    except ValueError:
        return False

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        customtkinter.deactivate_automatic_dpi_awareness()
        customtkinter.set_default_color_theme("Hades.json")
        
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

        self.button = customtkinter.CTkButton(self, text="Begin", command=self.button_cb)
        self.button.grid(row=4, column=0, padx=20, pady=20, sticky="ew")
        
        self.error_label = None  # Initialize error_label variable
        self.transfer_times = []
        
        self.transfer_button = None  # Initialize transfer_button variable

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
        self.dialog = customtkinter.CTkInputDialog(text="Enter the number of transfers:", title="Number of Transfers")
        num_transfers = self.dialog.get_input()
        try:
            num_transfers = int(num_transfers)
            if num_transfers < 1:
                self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid number greater than 0")
                self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
            elif num_transfers == 1:
                self.prompt_single_transfer()
            else:
                self.handle_transfers(num_transfers)
        except ValueError:
            self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid integer")
            self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

    def prompt_single_transfer(self):
        self.dialog = customtkinter.CTkInputDialog(text="Do you want to execute the transfer immediately? (yes/no)", title="Single Transfer Option")
        choice = self.dialog.get_input()
        if choice.lower() == "yes":
            # Execute transfer immediately
            self.execute_single_transfer()
        elif choice.lower() == "no":
            # Prompt for time delay
            self.prompt_time_delay()
        else:
            self.error_label = customtkinter.CTkLabel(self, text="Invalid choice! Please enter 'yes' or 'no'.")
            self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

    def execute_single_transfer(self):
        # Since user selected to execute immediately, do not show time input prompt
        self.log_message("Executing single transfer immediately")
        # Create and display the execute transfers button
        self.create_execute_button()

    def prompt_time_delay(self):
        self.dialog = customtkinter.CTkInputDialog(text="Enter the time delay in HH:MM:SS format:", title="Time Delay")
        time_input = self.dialog.get_input()
        while not validate_time_format(time_input):
            self.dialog = customtkinter.CTkInputDialog(text="Invalid format! Please enter time in HH:MM:SS format.", title="Invalid Input")
            time_input = self.dialog.get_input()
        self.log_message(f"Time delay specified: {time_input}")
        self.transfer_times.append(time_input)
        # Create and display the execute transfers button
        self.create_execute_button()

    def handle_transfers(self, num_transfers):
        if num_transfers == 0:
            self.error_label = customtkinter.CTkLabel(self, text="Cannot perform zero transfers!")
            self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
            return
        
        self.transfer_times = []
        self.prompt_next_transfer_time(1, num_transfers)

    def prompt_next_transfer_time(self, idx, total_transfers):
        if idx < total_transfers:
            self.dialog = customtkinter.CTkInputDialog(text=f"Enter the time delay in HH:MM:SS format between transfer {idx} and {idx + 1}:", title=f"Time Delay {idx} to {idx + 1}")
            time_input = self.dialog.get_input()
            while not validate_time_format(time_input):
                self.dialog = customtkinter.CTkInputDialog(text="Invalid format! Please enter time in HH:MM:SS format.", title="Invalid Input")
                time_input = self.dialog.get_input()
            self.log_message(f"Time delay specified for transfer {idx} to {idx + 1}: {time_input}")
            self.transfer_times.append(time_input)
            self.prompt_next_transfer_time(idx + 1, total_transfers)
        else:
            # All prompts completed, create and display the execute transfers button
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

        self.log_message(f"Executing transfers with the following delays:")

        for idx, time_str in enumerate(self.transfer_times):
            self.log_message(f"Time between transfer {idx + 1} and transfer {idx + 2}: {time_str}")

        self.current_transfer = 0
        self.execute_next_transfer()

    def execute_next_transfer(self):
        if self.current_transfer < len(self.transfer_times):
            time_str = self.transfer_times[self.current_transfer]
            self.start_countdown(time_str)
        else:
            self.log_message("Executing transfers now")

    def start_countdown(self, time_str):
        hours, minutes, seconds = map(int, time_str.split(':'))
        total_seconds = hours * 3600 + minutes * 60 + seconds
        self.countdown(total_seconds)

    def countdown(self, remaining_seconds):
        if remaining_seconds > 0:
            self.log_message(f"Time remaining: {str(timedelta(seconds=remaining_seconds))}")
            self.after(1000, self.countdown, remaining_seconds - 1)
        else:
            self.log_message("Executing transfers now")
            self.current_transfer += 1
            self.execute_next_transfer()



class roboControl():
    def init_robo(self):
        # Define path to relaxed_ik_core for build and ros inclusion
        path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

        # Init and spin ros node
        flag = rospy.set_param('hiro_robochem/flag', 'list')
        self.xController = XboxInput(flag=flag)

    # def closeGrip(self):
    #     rospy.Publisher()

if __name__ == '__main__':
    app = App()

    # rospy.init_node('hiro_robochem')
    app.mainloop()





