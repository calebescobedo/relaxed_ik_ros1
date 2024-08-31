#!/usr/bin/env python3

import customtkinter
import os
import rospy
from relaxed_ik_ros1.msg import GUIMsg
from datetime import datetime
from tkinter import DISABLED, INSERT, END, NORMAL

def validate_time_format(time_str):
    try:
        time_int = int(time_str)
        return 1 <= time_int <= 23
    except ValueError:
        return False

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        self.gohead_pub = rospy.Publisher("/gui_pub", GUIMsg, queue_size=1)
        self.xbox_sub = rospy.Subscriber("/gui_done", GUIMsg, self.gui_done_cb)
        script_dir = os.path.dirname(__file__)
        theme_path = os.path.join(script_dir, 'Hades.json')
        # customtkinter.deactivate_automatic_dpi_awareness()
        customtkinter.set_default_color_theme(theme_path)

        self.configure(fg_color=("darkblue", "#213844"))
        self.geometry("800x600")
        self.title("RoboDialysis GUI")
        self.font = customtkinter.CTkFont(family="Roboto", size=13)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(2, weight=0)
        self.grid_rowconfigure(3, weight=0)
        self.grid_rowconfigure(4, weight=0)
        self.grid_rowconfigure(5, weight=0)
        self.grid_rowconfigure(6, weight=0)

        self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
        self.textbox.tag_config("centered", justify="center")
        self.textbox.grid(row=0, column=0, padx=10, pady=20, sticky="nsew")
        self.textbox.insert(INSERT, "Welcome to HIRO Robochem GUI, press the start button to begin\n", "centered")
        self.textbox.configure(state=DISABLED)

        self.button = customtkinter.CTkButton(self, text="Start", command=self.start_button_cb)
        self.button.grid(row=4, column=0, padx=20, pady=20, sticky="ew")

        self.error_label = None  
        self.transfer_times = []

        self.transfer_button = None  
        self.view_time_button = None 

        self.start_flag = ""
        self.done_flag = ""
        self.remaining_seconds = 0
        self.countdown_active = False

        self.input_frame = None

    def log_message(self, message):
        self.textbox.configure(state=NORMAL)
        self.textbox.insert(END, message + "\n")
        self.textbox.configure(state=DISABLED)
        self.textbox.yview(END)

    def start_button_cb(self):
        if self.error_label and self.error_label.winfo_exists():
            self.error_label.destroy()
        self.show_input_form()

    def show_input_form(self):
        self.clear_input_frame()

        self.input_frame = customtkinter.CTkFrame(self)
        self.input_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # self.num_transfers_label = customtkinter.CTkLabel(self.input_frame, text="Enter the number of buckets (default is 6):")
        # self.num_transfers_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        # self.num_transfers_entry = customtkinter.CTkEntry(self.input_frame)
        # self.num_transfers_entry.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        self.choice_label = customtkinter.CTkLabel(self.input_frame, text="Do you want equal time between transfers or custom times? Enter 'equal' or 'custom':")
        self.choice_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")
        self.choice_entry = customtkinter.CTkEntry(self.input_frame)
        self.choice_entry.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

        self.confirm_button = customtkinter.CTkButton(self.input_frame, text="Confirm", command=self.handle_confirm)
        self.confirm_button.grid(row=4, column=0, padx=10, pady=20, sticky="ew")

    def handle_confirm(self):
        # num_transfers = self.num_transfers_entry.get()
        choice = self.choice_entry.get().strip().lower()

        # try:
        #     num_transfers = int(num_transfers)
        #     if num_transfers < 1:
        #         self.show_error_message("Please enter a valid number greater than 0")
        #         return
        # except ValueError:
        #     self.show_error_message("Please enter a valid integer")
        #     return

        if choice not in ['equal', 'custom']:
            self.show_error_message("Invalid choice! Please enter 'equal' or 'custom'.")
            return

        # self.num_transfers = num_transfers
        self.choice = choice
        self.show_time_input_form()

    def show_error_message(self, message):
        if self.error_label:
            self.error_label.destroy()
        self.error_label = customtkinter.CTkLabel(self, text=message)
        self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

    def show_time_input_form(self):
        self.clear_input_frame()

        self.input_frame = customtkinter.CTkFrame(self)
        self.input_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.inputs = []

        if self.choice == "equal":
            label = customtkinter.CTkLabel(self.input_frame, text=f"Enter the number of hours (1-23) between transfers:")
            label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
            entry = customtkinter.CTkEntry(self.input_frame)
            entry.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
            self.inputs.append(entry)
        # else:
        #     for i in range(self.num_transfers):
        #         label_text = f"Enter the number of hours (1-23) before transfer {i + 1}:" if i == 0 else f"Enter the number of hours (1-23) between transfer {i} and {i + 1}:"
        #         label = customtkinter.CTkLabel(self.input_frame, text=label_text)
        #         label.grid(row=i*2, column=0, padx=10, pady=5, sticky="w")
        #         entry = customtkinter.CTkEntry(self.input_frame)
        #         entry.grid(row=i*2+1, column=0, padx=10, pady=5, sticky="ew")
        #         self.inputs.append(entry)

        self.confirm_button = customtkinter.CTkButton(self.input_frame, text="Confirm", command=self.confirm_time_input)
        # self.confirm_button.grid(row=self.num_transfers*2, column=0, padx=10, pady=20, sticky="ew")
        self.confirm_button.grid(row=2, column=0, padx=10, pady=20, sticky="ew")
        
    def confirm_time_input(self):
        valid = True
        self.transfer_times = []
        for entry in self.inputs:
            time_input = entry.get()
            if not validate_time_format(time_input):
                valid = False
                break
            self.transfer_times.append(time_input)

        if valid:
            self.handle_transfers()
        else:
            self.show_error_message("Invalid input! Please enter a whole number of hours between 1 and 23.")

    def clear_input_frame(self):
        if self.input_frame:
            self.input_frame.destroy()

    def handle_transfers(self):
        self.create_execute_button()

    def create_execute_button(self):
        if not self.transfer_button:
            self.transfer_button = customtkinter.CTkButton(self, text="Execute Transfers", command=self.start_transfers)
            self.transfer_button.grid(row=6, column=0, padx=20, pady=20, sticky="ew")

    def create_view_time_button(self):
        if not self.view_time_button:
            self.view_time_button = customtkinter.CTkButton(self, text="View time until next transfer", command=self.display_time_remaining)
            self.view_time_button.grid(row=5, column=0, padx=20, pady=10, sticky="ew")

    def display_time_remaining(self):
        if self.remaining_seconds > 0:
            hours, remainder = divmod(self.remaining_seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            time_str = f"{int(hours):02}:{int(minutes):02}:{int(seconds):02}"
            self.log_message(f"Time remaining until next transfer: {time_str}")
        else:
            self.log_message("No upcoming transfer.")

    def start_transfers(self):
        self.clear_input_frame()

        self.geometry("800x600")
        self.grid_columnconfigure(0, weight=1)
        self.font = customtkinter.CTkFont(family="Roboto", size=13)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)
        self.grid_rowconfigure(2, weight=0)
        self.grid_rowconfigure(3, weight=0)
        self.grid_rowconfigure(4, weight=0)
        self.grid_rowconfigure(5, weight=0)
        self.grid_rowconfigure(6, weight=0)

        self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
        self.textbox.tag_config("centered", justify="center")
        self.textbox.grid(row=0, column=0, padx=10, pady=20, sticky="nsew")
        self.textbox.insert(INSERT, "Transfers are being executed\n", "centered")
        self.textbox.configure(state=DISABLED)

        self.transfer_button = None
        self.view_time_button = None

        self.create_view_time_button()

        self.execute_transfers()

    def execute_transfers(self):
        for time_input in self.transfer_times:
            self.schedule_transfer(time_input)
            self.remaining_seconds = int(time_input) * 3600
            self.update_countdown()
            self.wait_until_next_transfer()

    def schedule_transfer(self, time_input):
        # rospy.sleep(int(time_input) * 3600)
        gui_msg = GUIMsg()
        gui_msg.go_flag = "Go"
        self.gohead_pub.publish(gui_msg)
        self.log_message(f"Transfer executed at {datetime.now().strftime('%H:%M:%S')}")

    def update_countdown(self):
        if self.remaining_seconds > 0:
            self.remaining_seconds -= 1
            self.after(1000, self.update_countdown)

    def wait_until_next_transfer(self):
        while self.remaining_seconds > 0 and not rospy.is_shutdown():
            self.update_idletasks()
            self.update()
        gui_msg = GUIMsg()
        gui_msg.go_flag = "Go"
        self.gohead_pub.publish(gui_msg)
        self.log_message(f"Transfer executed at {datetime.now().strftime('%H:%M:%S')}")

    def gui_done_cb(self, msg):
        self.done_flag = msg.done

if __name__ == "__main__":
    rospy.init_node('gui_node', anonymous=True)
    app = App()
    while not rospy.is_shutdown():
        app.update_idletasks()
        app.update()
        rospy.sleep(0.05)