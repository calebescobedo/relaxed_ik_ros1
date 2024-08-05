#!/usr/bin/env python3

# import customtkinter
# import tkinter.font
# import os
# import hiro_grasp
# import rospy
# from relaxed_ik_ros1.msg import GUIMsg
# from datetime import datetime, timedelta
# from tkinter import *

# def validate_time_format(time_str):
#     if time_str == "0":
#         return False
#     try:
#         datetime.strptime(time_str, '%H')
#         return True
#     except ValueError:
#         return False

# class MultiInputDialog(customtkinter.CTkToplevel):
#     def __init__(self, parent, num_transfers, *args, **kwargs):
#         super().__init__(*args, **kwargs)
#         self.title("Time Delays")
#         self.geometry("400x400")
#         self.parent = parent
#         self.num_transfers = num_transfers
#         self.transfer_times = []

#         self.grid_columnconfigure(0, weight=1)
#         self.grid_rowconfigure(0, weight=1)

#         self.input_frame = customtkinter.CTkFrame(self)
#         self.input_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

#         self.inputs = []
#         for i in range(num_transfers):
#             label = customtkinter.CTkLabel(self.input_frame, text=f"Enter the number of hours (1-23) before transfer {i + 1}:" if i == 0 else f"Enter the number of hours (1-23) between transfer {i} and {i + 1}:")
#             label.grid(row=i*2, column=0, padx=10, pady=5, sticky="w")
#             entry = customtkinter.CTkEntry(self.input_frame)
#             entry.grid(row=i*2+1, column=0, padx=10, pady=5, sticky="ew")
#             self.inputs.append(entry)

#         self.confirm_button = customtkinter.CTkButton(self, text="Confirm", command=self.confirm)
#         self.confirm_button.grid(row=num_transfers*2, column=0, padx=10, pady=20, sticky="ew")
        
#         self.after(100, self.grab_set)

#     def confirm(self):
#         valid = True
#         self.transfer_times = []
#         for entry in self.inputs:
#             time_input = entry.get()
#             if not validate_time_format(time_input):
#                 valid = False
#                 break
#             self.transfer_times.append(time_input)

#         if valid:
#             self.parent.handle_transfers(self.transfer_times)
#             self.destroy()
#         else:
#             error_label = customtkinter.CTkLabel(self, text="Invalid input! Please enter a whole number of hours between 1 and 23.")
#             error_label.grid(row=len(self.inputs)*2+1, column=0, padx=10, pady=5, sticky="ew")

# class App(customtkinter.CTk):
#     def __init__(self):
#         super().__init__()
#         self.gohead_pub = rospy.Publisher("/gui_pub", GUIMsg, queue_size=1)
#         self.xbox_sub = rospy.Subscriber("/gui_done", GUIMsg, self.gui_done_cb)
#         script_dir = os.path.dirname(__file__)
#         theme_path = os.path.join(script_dir, 'Hades.json')
#         customtkinter.deactivate_automatic_dpi_awareness()
#         customtkinter.set_default_color_theme(theme_path)

#         self.configure(fg_color=("darkblue", "#213844"))
#         self.geometry("800x600")
#         self.title("RoboDialysis GUI")
#         self.font = customtkinter.CTkFont(family="Roboto", size=13)
#         self.grid_columnconfigure(0, weight=1)
#         self.grid_rowconfigure(0, weight=1)
#         self.grid_rowconfigure(1, weight=0)
#         self.grid_rowconfigure(2, weight=0)
#         self.grid_rowconfigure(3, weight=0)
#         self.grid_rowconfigure(4, weight=0)
#         self.grid_rowconfigure(5, weight=0)
#         self.grid_rowconfigure(6, weight=0)

#         self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
#         self.textbox.tag_config("centered", justify="center")
#         self.textbox.grid(row=0, column=0, padx=10, pady=20, sticky="nsew")
#         self.textbox.insert(INSERT, "Welcome to HIRO Robochem GUI, press the start button to begin\n", "centered")
#         self.textbox.configure(state=DISABLED)

#         self.button = customtkinter.CTkButton(self, text="Start", command=self.button_cb)
#         self.button.grid(row=4, column=0, padx=20, pady=20, sticky="ew")

#         self.error_label = None  
#         self.transfer_times = []

#         self.transfer_button = None  
#         self.view_time_button = None 

#         self.start_flag = ""
#         self.done_flag = ""
#         self.remaining_seconds = 0
#         self.countdown_active = False

#     def log_message(self, message):
#         self.textbox.configure(state=NORMAL)
#         self.textbox.insert(END, message + "\n")
#         self.textbox.configure(state=DISABLED)
#         self.textbox.yview(END)

#     def button_cb(self):
#         # Destroy error label if it exists
#         if self.error_label and self.error_label.winfo_exists():
#             self.error_label.destroy()

#         self.prompt_transfer_count()

#     def prompt_transfer_count(self):
#         self.dialog = customtkinter.CTkInputDialog(text="Enter the number of buckets (default is 6):", title="Number of transfers between buckets")
#         num_transfers = self.dialog.get_input()
#         try:
#             num_transfers = int(num_transfers)
#             if num_transfers < 1:
#                 self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid number greater than 0")
#                 self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
#             else:
#                 self.prompt_time_interval_choice(num_transfers)
#         except ValueError:
#             self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid integer")
#             self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

#     def prompt_time_interval_choice(self, num_transfers):
#         self.num_transfers = int(num_transfers)
#         self.dialog_choice = customtkinter.CTkInputDialog(text=f"Do you want equal time between transfers or custom times? Enter 'equal' or 'custom':", title="Time Interval Choice")
#         choice = self.dialog_choice.get_input().strip().lower()
#         if choice == 'equal':
#             self.handle_equal_time_intervals(num_transfers + 1)
#         elif choice == 'custom':
#             self.open_multi_input_dialog(num_transfers)
#         else:
#             self.error_label = customtkinter.CTkLabel(self, text="Invalid choice! Please enter 'equal' or 'custom'.")
#             self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")

#     def handle_equal_time_intervals(self, num_transfers):
#         label = customtkinter.CTkInputDialog(text=f"Enter the number of hours (1-8) between transfers:", title="Time Interval Choice")
#         interval = label.get_input()
#         self.transfer_times = [str(interval)] * (num_transfers - 1)
#         self.create_execute_button()

#     def open_multi_input_dialog(self, num_transfers):
#         self.multi_input_dialog = MultiInputDialog(self, num_transfers)
#         self.multi_input_dialog.grab_set() 

#     def handle_transfers(self, transfer_times):
#         self.transfer_times = transfer_times
#         self.create_execute_button()

#     def create_execute_button(self):
#         if not self.transfer_button:
#             self.transfer_button = customtkinter.CTkButton(self, text="Execute Transfers", command=self.start_transfers)
#             self.transfer_button.grid(row=6, column=0, padx=20, pady=20, sticky="ew")

#     def create_view_time_button(self):
#         if not self.view_time_button:
#             self.view_time_button = customtkinter.CTkButton(self, text="View time until next transfer", command=self.display_time_remaining)
#             self.view_time_button.grid(row=5, column=0, padx=20, pady=10, sticky="ew")

#     def display_time_remaining(self):
#         if self.remaining_seconds > 0:
#             hours, remainder = divmod(self.remaining_seconds, 3600)
#             minutes, seconds = divmod(remainder, 60)
#             time_str = f"{int(hours):02}:{int(minutes):02}:{int(seconds):02}"
#             self.log_message(f"Time remaining until next transfer: {time_str}")
#         else:
#             self.log_message("No upcoming transfer.")

#     def start_transfers(self):
#         for widget in self.winfo_children():
#             widget.destroy()

#         self.geometry("800x600")
#         self.grid_columnconfigure(0, weight=1)
#         self.font = customtkinter.CTkFont(family="Roboto", size=13)

#         self.textbox = customtkinter.CTkTextbox(self, font=self.font, border_width=1, fg_color="#375774", text_color="white")
#         self.textbox.grid(row=0, column=0, padx=10, pady=20, sticky="nsew")
#         self.textbox.configure(state=DISABLED)

#         for idx, time_str in enumerate(self.transfer_times):
#             if idx < 1:
#                 self.log_message(f"Hours until transfer {idx + 1}: {time_str}")
#             elif idx < 0:
#                 self.error_label = customtkinter.CTkLabel(self, text="Please enter a valid number greater than 0")
#                 self.error_label.grid(row=6, column=0, padx=20, pady=5, sticky="ew")
#             else:
#                 self.log_message(f"Hours between transfer {idx} and transfer {idx + 1}: {time_str}")

#         self.buckets = self.num_transfers
#         self.current_transfer = 0
#         self.execute_next_transfer()
#         self.create_view_time_button() 

#     def execute_next_transfer(self):
#         if self.current_transfer < len(self.transfer_times):
#             time_str = self.transfer_times[self.current_transfer]
#             self.remaining_seconds = int(time_str) * 60 * 60
#             self.current_transfer += 1
#             self.countdown_active = True
#             self.update_countdown()
#         else:
#             self.log_message("All transfers completed.")        

#     def update_countdown(self):
#         if not self.countdown_active and self.remaining_seconds > 0:
#             self.remaining_seconds -= 1
#             self.after(1000, self.update_countdown)
#         else:
#             self.log_message(f"Executing transfer {self.current_transfer}...")
#             self.timer_cb("Go")
#             self.countdown_active = False
#             self.execute_next_transfer()

#     def publisher(self, go_msg, buckets):
#         gui_msg = GUIMsg()
#         if go_msg:
#             gui_msg.go_flag = (go_msg)
#         gui_msg.num_transfers = buckets
#         self.gohead_pub.publish(gui_msg)

#     def buckets_cb(self, buckets):
#         self.buckets = buckets
#         self.publisher(buckets=self.buckets, go_msg=None)

#     def timer_cb(self, start_flag):
#         self.update()
#         self.start_flag = start_flag
#         if self.start_flag == "Go":
#             self.publisher(go_msg="Go", buckets=self.buckets)

#     def gui_done_cb(self, done_flag):
#         self.done_flag = done_flag
#         if done_flag:
#             self.publisher("")

# if __name__ == '__main__':
#     rospy.init_node('gui_option1')
#     app = App()
#     while not rospy.is_shutdown():
#         app.timer_cb("")
#         rospy.sleep(0.05)

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