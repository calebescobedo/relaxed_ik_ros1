#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import re
import os
from typing import List
from datetime import datetime
from threading import Thread, Event
import rospy
from relaxed_ik_ros1.msg import GUIMsg
from jsk_rviz_plugins.msg import OverlayText

class TransferManager:
    def __init__(self, transfer_times: List[float]):
        """
        Initialize the transfer manager.
        
        Args:
            transfer_times: List of times (in hours) to wait before spawning launch file
        """
        self.transfer_times = transfer_times  # Times in hours
        self.current_process = None
        self.package_name = 'relaxed_ik_ros1'
        self.launch_file = 'gui_v3_control.launch'
        self.max_retries = 3
        self.retry_delay = 3  # seconds
        self.start_time = None
        self.error_patterns = [
            r"Motion finished commanded, but the robot is still moving!.*velocity_limits_violation.*velocity_discontinuity.*acceleration_discontinuity",
            r"control_command_success_rate: 0\.95",
            r"control_command_success_rate: 0\.78",
            r"libfranka: Move command aborted: motion aborted by reflex!.*communication_constraints_violation"
        ]
        self.needs_restart = False
        self.restart_lock = False
        self.current_mode = 'xbox'
        self.exit_flag = Event()
        self.transition_flag = Event()
        self.setup_signal_handlers()
        
        # Initialize ROS node
        rospy.init_node('transfer_manager', anonymous=True)
        self.gui_pub = rospy.Publisher('/gui_msg', GUIMsg, queue_size=1)
        self.text_pub = rospy.Publisher("/text_overlay", OverlayText, queue_size=1)
        
    def format_time_remaining(self, hours_remaining: float) -> str:
        """Convert hours to HH:MM:SS format."""
        total_seconds = int(hours_remaining * 3600)
        hours = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        seconds = total_seconds % 60
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def publish_timer_update(self, hours_remaining: float):
        """Publish timer update to GUI."""
        msg = GUIMsg()
        msg.transfer_times = self.format_time_remaining(hours_remaining)
        msg.robot_mode = self.current_mode
        self.gui_pub.publish(msg)

    def setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown."""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """
        Handle termination signals.
        SIGINT (Ctrl+C) triggers transition, SIGTERM triggers exit
        """
        if signum == signal.SIGINT and self.current_mode == 'xbox':
            self.log_message("Ctrl+C received, transitioning to timer mode...")
            self.transition_flag.set()
        elif signum == signal.SIGTERM or self.current_mode == 'list':
            self.log_message(f"Termination signal received, cleaning up...")
            self.exit_flag.set()
            self.cleanup()
            sys.exit(0)

    def cleanup(self):
        """Perform thorough cleanup of processes."""
        try:
            if self.current_process:
                self.log_message("Cleaning up current process...")
                
                # Kill the process group
                try:
                    pgid = os.getpgid(self.current_process.pid)
                    os.killpg(pgid, signal.SIGTERM)
                    self.log_message(f"Sent SIGTERM to process group {pgid}")
                except ProcessLookupError:
                    pass
                except Exception as e:
                    self.log_message(f"Error killing process group: {e}", error=True)

                # Handle main process
                try:
                    self.current_process.terminate()
                    try:
                        self.current_process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.log_message("Process not responding, forcing kill...", error=True)
                        self.current_process.kill()
                        self.current_process.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.kill(self.current_process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                
                self.current_process = None

            # Cleanup any remaining processes
            try:
                subprocess.run(['pkill', '-f', f'roslaunch.*{self.launch_file}'], 
                             timeout=5)
            except Exception as e:
                self.log_message(f"Error during final cleanup: {e}", error=True)

        except Exception as e:
            self.log_message(f"Error during cleanup: {e}", error=True)

    def log_message(self, message: str, error: bool = False):
        """Log a message with timestamp."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        prefix = 'ERROR: ' if error else 'INFO: '
        print(f"[{timestamp}] {prefix}{message}")

    def check_error_patterns(self, line: str) -> bool:
        """Check if line matches any error patterns."""
        for pattern in self.error_patterns:
            if re.search(pattern, line):
                return True
        return False

    def stream_output(self, stream, prefix=''):
        """Stream output from a pipe to stdout."""
        for line in iter(stream.readline, b''):
            if self.exit_flag.is_set():
                break
            decoded_line = line.decode().rstrip()
            print(f"{prefix}{decoded_line}")
            
            if prefix == 'ERROR: ' and self.check_error_patterns(decoded_line):
                error_msg = f"Detected critical error: {decoded_line}"
                self.log_message(error_msg, error=True)
                if not self.restart_lock:
                    self.needs_restart = True

    def handle_process_restart(self):
        """Handle the process restart when an error is detected."""
        if self.needs_restart and not self.restart_lock:
            try:
                self.restart_lock = True
                self.log_message("Initiating emergency restart sequence", error=True)
                self.cleanup()
                time.sleep(2)
                self.current_process = self.execute_launch_file()
                self.log_message("Emergency restart completed successfully")
            except Exception as e:
                self.log_message(f"Error during emergency restart: {e}", error=True)
            finally:
                self.needs_restart = False
                self.restart_lock = False

    def execute_launch_file(self) -> subprocess.Popen:
        """Execute the launch file with retry logic."""
        retries = 0
        while retries < self.max_retries:
            try:
                cmd = ['roslaunch', self.package_name, self.launch_file, f'flag:={self.current_mode}']
                self.log_message(f"Starting launch with mode '{self.current_mode}' and command: {' '.join(cmd)}")
                
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=1,
                    universal_newlines=False,
                    preexec_fn=os.setsid
                )
                
                stdout_thread = Thread(target=self.stream_output, 
                                     args=(process.stdout,),
                                     daemon=True)
                stderr_thread = Thread(target=self.stream_output,
                                     args=(process.stderr, 'ERROR: '),
                                     daemon=True)
                
                stdout_thread.start()
                stderr_thread.start()
                
                if process.poll() is None:
                    return process
                else:
                    raise subprocess.SubprocessError("Process failed to start")
                
            except (subprocess.SubprocessError, OSError) as e:
                retries += 1
                self.log_message(f"Attempt {retries} failed: {str(e)}", error=True)
                
                if retries < self.max_retries:
                    self.log_message(f"Retrying in {self.retry_delay} seconds...")
                    time.sleep(self.retry_delay)
                else:
                    self.log_message("Maximum retry attempts reached. Exiting.", error=True)
                    sys.exit(1)
        
        self.log_message("Failed to start process after all retry attempts", error=True)
        sys.exit(1)

    def get_elapsed_time(self) -> float:
        """Get elapsed time in hours since timer start."""
        if self.start_time is None:
            return 0.0
        elapsed = datetime.now() - self.start_time
        return elapsed.total_seconds() / 3600

    def run(self):
        """Main execution loop."""
        try:
            while not self.exit_flag.is_set():
                try:
                    # Start with xbox mode
                    self.current_mode = 'xbox'
                    self.log_message("Starting initial launch with xbox mode...")
                    self.current_process = self.execute_launch_file()
                    
                    # Wait for xbox mode to be terminated or fail
                    while not self.transition_flag.is_set() and not self.exit_flag.is_set():
                        if self.current_process.poll() is not None:
                            self.log_message("Xbox mode process ended unexpectedly")
                            break
                        
                        if self.needs_restart:
                            self.handle_process_restart()
                        
                        # Publish mode status
                        msg = GUIMsg()
                        msg.robot_mode = self.current_mode
                        # msg.transfer_times = "00:00:00"
                        self.gui_pub.publish(msg)
                        
                        time.sleep(0.1)
                    
                    # Clean up xbox mode process
                    self.cleanup()
                    
                    if self.exit_flag.is_set():
                        break
                    
                    # Switch to list mode and start timer
                    self.current_mode = 'list'
                    self.start_time = datetime.now()
                    self.log_message("Switching to list mode and starting timer...")
                    
                    # Immediately start the launch file in list mode
                    self.current_process = self.execute_launch_file()
                    self.log_message("Launched list mode process")
                    self.transition_flag.clear()

                    # Wait for transfer time while updating display
                    target_time = self.transfer_times[0]
                    while not self.exit_flag.is_set():
                        elapsed_hours = self.get_elapsed_time()
                        remaining_hours = target_time - elapsed_hours
                        
                        # Publish timer update
                        msg = GUIMsg()
                        msg.robot_mode = self.current_mode
                        msg.transfer_times = self.format_time_remaining(remaining_hours)
                        self.gui_pub.publish(msg)
                        
                        if self.needs_restart:
                            self.handle_process_restart()
                        
                        if elapsed_hours >= target_time:
                            self.log_message(f"Transfer time reached: {target_time} hours")
                            self.current_process = self.execute_launch_file()
                            break
                            
                        time.sleep(0.1)  # Update display every 100ms

                    # Keep the final process running until interrupted
                    while not self.exit_flag.is_set():
                        if self.current_process.poll() is not None:
                            self.log_message("Process died unexpectedly, restarting...", error=True)
                            self.current_process = self.execute_launch_file()
                        
                        if self.needs_restart:
                            self.handle_process_restart()
                        
                        # Continue publishing timer status
                        msg = GUIMsg()
                        msg.robot_mode = self.current_mode
                        msg.transfer_times = "00:00:00"  # Reset timer after transfer
                        self.gui_pub.publish(msg)
                        
                        time.sleep(0.1)

                except KeyboardInterrupt:
                    if self.current_mode == 'xbox':
                        self.transition_flag.set()
                    else:
                        self.exit_flag.set()

        except Exception as e:
            self.log_message(f"Unexpected error: {e}", error=True)
            self.cleanup()
            sys.exit(1)

    def gui_callback(self, msg):
        """Handle messages from the GUI."""
        try:
            if msg.go_flag == "start":
                self.log_message("Received start command from GUI")
                
                # Parse transfer times from message
                if msg.transfer_times:
                    try:
                        self.transfer_times = [float(t) for t in msg.transfer_times.split(',')]
                        self.log_message(f"Updated transfer times: {self.transfer_times}")
                    except ValueError as e:
                        self.log_message(f"Error parsing transfer times: {e}", error=True)
                
                # Set transition flag to start transfers
                self.transition_flag.set()
        except Exception as e:
            self.log_message(f"Error in GUI callback: {e}", error=True)

def read_transfer_times(filename: str) -> List[float]:
    """Read transfer times from file, converting hours to floats."""
    try:
        with open(filename, 'r') as f:
            return [float(line.strip()) for line in f if line.strip()]
    except FileNotFoundError:
        print(f"Transfer times file '{filename}' not found. Using default of 1 hour.")
        return [1]
    except ValueError as e:
        print(f"Error parsing transfer times: {e}. Using default of 1.0 hour.")
        return [1]

def main():
    try:
        transfer_times = read_transfer_times('/home/caleb/robochem_steps/transfer_times.txt')
        manager = TransferManager(transfer_times)
        manager.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()