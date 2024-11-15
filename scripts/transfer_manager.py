#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import re
from typing import List
from datetime import datetime
from threading import Thread

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
        self.first_launch = True  # Track if this is the first launch
        self.error_patterns = [
            # Motion finished but still moving error
            r"Motion finished commanded, but the robot is still moving!.*velocity_limits_violation.*velocity_discontinuity.*acceleration_discontinuity",
            # Control command success rate errors
            r"control_command_success_rate: 0\.95",
            r"control_command_success_rate: 0\.78",
            # Reflex error
            r"libfranka: Move command aborted: motion aborted by reflex!.*communication_constraints_violation"
        ]
        self.needs_restart = False
        self.restart_lock = False

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
            decoded_line = line.decode().rstrip()
            print(f"{prefix}{decoded_line}")
            
            # Check for error patterns in stderr
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
                
                # Kill current process
                self.kill_current_process()
                
                # Wait briefly before restart
                time.sleep(2)
                
                # Start new process
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
                # Prepare launch command based on whether this is the first launch
                # if self.first_launch:
                #     cmd = ['roslaunch', self.package_name, self.launch_file]
                #     launch_type = "initial"
                # else:
                cmd = ['roslaunch', self.package_name, self.launch_file, 'flag:=list']
                launch_type = "subsequent"

                self.log_message(f"Starting {launch_type} launch with command: {' '.join(cmd)}")
                
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=1,
                    universal_newlines=False
                )
                
                # Create threads to handle stdout and stderr streams
                stdout_thread = Thread(target=self.stream_output, 
                                     args=(process.stdout,),
                                     daemon=True)
                stderr_thread = Thread(target=self.stream_output,
                                     args=(process.stderr, 'ERROR: '),
                                     daemon=True)
                
                # Start the output streaming threads
                stdout_thread.start()
                stderr_thread.start()
                
                # Check if process started successfully
                if process.poll() is None:  # Process is running
                    if self.first_launch:
                        self.first_launch = False  # Update flag after successful first launch
                    return process
                else:  # Process failed to start
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

    def kill_current_process(self):
        """Safely terminate the current process if it exists."""
        if self.current_process:
            try:
                self.current_process.terminate()
                self.log_message("Waiting for process to terminate...")
                try:
                    self.current_process.wait(timeout=5)  # Wait up to 5 seconds for graceful termination
                except subprocess.TimeoutExpired:
                    self.log_message("Process did not terminate gracefully, forcing kill...", error=True)
                    self.current_process.kill()  # Force kill if graceful termination fails
                self.log_message("Process terminated")
                self.current_process = None
            except Exception as e:
                self.log_message(f"Error while terminating process: {e}", error=True)

    def get_elapsed_hours(self) -> float:
        """Get elapsed time in hours since timer start."""
        if self.start_time is None:
            return 0.0
        elapsed = datetime.now() - self.start_time
        return elapsed.total_seconds() / 3600  # Convert to hours

    def run(self):
        """Main execution loop."""
        try:
            # Start initial launch file
            self.current_process = self.execute_launch_file()
            self.log_message("Initial launch file started. Waiting for control mode change...")

            # This part needs to be replaced with actual mode monitoring
            # For now, simulate with a keyboard interrupt to change modes
            try:
                while True:
                    if self.current_process.poll() is not None:
                        self.log_message("Process died unexpectedly, restarting...", error=True)
                        self.current_process = self.execute_launch_file()
                    
                    # Check for error-triggered restarts
                    if self.needs_restart:
                        self.handle_process_restart()
                    
                    time.sleep(0.1)  # Small sleep to prevent CPU overuse
                        
            except KeyboardInterrupt:
                self.log_message("Control mode change detected (simulated)")
                self.kill_current_process()
                self.start_time = datetime.now()
                self.log_message("Timer started")

                # Wait for first transfer time
                target_time = self.transfer_times[0]
                while True:
                    elapsed = self.get_elapsed_hours()
                    
                    # Check for error-triggered restarts
                    if self.needs_restart:
                        self.handle_process_restart()
                    
                    if elapsed >= target_time:
                        self.log_message(f"Transfer time reached: {target_time} hours")
                        self.current_process = self.execute_launch_file()
                        break
                    time.sleep(0.1)  # Check frequently but don't overload CPU

                # Keep the final process running until interrupted
                while True:
                    if self.current_process.poll() is not None:
                        self.log_message("Process died unexpectedly, restarting...", error=True)
                        self.current_process = self.execute_launch_file()
                    
                    # Check for error-triggered restarts
                    if self.needs_restart:
                        self.handle_process_restart()
                    
                    time.sleep(0.1)

        except KeyboardInterrupt:
            self.log_message("Interrupt received, cleaning up...")
            self.kill_current_process()
            sys.exit(0)
        except Exception as e:
            self.log_message(f"Unexpected error: {e}", error=True)
            self.kill_current_process()
            sys.exit(1)

def main():
    # Example usage - modify these values as needed
    transfer_times = [1.0]  # Time in hours when launch file should respawn
    
    try:
        manager = TransferManager(transfer_times)
        manager.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()