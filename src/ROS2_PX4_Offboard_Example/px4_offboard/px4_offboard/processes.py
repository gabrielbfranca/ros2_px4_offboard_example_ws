#!/usr/bin/env python3

import subprocess
import time

# List of commands to run
commands = {
    "MicroXRCEAgent": "MicroXRCEAgent udp4 -p 8888",
    "PX4_SITL": "cd ../EDRA/PX4-Autopilot && make px4_sitl gz_x500",
    # "QGroundControl": "cd ~/QGroundControl && ./QGroundControl.AppImage"  # Uncomment if needed
}

# Start tmux session
session_name = "ros_session"
subprocess.run(["tmux", "new-session", "-d", "-s", session_name])

# Loop through each command and create a new tmux window for it
for window_name, command in commands.items():
    subprocess.run(["tmux", "new-window", "-t", session_name, "-n", window_name, f"bash -c '{command}; exec bash'"])
    time.sleep(1)  # Give some time for the process to start

# Attach to tmux session (Optional: if you want to see the windows)
subprocess.run(["tmux", "attach-session", "-t", session_name])
