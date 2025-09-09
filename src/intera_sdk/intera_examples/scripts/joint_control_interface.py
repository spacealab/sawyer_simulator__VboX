#!/usr/bin/env python3

# Copyright (c) 2015-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Sawyer Joint Control Interface with Live Visualization

This script provides a graphical interface for controlling Sawyer robot joints
using keyboard input, with live display of joint angles and changes.
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import math

import rospy
import intera_interface
from sensor_msgs.msg import JointState

class JointControlInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Sawyer Joint Control Interface")
        self.root.geometry("800x600")

        # Initialize ROS
        rospy.init_node("joint_control_interface")
        self.limb = intera_interface.Limb('right')
        self.joints = self.limb.joint_names()

        # Resolution control (fixed at default 6 degrees)
        self.resolution_deg = 6.0
        self.delta = math.radians(self.resolution_deg)

        # Joint angles storage
        self.current_angles = {joint: 0.0 for joint in self.joints}
        self.previous_angles = {joint: 0.0 for joint in self.joints}
        
        # Selected joint for mouse control
        self.selected_joint = None

        # Create GUI
        self.create_widgets()

        # Start joint state subscriber
        self.joint_sub = rospy.Subscriber('/robot/joint_states', JointState, self.joint_state_callback)

        # Start keyboard listener and mouse wheel
        self.root.bind('<Key>', self.on_key_press)
        self.root.bind('<MouseWheel>', self.on_mouse_wheel)  # Windows
        self.root.bind('<Button-4>', self.on_mouse_wheel)    # Linux scroll up
        self.root.bind('<Button-5>', self.on_mouse_wheel)    # Linux scroll down
        self.root.focus_set()

        # Update display thread
        self.update_thread = threading.Thread(target=self.update_display)
        self.update_thread.daemon = True
        self.update_thread.start()

    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Title
        title_label = ttk.Label(main_frame, text="Sawyer Joint Control Interface",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=4, pady=10)

        # Resolution info
        res_frame = ttk.LabelFrame(main_frame, text="Resolution", padding="5")
        res_frame.grid(row=1, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        ttk.Label(res_frame, text="Fixed Resolution:").grid(row=0, column=0)
        ttk.Label(res_frame, text=f"{self.resolution_deg:.1f}°").grid(row=0, column=1)

        # Joint display frame
        joint_frame = ttk.LabelFrame(main_frame, text="Joint Angles", padding="5")
        joint_frame.grid(row=2, column=0, columnspan=4, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Headers
        ttk.Label(joint_frame, text="Joint", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(joint_frame, text="Current (°)", font=("Arial", 10, "bold")).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(joint_frame, text="Change (°)", font=("Arial", 10, "bold")).grid(row=0, column=2, padx=5, pady=2)
        ttk.Label(joint_frame, text="Controls", font=("Arial", 10, "bold")).grid(row=0, column=3, padx=5, pady=2)
        ttk.Label(joint_frame, text="Select", font=("Arial", 10, "bold")).grid(row=0, column=4, padx=5, pady=2)

        # Joint rows
        self.joint_labels = {}
        self.change_labels = {}
        self.joint_buttons = {}
        for i, joint in enumerate(self.joints):
            ttk.Label(joint_frame, text=joint).grid(row=i+1, column=0, padx=5, pady=2, sticky=tk.W)

            self.joint_labels[joint] = ttk.Label(joint_frame, text="0.00")
            self.joint_labels[joint].grid(row=i+1, column=1, padx=5, pady=2)

            self.change_labels[joint] = ttk.Label(joint_frame, text="0.00")
            self.change_labels[joint].grid(row=i+1, column=2, padx=5, pady=2)

            controls = self.get_joint_controls(i+1)
            ttk.Label(joint_frame, text=controls).grid(row=i+1, column=3, padx=5, pady=2)

            # Select button for each joint
            self.joint_buttons[joint] = ttk.Button(joint_frame, text="Select", 
                                                  command=lambda j=joint: self.select_joint(j))
            self.joint_buttons[joint].grid(row=i+1, column=4, padx=5, pady=2)

        # Status frame
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="5")
        status_frame.grid(row=3, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        self.status_label = ttk.Label(status_frame, text="Ready - No joint selected")
        self.status_label.grid(row=0, column=0, sticky=(tk.W, tk.E))

        # Selected joint info
        self.selected_label = ttk.Label(status_frame, text="Selected Joint: None", 
                                       font=("Arial", 10, "bold"))
        self.selected_label.grid(row=1, column=0, sticky=(tk.W, tk.E))

        # Instructions
        instr_frame = ttk.LabelFrame(main_frame, text="Instructions", padding="5")
        instr_frame.grid(row=4, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        instructions = """
        Use number keys to control joints:
        1/q: j0, 2/w: j1, 3/e: j2, 4/r: j3, 5/t: j4, 6/y: j5, 7/u: j6
        OR: Select a joint with the 'Select' button and use mouse wheel to control
        Fixed resolution: 6.0° per step
        Esc: Exit
        """
        ttk.Label(instr_frame, text=instructions, justify=tk.LEFT).grid(row=0, column=0)

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)

    def get_joint_controls(self, num):
        keys = ['1/q', '2/w', '3/e', '4/r', '5/t', '6/y', '7/u']
        return keys[num-1] if num <= len(keys) else ""

    def select_joint(self, joint):
        """Select/deselect a joint for mouse wheel control"""
        if self.selected_joint == joint:
            # Deselect if already selected
            self.selected_joint = None
            self.joint_buttons[joint].config(text="Select", state="normal")
            self.selected_label.config(text="Selected Joint: None")
            self.status_label.config(text="No joint selected - Select a joint for mouse wheel control")
        else:
            # Select new joint
            self.selected_joint = joint
            
            # Update all button styles
            for j, button in self.joint_buttons.items():
                if j == joint:
                    button.config(text="Selected", state="normal")
                else:
                    button.config(text="Select", state="normal")
            
            self.selected_label.config(text=f"Selected Joint: {joint}")
            self.status_label.config(text=f"Joint {joint} selected - Use mouse wheel to control")

    def on_mouse_wheel(self, event):
        """Handle mouse wheel events for selected joint control"""
        if self.selected_joint is None:
            self.status_label.config(text="No joint selected - Select a joint first!")
            return
        
        print(f"Raw event details: type={event.type}, delta={getattr(event, 'delta', 'N/A')}, num={getattr(event, 'num', 'N/A')}")
        
        # Check scroll direction more carefully
        wheel_direction = 0
        
        # For Linux X11 systems
        if hasattr(event, 'num'):
            if event.num == 4:  # Scroll up
                wheel_direction = 1
                print("Detected: Scroll UP (Linux Button-4)")
            elif event.num == 5:  # Scroll down
                wheel_direction = -1
                print("Detected: Scroll DOWN (Linux Button-5)")
        
        # For Windows/Mac systems
        elif hasattr(event, 'delta') and event.delta != 0:
            if event.delta > 0:  # Scroll up
                wheel_direction = 1
                print(f"Detected: Scroll UP (delta={event.delta})")
            else:  # Scroll down
                wheel_direction = -1
                print(f"Detected: Scroll DOWN (delta={event.delta})")
        
        # If no direction detected, skip
        if wheel_direction == 0:
            print("No scroll direction detected, skipping...")
            return
            
        delta = self.delta * wheel_direction
        
        current_angle = self.limb.joint_angle(self.selected_joint)
        new_angle = current_angle + delta
        joint_command = {self.selected_joint: new_angle}
        
        try:
            self.limb.set_joint_positions(joint_command)
            direction_text = "UP (increase)" if wheel_direction > 0 else "DOWN (decrease)"
            self.status_label.config(text=f"{self.selected_joint} scroll {direction_text} by {math.degrees(abs(delta)):.2f}°")
            print(f"Applied: {self.selected_joint}, Scroll: {direction_text}, Delta: {delta:.4f} rad ({math.degrees(delta):.2f}°)")
        except Exception as e:
            print(f"Error moving joint: {e}")
            self.status_label.config(text=f"Error moving {self.selected_joint}: {str(e)}")

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joints:
                self.previous_angles[name] = self.current_angles[name]
                self.current_angles[name] = math.degrees(msg.position[i])

    def update_display(self):
        while not rospy.is_shutdown():
            for joint in self.joints:
                current = self.current_angles[joint]
                previous = self.previous_angles[joint]
                change = current - previous

                self.joint_labels[joint].config(text=f"{current:.2f}")
                self.change_labels[joint].config(text=f"{change:+.2f}")

            time.sleep(0.1)

    def on_key_press(self, event):
        key = event.char
        joint_map = {
            '1': (self.joints[0], self.delta),      # j0 increase
            'q': (self.joints[0], -self.delta),     # j0 decrease
            '2': (self.joints[1], self.delta),      # j1 increase
            'w': (self.joints[1], -self.delta),     # j1 decrease
            '3': (self.joints[2], self.delta),      # j2 increase
            'e': (self.joints[2], -self.delta),     # j2 decrease
            '4': (self.joints[3], self.delta),      # j3 increase
            'r': (self.joints[3], -self.delta),     # j3 decrease
            '5': (self.joints[4], self.delta),      # j4 increase
            't': (self.joints[4], -self.delta),     # j4 decrease
            '6': (self.joints[5], self.delta),      # j5 increase
            'y': (self.joints[5], -self.delta),     # j5 decrease
            '7': (self.joints[6], self.delta),      # j6 increase
            'u': (self.joints[6], -self.delta),     # j6 decrease
        }

        if key in joint_map:
            joint, delta = joint_map[key]
            current_angle = self.limb.joint_angle(joint)
            new_angle = current_angle + delta
            joint_command = {joint: new_angle}
            self.limb.set_joint_positions(joint_command)
            direction = "increased" if delta > 0 else "decreased"
            self.status_label.config(text=f"{joint} {direction} by {math.degrees(abs(delta)):.2f}°")
            print(f"Key: {key}, Joint: {joint}, Delta: {delta:.4f} rad ({math.degrees(delta):.2f}°), New angle: {math.degrees(new_angle):.2f}°")

        elif key == '\x1b':  # Escape
            self.root.quit()

def main():
    root = tk.Tk()
    app = JointControlInterface(root)
    root.mainloop()

if __name__ == "__main__":
    main()
