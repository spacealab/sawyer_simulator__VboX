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
        self.root.title("Sawyer Joint Control Interface with Gripper Control")
        self.root.geometry("1000x700")

        # Initialize ROS
        rospy.init_node("joint_control_interface")
        self.limb = intera_interface.Limb('right')
        self.gripper = intera_interface.Gripper('right_gripper')  # Use full gripper name to avoid deprecated warning
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
        # Main frame with three columns: controls, front view, side view
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Left column - Controls
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))

        # Title
        title_label = ttk.Label(control_frame, text="Sawyer Joint Control Interface",
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=4, pady=10)

        # Resolution info
        res_frame = ttk.LabelFrame(control_frame, text="Resolution", padding="5")
        res_frame.grid(row=1, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        ttk.Label(res_frame, text="Fixed Resolution:").grid(row=0, column=0)
        ttk.Label(res_frame, text=f"{self.resolution_deg:.1f}°").grid(row=0, column=1)

        # Joint display frame
        joint_frame = ttk.LabelFrame(control_frame, text="Joint Angles", padding="5")
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
        status_frame = ttk.LabelFrame(control_frame, text="Status", padding="5")
        status_frame.grid(row=3, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        self.status_label = ttk.Label(status_frame, text="Ready - No joint selected")
        self.status_label.grid(row=0, column=0, sticky=(tk.W, tk.E))

        # Selected joint info
        self.selected_label = ttk.Label(status_frame, text="Selected Joint: None", 
                                       font=("Arial", 10, "bold"))
        self.selected_label.grid(row=1, column=0, sticky=(tk.W, tk.E))

        # Instructions
        instr_frame = ttk.LabelFrame(control_frame, text="Robot Joint Control Guide", padding="5")
        instr_frame.grid(row=4, column=0, columnspan=4, pady=5, sticky=(tk.W, tk.E))

        instructions = """
        JOINT LOCATIONS & CONTROLS:
        1/q: j0 (BASE ROTATION) - Rotates entire arm left/right at base
        2/w: j1 (SHOULDER) - Moves arm up/down at shoulder joint
        3/e: j2 (ELBOW) - Bends/straightens elbow joint
        4/r: j3 (FOREARM ROLL) - Rotates forearm (twist wrist area)
        5/t: j4 (WRIST PITCH) - Bends wrist up/down
        6/y: j5 (WRIST ROLL) - Rotates wrist left/right        
        7/u: j6 (END-EFFECTOR) - Final rotation at gripper attachment
        
        GRIPPER CONTROL (for picking boxes):
        O = Open Gripper (release objects)
        P = Close Gripper (grab objects)
        C = Calibrate Gripper (REQUIRED first!)
        
        MOUSE CONTROL: Select joint + mouse wheel
        IMPORTANT: Always press 'C' first!    Exit: Esc
        """
        ttk.Label(instr_frame, text=instructions, justify=tk.LEFT, font=("Arial", 8)).grid(row=0, column=0)

        # Gripper Control Frame (right column)
        gripper_frame = ttk.LabelFrame(main_frame, text="Gripper Control & Pick & Place Guide", padding="10")
        gripper_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))

        # Status and reminder frame
        status_reminder_frame = ttk.LabelFrame(gripper_frame, text="Current Status", padding="5")
        status_reminder_frame.grid(row=0, column=0, columnspan=3, pady=5, sticky=(tk.W, tk.E))

        # Current status label
        self.current_action_label = ttk.Label(status_reminder_frame, 
                                            text="Ready to start - Press 'C' first!", 
                                            font=("Arial", 10, "bold"), foreground="blue")
        self.current_action_label.grid(row=0, column=0, pady=5)

        # Gripper status
        self.gripper_status = ttk.Label(gripper_frame, text="Gripper Status: Ready", 
                                       font=("Arial", 12, "bold"), foreground="green")
        self.gripper_status.grid(row=1, column=0, columnspan=3, pady=10)

        # Gripper control buttons
        button_frame = ttk.Frame(gripper_frame)
        button_frame.grid(row=1, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Open Gripper (O)", 
                  command=self.open_gripper, width=18).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(button_frame, text="Close Gripper (P)", 
                  command=self.close_gripper, width=18).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(button_frame, text="Calibrate (C)", 
                  command=self.calibrate_gripper, width=18).grid(row=0, column=2, padx=5, pady=5)

        # Gripper position display
        pos_frame = ttk.LabelFrame(gripper_frame, text="Gripper Position", padding="5")
        pos_frame.grid(row=2, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        self.gripper_position_label = ttk.Label(pos_frame, text="Position: 0.00 mm", font=("Arial", 11))
        self.gripper_position_label.grid(row=0, column=0)

        # Pick and place guide
        guide_frame = ttk.LabelFrame(gripper_frame, text="Box Pick & Place Guide", padding="5")
        guide_frame.grid(row=3, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        guide_text = """
        BOX PICKUP STEPS:
        
        1. Position above box:
           Use: 1/q (BASE ROTATION - left/right entire arm)
           Use: 2/w (SHOULDER - up/down arm movement)
           Use: 3/e (ELBOW - extend/bend arm reach)
        
        2. Lower robot to box level:
           Press 'w' (SHOULDER down) multiple times
           Watch arm go down toward box
        
        3. Grab the box:
           Press 'P' (Close Gripper)
           Wait for "Box grabbed!" message
        
        4. Lift box up:
           Press '2' (SHOULDER up) to lift
           Box should rise with arm
        
        5. Move box to destination:
           Use various joint keys to navigate
           4/r: FOREARM ROLL (twist wrist area)
           5/t: WRIST PITCH (wrist up/down)
        
        6. Release box:
           Lower with 'w' (SHOULDER down)
           Press 'O' (Open Gripper)
           Move away with '2' (SHOULDER up)
        """
        ttk.Label(guide_frame, text=guide_text, justify=tk.LEFT, font=("Arial", 8)).grid(row=0, column=0)

        # Quick reference frame
        quick_frame = ttk.LabelFrame(gripper_frame, text="Quick Reference - Joint Locations", padding="5")
        quick_frame.grid(row=4, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        quick_text = """
        MAIN KEYS FOR BOX PICKUP:
        
        VERTICAL MOVEMENT (Most Important):
           2 = SHOULDER UP (lift entire arm)
           w = SHOULDER DOWN (lower entire arm)
        
        HORIZONTAL POSITIONING:
           1 = BASE LEFT (rotate whole arm left)
           q = BASE RIGHT (rotate whole arm right)
           3 = ELBOW EXTEND (stretch arm forward)
           e = ELBOW BEND (pull arm back)
        
        FINE ADJUSTMENTS:
           4 = FOREARM TWIST RIGHT    r = FOREARM TWIST LEFT
           5 = WRIST PITCH UP         t = WRIST PITCH DOWN
           6 = WRIST ROLL RIGHT       y = WRIST ROLL LEFT
           7 = END-EFFECTOR RIGHT     u = END-EFFECTOR LEFT
        
        GRIPPER CONTROL:
           P = CLOSE (grab box)       O = OPEN (release box)
           C = CALIBRATE (do this first!)
        
        TIP: Start with SHOULDER (2/w) and BASE (1/q) for major movements!
        """
        ttk.Label(quick_frame, text=quick_text, justify=tk.LEFT, font=("Arial", 7)).grid(row=0, column=0)

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)  # Control column
        main_frame.columnconfigure(1, weight=1)  # Gripper column  
        main_frame.rowconfigure(0, weight=1)

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

    def open_gripper(self):
        """Open the gripper"""
        try:
            self.gripper.open()
            self.gripper_status.config(text="Gripper Status: Opening...", foreground="orange")
            self.current_action_label.config(text="Gripper opening - Ready to release box")
            self.root.after(2000, lambda: [
                self.gripper_status.config(text="Gripper Status: Open", foreground="blue"),
                self.current_action_label.config(text="Box released! Move robot away")
            ])
            rospy.loginfo("Gripper opened")
        except Exception as e:
            self.gripper_status.config(text=f"Error: {str(e)}", foreground="red")
            self.current_action_label.config(text="Error opening gripper")
            rospy.logerr(f"Failed to open gripper: {e}")

    def close_gripper(self):
        """Close the gripper"""
        try:
            self.gripper.close()
            self.gripper_status.config(text="Gripper Status: Closing...", foreground="orange")
            self.current_action_label.config(text="Gripper closing - Grabbing box")
            self.root.after(2000, lambda: [
                self.gripper_status.config(text="Gripper Status: Closed", foreground="red"),
                self.current_action_label.config(text="Box grabbed! Now lift it up (press '2')")
            ])
            rospy.loginfo("Gripper closed")
        except Exception as e:
            self.gripper_status.config(text=f"Error: {str(e)}", foreground="red")
            self.current_action_label.config(text="Error closing gripper")
            rospy.logerr(f"Failed to close gripper: {e}")

    def calibrate_gripper(self):
        """Calibrate the gripper"""
        try:
            self.gripper_status.config(text="Gripper Status: Calibrating...", foreground="purple")
            self.current_action_label.config(text="Calibrating... Please wait")
            self.gripper.calibrate()
            self.gripper_status.config(text="Gripper Status: Calibrated", foreground="green")
            self.current_action_label.config(text="Ready! Now move robot above the box")
            rospy.loginfo("Gripper calibrated")
        except Exception as e:
            self.gripper_status.config(text=f"Error: {str(e)}", foreground="red")
            self.current_action_label.config(text="Error calibrating gripper")
            rospy.logerr(f"Failed to calibrate gripper: {e}")

    def update_gripper_position(self):
        """Update gripper position display"""
        try:
            position = self.gripper.get_position()
            self.gripper_position_label.config(text=f"Position: {position:.2f} mm")
        except:
            self.gripper_position_label.config(text="Position: Unknown")

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joints:
                self.previous_angles[name] = self.current_angles[name]
                self.current_angles[name] = math.degrees(msg.position[i])

    def get_joint_controls(self, num):
        keys = ['1/q', '2/w', '3/e', '4/r', '5/t', '6/y', '7/u']
        return keys[num-1] if num <= len(keys) else ""

    def update_display(self):
        while not rospy.is_shutdown():
            for joint in self.joints:
                current = self.current_angles[joint]
                previous = self.previous_angles[joint]
                change = current - previous

                self.joint_labels[joint].config(text=f"{current:.2f}")
                self.change_labels[joint].config(text=f"{change:+.2f}")

            # Update gripper position
            self.update_gripper_position()

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

        # Handle joint movements
        if key in joint_map:
            joint, delta = joint_map[key]
            current_angle = self.limb.joint_angle(joint)
            new_angle = current_angle + delta
            joint_command = {joint: new_angle}
            self.limb.set_joint_positions(joint_command)
            direction = "increased" if delta > 0 else "decreased"
            
            # Update status with helpful message
            joint_descriptions = {
                self.joints[0]: "BASE (left/right rotation)",
                self.joints[1]: "SHOULDER (up/down movement)", 
                self.joints[2]: "ELBOW (extend/bend)",
                self.joints[3]: "FOREARM ROLL (twist)",
                self.joints[4]: "WRIST PITCH (up/down)", 
                self.joints[5]: "WRIST ROLL (left/right)",
                self.joints[6]: "END-EFFECTOR (final rotation)"
            }
            
            joint_desc = joint_descriptions.get(joint, joint)
            self.status_label.config(text=f"Joint {joint_desc} {direction} ({math.degrees(abs(delta)):.1f} degrees)")
            
            # Special guidance for important joints
            if joint == self.joints[1]:  # j1 - vertical movement
                if delta > 0:
                    self.current_action_label.config(text="ARM LIFTED - To grab box press 'w' to go down")
                else:
                    self.current_action_label.config(text="ARM LOWERED - Near box? Press 'P' to grab")
            
            print(f"Key: {key}, Joint: {joint}, Delta: {delta:.4f} rad ({math.degrees(delta):.2f}°), New angle: {math.degrees(new_angle):.2f}°")

        # Handle gripper controls
        elif key == 'o':  # Open gripper
            self.open_gripper()
            print("Gripper opened via keyboard")
        elif key == 'p':  # Close gripper
            self.close_gripper()
            print("Gripper closed via keyboard")
        elif key == 'c':  # Calibrate gripper
            self.calibrate_gripper()
            print("Gripper calibrated via keyboard")

        elif key == '\x1b':  # Escape
            self.root.quit()

def main():
    root = tk.Tk()
    app = JointControlInterface(root)
    root.mainloop()

if __name__ == "__main__":
    main()
