#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import threading
import time
import math

import rospy
import intera_interface
from sensor_msgs.msg import JointState

class SimpleJointControl:
    def __init__(self, root):
        self.root = root
        self.root.title("Sawyer Robot Control - Simple Version")
        self.root.geometry("800x600")

        # Initialize ROS
        rospy.init_node("simple_joint_control")
        self.limb = intera_interface.Limb('right')
        self.gripper = intera_interface.Gripper('right_gripper')
        self.joints = self.limb.joint_names()

        # Resolution control
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

        # Start keyboard listener
        self.root.bind('<Key>', self.on_key_press)
        self.root.focus_set()

        # Update display thread
        self.update_thread = threading.Thread(target=self.update_display)
        self.update_thread.daemon = True
        self.update_thread.start()

    def create_widgets(self):
        # Main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title
        title_label = tk.Label(main_frame, text="Sawyer Robot Joint Control", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        # Status
        self.status_label = tk.Label(main_frame, text="Ready - Press 'C' to calibrate gripper first!", 
                                   font=("Arial", 10), fg="blue")
        self.status_label.pack(pady=5)

        # Joint angles frame
        angles_frame = tk.LabelFrame(main_frame, text="Current Joint Angles", font=("Arial", 10))
        angles_frame.pack(fill=tk.X, pady=10)

        self.angle_labels = {}
        for i, joint in enumerate(self.joints):
            frame = tk.Frame(angles_frame)
            frame.pack(fill=tk.X, pady=2)
            
            # Joint description
            descriptions = ["BASE (left/right)", "SHOULDER (up/down)", "ELBOW (extend/bend)", 
                          "FOREARM ROLL", "WRIST PITCH", "WRIST ROLL", "END-EFFECTOR"]
            
            tk.Label(frame, text=f"{joint} - {descriptions[i]}:", width=25, anchor=tk.W).pack(side=tk.LEFT)
            label = tk.Label(frame, text="0.00 deg", width=10, anchor=tk.E, font=("Arial", 9))
            label.pack(side=tk.RIGHT)
            self.angle_labels[joint] = label

        # Gripper frame
        gripper_frame = tk.LabelFrame(main_frame, text="Gripper Control", font=("Arial", 10))
        gripper_frame.pack(fill=tk.X, pady=10)

        self.gripper_status = tk.Label(gripper_frame, text="Gripper: Not calibrated", 
                                     font=("Arial", 10), fg="red")
        self.gripper_status.pack(pady=5)

        button_frame = tk.Frame(gripper_frame)
        button_frame.pack(pady=5)

        tk.Button(button_frame, text="Calibrate (C)", command=self.calibrate_gripper, 
                 width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Close (P)", command=self.close_gripper, 
                 width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Open (O)", command=self.open_gripper, 
                 width=12).pack(side=tk.LEFT, padx=5)

        # Instructions frame
        instr_frame = tk.LabelFrame(main_frame, text="Controls", font=("Arial", 10))
        instr_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        instructions = """JOINT CONTROLS:
1/q: j0 BASE - Rotate arm left/right
2/w: j1 SHOULDER - Move arm up/down (MAIN MOVEMENT)
3/e: j2 ELBOW - Extend/bend arm
4/r: j3 FOREARM - Twist wrist area
5/t: j4 WRIST PITCH - Bend wrist up/down
6/y: j5 WRIST ROLL - Roll wrist
7/u: j6 END-EFFECTOR - Final rotation

GRIPPER CONTROLS:
C: Calibrate gripper (DO THIS FIRST!)
P: Close gripper (grab box)
O: Open gripper (release box)

BOX PICKUP STEPS:
1. Press 'C' to calibrate
2. Use 1/q and 2/w to position above box
3. Press 'w' to lower arm to box
4. Press 'P' to grab box
5. Press '2' to lift box
6. Move to destination
7. Press 'w' to lower, 'O' to release

Esc: Exit"""

        tk.Label(instr_frame, text=instructions, justify=tk.LEFT, 
                font=("Courier", 9)).pack(anchor=tk.W, padx=10, pady=10)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joints:
                self.previous_angles[name] = self.current_angles[name]
                self.current_angles[name] = math.degrees(msg.position[i])

    def update_display(self):
        while not rospy.is_shutdown():
            for joint in self.joints:
                current = self.current_angles[joint]
                self.angle_labels[joint].config(text=f"{current:.1f} deg")
            time.sleep(0.1)

    def calibrate_gripper(self):
        try:
            self.gripper_status.config(text="Gripper: Calibrating...", fg="orange")
            self.status_label.config(text="Calibrating gripper - please wait...")
            self.gripper.calibrate()
            self.gripper_status.config(text="Gripper: Ready", fg="green")
            self.status_label.config(text="Gripper calibrated! Ready to pick boxes")
        except Exception as e:
            self.gripper_status.config(text=f"Gripper Error: {str(e)}", fg="red")

    def close_gripper(self):
        try:
            self.gripper.close()
            self.gripper_status.config(text="Gripper: Closed", fg="red")
            self.status_label.config(text="Gripper closed - box grabbed!")
        except Exception as e:
            self.gripper_status.config(text=f"Gripper Error: {str(e)}", fg="red")

    def open_gripper(self):
        try:
            self.gripper.open()
            self.gripper_status.config(text="Gripper: Open", fg="blue")
            self.status_label.config(text="Gripper opened - box released!")
        except Exception as e:
            self.gripper_status.config(text=f"Gripper Error: {str(e)}", fg="red")

    def on_key_press(self, event):
        key = event.char
        joint_map = {
            '1': (self.joints[0], self.delta),
            'q': (self.joints[0], -self.delta),
            '2': (self.joints[1], self.delta),
            'w': (self.joints[1], -self.delta),
            '3': (self.joints[2], self.delta),
            'e': (self.joints[2], -self.delta),
            '4': (self.joints[3], self.delta),
            'r': (self.joints[3], -self.delta),
            '5': (self.joints[4], self.delta),
            't': (self.joints[4], -self.delta),
            '6': (self.joints[5], self.delta),
            'y': (self.joints[5], -self.delta),
            '7': (self.joints[6], self.delta),
            'u': (self.joints[6], -self.delta),
        }

        if key in joint_map:
            joint, delta = joint_map[key]
            current_angle = self.limb.joint_angle(joint)
            new_angle = current_angle + delta
            joint_command = {joint: new_angle}
            self.limb.set_joint_positions(joint_command)
            
            direction = "UP" if delta > 0 else "DOWN"
            self.status_label.config(text=f"{joint} moved {direction} by {math.degrees(abs(delta)):.1f} degrees")

        elif key == 'c':
            self.calibrate_gripper()
        elif key == 'p':
            self.close_gripper()
        elif key == 'o':
            self.open_gripper()
        elif key == '\x1b':  # Escape
            self.root.quit()

def main():
    root = tk.Tk()
    app = SimpleJointControl(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
