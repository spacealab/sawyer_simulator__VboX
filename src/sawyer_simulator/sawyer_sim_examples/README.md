# Sawyer Simulation Examples

This package contains examples for controlling the Sawyer robot in the Gazebo simulator.

## Scripts

### Joint Control Interface
- **File:** `intera_sdk/intera_examples/scripts/joint_control_interface.py`
- **Description:** A graphical user interface (GUI) for manually controlling the Sawyer robot's joints using keyboard and mouse input. It provides a live display of joint angles and allows for direct control over each joint and the gripper.
- **Note:** The GUI and all associated text in this script have been updated to be entirely in English.

### Quick Box Pickup Guide
- **File:** `intera_sdk/intera_examples/scripts/QUICK_BOX_PICKUP_GUIDE.md`
- **Description:** A user-friendly guide on how to use the `joint_control_interface.py` script to manually perform a pick-and-place task.
- **Note:** This guide has been translated from Persian to English.

---

### Inverse Kinematics Pick and Place Demos

These examples demonstrate how to use Inverse Kinematics (IK) to perform a pick-and-place task.

#### 1. Static Pick and Place Demo
- **File:** `scripts/pick_and_place_static.py`
- **Description:** This script commands the robot to pick up a block from a specified location and place it at another. The target coordinates for the block are provided as command-line arguments.
- **How to Run:**
  1. Launch the Gazebo world: `roslaunch sawyer_gazebo sawyer_world.launch`
  2. In a new terminal, run the script with your desired coordinates:
     ```bash
     rosrun sawyer_sim_examples pick_and_place_static.py --x 0.6 --y 0.2 --z -0.129
     ```

#### 2. Dynamic Pick and Place Demo
- **File:** `scripts/pick_and_place_dynamic.py`
- **Description:** This script demonstrates a more advanced pick-and-place scenario where the robot waits for the block's position to be published on a ROS topic (`/box_pose`). This simulates a system where a perception module (like a camera) provides the object's location dynamically.
- **Helper Script:** `scripts/mock_perception_publisher.py`
  - This script acts as a simulated perception system. It publishes a single coordinate to the `/box_pose` topic and then exits.
- **How to Run:**
  1. Launch the Gazebo world: `roslaunch sawyer_gazebo sawyer_world.launch`
  2. In a new terminal, run the dynamic script. It will wait for a message:
     ```bash
     rosrun sawyer_sim_examples pick_and_place_dynamic.py
     ```
  3. In a third terminal, run the publisher to send the pose and trigger the robot's action:
     ```bash
     rosrun sawyer_sim_examples mock_perception_publisher.py
     ```
