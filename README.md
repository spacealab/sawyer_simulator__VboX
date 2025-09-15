# Sawyer Simulator

A ROS-based simulation environment for the Sawyer robotic arm from Rethink Robotics, featuring Gazebo integration, Python interfaces, and advanced inverse kinematics capabilities.

## Overview

This repository provides a complete simulation setup for the Sawyer robot, including:

- **Gazebo Simulation**: Full physics-based simulation of the Sawyer robot arm
- **Intera SDK**: Python API for robot control and interaction
- **Inverse Kinematics**: SNS-IK library for advanced kinematic solving
- **Examples**: Ready-to-run demonstration scripts and launch files
- **Hardware Interface**: Emulated interfaces matching real Sawyer hardware

## Repository Structure

```
sawyer_simulator__VboX/
├── src/
│   ├── intera_common/          # Common messages and end-effector descriptions
│   ├── intera_sdk/             # Python SDK and examples
│   ├── sawyer_robot/           # Sawyer-specific robot description
│   ├── sawyer_simulator/       # Gazebo simulation packages
│   │   ├── sawyer_gazebo/      # Gazebo interface and world setup
│   │   ├── sawyer_sim_controllers/  # Controller plugins
│   │   ├── sawyer_sim_examples/     # Simulation-specific examples
│   │   └── sawyer_hardware_interface/ # Hardware emulation
│   └── sns_ik/                 # Saturation in Null-Space IK library
├── LICENSE.md
└── README.md
```

## Prerequisites

- **ROS**: Kinetic, Melodic, or Noetic (recommended: Noetic)
- **Gazebo**: Version compatible with your ROS distribution
- **Python**: 2.7 or 3.x (depending on ROS version)
- **Catkin Tools**: For building the workspace

### Installing ROS

For Ubuntu 20.04 (ROS Noetic):
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### Installing Dependencies

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/spacealab/sawyer_simulator__VboX.git
   cd sawyer_simulator__VboX
   ```

2. **Initialize the workspace**:
   ```bash
   cd src
   catkin_init_workspace
   cd ..
   ```

3. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the workspace**:
   ```bash
   catkin_make
   ```

5. **Source the setup**:
   ```bash
   source devel/setup.bash
   ```

## Usage

### Starting the Simulation

1. **Launch Sawyer in Gazebo**:
   ```bash
   roslaunch sawyer_gazebo sawyer_world.launch
   ```

   Optional arguments:
   - `electric_gripper:=true` - Load the electric gripper
   - `pedestal:=true` - Include the robot pedestal
   - `gui:=false` - Run headless
   - `paused:=true` - Start simulation paused

2. **Run example demonstrations**:
   ```bash
   # Pick and place demo
   roslaunch sawyer_sim_examples sawyer_pick_and_place_demo.launch

   # Manual control demo
   roslaunch sawyer_sim_examples manual_pick_and_place.launch

   # Vision demo
   roslaunch sawyer_sim_examples vision_demo.launch
   ```

### Using the Python SDK

```python
import rospy
import intera_interface

rospy.init_node('sawyer_example')
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right_gripper')

# Basic operations
limb.move_to_neutral()
gripper.calibrate()  # Required!
gripper.open()
gripper.close()
```

### Joint Control Interface

Interactive robot control with GUI:

```bash
rosrun intera_examples joint_control_interface.py
```

**Quick Controls:**
- **C** = Calibrate gripper (do first!)
- **2/w** = Up/down movement (SHOULDER)
- **1/q** = Left/right positioning (BASE)  
- **P** = Grab, **O** = Release
- **Mouse** = Select joint + wheel for precision

**Joint Locations:**
- j0 (BASE): 1/q - Rotate entire arm
- j1 (SHOULDER): 2/w - Main up/down
- j2 (ELBOW): 3/e - Extend/bend reach
- j3-j6: Fine wrist adjustments

### Examples

Use the provided `intera.sh` script for convenient environment setup:

```bash
# Copy to workspace root
cp src/intera_sdk/intera.sh .

# Edit configuration (robot_hostname, your_ip, ros_version)
nano intera.sh

# Source the environment
./intera.sh sim  # For simulation mode
```

## Key Components

### Intera Common
- ROS messages for robot communication
- URDF descriptions for end-effectors
- Common utilities

### Intera SDK
- Python API for robot control
- Example scripts and tutorials
- Interface classes for limbs, grippers, and cameras

### Sawyer Robot
- Robot-specific URDF and meshes
- Configuration files
- Named poses and parameters

### Sawyer Simulator
- Gazebo world setup
- Hardware interface emulation
- Controller plugins
- Simulation-specific examples

### SNS-IK Library
- Advanced inverse kinematics algorithms
- Handles redundant kinematic chains
- Joint limit and velocity constraints
- Multiple task prioritization

## API Documentation

- [Intera SDK Documentation](http://sdk.rethinkrobotics.com/intera/)
- [Python API Reference](http://rethinkrobotics.github.io/intera_sdk_docs)
- [ROS Wiki](http://wiki.ros.org/)

## Examples

### Basic Joint & Gripper Control
```python
import rospy
from intera_interface import Limb, Gripper

rospy.init_node('robot_control')
limb = Limb('right')
gripper = Gripper('right_gripper')

# Basic operations
gripper.calibrate()          # Required first!
limb.move_to_neutral()       # Safe position
gripper.close()              # Grab
gripper.open()               # Release
```

### Simple Pick & Place
```python
# Get joint angles and modify for pickup
angles = limb.joint_angles()
angles['right_j1'] = -0.5    # Lower shoulder
limb.move_to_joint_positions(angles)
gripper.close()              # Grab

angles['right_j1'] = 0.2     # Lift up
angles['right_j0'] = 0.5     # Rotate to new position
limb.move_to_joint_positions(angles)
gripper.open()               # Release
```

## Troubleshooting

**Common Fixes:**
- **Gazebo crash**: Try `gui:=false` for headless mode
- **Dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`
- **GUI errors**: Check graphics drivers or run headless
- **Gripper warnings**: Use `Gripper('right_gripper')` instead of `Gripper('right')`
- **Interface issues**: Ensure ROS master running (`roscore`) and workspace sourced

### Getting Help

- [GitHub Issues](https://github.com/RethinkRobotics/sawyer_simulator/issues)
- [Rethink Robotics Support](https://support.rethinkrobotics.com/support/home)
- [ROS Answers](https://answers.ros.org/)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly in simulation
5. Submit a pull request

Please follow the [Contributing Guidelines](src/intera_common/CONTRIBUTING.md) for each component.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- Original development by Rethink Robotics
- SNS-IK algorithms by Fabrizio Flacco, Alessandro De Luca, and Oussama Khatib
- ROS and Gazebo communities

## Version History

- **Current**: Based on Rethink Robotics Sawyer Simulator v5.3.0
- **ROS Support**: Indigo, Kinetic, Melodic, Noetic
- **Gazebo**: Compatible with ROS distribution versions

## Exercises

### Enhanced Joint Control with GUI Interface
- **Files**: 
  - `src/intera_sdk/intera_examples/scripts/joint_position_keyboard.py` (Variable resolution control)
  - `src/intera_sdk/intera_examples/scripts/joint_control_interface.py` (GUI interface with gripper)

- **Features**:
  - **Variable Resolution**: Adjustable movement precision (0.05, 0.1, 0.2 rad) using keys 'a', 's', 'd'
  - **GUI Interface**: Real-time joint monitoring with dual control methods
  - **Gripper Control**: Complete gripper functionality (open/close/calibrate)
  - **Pick & Place Guide**: Step-by-step box pickup instructions
  - **Joint Reference**: Clear explanation of each joint location and function

- **Quick Usage**:
  ```bash
  # Variable resolution control
  rosrun intera_examples joint_position_keyboard.py
  
  # GUI interface with gripper
  rosrun intera_examples joint_control_interface.py
  ```

- **Key Controls**:
  - **C**: Calibrate gripper (required first!)
  - **1/q**: j0 (BASE) - Rotate arm left/right
  - **2/w**: j1 (SHOULDER) - Move arm up/down (main movement)
  - **3/e**: j2 (ELBOW) - Extend/bend arm
  - **P**: Close gripper (grab), **O**: Open gripper (release)
  - **Mouse**: Select joint + wheel for precise control

- **Box Pickup Steps**:
  1. Press 'C' → 2. Position above box → 3. Lower with 'w' → 4. Grab with 'P' → 5. Lift with '2' → 6. Move to destination → 7. Lower with 'w' → 8. Release with 'O'
