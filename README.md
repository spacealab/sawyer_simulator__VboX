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

The Intera SDK provides Python interfaces for robot control:

```python
import rospy
import intera_interface

# Initialize ROS node
rospy.init_node('sawyer_example')

# Get robot interfaces
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')

# Move to neutral position
limb.move_to_neutral()

# Control gripper
gripper.open()
gripper.close()
```

### Environment Setup Script

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

### Basic Joint Control
```python
from intera_interface import Limb
import rospy

rospy.init_node('joint_control')
limb = Limb('right')

# Get current joint angles
angles = limb.joint_angles()

# Move to specific joint positions
joint_command = {'right_j0': 0.0, 'right_j1': 0.0, 'right_j2': 0.0,
                 'right_j3': 0.0, 'right_j4': 0.0, 'right_j5': 0.0,
                 'right_j6': 0.0}
limb.move_to_joint_positions(joint_command)
```

### Cartesian Control
```python
# Move to Cartesian pose
pose = limb.endpoint_pose()
pose['position'].x += 0.1  # Move 10cm in x
limb.move_to_cartesian_pose(pose)
```

## Troubleshooting

### Common Issues

1. **Gazebo crashes on startup**:
   - Ensure graphics drivers are properly installed
   - Try running with `gui:=false` for headless mode

2. **ROS dependency errors**:
   - Run `rosdep install --from-paths src --ignore-src -r -y`
   - Ensure ROS environment is sourced

3. **Python import errors**:
   - Check Python path: `echo $PYTHONPATH`
   - Ensure workspace is built and sourced

4. **Joint limits exceeded**:
   - Use the SNS-IK library for constraint-aware IK solving

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

### Exercise 1: Modifying Joint Movement Resolution
- **File**: `src/intera_sdk/intera_examples/scripts/joint_position_keyboard.py`
- **Description**: Added functionality to change the resolution (delta) of joint movements using keyboard keys.
- **Changes**:
  - Added `delta` variable for resolution control.
  - Created `change_resolution()` function to update delta.
  - Updated `bindings` dictionary to use dynamic delta.
  - Added keys 'a' (0.05 rad), 's' (0.1 rad), 'd' (0.2 rad) for resolution switching.
- **Usage**: Run the script and press the keys to adjust movement precision.
