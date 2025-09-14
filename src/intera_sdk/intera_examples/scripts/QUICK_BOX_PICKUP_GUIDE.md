# 🤖 Quick Guide for Picking Up a Box with Sawyer

## 🚀 Quick Start

### 1️⃣ Preparation
```bash
# Navigate to your Sawyer workspace
cd /path/to/your/sawyer_ws
# Source the environment
source devel/setup.bash
# Run the manual control interface
rosrun intera_examples joint_control_interface.py
```

### 2️⃣ Calibration (Mandatory!)
- Press the **`C`** key.
- Wait until "Calibrated" is displayed ✅.

## 🎯 Steps for Picking Up a Box (Step-by-Step)

### Step 1: Position Above the Box
- **`1`** = Move Left
- **`q`** = Move Right
- **`2`** = Move Up
- **`w`** = Move Down ⬇️

### Step 2: Approach the Box
- Press **`w`** a few times to get close to the box.
- Be careful not to go too low!

### Step 3: Grab the Box
- Press **`P`** (Close Gripper) 🔒.
- Wait for the "Box Grabbed" message.

### Step 4: Lift the Box
- Press **`2`** to lift the box ⬆️.

### Step 5: Move the Box
- Use keys **`1,q,3,e,4,r`** to move to the destination.

### Step 6: Release the Box
- Press **`w`** to lower the box.
- Press **`O`** (Open Gripper) 🔓.
- Press **`2`** to move the robot away.

## 🎮 Main Keys

| Key | Function | Usage for Box |
|------|-----------|-------------------|
| **C** | Calibrate | ✅ Mandatory Start |
| **2** | Up | ⬆️ Lifting |
| **w** | Down | ⬇️ Approaching |
| **1** | Left | ←→ Positioning |
| **q** | Right | ←→ Positioning |
| **P** | Close Gripper | 🔒 Grabbing Box |
| **O** | Open Gripper | 🔓 Releasing Box |

## ⚠️ Important Notes

1.  **Always press C first!**
2.  **Move slowly** - press each key once at a time.
3.  **Read the on-screen messages** - they provide guidance.
4.  **If you get stuck**: Press Esc and start over.

## 🎬 Full Sequence for a Successful Pickup:
```
C → Position (1,q,2,w) → w (approach box) → P (grab) → 2 (lift) → Move → w (lower) → O (release) → 2 (move away)
```

**Good luck! 🎉**
