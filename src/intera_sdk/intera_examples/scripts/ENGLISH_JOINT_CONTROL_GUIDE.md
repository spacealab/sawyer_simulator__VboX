# 🤖 Sawyer Robot Joint Control & Box Pickup Guide

## 🚀 Quick Start

### 1️⃣ Setup
```bash
cd /home/ali/Downloads/ubuntu/sawyerws_v1
source devel/setup.bash
rosrun intera_examples joint_control_interface.py
```

### 2️⃣ Calibration (REQUIRED!)
- Press **`C`** key
- Wait for "Calibrated ✅" message

## 🦾 Joint Locations & Functions

### 📍 Understanding Robot Anatomy:
```
        j6 (END-EFFECTOR)
           ↓
    j5 (WRIST ROLL) ←→
           |
    j4 (WRIST PITCH) ↕
           |
    j3 (FOREARM ROLL) ↻
           |
    j2 (ELBOW) ⌐
           |
    j1 (SHOULDER) ↕ (MOST IMPORTANT!)
           |
    j0 (BASE) ↻ (ROTATES ENTIRE ARM)
```

### 🎮 Joint Control Keys:

| Key | Joint | Location | Function |
|-----|-------|----------|----------|
| **1/q** | j0 | Base | **Rotates entire arm left/right** |
| **2/w** | j1 | Shoulder | **Lifts/lowers entire arm** (MAIN MOVEMENT) |
| **3/e** | j2 | Elbow | **Extends/bends arm forward/back** |
| **4/r** | j3 | Forearm | Twists wrist area left/right |
| **5/t** | j4 | Wrist | Bends wrist up/down |
| **6/y** | j5 | Wrist | Rolls wrist left/right |
| **7/u** | j6 | End-effector | Final rotation at gripper |

## 🎯 Box Pickup Steps (Detailed)

### Step 1: Position Above Box
- **`1`** = Rotate arm LEFT (entire arm moves left)
- **`q`** = Rotate arm RIGHT (entire arm moves right)  
- **`2`** = Lift arm UP (shoulder lifts entire arm)
- **`w`** = Lower arm DOWN (shoulder lowers entire arm)
- **`3`** = Extend arm FORWARD (elbow stretches out)
- **`e`** = Pull arm BACK (elbow bends in)

### Step 2: Approach Box
- Press **`w`** multiple times (SHOULDER DOWN)
- Watch arm descend toward box level
- Stop when gripper is just above box

### Step 3: Grab Box
- Press **`P`** (Close Gripper) 🔒
- Wait for "Box grabbed!" message

### Step 4: Lift Box
- Press **`2`** (SHOULDER UP) ⬆️
- Box should rise with arm

### Step 5: Transport Box
- Use combination of keys to move:
  - **1/q**: BASE rotation for left/right positioning
  - **2/w**: SHOULDER for up/down movement
  - **3/e**: ELBOW for forward/back reach
  - **4/r**: FOREARM twist for fine orientation

### Step 6: Release Box
- Press **`w`** to lower (SHOULDER DOWN)
- Press **`O`** (Open Gripper) 🔓
- Press **`2`** to move away (SHOULDER UP)

## 🎮 Key Controls Summary

### 🔄 **Primary Movement Keys (Most Important):**
- **`2`** = SHOULDER UP (lifts entire arm)
- **`w`** = SHOULDER DOWN (lowers entire arm)
- **`1`** = BASE LEFT (rotates whole arm left)
- **`q`** = BASE RIGHT (rotates whole arm right)

### 🤏 **Gripper Controls:**
- **`C`** = Calibrate (DO THIS FIRST!)
- **`P`** = Close (grab box) 🟢
- **`O`** = Open (release box) 🔴

### 🔧 **Fine Adjustment Keys:**
- **`3/e`** = ELBOW extend/bend
- **`4/r`** = FOREARM twist
- **`5/t`** = WRIST pitch
- **`6/y`** = WRIST roll
- **`7/u`** = END-EFFECTOR rotation

## 💡 Pro Tips

1. **Start with big movements**: Use SHOULDER (2/w) and BASE (1/q) first
2. **Then fine-tune**: Use ELBOW (3/e) and other joints for precision
3. **Watch the interface**: Status messages guide you through each step
4. **Move slowly**: Press each key once, observe movement, then continue
5. **Emergency stop**: Press `Esc` to exit

## 🎬 Complete Pickup Sequence:
```
C → Position (1,q,2,w,3,e) → w (approach) → P (grab) → 2 (lift) → 
Transport → w (lower) → O (release) → 2 (move away)
```

**Good luck with your robot control! 🎉**
