# 🦿 Gait Control - Sprint Movement (Ainex Robot)

This module demonstrates basic forward walking using the Ainex `GaitManager`. It's typically used for the **Sprint** event in the FIRA HuroCup, where the robot must walk straight and quickly over a flat surface.

---

## ✅ FEATURE LIST – move\_forward.py

### 🧠 Core Functionality

* Uses Ainex `GaitManager` to control dynamic walking.
* Moves the robot forward for a set duration (default: 17 seconds).
* Allows fine-tuning of walking parameters like step height and pelvis offset.

### ⚙️ Gait Parameter Configuration

* `pelvis_offset`: Horizontal offset of the pelvis during walk cycle.
* `step_height`: Maximum foot lift height.
* `z_swap_amplitude`: Smoothness of weight transfer.
* `body_height`: Torso elevation during walking.

### ✅ ROS Integration

* Initializes as a ROS node (`gait_control_demo`).
* Can be run as a standalone Python script or via `rosrun`.

### 🛑 Clean Stop

* Uses `gait_manager.stop()` to end movement safely.

---

## ✂️ Tunable Snippets for Students

### ⚙️ Modify Gait Parameters

```python
gait_param = gait_manager.get_gait_param()
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.008
gait_param['body_height'] = 0.025
```

### 🚶‍♂️ Start Walking

```python
gait_manager.move(1, 0.02, 0, 0)  # move(forward_speed, x, y, yaw)
```

### ⏱️ Control Walking Duration

```python
time.sleep(17)  # walk forward for 17 seconds
```

### 🛑 Stop Gait

```python
gait_manager.stop()
```

---

## 🚀 Script Overview

### `move_forward.py`

* Walks the robot forward at a steady pace.
* Uses ROS-compatible motion startup.
* Fine-tuned for stable straight-line walking.

📂 File location:

```
~/ros_ws/src/ainex_example/scripts/marker/hurocup/Gait_Control
```

▶️ Run with:

```bash
python3 move_forward.py
```

Or with ROS:

```bash
rosrun ainex_tutorial move_forward.py
```

---

## 🧠 Behavior Summary

* Robot initializes gait and motion managers.
* Loads walking parameters.
* Starts walking forward.
* Stops after specified duration.

---

## 📦 Requirements

* ROS (Melodic/Noetic)
* `ainex_sdk`, `ainex_kinematics`
* Proper path to action group folder:

```python
MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')
```

---

## 🎓 For Students: Learning Objectives

* Understand gait cycle tuning in humanoid robots.
* Practice safe motion startup and stop.
* Explore effects of step height, body pitch, and offset.
* Learn timing and motion sequencing for competitions.

---
