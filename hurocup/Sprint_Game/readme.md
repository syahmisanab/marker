# 🏃 Sprint Game Nodes - Ainex Robot (FIRA HuroCup)

This folder contains ROS nodes used to execute and trigger the **Sprint** event routine in FIRA HuroCup. The robot walks forward, detects a white line, and returns to the starting point.

---

## ✅ FEATURE LIST – Sprint Nodes

### 🧠 Core Functionality

#### `sprint_line_trigger.py` (Advanced)

* Waits for a white line to appear on `/object/pixel_coords`
* Walks forward until the line is detected
* After detection, continues briefly, then reverses
* Dynamically adjusts head tilt (looking down to up)

#### `sprint_game_node.py`

* Walks forward for 17 seconds
* Walks backward for 16 seconds
* Uses `GaitManager` for motion
* Uses optional gait parameter tuning

---

## 🔁 Behavior Summary

### `sprint_line_trigger.py` (Advanced Sprint)

* Set head downward for line scanning
* Start walking forward
* Detect white line → continue walking briefly → reverse back
* Stop and raise head

### `sprint_game_node.py`

* Start walking forward for 17 seconds
* Walk backward for 16 seconds
* Stop after completion

---

## ✂️ Tunable Snippets for Students

### 🎯 Gait Tuning Example

```python
gait_param = gait_manager.get_gait_param()
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.008
gait_param['body_height'] = 0.025
```

### ⏱ Walking Durations

```python
time.sleep(17)  # Forward
...
time.sleep(16)  # Return (backward)
```

### 🤖 Trigger Walk from White Line

```python
if obj.type == 'line':
    self.line_detected = True
```

### 🎥 Echo detected objects

```bash
rostopic echo /object/pixel_coords
```

---

## 🚀 Script Usage

### File Locations

```
~/ros_ws/src/ainex_example/scripts/marker/hurocup
```

### `sprint_line_trigger.py`

Detects white line trigger and performs a reverse move.

```bash
rosrun ainex_example sprint_line_trigger.py
```

### `sprint_game_node.py`

Performs basic forward and reverse sprint.

```bash
rosrun ainex_tutorial sprint_game_node.py
```

---

## 📆 Requirements

* ROS (Melodic/Noetic)
* Python 3
* `ainex_kinematics`, `ainex_sdk`, and `ainex_controller`
* Proper `/object/pixel_coords` topic for color detection
* Action group path (used by `MotionManager`):

```python
MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')
```

---

## 🎓 For Students: Learning Objectives

* Learn gait tuning for straight motion
* Use visual triggers (white line) to initiate robot behavior
* Combine head control + motion sequences
* Practice timing-based task control in ROS

---

Let me know if you want to chain this with other nodes or develop a more sensor-based sprint system!
