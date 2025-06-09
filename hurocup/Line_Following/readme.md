# 🧭 Visual Line Patrol Node - Ainex Robot

This node enables real-time black line following using the Ainex robot's head-mounted camera. It is optimized for the **Marathon** event in FIRA HuroCup, where the robot follows a predefined track using image-based line detection.

---

## ✅ FEATURE LIST – VisualPatrolNode

### 🎯 Core Functionality

#### Line Detection & Tracking

* Processes vision data to follow a black line on the ground.
* Scans 3 ROIs (top, middle, bottom) vertically in the frame.
* Immediately reacts to a detected line by adjusting walking direction.

#### Real-Time Gait Control

* Uses `VisualPatrol` for PID-style steering.
* Integrates `GaitManager` for smooth forward walking and lateral adjustments.

#### ROS Integration

* Initializes as a ROS node with name and parameters.
* Starts tracking on launch if `~start` param is `true`.
* Sets head posture using `MotionManager` (e.g., tilt down).

---

## ✂️ Tunable Snippets for Students

### 🕒 Change Image Processing Size

```python
self.image_process_size = [160, 120]  # width x height
```

### 🎯 ROI Zones (Y & X ratios of frame)

```python
self.line_roi = [
    (5 / 12, 6 / 12, 1 / 4, 3 / 4),
    (6 / 12, 7 / 12, 1 / 4, 3 / 4),
    (7 / 12, 8 / 12, 1 / 4, 3 / 4)
]
```

### 🤖 Head Pose Initialization

```python
self.head_pan_init = 500
self.head_tilt_init = 260
```

### 🎨 Change Line Color from Launch Param or Terminal

```bash
rosparam set /visual_patrol/color black
```

Or call the service:

```bash
rosservice call /visual_patrol/set_color "data: 'black'"
```

---

## 🚀 Script Overview

### `visual_patrol_node.py`

* Subscribes to `/object/pixel_coords` for color object info.
* Sends ROI-based config to `/color_detect/input`.
* Detects line position and width to adjust robot's trajectory.

📂 File location:

```
~/ros_ws/src/ainex_example/scripts/marker/hurocup/Line Following (Visual Patrol)
```

▶️ Run with:

```bash
rosrun ainex_example visual_patrol_node.py
```

💬 To monitor detection result:

```bash
rostopic echo /object/pixel_coords
```

---

## 🧠 Behavior Summary

* Initializes head posture and `walk_ready` pose.
* Configures color detection to track a black line.
* Continuously processes recognition data and steers accordingly.
* Remains reactive while `self.running` and `self.start` are `True`.

---

## 📦 Requirements

* ROS (Melodic/Noetic)
* `ainex_sdk`, `ainex_interfaces`, `ainex_kinematics`
* Proper camera topic: `/camera/image_raw`
* Color detection nodes and models configured for `black`

---

## 🎓 For Students: Learning Outcomes

* Understand ROI-based image processing in mobile robotics
* Tune vision and walking control integration
* Explore color detection using services and real-time control loops
* Debug and observe sensor-actuator loops using ROS topics

---
