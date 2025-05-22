# 🧭 Visual Line Patrol Node - Ainex Robot

This node is used for real-time line following using the robot's head-mounted camera. It tracks a black line on the ground and dynamically adjusts walking direction.

This is typically used for the **Marathon** event in FIRA HuroCup.

---

## 🚀 Script Overview

### `visual_patrol_node.py`

- Subscribes to color detection results
- Detects and follows black lines using predefined ROIs
- Uses `VisualPatrol` + `GaitManager` for real-time correction
- Includes head positioning and walk-ready initialization

📍 File location:
```
~/ros_ws/src/ainex_example/scripts/marker/hurocup/Line Following (Visual Patrol)

```

▶️ Run with:
```bash
rosrun ainex_example visual_patrol_node.py
```

---

## 🛠 Features

- Three ROIs are scanned (top-down)
- If any black line is detected → reacts immediately
- Automatically tilts head at init
- Uses `/object/pixel_coords` and sets color detection params via service

---

## 📦 Requirements

- ROS (Melodic/Noetic)
- `ainex_sdk`, `ainex_interfaces`, `ainex_kinematics`
- Camera stream to `/camera/image_raw`
- Pre-configured black line detection model

---

Make sure your robot is placed in a "ready" position before launching this node. Adjust gait or camera settings in `visual_patrol.py` or launch parameters if needed.
