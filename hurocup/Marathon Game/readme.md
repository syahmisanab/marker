# 🏁 Marathon Node - Ainex Robot (Line + Marker Navigation)

This script controls the robot for the **Marathon** game in FIRA HuroCup. It enables the robot to follow a black line and respond to arrow markers (`left`, `right`, `forward`) as it progresses along the course.

---

## 🚀 Script Overview

### `marathon_node.py`

- **Phase 1**: Follows a black line using `VisualPatrol`
- **Phase 2**: Upon losing the line, transitions to arrow marker detection
- Detects direction from `/arrow/shape_direction`
- Turns left/right or continues forward based on marker
- Uses `GaitManager` to walk and rotate

📍 File location:
```
/home/ubuntu/ros_ws/src/ainex_example/scripts/marker/marathon_node.py
```

▶️ Run with:
```bash
rosrun ainex_example marathon_node.py
```

---

## 🧠 Behavior Logic

- Robot follows line using pixel position + width
- If line is lost for 0.3 seconds, switches to arrow mode
- Listens for `"left"`, `"right"`, or `"forward"` arrow messages
- Rotates appropriately and resumes walking forward
- Waits for the next marker before stopping again

---

## 📦 Requirements

- ROS (Melodic/Noetic)
- `ainex_sdk`, `ainex_interfaces`, `ainex_kinematics`
- `/arrow/shape_direction` topic from arrow detection node
- Properly tuned gait parameters

---

💡 Use this node in conjunction with:
- `arrow_detection_node.py`
- Proper black line and arrow field layout

Make sure the robot is in `walk_ready` pose before starting this node.
