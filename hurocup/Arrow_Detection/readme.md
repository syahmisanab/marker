# 🎯 Arrow Detection & Turning - Ainex Robot (FIRA HuroCup)

This folder contains two ROS nodes used for detecting arrow shapes and triggering corresponding turning motions on the Ainex humanoid robot.

These are typically used during the **Marathon** event in FIRA HuroCup, where the robot must:
- Follow a black line
- Detect arrow markers
- Turn left or right accordingly

---

## 🚀 Nodes Overview

### 1. `arrow_detection_node.py`

- Subscribes to camera image (`/camera/image_raw`)
- Detects arrow shapes using contours
- Publishes arrow direction to `/arrow/shape_direction`
- Optionally displays annotated image

📍 Location:
```
~/ros_ws/src/ainex_example/scripts/marker/hurocup/Arrow Detection
```

▶️ Run with:
```bash
rosrun ainex_example arrow_detection_node.py
```

---

### 2. `turn_arrow_node.py`

- Listens to `/arrow/shape_direction`
- Turns robot left or right if a consistent arrow is detected
- Uses `GaitManager` for walking
- Includes a reset service `/arrow_turn/reset` to re-enable turning after one turn

📍 Location:
```
/home/ubuntu/ros_ws/src/ainex_example/scripts/marker/turn_arrow_node.py
```

▶️ Run with:
```bash
rosrun ainex_example turn_arrow_node.py
```

---

## 🛠 How to Make Scripts Executable

If not already executable:

```bash
chmod +x arrow_detection_node.py
chmod +x turn_arrow_node.py
```

---
