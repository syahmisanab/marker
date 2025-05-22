# 🦿 Gait Control - Sprint Movement (Ainex Robot)

This directory contains a basic walking demo script used for the **Sprint** event in the FIRA HuroCup. It demonstrates forward walking using Ainex's `GaitManager`.

---

## 🚀 Script Overview

### `move_forward.py`

- Walks the robot forward for 17 seconds at a steady speed
- Uses `GaitManager` for continuous gait control
- Optionally modifies pelvis offset, step height, and body posture
- Requires Ainex robot with correct action group path

📍 File location:
```
/home/ubuntu/ros_ws/src/ainex_tutorial/scripts/gait_control/move_forward.py
```

▶️ Run with:
```bash
python3 move_forward.py
```
or using ROS:
```bash
rosrun ainex_tutorial move_forward.py
```

---

## 🛠 Dependencies

- ROS (Melodic or Noetic)
- `ainex_kinematics` package
- Python 3
- Ainex hardware platform

---

Make sure the robot is standing in `walk_ready` posture before running this script. You can use `motion_manager.run_action('walk_ready')` if needed.

