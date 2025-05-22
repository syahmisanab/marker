# 🏃 Sprint Game Node - Ainex Robot (FIRA HuroCup)

This script performs the **Sprint** event routine as part of the FIRA HuroCup competition. The robot walks forward, turns 180°, and walks back to the starting point using timed motion commands.

---

## 🚀 Script Overview

### `sprint_game_node.py`

- Walks forward for 17 seconds
- Performs an in-place 180° turn (yaw rotation)
- Walks back for 16 seconds
- Stops walking and completes the sprint routine

📍 File location:
```
~/ros_ws/src/ainex_example/scripts/marker/hurocup
```

▶️ Run with:
```bash
python3 sprint_game_node.py
```
or using ROS:
```bash
rosrun ainex_tutorial sprint_game_node.py
```

---

## 🧠 Behavior Summary

- Uses `GaitManager` for dynamic gait
- Yaw speed and durations are tuned to achieve approx. 180° rotation
- Executes timed gait sequences with pauses between actions
- Uses optional gait parameter tuning (`pelvis_offset`, `step_height`, etc.)

---

## 📦 Requirements

- ROS (Melodic or Noetic)
- `ainex_kinematics` and `ainex_controller` path set correctly
- Python 3
- Ainex robot hardware platform

---

💡 This node is useful for demo purposes or sprint challenge rehearsal. Tune timings if needed to better match surface and performance conditions.
