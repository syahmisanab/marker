# 🏁 Marathon Node - Ainex Robot (Line + Marker Navigation)

This script controls the robot for the **Marathon** game in FIRA HuroCup. It enables the robot to follow a black line and respond to arrow markers (`left`, `right`, `forward`) as it progresses along the course.

---

## ✅ FEATURE LIST – VisualPatrolNode

### 🎯 Core Functionality

#### Line Following

* Uses visual input to track a colored line.
* Uses configurable ROI (region of interest) to detect the line in different vertical zones.
* Continuously adjusts robot movement to follow the line using `VisualPatrol`.

#### Marker Recognition

* Subscribes to `/arrow/shape_direction` topic.
* Detects directional markers: `"left"`, `"right"`, `"forward"`.
* Remembers the last seen marker while the line is still visible.

#### Line Loss Detection

* Detects when the line disappears.
* Waits for 0.2 seconds (`line_loss_hold_time`) before transitioning to the next phase.
* Triggers marker-based navigation after line loss.

#### Marker-Based Navigation

* Responds to the previously seen marker direction.
* Executes a turn (left/right) or forward walk depending on the marker.
* Uses predefined rotation and time durations for movement.

#### Line Reacquisition (Search Mode)

* After turning, robot enters a line search phase.
* Slowly walks forward while looking for a line.
* As soon as a new line is detected → resumes line following.

---

### 🛠️ ROS Integration

#### ROS Node Initialization

* Sets up a ROS node with configurable name and parameters.

#### ROS Parameters

* `~start`: Whether to begin tracking immediately.
* `~color`: The color of the line to track (e.g., `"black"`).

#### ROS Services

* `~set_color`: Dynamically change the target line color via service call.

#### ROS Topics

* Subscribes to:

  * `/arrow/shape_direction`: for marker direction
  * `/object/pixel_coords`: for line detection
* Publishes to:

  * `/color_detect/input`: sends line tracking configuration

---

### 🤖 Motion Control

#### Motion Manager Integration

* Executes `walk_ready`, `stand`, and `move()` actions via Ainex SDK.
* Allows both linear and rotational motion using `gait_manager`.

#### Failsafe / Shutdown Handling

* Listens for Ctrl+C (`SIGINT`) and safely stops the robot.

---

### 🔁 Phased Behavior Loop

Your robot has a clean, cyclical control structure with:

* **Phase 1**: Line Following
* **Phase 2**: Marker Navigation (turn once)
* **Phase 3**: Line Search (walk forward until line is found)

---

## ✂️ Tunable Snippets for Students

#### 🔧 Adjust how long the robot waits before reacting to line loss:

```python
self.line_loss_hold_time = 0.2  # seconds
```

#### 🎯 Set your image processing resolution:

```python
self.image_process_size = [160, 120]
```

#### 🖼️ Line detection zone (ROI):

```python
self.line_roi = [
    (5 / 12, 6 / 12, 1 / 4, 3 / 4),  # top zone
    (6 / 12, 7 / 12, 1 / 4, 3 / 4),  # middle zone
    (7 / 12, 8 / 12, 1 / 4, 3 / 4)   # bottom zone
]
```

#### 🤖 Tune marker turn behavior:

```python
if self.last_arrow_direction == "left":
    self.gait_manager.move(1, 0, 0, -10)  # rotate left
    time.sleep(5.3)
elif self.last_arrow_direction == "right":
    self.gait_manager.move(1, 0, 0, 10)   # rotate right
    time.sleep(5.3)
```

#### 🎨 Change target color dynamically:

```bash
rosparam set /visual_patrol/color blue
```

Or using service call:

```bash
rosservice call /visual_patrol/set_color "data: 'blue'"
```

---

## 🚀 Script Overview

### `marathon_node.py`

* **Line Following**: Track a colored line using image detection.
* **Marker Recognition**: While following the line, the robot monitors for directional markers.
* **Line Loss + Action**: When the line is lost, the robot executes a turn or move based on the last marker seen.
* **Line Search**: After turning, it slowly walks forward until the line is found again, then resumes following.

📍 File location:

```
~/ros_ws/src/ainex_example/scripts/marker/hurocup
```

▶️ Run with:

```bash
rosrun ainex_example marathon_node.py
```

---

## 🧠 Behavior Logic

* The robot continuously follows the line.
* While following, it listens for arrow marker messages.
* If the line is lost for more than 0.2s:

  * It uses the last known marker direction to rotate or move.
  * Then it walks forward slowly, looking for a new line.
* Once the new line is detected, it resumes following.
* The cycle repeats: line → marker → turn → line reacquire → line.

---

## 📦 Requirements

* ROS (Melodic/Noetic)
* `ainex_sdk`, `ainex_interfaces`, `ainex_kinematics`
* `/arrow/shape_direction` topic from arrow detection node
* Properly tuned gait parameters

💡 Use this node in conjunction with:

* `arrow_detection_node.py`
* Proper black line and arrow field layout

Make sure the robot is in `walk_ready` pose before starting this node.

---

## 🎓 For Students: Learning Outcomes

* Understanding ROS topics, services, and parameters
* Implementing real-time robot control using vision
* Designing finite state machines
* Tuning movement and perception systems for real-world conditions
* Debugging sensor-driven behavior in a competitive robotics scenario

---

Let me know if you’d like to add launch files, simulation support, or more visualization tools!
