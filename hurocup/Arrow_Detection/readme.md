# 🎯 Arrow Detection & Turning - Ainex Robot (FIRA HuroCup)

This module includes two ROS nodes for enabling the Ainex robot to visually detect arrow shapes (left, right, forward) and execute corresponding turning motions. It is typically used during the **Marathon** event of FIRA HuroCup.

---

## ✅ FEATURE LIST – Arrow Detection & Turn

### 🔍 Arrow Detection Node (`arrow_detection_node.py`)

* Subscribes to raw camera images (`/camera/image_raw`).
* Applies image preprocessing (grayscale, blur, canny, dilation, erosion).
* Detects arrows based on contour shape, convexity, and number of points.
* Supports detection of `left`, `right`, and `forward` arrows.
* Publishes direction to `/arrow/shape_direction`.
* Optionally displays debug output window with contour and label.

### 🔄 Arrow Turn Node (`turn_arrow_node.py`)

* Listens to `/arrow/shape_direction`.
* Waits for stable detection (e.g., `0.2s` consistent).
* Executes rotation using `GaitManager` to turn left or right.
* Includes one-turn safety logic using `self.has_turned`.
* Provides `/arrow_turn/reset` service to manually re-enable turning.

---

## ✂️ Tunable Snippets for Students

### 🕒 Detection Hold Duration (in turn node)

```python
self.detection_hold_time = 0.2  # seconds
```

### ⚙️ Turn Speed and Duration

```python
self.turn_speed = 10
self.turn_duration = 2.6  # approx 90 degrees
```

### ♻️ Reset Wait Time

```python
self.reset_wait_time = 3.0  # pause before turn resets
```

### 🖼️ Display Toggle in Arrow Detection

```bash
rosparam set /arrow_shape_node/enable_display false  # headless mode
```

---

## 🚀 Nodes Overview

### 1. `arrow_detection_node.py`

* Subscribes to: `/camera/image_raw`
* Publishes:

  * `/arrow/shape_direction` (String)
  * `/arrow/shape_result` (debug image)
* Detects arrow direction based on shape points:

  * `7 points`: forward
  * `9 points`: left or right (based on geometry)

▶️ Run with:

```bash
rosrun ainex_example arrow_detection_node.py
```

📂 Location:

```
~/ros_ws/src/ainex_example/scripts/marker/hurocup/Arrow_Detection
```

💬 View arrow detection output:

```bash
rostopic echo /arrow/shape_direction
```

---

### 2. `turn_arrow_node.py`

* Subscribes to: `/arrow/shape_direction`
* Publishes: uses gait manager internally (no ROS publisher)
* Services:

  * `/arrow_turn/reset` to manually allow next turn

▶️ Run with:

```bash
rosrun ainex_example turn_arrow_node.py
```

📂 Location:

```
~/ros_ws/src/ainex_example/scripts/marker/turn_arrow_node.py
```

---

## 🧠 Behavior Logic Summary

### Arrow Detection:

* Processes each frame from the camera.
* Applies filters and contour detection.
* Checks contour shape, convexity, aspect ratio, and number of points.
* Assigns direction and publishes the result.

### Arrow Turn:

* Listens for direction string.
* If stable for `0.2s` and hasn't turned yet:

  * Turns the robot
  * Locks out further turning until `/arrow_turn/reset` is called

---

## 📦 Dependencies

* ROS (Melodic/Noetic)
* OpenCV (via `cv2`)
* `cv_bridge`
* `ainex_sdk`, `ainex_kinematics`
* Proper topic remapping for camera input (e.g., `/camera/image_raw`)

---

## 🛠 Make Executable

```bash
chmod +x arrow_detection_node.py
chmod +x turn_arrow_node.py
```

---

## 🎓 For Students: Learning Objectives

* Learn how to process images using OpenCV in ROS
* Understand shape detection via contours and polygon approximation
* Practice event-based robot response using `std_msgs/String`
* Implement consistent detection hold logic
* Work with services to control stateful behavior (e.g. one-time turns)

---

Let me know if you want to integrate this with the line-following node or combine both into a single decision framework.
