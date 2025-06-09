
# 🏁 Ainex Competition Training 

**Audience:** Robotics team students  
**Goal:** Prepare students to understand and operate Ainex for sprint and marathon competition tasks.

---

## 🕐 Schedule Overview

| Time            | Topic                              | Description |
|-----------------|-------------------------------------|-------------|
| **9:00 – 9:30** | Introduction                      | Show robot capabilities and the competition context |
| **9:30 – 10:30**| Network Setup + VNC               | Connect to robot via IP, use VNC viewer |
| **10:30 – 11:30**| Python + ROS Basics              | Topics: scripts, `rostopic`, `rosrun`, messages |
| **11:30 – 12:00**| Docker on Ainex + Ainex Controller| Start/use containers, no deep dive |
| **12:00 – 1:00** | Lunch Break                      | - |
| **1:00 – 2:00** | Sprint – Gait Control             | Control walking: forward/reverse with `gait.move()` |
| **2:00 – 2:30** | Sprint Game Implementation        | Use `sprint.py`, run through the game |
| **2:30 – 3:30** | Line Following with VisualPatrol  | How the robot follows black lines |
| **3:30 – 4:00** | Short Break                       | - |
| **4:00 – 4:30** | Arrow Detection Recap             | Test and tune arrow direction detection |
| **4:30 – 5:30** | Marathon Game Integration         | Combine line following + arrow logic (`marathon.py`) |
| **5:30 – 6:00** | Recap + Free Practice + Q&A       | Let students test, edit, and experiment |

---

## 1. Introduction
- Overview of Ainex robot and competition structure
- What students will build and achieve by the end of the day
- Live demo of Sprint and Marathon games

## 2. Network Configuration
- Connecting to Ainex over LAN/Wi-Fi
- Accessing Ainex using **VNC Viewer**
- Troubleshooting connection issues

## 3. Python & ROS Basics
- Basic Python scripting (variables, loops, functions)
- Core ROS concepts: nodes, topics, messages, services
- Using `rostopic`, `rosrun`, and viewing messages

## 4. Docker Basics & Ainex Controller
- Using Docker to launch the Ainex environment
- How to enter and interact with the ROS container
- Introduction to the **Ainex Controller GUI**:
  - Move individual joints
  - Load and play motion groups (like `walk_ready`)
  - Explore joint calibration 

## 5. Gait Control (Sprint – Movement)
- Using `GaitManager` to walk forward, reverse, and turn
- Understanding movement parameters (`x`, `y`, `yaw`)
- Stopping gait safely

## 6. Sprint Game (`sprint.py`)
- Full movement loop: walk → reverse
- Structure of `sprint.py` code
- Editing duration and speed
- Running the sprint mission

## 7. Line Following (Visual Patrol)
- Explaining line detection using color
- Understanding ROIs and how the camera is processed
- Running and tuning `VisualPatrol` for stability

## 8. Arrow Detection
- Using OpenCV to detect left, right, and forward arrows
- Understanding 7/9-point contour logic
- How detection is published as `/arrow/shape_direction`

## 9. Marathon Game (`marathon.py`)
- Transition logic: line follow → arrow detect → react
- Handling turns and forward walking
- Testing full mission from start to finish


---
## 🎯 Deliverables

By the end of the day, students should be able to:
- Operate the robot via VNC
- Control gait manually
- Modify and run `sprint.py` and `marathon.py`
- Understand ROS topics used in competition
- Debug and tweak parameters like turn speed and line color

---


