# 🤖 Ainex Robot Codebase for FIRA HuroCup (Sprint & Marathon)

This repository contains ROS + Python code used to control the Ainex humanoid robot for the FIRA HuroCup competition. It covers:

- 🏃 Sprint Game: Move forward, reverse, and complete a straight-line run
- 🏁 Marathon Game: Line following, arrow detection, and turn-based walking
- ⚙️ Real-time camera + gait integration

This repo is designed for team members and students working with the Ainex robot in preparation for FIRA HuroCup.

---

## 🚀 Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/syahmisanab/marker.git
cd marker
```

### 2. Make scripts executable

```bash
find . -name "*.py" -exec chmod +x {} \;
```

---

## 📦 Requirements

- Ainex robot (with head-mounted camera)
- ROS environment (e.g., Melodic or Noetic)
- Installed `ainex_sdk`, `ainex_kinematics`, `ainex_interfaces`
- OpenCV (`cv2`) in Python 3

---
