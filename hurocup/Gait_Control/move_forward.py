#!/usr/bin/env python3
# encoding: utf-8
# Date: 2023/07/10

import time
import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

# Initialize ROS node
rospy.init_node('gait_control_demo')

# Initialize motion manager (for posture and static actions)
motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')

# Initialize gait manager (for dynamic walking)
gait_manager = GaitManager()
time.sleep(0.2)  # Small delay to ensure stability

# Optional: Retrieve and modify walking parameters
gait_param = gait_manager.get_gait_param()
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.008
gait_param['body_height'] = 0.025

# Optional DSP config for step-level control (not used directly here)
dsp = [400, 0.2, 0.02]

# Walk forward at a constant speed
gait_manager.move(1, 0.02, 0, 0)  # move(forward_speed, x, y, yaw)
time.sleep(17)  # Move forward for 17 seconds

# Stop walking
gait_manager.stop()
