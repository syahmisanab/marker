#!/usr/bin/env python3
# encoding: utf-8
# Date: 2023/07/10

import time
import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

# Initialize the ROS node
rospy.init_node('gait_control_demo')

# Initialize motion manager for predefined actions (poses)
motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')

# Initialize gait manager for dynamic walking
gait_manager = GaitManager()
time.sleep(0.2)  # Allow systems to stabilize

# Optional: Retrieve and modify gait parameters
gait_param = gait_manager.get_gait_param()
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.008
gait_param['body_height'] = 0.025

# DSP step settings (used in manual step-based gait, not used here)
dsp = [400, 0.2, 0.02]

# Begin walking forward
gait_manager.move(1, 0.02, 0, 0)  # Forward speed = 0.02
time.sleep(3)  # Move forward for 3 seconds


# Begin backward forward
gait_manager.move(1, -0.02, 0, 0)  # Forward speed = 0.02
time.sleep(3)  # Move forward for 3 seconds

# Stop walking
gait_manager.stop()

# === Additional Examples ===
# The following lines are examples and not used in this script:

# Example: set_step for customized walking with arm swing control
# gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, arm_swap=30, step_num=3)

# Example: disable gait manager before running static action
# gait_manager.disable()
# motion_manager.run_action('left_shot')

# Example: play a hand open action
# motion_manager.run_action('hand_open')
