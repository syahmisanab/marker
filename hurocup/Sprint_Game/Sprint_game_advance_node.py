#!/usr/bin/env python3
# encoding: utf-8

import time
import rospy
from std_msgs.msg import String
from ainex_interfaces.msg import ObjectsInfo
from ainex_kinematics.gait_manager import GaitManager
from ainex_example.color_common import Common

class SprintLineTrigger(Common):
    def __init__(self):
        rospy.init_node('sprint_line_trigger_node')
        self.running = True

        # Initialize gait manager
        self.gait_manager = GaitManager()
        time.sleep(0.2)

        # Head servo values
        self.head_pan_init = 500
        self.head_tilt_down = 260  # Looking down
        self.head_tilt_up = 500    # Looking forward/up

        # State
        self.line_detected = False

        # Init base class for head control
        super().__init__('sprint_line_trigger', self.head_pan_init, self.head_tilt_down)

        # Subscribe to color detection topic
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.line_callback)

    def line_callback(self, msg):
        if not self.line_detected:
            for obj in msg.data:
                if obj.type == 'line':
                    rospy.loginfo("White line detected!")
                    self.line_detected = True
                    break

    def run(self):
        rospy.loginfo("Starting Sprint Task...")

        # Set head down
        self.init_action(self.head_pan_init, self.head_tilt_down)
        time.sleep(0.5)

        # Start walking forward
        self.gait_manager.move(1, 0.02, 0, 0)
        rospy.loginfo("Walking forward, looking for white line...")

        # Wait until white line is detected
        while not self.line_detected and not rospy.is_shutdown():
            time.sleep(0.01)

        if rospy.is_shutdown():
            return

        # Continue forward briefly to "pass" the line
        rospy.loginfo("Passing the line...")
        time.sleep(1.0)

        # Walk backward
        rospy.loginfo("Reversing...")
        self.gait_manager.move(1, -0.02, 0, 0)
        time.sleep(5.0)

        # Stop walking
        self.gait_manager.stop()

        # Head back up
        rospy.loginfo("Raising head back up...")
        self.init_action(self.head_pan_init, self.head_tilt_up)

        rospy.loginfo("Sprint task complete.")
        rospy.signal_shutdown("Finished")

if __name__ == "__main__":
    try:
        SprintLineTrigger().run()
    except rospy.ROSInterruptException:
        pass
