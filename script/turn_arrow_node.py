#!/usr/bin/env python3
# encoding: utf-8

import rospy
from std_msgs.msg import String
from ainex_kinematics.gait_manager import GaitManager

class ArrowTurnNode:
    def __init__(self):
        rospy.init_node("arrow_turn_node")
        rospy.loginfo("ArrowTurnNode initialized. Waiting for direction...")

        # Initialize GaitManager
        self.gait_manager = GaitManager()

        # Turn config (tuned for ~90°)
        self.turn_speed = 10      # yaw speed
        self.turn_duration = 2.6  # seconds for ~90°

        self.has_turned = False   # prevent multiple triggers

        rospy.Subscriber("/arrow/shape_direction", String, self.arrow_callback)

    def arrow_callback(self, msg):
        if self.has_turned:
            return

        direction = msg.data.lower().strip()
        rospy.loginfo(f"Arrow detected: {direction}")

        if direction == "left":
            self.gait_manager.move(1, 0, 0, -self.turn_speed)
            rospy.sleep(self.turn_duration)
            self.gait_manager.stop()
            rospy.loginfo("Turned LEFT 90 degrees.")
            self.has_turned = True

        elif direction == "right":
            self.gait_manager.move(1, 0, 0, self.turn_speed)
            rospy.sleep(self.turn_duration)
            self.gait_manager.stop()
            rospy.loginfo("Turned RIGHT 90 degrees.")
            self.has_turned = True

        else:
            rospy.loginfo("Arrow is not left or right. Ignoring.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        ArrowTurnNode().run()
    except rospy.ROSInterruptException:
        pass
