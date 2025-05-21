
#!/usr/bin/env python3
# encoding: utf-8

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from ainex_kinematics.gait_manager import GaitManager
import time

class ArrowTurnNode:
    def __init__(self):
        rospy.init_node("arrow_turn_node")
        rospy.loginfo("ArrowTurnNode initialized.")

        self.gait_manager = GaitManager()

        self.turn_speed = 10
        self.turn_duration = 2.6  # for 90 degrees
        self.has_turned = False

        # Detection hold
        self.current_direction = "none"
        self.direction_start_time = None
        self.detection_hold_time = 0.2  # seconds of consistent detection

        # Reset service
        rospy.Service("/arrow_turn/reset", Empty, self.reset_callback)  # << You can remove this line later
        self.reset_wait_time = 3.0  # << To change delay, adjust this value

        rospy.Subscriber("/arrow/shape_direction", String, self.arrow_callback)

    def reset_callback(self, req):
        rospy.loginfo("Resetting arrow turn logic...")
        self.has_turned = False
        self.current_direction = "none"
        self.direction_start_time = None
        rospy.sleep(self.reset_wait_time)  # << Wait 3 seconds before listening again
        return EmptyResponse()

    def arrow_callback(self, msg):
        direction = msg.data.lower().strip()

        if self.has_turned or direction not in ["left", "right"]:
            self.current_direction = "none"
            self.direction_start_time = None
            return

        # If it's a new direction
        if direction != self.current_direction:
            self.current_direction = direction
            self.direction_start_time = time.time()
            return

        # Same direction is being held
        held_duration = time.time() - self.direction_start_time
        if held_duration < self.detection_hold_time:
            return  # not stable long enough

        # Detected consistently ? now turn
        if direction == "left":
            rospy.loginfo("Turning LEFT")
            self.gait_manager.move(1, 0, 0, self.turn_speed)
            rospy.sleep(self.turn_duration)
            self.gait_manager.stop()

        elif direction == "right":
            rospy.loginfo("Turning RIGHT")
            self.gait_manager.move(1, 0, 0, -self.turn_speed)
            rospy.sleep(self.turn_duration)
            self.gait_manager.stop()

        self.has_turned = True
        rospy.loginfo("Turn completed. Waiting for reset.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        ArrowTurnNode().run()
    except rospy.ROSInterruptException:
        pass
