#!/usr/bin/env python3
# encoding: utf-8

import time
import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

class MotionSequenceNode:
    def __init__(self):
        rospy.init_node('motion_sequence_node')
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager('/home/ubuntu/software/ainex_controller/ActionGroups')

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("✅ MotionSequenceNode started.")
        time.sleep(0.2)

    def shutdown(self):
        rospy.loginfo("🛑 Stopping gait.")
        self.gait_manager.stop()

    def step_forward(self, duration=2.0, speed=0.01):
        rospy.loginfo(f"⬆ Walking forward for {duration} sec")
        self.gait_manager.move(2, speed, 0, 0)
        time.sleep(duration)
        self.gait_manager.stop()

    def run_action(self, action_name=""):
        if action_name:
            rospy.loginfo(f"🎬 Running action: {action_name}")
            self.motion_manager.run_action(action_name)
        else:
            rospy.loginfo("🎬 Skipping action (none defined)")

    def run(self):
        # Step 1
        self.step_forward(duration=2.5)

        # Action 1
        self.run_action("")  # TODO: Replace with real action name later

        # Step 2
        self.step_forward(duration=2.0)

        # Action 2
        self.run_action("")  # TODO: Replace with real action name later

        # Step 3
        self.step_forward(duration=1.5)

        rospy.loginfo("✅ Motion sequence completed.")

if __name__ == "__main__":
    try:
        MotionSequenceNode().run()
    except rospy.ROSInterruptException:
        pass
