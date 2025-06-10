#!/usr/bin/env python3
# encoding: utf-8
# Date: 2025-06-11  Basket approach based on rectangle tracking

import rospy
import time
from ainex_interfaces.msg import ObjectsInfo
from ainex_kinematics.gait_manager import GaitManager

class BasketApproachNode:
    def __init__(self):
        rospy.init_node('basket_approach_node')
        self.gait_manager = GaitManager()
        self.object_info = None
        self.running = True

        # Subscribe to the basket tracking output
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.object_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("BasketApproachNode started.")

    def object_callback(self, msg):
        if msg.data:
            self.object_info = msg.data[0]
            rospy.logdebug(f"Detected object_info: {self.object_info}")

    def shutdown(self):
        rospy.loginfo("BasketApproachNode shutting down.")
        self.gait_manager.stop()
        self.running = False

    def run(self):
        # Parameters (tune these as needed)
        image_width = 160
        center_tolerance = 20          # pixels off-center allowed
        close_area_threshold = 3000    # area at which we consider "close enough"
        rate = rospy.Rate(10)          # 10 Hz loop

        while not rospy.is_shutdown() and self.running:
            if self.object_info:
                # Extract center and size
                center_x = self.object_info[2][0]
                width   = self.object_info[3][0]
                height  = self.object_info[3][1]
                area    = width * height

                image_center_x = image_width / 2

                # Check if close enough
                if area > close_area_threshold:
                    rospy.loginfo("? Reached basket (area {:.0f}). Stopping.".format(area))
                    self.gait_manager.stop()

                # Need to turn left
                elif center_x < image_center_x - center_tolerance:
                    rospy.loginfo("? Turning left. center_x={:.0f}".format(center_x))
                    # mode=2: walking, forward=0, sidestep=0, rotate negative = left
                    self.gait_manager.move(2, 0, 0, -3)

                # Need to turn right
                elif center_x > image_center_x + center_tolerance:
                    rospy.loginfo("? Turning right. center_x={:.0f}".format(center_x))
                    # positive rotate = right
                    self.gait_manager.move(2, 0, 0, 3)

                # Aligned: move forward
                else:
                    rospy.loginfo("? Aligned. Moving forward.")
                    # small forward speed
                    self.gait_manager.move(2, 0.01, 0, 0)

            else:
                # No detection: stop and wait
                rospy.loginfo_throttle(5, "…No basket detected; standing by.")
                self.gait_manager.stop()

            rate.sleep()

if __name__ == '__main__':
    try:
        BasketApproachNode().run()
    except rospy.ROSInterruptException:
        pass
