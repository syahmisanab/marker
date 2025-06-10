#!/usr/bin/env python3
# encoding: utf-8
# Robot walks toward the basket based on vision input

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

        # Subscribe to object detection
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.object_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("BasketApproachNode started.")

    def object_callback(self, msg):
        if msg.data:
            self.object_info = msg.data[0]

    def shutdown(self):
        rospy.loginfo("BasketApproachNode shutting down.")
        self.gait_manager.stop()
        self.running = False

    def run(self):
        image_width = 160
        center_tolerance = 20
        close_area_threshold = 3000
        rate = rospy.Rate(10)

        last_area = -1  # for detecting stale data

        while not rospy.is_shutdown() and self.running:
            if self.object_info is not None:
                # Extract data safely
                center_x = self.object_info.x
                center_y = self.object_info.y
                width    = self.object_info.width
                height   = self.object_info.height
                area     = width * height

                # Log what we're getting
                rospy.loginfo(f"x: {center_x}, y: {center_y}, w: {width}, h: {height}, area: {area}")

                image_center_x = image_width / 2

                # Check for stale/frozen detection
                if area == last_area:
                    rospy.logwarn_throttle(3, "⚠ Basket detection appears frozen — area hasn't changed.")
                last_area = area

                # Behavior logic
                if area > close_area_threshold:
                    rospy.loginfo("✅ Reached basket (area: {}). Stopping.".format(area))
                    self.gait_manager.stop()

                elif center_x < image_center_x - center_tolerance:
                    rospy.loginfo("← Turning left. center_x={}".format(center_x))
                    self.gait_manager.move(2, 0, 0, -3)

                elif center_x > image_center_x + center_tolerance:
                    rospy.loginfo("→ Turning right. center_x={}".format(center_x))
                    self.gait_manager.move(2, 0, 0, 3)

                else:
                    rospy.loginfo("↑ Aligned. Moving forward.")
                    self.gait_manager.move(2, 0.01, 0, 0)

            else:
                rospy.loginfo_throttle(5, "…No basket detected; standing by.")
                self.gait_manager.stop()

            rate.sleep()

if __name__ == '__main__':
    try:
        BasketApproachNode().run()
    except rospy.ROSInterruptException:
        pass
