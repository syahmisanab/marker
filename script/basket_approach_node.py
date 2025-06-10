#!/usr/bin/env python3
# encoding: utf-8

import rospy
import time
from ainex_interfaces.msg import ObjectInfo
from ainex_kinematics.gait_manager import GaitManager

class BasketApproachNode:
    def __init__(self):
        rospy.init_node('basket_approach_node')
        self.gait_manager = GaitManager()
        self.object_info = None
        self.running = True

        rospy.Subscriber('/basket/target', ObjectInfo, self.object_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("BasketApproachNode started.")

    def object_callback(self, msg):
        self.object_info = msg

    def shutdown(self):
        rospy.loginfo("BasketApproachNode shutting down.")
        self.gait_manager.stop()
        self.running = False

    def run(self):
        image_width = 640  # Full camera resolution
        center_tolerance = 30
        close_area_threshold = 8000  # You can tune this
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.running:
            if self.object_info and self.object_info.width > 0 and self.object_info.height > 0:
                center_x = self.object_info.x
                width = self.object_info.width
                height = self.object_info.height
                area = width * height

                rospy.loginfo(f"x: {center_x}, y: {self.object_info.y}, w: {width}, h: {height}, area: {area}")

                image_center_x = image_width / 2

                if width >= 600 and height >= 400:
                    rospy.logwarn_throttle(3, "⚠ Ignoring full-frame detection; likely invalid.")
                    self.gait_manager.stop()
                    rate.sleep()
                    continue

                if area >= close_area_threshold:
                    rospy.loginfo(f"✅ Reached basket (area: {area}). Stopping.")
                    self.gait_manager.stop()

                elif center_x < image_center_x - center_tolerance:
                    rospy.loginfo(f"← Turning left. center_x={center_x}")
                    self.gait_manager.move(2, 0, 0, -3)

                elif center_x > image_center_x + center_tolerance:
                    rospy.loginfo(f"→ Turning right. center_x={center_x}")
                    self.gait_manager.move(2, 0, 0, 3)

                else:
                    rospy.loginfo("↑ Aligned. Moving forward.")
                    self.gait_manager.move(2, 0.01, 0, 0)
            else:
                rospy.loginfo_throttle(5, "…No valid basket detected yet. Standing by.")
                self.gait_manager.stop()

            rate.sleep()

if __name__ == '__main__':
    try:
        BasketApproachNode().run()
    except rospy.ROSInterruptException:
        pass
