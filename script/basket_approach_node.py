#!/usr/bin/env python3
# encoding: utf-8

import rospy
from ainex_interfaces.msg import ObjectsInfo, ObjectInfo
from ainex_kinematics.gait_manager import GaitManager

class BasketNavigatorNode:
    def __init__(self):
        rospy.init_node('basket_navigator_node')
        self.gait_manager = GaitManager()
        self.target_object = None

        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.detection_callback)
        rospy.on_shutdown(self.shutdown)

        self.image_width = 640
        self.center_tolerance = 30
        self.close_area_threshold = 8000

        rospy.loginfo("? BasketNavigatorNode started.")

    def shutdown(self):
        rospy.loginfo("?? Shutting down.")
        self.gait_manager.stop()

    def detection_callback(self, msg):
        for obj in msg.data:
            if obj.label in ["red", "pink"] and obj.type == "rect":
                self.target_object = obj
                return
        self.target_object = None

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.target_object:
                x = self.target_object.x
                y = self.target_object.y
                w = self.target_object.width
                h = self.target_object.height
                area = w * h
                image_center = self.image_width // 2

                rospy.loginfo(f"?? Basket: x={x}, w={w}, area={area}")

                if w >= 600 and h >= 400:
                    rospy.logwarn_throttle(3, "? Full-frame detection, ignoring.")
                    self.gait_manager.stop()
                    rate.sleep()
                    continue

                if area >= self.close_area_threshold:
                    rospy.loginfo("? Reached basket. Stopping.")
                    self.gait_manager.stop()
                elif x < image_center - self.center_tolerance:
                    rospy.loginfo("? Turning left")
                    self.gait_manager.move(2, 0, 0, -3)
                elif x > image_center + self.center_tolerance:
                    rospy.loginfo("? Turning right")
                    self.gait_manager.move(2, 0, 0, 3)
                else:
                    rospy.loginfo("? Moving forward")
                    self.gait_manager.move(2, 0.01, 0, 0)
            else:
                rospy.loginfo_throttle(5, "…No target basket detected.")
                self.gait_manager.stop()

            rate.sleep()

if __name__ == "__main__":
    try:
        BasketNavigatorNode().run()
    except rospy.ROSInterruptException:
        pass
