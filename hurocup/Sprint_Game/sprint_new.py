#!/usr/bin/env python3
# encoding: utf-8

import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ainex_example.color_common import Common
from ainex_kinematics.gait_manager import GaitManager


class LineCrossTask(Common):
    def __init__(self):
        rospy.init_node('line_cross_task')
        self.gait_manager = GaitManager()
        self.bridge = CvBridge()
        self.frame = None
        self.line_confirmed = False
        self.detection_start_time = None
        self.required_duration = 0.5
        self.detecting = False

        self.head_pan_init = 500
        self.head_tilt_down = 260
        self.head_tilt_up = 500
        super().__init__('line_cross_task', self.head_pan_init, self.head_tilt_down)

        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        rospy.sleep(0.5)
        self.init_action(self.head_pan_init, self.head_tilt_down)
        rospy.loginfo("🔽 Head tilted down. Starting to walk forward.")

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("❌ CV Bridge error: %s", e)

    def detect_line(self):
        if self.frame is None:
            return False

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

        height = thresh.shape[0]
        roi = thresh[int(height * 0.55):int(height * 0.85), :]
        image_width = roi.shape[1]

        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            if w > image_width * 0.6 and aspect_ratio > 2.0:
                return True
        return False

    def run(self):
        rate = rospy.Rate(20)
        self.gait_manager.move(1, 0.01, 0, 0)  # Start walking forward

        rospy.loginfo("🏃 Walking forward, looking for white line...")

        while not rospy.is_shutdown():
            if self.detect_line():
                if not self.detecting:
                    self.detection_start_time = time.time()
                    self.detecting = True
                    rospy.loginfo("⏱ Line detected — starting confirmation timer...")
                elif time.time() - self.detection_start_time >= self.required_duration:
                    self.line_confirmed = True
                    rospy.loginfo("✅ White line confirmed.")
                    break
            else:
                if self.detecting:
                    rospy.loginfo("❌ Line lost — resetting timer.")
                self.detecting = False
                self.detection_start_time = None
            rate.sleep()

        if not self.line_confirmed:
            rospy.loginfo("⚠️ Line never confirmed. Shutting down.")
            self.gait_manager.stop()
            rospy.signal_shutdown("No line detected.")
            return

        # Continue forward 2 more seconds
        rospy.loginfo("➡️ Continuing forward for 2 seconds...")
        time.sleep(2.0)

        # Stop
        self.gait_manager.stop()
        rospy.loginfo("🛑 Stopping.")

        # Move backward
        rospy.loginfo("⬅️ Moving backward for 5 seconds...")
        self.gait_manager.move(1, -0.02, 0, 0)
        time.sleep(5.0)
        self.gait_manager.stop()

        # Tilt head up
        rospy.loginfo("🔼 Raising head back up.")
        self.init_action(self.head_pan_init, self.head_tilt_up)

        rospy.loginfo("🏁 Task complete.")
        rospy.signal_shutdown("Finished")


if __name__ == '__main__':
    try:
        LineCrossTask().run()
    except rospy.ROSInterruptException:
        pass
