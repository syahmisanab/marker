#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ainex_example.color_common import Common


class WhiteLineDetectorLive(Common):
    def __init__(self, show_gui=True):
        self.head_pan_init = 500
        self.head_tilt_down = 260
        self.head_tilt_up = 500
        self.show_gui = show_gui

        rospy.init_node('white_line_check_live')
        super().__init__('white_line_check_live', self.head_pan_init, self.head_tilt_down)
        rospy.loginfo("white_line_check_live init_finish")

        self.bridge = CvBridge()
        self.frame = None
        self.detecting = False
        self.detection_start_time = None
        self.required_duration = 0.5  # seconds
        self.line_confirmed = False

        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Tilt head down to begin
        rospy.sleep(0.5)
        self.init_action(self.head_pan_init, self.head_tilt_down)
        rospy.loginfo("👁️ Head tilted down")

        # Start real-time detection loop
        self.run()

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)

    def run(self):
        rate = rospy.Rate(30)
        height = None

        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            # Convert to thresholded grayscale
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

            if height is None:
                height = thresh.shape[0]

            # ROI: from 55% to 85% of height
            roi_start = int(height * 0.55)
            roi_end = int(height * 0.85)
            roi = thresh[roi_start:roi_end, :]

            # Check margin strips
            margin = 10
            left_strip = roi[:, :margin]
            right_strip = roi[:, -margin:]

            left_white = np.count_nonzero(left_strip == 255)
            right_white = np.count_nonzero(right_strip == 255)

            left_ok = left_white > 500
            right_ok = right_white > 500

            # Show ROI
            if self.show_gui:
                # Show ROI in green on original image
                debug_img = self.frame.copy()
                cv2.rectangle(debug_img, (0, roi_start), (debug_img.shape[1], roi_end), (0, 255, 0), 2)
                cv2.imshow("Camera + ROI", debug_img)
                cv2.imshow("White Line ROI", roi)
                cv2.waitKey(1)

            # Detection logic
            if left_ok and right_ok:
                if not self.detecting:
                    self.detection_start_time = time.time()
                    self.detecting = True
                    rospy.loginfo("🕒 Line detected — starting confirmation timer...")
                elif time.time() - self.detection_start_time >= self.required_duration:
                    rospy.loginfo("✅ Line confirmed after 0.5s!")
                    self.line_confirmed = True
                    break
            else:
                if self.detecting:
                    rospy.loginfo("⛔ Detection interrupted — resetting timer.")
                self.detecting = False
                self.detection_start_time = None

            rate.sleep()

        # Final head control
        if self.line_confirmed:
            self.init_action(self.head_pan_init, self.head_tilt_up)
            rospy.loginfo("👁️ Head tilted back up after detection")
        else:
            rospy.loginfo("❌ No line detected — head remains down")

        rospy.signal_shutdown("Detection complete")


if __name__ == '__main__':
    import sys
    show_gui = True
    if "--show_gui=false" in sys.argv or "--show_gui False" in sys.argv:
        show_gui = False

    try:
        WhiteLineDetectorLive(show_gui=show_gui)
    except rospy.ROSInterruptException:
        pass
