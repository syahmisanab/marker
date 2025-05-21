
#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ainex_example.color_common import Common
from ainex_kinematics.gait_manager import GaitManager


class WhiteLineWalker(Common):
    def __init__(self, show_gui=True):
        self.head_pan_init = 500
        self.head_tilt_down = 260
        self.head_tilt_up = 500
        self.show_gui = show_gui

        rospy.init_node('white_line_with_motion')
        super().__init__('white_line_with_motion', self.head_pan_init, self.head_tilt_down)
        rospy.loginfo("white_line_with_motion init_finish")

        self.bridge = CvBridge()
        self.frame = None
        self.detecting = False
        self.detection_start_time = None
        self.required_duration = 0.5
        self.line_confirmed = False
        self.motion_started = False
        self.gait_manager = GaitManager()

        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Tilt head down
        rospy.sleep(0.5)
        self.init_action(self.head_pan_init, self.head_tilt_down)
        rospy.loginfo("早・・Head tilted down")

        self.start_time = time.time()

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
            if not self.motion_started:
                self.gait_manager.move(1, 0.02, 0, 0)  # Walk forward
                rospy.loginfo("垳 Walking forward...")
                self.motion_started = True

            if time.time() - self.start_time > 10:
                rospy.loginfo("竢ｱ・・Timeout 窶・stopping after 10 seconds.")
                break

            if self.frame is None:
                rate.sleep()
                continue

            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 130, 255, cv2.THRESH_BINARY)


            if height is None:
                height = thresh.shape[0]

            roi_start = int(height * 0.5)
            roi_end = int(height * 0.9)
            roi = thresh[roi_start:roi_end, :]

            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            image_width = roi.shape[1]

            valid_line_found = False

            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                #rospy.loginfo(f"Contour w={w}, h={h}, ratio={aspect_ratio:.2f}")
                if w > image_width * 0.6 and aspect_ratio > 2.0:
                    rospy.loginfo("笨・Valid line contour matched!")
                    valid_line_found = True
                    break

            if self.show_gui:
                cv2.imshow("White Line ROI", roi)
                cv2.waitKey(1)

            if valid_line_found:
                if not self.detecting:
                    self.detection_start_time = time.time()
                    self.detecting = True
                    rospy.loginfo("葡 Line shape detected 窶・starting timer...")
                elif time.time() - self.detection_start_time >= self.required_duration:
                    rospy.loginfo("笨・Line confirmed after 0.5s!")
                    self.line_confirmed = True
                    break
            else:
                if self.detecting:
                    rospy.loginfo("笵・Shape lost 窶・resetting timer.")
                self.detecting = False
                self.detection_start_time = None

            rate.sleep()

        # Final wrap-up
        self.gait_manager.stop()
        rospy.loginfo("尅 Robot stopped walking")

        if self.line_confirmed:
            self.init_action(self.head_pan_init, self.head_tilt_up)
            rospy.loginfo("早・・Head tilted back up after detection")
            rospy.loginfo("?? Moving forward for 1 second...")
            self.gait_manager.move(1, 0.02, 0, 0)
            rospy.sleep(1.5)
            self.gait_manager.stop()
            rospy.sleep(1)
            rospy.loginfo("?? Moving backward for 3 second...")
            self.gait_manager.move(1, -0.02, 0, 0)
            rospy.sleep(3)
            self.gait_manager.stop()
            rospy.loginfo("stopped")

        else:
            rospy.loginfo("笶・No valid line detected 窶・head stays down")

        rospy.signal_shutdown("笨・Motion and detection complete")


if __name__ == '__main__':
    import sys
    show_gui = True
    if "--show_gui=false" in sys.argv or "--show_gui False" in sys.argv:
        show_gui = False

    try:
        WhiteLineWalker(show_gui=show_gui)
    except rospy.ROSInterruptException:
        pass
