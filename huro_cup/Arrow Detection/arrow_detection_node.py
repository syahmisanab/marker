#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class ArrowShapeNode:
    def __init__(self):
        rospy.init_node("arrow_shape_node", anonymous=True)
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        # Parameters
        self.enable_display = rospy.get_param("~enable_display", True)
        self.image_topic = rospy.get_param("~camera_topic", "/camera/image_raw")

        # Publishers
        self.arrow_pub = rospy.Publisher("/arrow/shape_direction", String, queue_size=1)
        self.image_pub = rospy.Publisher("/arrow/shape_result", Image, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("Arrow shape detector node initialized.")

    def getContours(self, inImg, outImg):
        direction = "none"
        contours, _ = cv2.findContours(inImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000 or area > 50000:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True) # change tolerance # lower the value, stricter shape  
            num_points = len(approx)

            if num_points not in [7, 9]:
                continue

            if cv2.isContourConvex(approx):
                continue

            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if aspect_ratio < 0.5 or aspect_ratio > 2.5:
                continue

            if num_points == 9:
                # Likely left or right
                x_vals = approx[:, 0, 0]
                arrow_center = (max(x_vals) + min(x_vals)) / 2
                if np.median(x_vals) < arrow_center:
                    direction = "left"
                else:
                    direction = "right"

            elif num_points == 7:
                # Likely forward
                direction = "forward"

            # Draw and annotate
            cv2.drawContours(outImg, [approx], -1, (0, 255, 0), 2)
            cv2.rectangle(outImg, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(outImg, direction, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    
            break  # Stop after first confident detection

        return direction, outImg

    def image_callback(self, ros_image):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", str(e))
            return

        imgOut = img.copy()
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (3, 3), 0)

        imgCanny = cv2.Canny(imgBlur, 30, 100)
        imgDilated = cv2.dilate(imgCanny, self.kernel, iterations=2)
        imgEroded = cv2.erode(imgDilated, self.kernel, iterations=1)

        direction, result = self.getContours(imgEroded, imgOut)

        # Publish arrow direction
        self.arrow_pub.publish(direction)

        # Publish image
        ros_result = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.image_pub.publish(ros_result)

        # Display
        if self.enable_display:
            cv2.imshow("Arrow Shape Detection", result)
            if cv2.waitKey(1) == 27:  # ESC to exit
                rospy.signal_shutdown("User closed display")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ArrowShapeNode().run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
