#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from ainex_kinematics.gait_manager import GaitManager

class IMUYawTurn:
    def __init__(self):
        rospy.init_node("imu_yaw_turn")
        self.gait = GaitManager()
        self.yaw = 0.0

        # Subscribe to IMU orientation
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.sleep(0.5)  # let IMU stabilize

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.yaw = yaw  # in radians

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        import math
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def turn_180(self, direction="right"):
        rospy.sleep(1.0)  # extra buffer

        start_yaw = self.yaw
        target_diff = 3.14  # 180° in radians
        sign = 1 if direction == "right" else -1

        rospy.loginfo(f"Starting 180° {direction} turn from yaw={start_yaw:.2f}")

        self.gait.move(1, 0, 0, 10 * sign)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            current_yaw = self.yaw
            diff = self.normalize_angle(current_yaw - start_yaw)

            rospy.loginfo_throttle(0.5, f"Turned: {abs(diff):.2f} / {target_diff:.2f} rad")

            if abs(diff) >= target_diff - 0.05:  # margin for noise
                break
            rate.sleep()

        self.gait.stop()
        rospy.loginfo("Turn complete.")

    def run(self):
        rospy.loginfo("Moving forward 17 sec...")
        self.gait.move(1, 0.02, 0, 0)
        time.sleep(17)
        self.gait.stop()

        self.turn_180(direction="right")

        rospy.loginfo("Moving forward 16 sec...")
        self.gait.move(1, 0.02, 0, 0)
        time.sleep(16)
        self.gait.stop()

if __name__ == "__main__":
    try:
        IMUYawTurn().run()
    except rospy.ROSInterruptException:
        pass
