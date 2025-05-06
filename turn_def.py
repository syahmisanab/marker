def turn_180(self, direction="right"):
    rospy.sleep(1.0)
    sign = 1 if direction == "right" else -1

    self.gait.move(1, 0, 0, 10 * sign)

    total_angle = 0.0
    rate = rospy.Rate(100)
    last_yaw = self.yaw
    last_time = rospy.Time.now().to_sec()

    rospy.loginfo("Starting 180° turn using IMU...")

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        dt = now - last_time
        last_time = now

        current_yaw = self.yaw
        delta_yaw = self.normalize_angle(current_yaw - last_yaw)
        last_yaw = current_yaw

        # Track only yaw in the correct direction
        if direction == "right" and delta_yaw > 0:
            total_angle += delta_yaw
        elif direction == "left" and delta_yaw < 0:
            total_angle -= delta_yaw

        rospy.loginfo_throttle(0.5, f"Total yaw: {total_angle:.2f} rad")

        if total_angle >= 3.14:
            break

        rate.sleep()

    self.gait.stop()
    rospy.loginfo("IMU-based 180° turn complete.")
