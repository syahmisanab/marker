#!/usr/bin/env python3
# encoding: utf-8

import time
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class VisualPatrolReactiveNode(Common):
    line_roi = [
        (5 / 12, 6 / 12, 1 / 4, 3 / 4),
        (6 / 12, 7 / 12, 1 / 4, 3 / 4),
        (7 / 12, 8 / 12, 1 / 4, 3 / 4)
    ]
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.objects_info = []
        self.head_pan_init = 500
        self.head_tilt_init = 260
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.visual_patrol = VisualPatrol(self.gait_manager)
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.Subscriber("/arrow/shape_direction", String, self.arrow_callback)
        self.arrow_direction = "none"
        self.arrow_last_handled_time = 0
        self.arrow_cooldown = 3.0  # seconds
        self.handled_arrow = False

        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)

        self.motion_manager.run_action('walk_ready')

        if rospy.get_param('~start', True):
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))
            self.start_srv_callback(None)
            common.loginfo('start track %s lane' % target_color)

    def arrow_callback(self, msg):
        self.arrow_direction = msg.data.lower().strip()

    def shutdown(self, signum, frame):
        with self.lock:
            self.motion_manager.run_action('stand')
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    def set_color_srv_callback(self, msg):
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'line'
        param.image_process_size = self.image_process_size

        param.line_roi.up.y_min = int(self.line_roi[0][0] * self.image_process_size[1])
        param.line_roi.up.y_max = int(self.line_roi[0][1] * self.image_process_size[1])
        param.line_roi.up.x_min = int(self.line_roi[0][2] * self.image_process_size[0])
        param.line_roi.up.x_max = int(self.line_roi[0][3] * self.image_process_size[0])

        param.line_roi.center.y_min = int(self.line_roi[1][0] * self.image_process_size[1])
        param.line_roi.center.y_max = int(self.line_roi[1][1] * self.image_process_size[1])
        param.line_roi.center.x_min = int(self.line_roi[1][2] * self.image_process_size[0])
        param.line_roi.center.x_max = int(self.line_roi[1][3] * self.image_process_size[0])

        param.line_roi.down.y_min = int(self.line_roi[2][0] * self.image_process_size[1])
        param.line_roi.down.y_max = int(self.line_roi[2][1] * self.image_process_size[1])
        param.line_roi.down.x_min = int(self.line_roi[2][2] * self.image_process_size[0])
        param.line_roi.down.x_max = int(self.line_roi[2][3] * self.image_process_size[0])

        param.min_area = 1
        param.max_area = self.image_process_size[0] * self.image_process_size[1]
        self.detect_pub.publish([param])
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']

    def get_color_callback(self, msg):
        self.objects_info = msg.data

    def run(self):
        rate = rospy.Rate(30)

        while self.running:
            line_data = None
            for object_info in self.objects_info:
                if object_info.type == 'line':
                    line_data = object_info

            # Real-time line following (always active)
            if line_data is not None:
                self.visual_patrol.process(line_data.x, line_data.width)

            # Real-time marker detection (with cooldown)
            now = time.time()
            if self.arrow_direction in ["left", "right", "forward"] and not self.handled_arrow:
                rospy.loginfo(f"Arrow marker detected: {self.arrow_direction}")

                if self.arrow_direction == "left":
                    self.gait_manager.move(1, 0, 0, -10)
                    time.sleep(5.3)
                    self.gait_manager.stop()
                elif self.arrow_direction == "right":
                    self.gait_manager.move(1, 0, 0, 10)
                    time.sleep(5.3)
                    self.gait_manager.stop()

                # Walk forward after any arrow
                self.gait_manager.move(1, 0.02, 0, 0)
                time.sleep(2.5)
                self.gait_manager.stop()

                # Reset detection
                self.handled_arrow = True
                self.arrow_last_handled_time = now
                self.arrow_direction = "none"

            # Cooldown reset
            if self.handled_arrow and now - self.arrow_last_handled_time > self.arrow_cooldown:
                self.handled_arrow = False

            rate.sleep()

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrolReactiveNode('visual_patrol_reactive').run()
