#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/03
# @author:aiden
# (visual line following node)
import time
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class VisualPatrolNode(Common):
    # (check three ROIs in order, and immediately exit if a black line is detected)
    # (y_min, y_max, x_min, and x_max represent respectively the proportion of the image occupied. The actual size is y_min*height)
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]
    # (When process the image, scale it to this resolution)
    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.objects_info = []
        self.head_pan_init = 500   # (initial value of the left-right servo)
        self.head_tilt_init = 260  # initial value of the up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        # (create line following control instance)
        self.visual_patrol = VisualPatrol(self.gait_manager)
        # (set exit processing function)
        signal.signal(signal.SIGINT, self.shutdown)

        # (subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        # (initialize color setting service)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  
        self.motion_manager.run_action('walk_ready')     # (play ready posture action)
        # (If the autonomous start parameter is ture, enable node)
        if rospy.get_param('~start', True):
            # (Notify the color recognition to prepare, at this time only display the camera original image)
            target_color = rospy.get_param('~color', 'black')  # (set to recognize black)
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  
            self.start_srv_callback(None)  # (enable color recognition)
            common.loginfo('start track %s lane' % target_color)
    # (node closing callback function)
    def shutdown(self, signum, frame):
        with self.lock:
            self.motion_manager.run_action('stand')
            self.running = False 
            common.loginfo('%s shutdown' % self.name)
            
    # (set color service callback function)
    def set_color_srv_callback(self, msg):
        # (generate color recognition parameter)
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'line'  # (specify the detection type to line)
        param.image_process_size = self.image_process_size
        # (set ROI parameter)
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
        # (area filtering parameter)
        param.min_area = 1
        param.max_area = self.image_process_size[0] * self.image_process_size[1]
        # (publish color recognition parameter)
        self.detect_pub.publish([param])
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']
    
    # (publish color recognition parameter)
    def get_color_callback(self, msg):
        # (obtain color recognition result)
        self.objects_info = msg.data

    # (main loop function)
    def run(self):
        while self.running:
            if self.start:
                # (obtain recognition result)
                line_data = None
                # stairs_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                # (If there is recognition result, control line following)
                if line_data is not None:
                    self.visual_patrol.process(line_data.x, line_data.width)
                time.sleep(0.01)
            else:
                time.sleep(0.01)
        # (action before exiting)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrolNode('visual_patrol').run()
