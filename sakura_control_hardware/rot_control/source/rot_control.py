#!/usr/bin/env python

#Default Sakura
#Author : RTU
#Version : 1.0

import sys
import rospy
import math
import cv2
import numpy as np

from supply.servo import Servo

rospy.init_node('rot_control')

class RotControl:

    def __init__(self):
        self.rot = Servo(6, '/sakura/rot/command', 560, 130, 5, 210)
        self.camy = Servo(7, '/sakura/cam_y/command', 100, 620, 90, -90)


if __name__ == '__main__':

    control = RotControl()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
