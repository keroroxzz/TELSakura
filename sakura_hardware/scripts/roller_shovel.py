#!/usr/bin/env python

#Default Sakura
#Author : RTU
#Version : 1.0

import sys
import rospy
import math
import cv2
import numpy as np

from std_msgs.msg import Float64
from supply.servo import Servo

rospy.init_node('roller_shovel_control')

class RotControl:

    def __init__(self):
        self.shovel = Servo(8, '/sakura/shovel/command', 350, 390, 5, 210)
        self.r_arm = Servo(9, '/sakura/roller/arm/command', 370, 380, -1.0, 1.0, scale=1.0)
        self.r_slider = Servo(10, '/sakura/roller/slider/command', 370, 380, -1.0, 1.0, scale=1.0)
        self.r_roller = Servo(11, '/sakura/roller/roller', 270, 470, -1.0, 1.0, scale=1.0)


if __name__ == '__main__':

    control = RotControl()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
