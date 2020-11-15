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

#native msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

rospy.init_node('arm_control')

class ArmControl:

    def __init__(self):
        self.base_s = Servo(0, '/sakura/arm/base/command', 172, 566,-90,90) 
        self.j1_s = Servo(1, '/sakura/arm/j1/command', 160, 550,90,-90) 
        self.j2_s = Servo(2, '/sakura/arm/j2/command', 150, 540,-90,90) 
        self.j3_s = Servo(3, '/sakura/arm/j3/command', 155, 555,-90,90) 
        self.j4_s = Servo(4, '/sakura/arm/j4/command', 140, 525,90,-90) 
        self.gripper_s = Servo(5, '/sakura/arm/f1/command', 185, 376, 0.0, 1.0)


if __name__ == '__main__':

    control = ArmControl()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
