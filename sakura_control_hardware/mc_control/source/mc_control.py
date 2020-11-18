#!/usr/bin/env python

#Default Sakura
#Author : RTU
#Version : 1.0

import sys
import rospy
import serial
import math
import cv2
import numpy as np

#native msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

rospy.init_node('mc_control')

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.001)

class McControl:

    apb = 0.25
    radius = 0.025
    rate = rospy.Rate(20);
    vx = 0.0
    vy = 0.0
    va = 0.0

    def __init__(self):
        self.vel = rospy.Subscriber('/sakura/cmd_vel', Twist, callback=self.callback, queue_size=1)
	
    def callback(self, cmd):

        self.vx = cmd.linear.x*15
        self.vy = cmd.linear.y*15
        self.va = cmd.angular.z*0.8

	print(self.vx, self.vy, self.va)

    def loop(self):
        
        try:
            while not rospy.is_shutdown():

                ser.write('x{0:.3f}y{1:.3f}z{2:.3f}'.format(self.vx, self.vy, self.va))
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':

    control = McControl()

    control.loop()
