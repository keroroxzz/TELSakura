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
    v_fr = 0.0
    v_fl = 0.0
    v_br = 0.0
    v_bl = 0.0

    def __init__(self):
        self.vel = rospy.Subscriber('/sakura/cmd_vel', Twist, callback=self.callback, queue_size=1)
	
    def callback(self, cmd):

        vx = -cmd.linear.x*15
        vy = cmd.linear.y*15
        va = cmd.angular.z*15

        #kinemetics
        self.v_fr = int((vx + vy + va*self.apb)/self.radius)
        self.v_fl = int((vx - vy - va*self.apb)/self.radius)
        self.v_br = int((vx - vy + va*self.apb)/self.radius)
        self.v_bl = int((vx + vy - va*self.apb)/self.radius)
	print(self.v_fr, self.v_fl, self.v_br, self.v_bl)

    def loop(self):
        
        try:
            while not rospy.is_shutdown():

                ser.write('a{0}b{1}c{2}d{3}'.format(-self.v_fr, self.v_fl, -self.v_br, self.v_bl))
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':

    control = McControl()

    control.loop()
