#!/usr/bin/env python

import sys
import rospy
import numpy as np
import cv2

import Adafruit_PCA9685

from std_msgs.msg import Float64

pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)

rtd = 180/np.pi

class Servo:
    
    def __init__(self, id_, topic, min_, max_, a_min, a_max, scale_=rtd):
        self.id = id_
        self.max = max_
        self.min = min_
        self.a_min = a_min
        self.a_max = a_max
        self.scale = scale_
        self.sub = rospy.Subscriber(topic, Float64, self.set, queue_size=1)

    def set(self, msg):
        deg = msg.data*self.scale
        val = np.clip((deg-self.a_min)/(self.a_max-self.a_min), 0.0, 1.0) * (self.max-self.min) + self.min
        v2 = int(val)
        pwm.set_pwm(self.id, 0, v2)

    def setClippped(self, msg, a, b):
        deg = msg.data*rtd
        val = np.clip((deg-self.a_min)/(self.a_max-self.a_min), 0.0, 1.0) * (self.max-self.min) + self.min
        v2 = int(val)
        pwm.set_pwm(self.id, 0, v2)

    def setInterpolated(self, msg):
        deg = msg.data*rtd
        val = (deg-self.a_min)/(self.a_max-self.a_min) * (self.max-self.min) + self.min
        v2 = int(val)
        pwm.set_pwm(self.id, 0, v2)


#for test one servo
if __name__ == '__main__':

    id = [int(i) for i in sys.argv[1:]]
    print(id)

    rospy.init_node('servo_tester')
    value = 355

    def callback(v):
        print(v) 
	
	for i in id:
        	pwm.set_pwm(i, 0, v)

        cv2.imshow('Control', np.zeros((1,1)))
        cv2.waitKey(1)

    cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('servo', 'Control', value, 750, callback)

    while not rospy.is_shutdown():
        cv2.imshow('Control', np.zeros((1,1)))
        cv2.waitKey(1)
        rospy.Rate(100).sleep()
