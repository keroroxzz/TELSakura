#!/usr/bin/env python

#Obstical node for Instruction of the 2020 Autorace
#Author : RTU
#Version : 1.3

import rospy
import math
import cv2
import numpy as np

#native msgs
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float64

from sakura_common.tool import Q2E_z,clamp,smoothStep,fuzzy
from sakura_common.frame import Frame
from sakura_common.PID import PID, SPPID
from sakura_common.moniter import cvShow, Moniter
from sakura_common.xh_arm import robot_arm
from sakura_common.camera_rot import camrot

from sakura_common.tool import inRangeHSV, clamp
from cv_bridge import CvBridge

bridge = CvBridge()

red = (np.array([165,100,100]),np.array([20,256,256]))
green = (np.array([30,50,50]),np.array([75,256,256]))
blue = (np.array([95,60,50]),np.array([125,256,256]))

targets = [red, green]

def cv2LineFloat(img, x1, y1, x2, y2, color, thickness):
	cv2.line(img, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, thickness)

###################################
# Classes
###################################

class Ballon():
	path_length=0.0

	currentAng = 0.0
	desireAng = 0.0
	fixAngle = None
	back_diff_fix = 0.0
	back_diff = 0.0

	index = -1
	dir = [-1.0,1.0,-1.0, 1.0]
	dist = 0.0

	safeDirection=0.0

	ballon_x = 180.0

	ballon_state = 0

	pid_x = SPPID(0.6,0.0,0.03, 0.5)
	pid_y = SPPID(0.6,0.0,0.04, 0.5)
	pid_z = SPPID(1.0,0.0,0.05, 0.5)

	dx = 160
	dy = 150
	dr = 50

	def __init__(self):


		self.image_sub = rospy.Subscriber('/sakura/camera_image/image', Image, self.imageListener, queue_size = 1)
		self.status=1

	def status_callback(self, msg):
		self.status = msg.data

	def imageListener(self, image_msg):
		img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#generate the mask of roi
		mask = np.zeros(img.shape[:2], dtype="uint8")
		cv2.circle(mask, (self.dx, self.dy), self.dr, 1, -1)

		#get the roi
		masked = cv2.bitwise_and(hsv, hsv, mask=mask)

		for color in targets:

			hue_range = inRangeHSV(masked, color[0], color[1])
			_,cnt,_ = cv2.findContours(hue_range,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			
			if len(cnt) > 0:
				M=cv2.moments(cnt[0])
				print(M['m00'])

				cv2.namedWindow('themask', cv2.WINDOW_NORMAL)
				cv2.imshow('themask', masked)
				cv2.waitKey(1)

###################################
# MAIN
###################################

def main():
  rospy.init_node('tunnel')
  
  ballon = Ballon()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
	main()
