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
from std_msgs.msg import Int16
from vision.msg import Lane_msg
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionFeedback

from lidar.service import obstacleService
from supply.tool import Q2E_z,clamp,smoothStep,fuzzy
from supply.frame import Frame
from supply.PID import PID
from supply.moniter import cvShow

from supply.tool import inRangeHSV
from cv_bridge import CvBridge

bridge = CvBridge()

pid=PID(1.0,0.0,0.02)

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

	ballon_x = 180.0

	ballon_state = 0

	def __init__(self):

		#publisher
		self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)

		self.odom_sub = rospy.Subscriber('/sakura/odom', Odometry, self.poseListener, queue_size = 1)
		self.scan_sub = rospy.Subscriber('/sakura/re_scan', LaserScan, self.lidarListener, queue_size = 5)
		self.image_sub = rospy.Subscriber('/sakura/camera_image/image', Image, self.imageListener, queue_size = 1)

	def imageListener(self, img):
		img = bridge.imgmsg_to_cv2(img)

		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		mask = inRangeHSV(hsv,np.array([50,50,50]),np.array([80,256,256]))
		
		circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, len(mask)/2, param1 = 100, param2 = 10, minRadius=40)

		if circles is not None and len(circles)>0 and self.ballon_state<2:
			circles = circles[0,:,:]
			cv2.circle(mask, (circles[0][0],circles[0][1]), circles[0][2], (255,0,0), 2)
			self.ballon_x = circles[0][0]
			self.ballon_state=1
		else:
			cv2.namedWindow('test', cv2.WINDOW_NORMAL)
			cv2.imshow('test', img)
			cv2.waitKey(1)
			return False

		cv2.namedWindow('test', cv2.WINDOW_NORMAL)
		cv2.imshow('test', mask)
		cv2.waitKey(1)

	def extract_wall(self, dir, lidar):

		dtr = math.pi/180.0

		point_set = []

		#extract right side
		i = int(dir)%360
		while True:
			dist_i = lidar[i]
			dist_j = lidar[(i+1)%360]

			if abs(dist_i-dist_j)<0.2:
				a=i*dtr
				point = (math.cos(a)*dist_i, math.sin(a)*dist_i)
				point_set.append(point)
			else:
				break
			
			i+=1
			if i>=360:
				break


		#extract left side
		i = int(dir)%360
		while True:
			dist_i = lidar[i]
			dist_j = lidar[(i+1)%360]

			if abs(dist_i-dist_j)<0.2:
				a=i*dtr
				point = (math.cos(a)*dist_i, math.sin(a)*dist_i)
				point_set.append(point)
			else:
				break
			
			i-=1
			if i<=-360:
				break
		if point_set is not None and len(point_set)>2:
		
			line = cv2.fitLine(np.array(point_set),cv2.DIST_L2,0,0.01,0.01)
			return lidar[int(dir)%360], -math.atan(line[1][0]/line[0][0])/7.5

		return lidar[int(dir)%360], 0.0


	def find_direction(self, lidar):

		left = [d for d in lidar[90:180] if d<0.7]
		right = [d for d in lidar[180:270] if d<0.7]

		self.back_diff = len(left)-len(right)


	cmd = Twist()
	angular_z = 0.0
	error = 5.0
	def lidarListener(self,msg):

		#used to help choose the right direction (old method)
		self.find_direction(msg.ranges)

		#Try to face to the wall vertiaclly when tracing the ballon
		if self.ballon_state==1:

			dir = -(self.ballon_x-180)/180.0*25.0

			d,a = self.extract_wall(dir, msg.ranges)
			b = (180 - self.ballon_x)

			self.cmd.linear.x = smoothStep(d-0.35,0.2,-0.2,0.2,-0.2)*0.5 + self.cmd.linear.x*0.5
			self.cmd.linear.y = b/600.0 * 0.25 + self.cmd.linear.y*0.6
			self.cmd.angular.z = (a*10.0 + smoothStep(b,180,-180,1.0,-1.0))*0.1 + self.cmd.angular.z*0.9

			self.angular_z = self.angular_z*0.95 + self.cmd.angular.z*0.05

			cmd = Twist()
			cmd.linear.x = self.cmd.linear.x
			cmd.linear.y = self.cmd.linear.y
			cmd.angular.z = abs(self.angular_z) * self.cmd.angular.z

			self.error = self.error*0.85 + (abs(d) + abs(a) + abs(b)/10.0)*0.15

			if self.error <1.75:
				self.ballon_state=2
				print('STOP!')

			self.vel_pub.publish(cmd)

	def poseListener(self,msg):

		self.currentAng=Q2E_z(msg.pose.pose.orientation)

		if self.fixAngle is None:
			self.fixAngle = self.currentAng

		self.obsMain_dc()

	def obsMain_dc(self):

		if self.fixAngle is None or self.ballon_state==1:
			return 

		cmd = Twist()
		cmd.linear.x = 0.20

		ang = self.fixAngle - self.currentAng + self.back_diff_fix
		searchAng = 0.0#math.atan2(math.sin(ang),math.cos(ang))

		obs_msg = obstacleService(searchAng, 0.8)
		left = obs_msg.leftAngle
		right = obs_msg.rightAngle
		fd  = obs_msg.frontDifference

		left_0 = np.cos(left-searchAng)
		right_0 = np.cos(right-searchAng)

		if left_0 > right_0:
			error = left
		else:
			error = right

		if abs(left_0-right_0)<0.05 and left_0!=right_0:
			print('Oscillating!',self.back_diff_fix)
			self.back_diff_fix += self.back_diff/20.0 if abs(self.back_diff_fix)<0.5 else 0.0

		angularVel = pid.resp(error)

		cmd.angular.z = clamp(angularVel*2.0,1.6,-1.6)
		cmd.linear.x *= smoothStep(abs(cmd.angular.z), 1.6,0.4,0.5,1.0)
			
		self.vel_pub.publish(cmd)

		self.back_diff_fix/=1.2

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
