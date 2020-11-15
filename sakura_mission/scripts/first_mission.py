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
from std_msgs.msg import Int16, Float64

from sakura_common.tool import Q2E_z,clamp,smoothStep,fuzzy
from sakura_common.frame import Frame
from sakura_common.PID import PID
from sakura_common.moniter import cvShow, Moniter
from sakura_common.xh_arm import robot_arm
from sakura_common.camera_rot import camrot

from sakura_common.tool import inRangeHSV
from cv_bridge import CvBridge

bridge = CvBridge()

pid=PID(1.0,0.0,0.02)

red = (np.array([165,100,100]),np.array([20,256,256]))
green = (np.array([30,50,50]),np.array([75,256,256]))
blue = (np.array([95,60,50]),np.array([125,256,256]))

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

	arm = robot_arm()
	camera = camrot()

	def __init__(self):

		#publisher
		self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)

		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidarListener, queue_size = 5)
		self.image_sub = rospy.Subscriber('/sakura/camera_image/image', Image, self.imageListener, queue_size = 1)

		self.camera.set(rot=0.0)

	def imageListener(self, img):
		img = bridge.imgmsg_to_cv2(img)

		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		mask = inRangeHSV(hsv,blue[0], blue[1])
		mask2 = cv2.medianBlur(mask, 3)
		mask2 = cv2.Canny(mask2,50, 100)
		print(hsv[50,50])
		circles = cv2.HoughCircles(mask2, cv2.HOUGH_GRADIENT, 1, len(mask2), param1 = 100, param2 = 20, minRadius=10)

		if circles is not None and len(circles)>0 and self.ballon_state<2:
			circles = circles[0,:,:]
			cv2.circle(mask2, (circles[0][0],circles[0][1]), circles[0][2], (255,0,0), 2)
			self.ballon_x = circles[0][0]
			self.ballon_state=1
		else:
			cv2.namedWindow('test', cv2.WINDOW_NORMAL)
			cv2.imshow('test', img)
			cv2.waitKey(1)
			return False

		cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
		cv2.imshow('mask', mask2)
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


			img = np.zeros((500,500))
			for p in point_set:
				cv2LineFloat(img, p[0]*100+250.0, p[1]*100+250.0, p[0]*100+250.0,p[1]*100+250.0, 255, 3)
			cv2.namedWindow('ps',cv2.WINDOW_NORMAL)
			cv2.imshow('ps', img)
			cv2.waitKey(1)

		
			line = cv2.fitLine(np.array(point_set),cv2.DIST_L2,0,0.01,0.01)
			return lidar[int(dir)%360], -math.atan(line[0][0]/line[1][0])

		return lidar[int(dir)%360], 0.0

	cmd = Twist()
	angular_z = 0.0
	error = 5.0
	def lidarListener(self,msg):

		#Try to face to the wall vertiaclly when tracing the ballon
		if self.ballon_state==1:

			dir = -(self.ballon_x-180)/180.0*25.0

			d,a = self.extract_wall(dir, msg.ranges)
			b = (180 - self.ballon_x)

			self.cmd.linear.x = smoothStep(d-0.25,0.2,-0.2,0.2,-0.2)*0.5 + self.cmd.linear.x*0.5
			self.cmd.linear.y = b/600.0 * 0.7 + self.cmd.linear.y*0.3
			self.cmd.angular.z = -(a*5.0 + smoothStep(b,180,-180,1.0,-1.0))*0.1 + self.cmd.angular.z*0.9

			self.angular_z = self.angular_z*0.95 + self.cmd.angular.z*0.05

			cmd = Twist()
			cmd.linear.x = self.cmd.linear.x*0.5
			cmd.linear.y = -self.cmd.linear.y*0.6
			cmd.angular.z = 0.0#-abs(self.angular_z) * self.cmd.angular.z * 1.0

			'''if cmd.angular.z > 0.35:
				cmd.angular.z = 0.35
			if cmd.angular.z < -0.35:
				cmd.angular.z = -0.35 '''

			self.error = self.error*0.85 + (abs(d) + abs(a) + abs(b)/10.0)*0.15
			if self.error > 100.0:
				self.error=(abs(d) + abs(a) + abs(b)/10.0)*0.15

			print('error:%.2f  dist:%.2f  ang:%.2f  posx:%.2f'%(self.error, d, a, b))
			print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, d, a))
			self.vel_pub.publish(cmd)

			if self.error <1.8:

				cmd.linear.x = 0.0
				cmd.linear.y = 0.0
				cmd.angular.z = 0.0
				self.vel_pub.publish(cmd)

				self.dist = msg.ranges[0]
				self.ballon_state=3
				self.index+=1

				self.arm.settarget(t0=0.0,t1=0.0,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
				self.arm.settarget(t0=0.45,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=5)
				self.arm.settarget(t0=0.45,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
				self.arm.grep()
				self.arm.settarget(t0=0,t1=0,t2=0,t3=0,t4=0,tf=0.5,TIME=3)

				cmd = Twist()
				cmd.angular.z = 0.0
				cmd.linear.x = 0.0
				cmd.linear.y = self.dir[self.index]*0.065

				self.vel_pub.publish(cmd)

				print('STOP!', self.index, self.ballon_state)


		elif self.ballon_state==3:
			if msg.ranges[0]>self.dist+0.1 and msg.ranges[30]>0.3 and msg.ranges[330]>0.3:
				self.ballon_state=0

	def loop(self):

		if self.fixAngle is None or self.ballon_state==1:
			return 

		cmd = Twist()
		cmd.angular.z = 0.0
		cmd.linear.x = 0.0
		cmd.linear.y = self.dir[self.index]*0.15
			
		self.vel_pub.publish(cmd)

###################################
# MAIN
###################################

def main():
  rospy.init_node('tunnel')
  
  ballon = Ballon()

  rate = rospy.Rate(10)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
	main()
