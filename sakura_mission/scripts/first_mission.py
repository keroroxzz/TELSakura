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

	pid_x = SPPID(0.6,0.0,0.03, 0.5)
	pid_y = SPPID(0.6,0.0,0.04, 0.5)
	pid_z = SPPID(1.0,0.0,0.05, 0.5)

	def __init__(self):

		#publisher
		self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)
		self.status_pub = rospy.Publisher('/sakura/status', Int32, queue_size=10)

		self.scan_sub = rospy.Subscriber('/sakura/scan', LaserScan, self.lidarListener, queue_size = 5)
		self.image_sub = rospy.Subscriber('/sakura/camera_image/image', Image, self.imageListener, queue_size = 1)
		self.status_sub = rospy.Subscriber('/sakura/status', Int32, self.status_callback, queue_size=10)
		self.status=0

		self.camera.set(rot=0.0)

	def status_callback(self, msg):
		self.status = msg.data

	def imageListener(self, image_msg):
		img = bridge.imgmsg_to_cv2(image_msg, "bgr8")

		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		mask = inRangeHSV(hsv,blue[0], blue[1])
		mask2 = cv2.medianBlur(mask, 3)
		mask2 = cv2.Canny(mask2,50, 100)
		circles = cv2.HoughCircles(mask2, cv2.HOUGH_GRADIENT, 1, len(mask2), param1 = 100, param2 = 20, minRadius=10)

		'''if circles is not None and len(circles)>0 and self.ballon_state<2:
			circles = circles[0,:,:]
			cv2.circle(mask2, (circles[0][0],circles[0][1]), circles[0][2], (255,0,0), 2)
			self.ballon_x = circles[0][0]*0.85 + self.ballon_x*0.15
			self.ballon_state=1'''
			
		'''cv2.namedWindow('img', cv2.WINDOW_NORMAL)
		cv2.imshow('img', img)

		cv2.namedWindow('themask', cv2.WINDOW_NORMAL)
		cv2.imshow('themask', mask2)
		cv2.waitKey(0)
		print('img shown')'''

	def extract_wall(self, dir, lidar):

		dtr = math.pi/180.0

		d = lidar[int(dir)%360]
		d = d if d<1.0 else 1.0

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
		
			line = cv2.fitLine(np.array(point_set),cv2.DIST_L2,0,0.01,0.01)

			'''d = lidar[int(dir)%360]
			d = d if d<1.0 else 1.0'''

			dot = line[1][0]*line[2][0] + line[0][0]*line[3][0]

			if dot<0:
				dx = line[1][0]*0.25+line[2][0] 
				dy = line[0][0]*0.25+line[3][0]
			else:
				dx = -line[1][0]*0.25+line[2][0] 
				dy = -line[0][0]*0.25+line[3][0]

			cv2LineFloat(img, dx*100+250.0, dy*100+250.0, dx*100+250.0, dy*100+250.0, 200, 10)
			cv2LineFloat(img, 250.0, 250.0, 250.0, 250.0, 200, 1)

			cv2.namedWindow('ps',cv2.WINDOW_NORMAL)
			cv2.imshow('ps', img)
			cv2.waitKey(1)


			return dx, dy, -math.atan(line[0][0]/line[1][0])

		return 0.0, 0.0, 0.0

	def lidarListener(self,msg):

		if not self.status == 1:
			return


		if self.ballon_state==0:

			dx,dy,a = self.extract_wall(0.0, msg.ranges)
			if dx > 0.0:
				self.ballon_state=1

		#Try to face to the wall vertiaclly when tracing the ballon
		elif self.ballon_state==1:

			dir = -(self.ballon_x-180)/180.0*25.0

			'''d,a = self.extract_wall(dir, msg.ranges)
			b = (180.0 - self.ballon_x)/360.0
			d = d-0.25

			cmd = Twist()
			cmd.linear.x = self.pid_x.resp(d) 
			cmd.linear.y = self.pid_y.resp(b)
			cmd.angular.z = clamp(self.pid_z.resp(a)*5.0, 1.0, -1.0)'''

			dx,dy,a = self.extract_wall(0.0, msg.ranges)

			cmd = Twist()
			cmd.linear.x = self.pid_x.resp(dx*1.5) 
			cmd.linear.y = self.pid_y.resp(dy*2.0)
			cmd.angular.z = clamp(self.pid_z.resp(a)*5.0, 1.0, -1.0)

			#print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
			print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
			self.vel_pub.publish(cmd)

			if self.pid_x.errShort() < 0.025 and self.pid_z.errShort() < 0.075 and self.pid_y.errShort() < 0.025:

				#STOP
				cmd.linear.x = 0.0
				cmd.linear.y = 0.0
				cmd.angular.z = 0.0
				self.vel_pub.publish(cmd)

				#save data
				self.dist = msg.ranges[0]
				self.ballon_state=3
				self.index+=1

				#Arm working
				'''self.arm.settarget(t0=0.0,t1=0.0,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
				self.arm.settarget(t0=0.45,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=2)
				self.arm.settarget(t0=0.45,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
				self.arm.grep()
				self.arm.ungrep()
				self.arm.grep()
				self.arm.ungrep()
				self.arm.grep()
				self.arm.ungrep()
				self.arm.settarget(t0=0.45,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
				self.arm.settarget(t0=0.45,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=1)
				self.arm.settarget(t0=0.0,t1=0.0,t2=0,t3=0.0,t4=0,tf=0,TIME=1)'''

				cmd = Twist()
				cmd.angular.z = 0.0
				cmd.linear.x = 0.0
				cmd.linear.y = -self.dir[self.index]*0.1

				self.vel_pub.publish(cmd)

				print('STOP!', self.index, self.ballon_state)


		elif self.ballon_state==3:

			if msg.ranges[0]>self.dist+0.1 and msg.ranges[30]>0.45 and msg.ranges[330]>0.35:

				self.ballon_state=0

				if(self.index>=2):

					self.status_pub.publish(2)
					#STOP
					cmd = Twist()
					cmd.linear.x = 0.1
					cmd.linear.y = 0.0
					cmd.angular.z = 0.0
					self.vel_pub.publish(cmd)

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
