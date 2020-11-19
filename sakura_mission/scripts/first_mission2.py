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
ballon_color_range = [red, green, blue]


#### the target ballons ####
targets = [0, 2]



def cv2LineFloat(img, x1, y1, x2, y2, color, thickness):
	cv2.line(img, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, thickness)

###################################
# Classes
###################################

class Ballon():

	index = -1
	dir = [-1.0,1.0,-1.0, 1.0]
	dist = 0.0

	ballon_state = 0

	arm_init = False
	arm = robot_arm()
	camera = camrot()

	pid_x = SPPID(0.6,0.0,0.02, 0.5)
	pid_y = SPPID(0.8,0.15,0.06, 0.5)
	pid_z = SPPID(1.0,0.1,0.05, 0.5)

	dx = 160
	dy = 80
	dr = 50

	dist = 0.24

	ballon_detect = -1
	ballon_area = 0.0
	MAX_AREA = 7578.5

	def __init__(self):

		#publisher
		self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)
		self.status_pub = rospy.Publisher('/sakura/status', Int32, queue_size=10)

		self.scan_sub = rospy.Subscriber('/sakura/scan', LaserScan, self.lidarListener, queue_size = 5)
		self.image_sub = rospy.Subscriber('/sakura/camera_image/image', Image, self.imageListener, queue_size = 1)
		self.status_sub = rospy.Subscriber('/sakura/status', Int32, self.status_callback, queue_size=10)

		
		self.dts_pub = rospy.Publisher('/sakura/detection_state', Int32, queue_size=1)
		self.ballon_pub = rospy.Subscriber('/sakura/ballon', Int32, self.ballon_callback, queue_size=1)
		
		self.status=1

	def initializing(self):
		self.arm.settarget(t0=1.57,t1=1.25,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=1)
		self.arm.settarget(t0=1.57,t1=1.05,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=1)
		self.arm.settarget(t0=1.2,t1=0.95,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=1)
		self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
		self.arm_init = True

	def ballon_callback(self, msg):
		self.ballon_detect = msg.data

	def status_callback(self, msg):
		self.status = msg.data

	def imageListener(self, image_msg):

		#jump out if it is not mission 1
		if not self.status == 1:
			return

		img = bridge.imgmsg_to_cv2(image_msg)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#generate the mask of roi
		mask = np.zeros(img.shape[:2], dtype="uint8")
		cv2.circle(mask, (self.dx, self.dy), self.dr, 1, -1)

		#get the roi
		masked = cv2.bitwise_and(hsv, hsv, mask=mask)


		for i in targets:
			color = ballon_color_range[i]

			hue_range = inRangeHSV(masked, color[0], color[1])
			_,cnt,_ = cv2.findContours(hue_range,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

			#cvShow('ass', hue_range)
			
			if len(cnt) > 0:
				M=cv2.moments(cnt[0])
				self.ballon_area = M['m00']/self.MAX_AREA*0.2 + self.ballon_area*0.8


	def extractside(self, dir, lidar, step, point_set):

		dtr = math.pi/180.0

		#extract right side
		i = int(dir)%360
		while True:
			dist_i = lidar[i]
			
			dist_j = lidar[(i+step)%360]

			if abs(dist_i-dist_j)<0.2:
				a=i*dtr
				point = (math.cos(a)*dist_i, math.sin(a)*dist_i)
				point_set.append(point)
			else:
				break
			
			i+=step
			if (step>0 and i>=360) or (step<0 and i<=-360):
				break

		return point_set

	def anylize_wall(self, dir, lidar):

		point_set = self.extractside(dir, lidar, 1, [])
		point_set = self.extractside(dir, lidar, -1, point_set=point_set)

		if point_set is not None and len(point_set)>10:

			
			line = cv2.fitLine(np.array(point_set),cv2.DIST_L2,0,0.01,0.01)

			# find the right side to go
			dot = line[1][0]*line[2][0] + line[0][0]*line[3][0]
			if dot<0:
				dx = line[1][0]*self.dist+line[2][0] 
				dy = -line[0][0]*self.dist+line[3][0]
			else:
				dx = -line[1][0]*self.dist+line[2][0] 
				dy = line[0][0]*self.dist+line[3][0]

			#visualization
			img = np.zeros((500,500))
			
			cv2LineFloat(img, dx*200+250.0, dy*200+250.0, dx*200+250.0, dy*200+250.0, 200, 5)
			cv2LineFloat(img, 250.0, 250.0, 250.0, 250.0, 200, 1)

			for p in point_set:
				cv2LineFloat(img, p[0]*200+250.0, p[1]*200+250.0, p[0]*200+250.0,p[1]*200+250.0, 255, 1)

			cv2.namedWindow('ps',cv2.WINDOW_NORMAL)
			cv2.imshow('ps', img)
			cv2.waitKey(1)
			line[1][0] += 0.000000000001
			return dx, dy, -math.atan(line[0][0]/line[1][0]) #+ 0.09 # offset

		return 0.0, 0.0, 0.0

	def lidarListener(self,msg):

		#jump out if it is not mission 1
		if not self.status == 1:
			return

		# Initializing
		if not self.arm_init:
			self.initializing()

		self.camera.set(rot=0.0, cy=0.0)

		if self.ballon_state==0:

			dx,dy,a = self.anylize_wall(0.0, msg.ranges)
			if dx > 0.0:
				self.ballon_state=1
				
				# start detection
				self.dts_pub.publish(1)

		# wall tracing
		elif self.ballon_state==1:
			dx,dy,a = self.anylize_wall(0.0, msg.ranges)
			print(a)

			cmd = Twist()
			cmd.linear.x = self.pid_x.resp(dx*1.0) 
			cmd.linear.y = self.pid_y.resp(dy*1.5)
			cmd.angular.z = clamp(self.pid_z.resp(a)*1.25, 1.0, -1.0)

			#print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
			print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
			self.vel_pub.publish(cmd)

			if self.pid_x.errShort() < 0.045 and self.pid_z.errShort() < 0.09 and self.pid_y.errShort() < 0.045:

				# stop detection
				self.dts_pub.publish(0)

				#STOP
				cmd.linear.x = 0.0
				cmd.linear.y = 0.0
				cmd.angular.z = 0.0
				self.vel_pub.publish(cmd)

				#save data
				self.dist = msg.ranges[0]
				self.ballon_state=3
				self.index+=1

				if self.ballon_area > 0.3 or self.ballon_detect in targets:	# if the color detection or yolov4 detect the ballon

					#Arm working
					self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
					self.arm.settarget(t0=0.45,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=2)
					self.arm.settarget(t0=0.45,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
					self.arm.settarget(t0=0.45,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
					self.arm.settarget(t0=0.45,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=1)
					self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)

				cmd = Twist()
				cmd.angular.z = 0.0
				cmd.linear.x = 0.0
				cmd.linear.y = -self.dir[self.index]*0.2

				self.vel_pub.publish(cmd)
				self.ballon_area = 0.0
				print('STOP!', self.index, self.ballon_state)


		elif self.ballon_state==3:

			left = min([d for d in msg.ranges[0:40] if d>0.18])
			right = min([d for d in msg.ranges[320:360] if d>0.18])

			print('Left : %.2f     Right : %.2f'%(left, right))
			if msg.ranges[0]>self.dist+0.1 and (left>0.45 or left<=0.0) and (right>0.37 or right<=0.0):

				# if not fin yet
				if(self.index<2):
					self.ballon_state=0

				else:
					#STOP
					cmd = Twist()
					cmd.linear.x = 0.15
					cmd.linear.y = 0.0
					cmd.angular.z = 0.0
					self.vel_pub.publish(cmd)

					right_front = [d for d in msg.ranges[270:360] if d < 0.7 and d > 0.32]
					print('Obs index : %d'%(len(right_front)))

					if len(right_front)==0:
						self.status_pub.publish(2)
						print('---MISSION 1 FIN!---')

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
