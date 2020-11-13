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

pid=PID(1.0,0.0,0.02)

###################################
# Classes
###################################

class Tunnel():
	path_length=0.0

	currentAng = 0.0
	desireAng = 0.0
	fixAngle = None


	laneTendncy = 0.0
	laneState = 0
	local_goal_d=-1.0
	stuck_time = -1

	getInTime = -1

	pos_movebase = None
	path_movebase = None

	def __init__(self):

		#publisher
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.poseListener, queue_size = 1)

	def poseListener(self,msg):

		self.currentAng=Q2E_z(msg.pose.pose.orientation)

		if self.fixAngle is None:
			self.fixAngle = self.currentAng

		self.obsMain_dc()

	def obsMain_omi(self):

		if self.fixAngle is None:
			return 

		cmd = Twist()

		cmd.angular.z = self.laneTendncy

		ang = self.fixAngle - self.currentAng 
		searchAng = math.atan2(math.sin(ang),math.cos(ang))

		obs_msg = obstacleService(searchAng, 0.8)
		left = obs_msg.leftAngle
		right = obs_msg.rightAngle
		fd  = obs_msg.frontDifference

		left_0 = np.cos(left-searchAng)
		right_0 = np.cos(right-searchAng)

		if left_0 > right_0:
			a = left 
		else:
			a = right

		cmd.linear.x  = math.cos(a)*0.20
		cmd.linear.y  = math.sin(a)*0.20
			
		self.vel_pub.publish(cmd)

	def obsMain_dc(self):

		if self.fixAngle is None:
			return 

		cmd = Twist()
		cmd.linear.x = 0.20

		cmd.angular.z = self.laneTendncy

		ang = self.fixAngle - self.currentAng 
		searchAng = math.atan2(math.sin(ang),math.cos(ang))

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
			
		if abs(left_0-right_0)<0.05:
			print('Oscillating!')

		angularVel = pid.resp(error)

		cmd.angular.z = clamp(angularVel*2.0,1.6,-1.6)
		cmd.linear.x *= smoothStep(abs(cmd.angular.z), 1.6,0.4,0.2,1.0)
			
		#self.vel_pub.publish(cmd)

###################################
# MAIN
###################################

def main():
  rospy.init_node('tunnel')
  
  tunnel = Tunnel()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
	main()
