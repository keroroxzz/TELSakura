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

from sakura_common.tool import inRangeHSV
from cv_bridge import CvBridge

bridge = CvBridge()

pid=PID(1.0,0.0,0.02)

###################################
# Classes
###################################

class Ballon():
	
	arm = robot_arm()

	def __init__(self):
		pass

	def test(self):
		
		self.arm.settarget(t0=0.0,t1=0.0,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
		self.arm.settarget(t0=0.45,t1=0.0,t2=-1.2,t3=-1.2,t4=0,tf=0,TIME=5)
		self.arm.grep()
		self.arm.settarget(t0=0,t1=0,t2=0,t3=0,t4=0,tf=0.5,TIME=3)

###################################
# MAIN
###################################

def main():
  rospy.init_node('tunnel')
  
  ballon = Ballon()
  ballon.test()
  rospy.spin()

if __name__ == '__main__':
	main()
