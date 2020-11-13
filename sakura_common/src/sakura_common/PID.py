#PID controller
#Author : RTU
#Version : 1.2
#add threshold

import rospy
import math
from supply.tool import clamp

class PID():
	integral=0.0
	lastInput=0.0
	t=-1

	def __init__(self,p,i,d, d_thres=0.0):
		self.P=p
		self.I=i
		self.D=d
		self.d_thres = d_thres

	#response for the input
	def resp(self,error):

		differential = 0.0

		if self.t>0.0:
			dt = rospy.get_time()-self.t

			self.integral += error * dt
			d = error-self.lastInput

			if self.d_thres>0.0:
				d = clamp(d,-self.d_thres,self.d_thres) 

			differential = d/dt if dt>0.0 else 0.0

		self.lastInput = error
		self.t=rospy.get_time()

		return error*self.P + self.integral*self.I + differential*self.D

	def reset(self):
		self.integral=0.0
		self.lastInput=0.0
		self.t=-1

