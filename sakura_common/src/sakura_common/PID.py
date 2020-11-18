#PID controller
#Author : RTU
#Version : 1.2
#add threshold

import rospy
import math
from sakura_common.tool import clamp,smoothStep

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
				d = clamp(d, self.d_thres, -self.d_thres) 

			differential = d/dt if dt>0.0 else 0.0

		self.lastInput = error
		self.t=rospy.get_time()

		return error*self.P + self.integral*self.I + differential*self.D

	def reset(self):
		self.integral=0.0
		self.lastInput=0.0
		self.t=-1

class SPPID():

	err_short = 1.0
	err_long = 1.0

	dsa=0.0
	dsb=0.0
	dla=0.0
	dlb=0.0

	integral=0.0
	lastInput=0.0
	t=-1

	def __init__(self, p, i, d, d_thres=0.0, decay=0.2, dratio=0.1, suppress_p1=0.0, suppress_p2=1.0, min_react=0.5):
		self.P=p
		self.I=i
		self.D=d
		self.d_thres = d_thres

		self.dsb = decay
		self.dsa = 1.0-decay
		self.dlb = decay*dratio
		self.dla = 1.0-self.dlb

		self.sp1 = suppress_p1
		self.sp2 = suppress_p2

		self.mreact = min_react

	def err_calc(self, err):
		self.err_short = self.err_short*self.dsa + abs(err)*self.dsb
		self.err_long = self.err_long*self.dla + abs(err)*self.dlb

	def suppress(self):
		return smoothStep(self.err_short/self.err_long, 0.0, 1.0, self.mreact, 1.0) #* smoothStep(self.err_short, self.sp1, self.sp2, self.mreact, 1.0)

	#response for the input
	def resp(self, error):

		differential = 0.0

		self.err_calc(error)

		if self.t>0.0:
			dt = rospy.get_time()-self.t

			self.integral += error * dt
			d = error-self.lastInput

			if self.d_thres>0.0:
				d = clamp(d, self.d_thres, -self.d_thres) 

			differential = d/dt if dt>0.0 else 0.0

		self.lastInput = error
		self.t=rospy.get_time()

		return (error*self.P + self.integral*self.I + differential*self.D)*self.suppress()

	def errLong(self):
		return self.err_long

	def errShort(self):
		return self.err_short

	def reset(self):
		self.integral=0.0
		self.lastInput=0.0
		self.t=-1

