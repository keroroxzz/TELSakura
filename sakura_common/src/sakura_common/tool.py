#Tools
#Author : RTU
#Version : 1.0

import rospy,math,cv2
import numpy as np

#fuctions
def Q2E_z(Q):
	return math.atan2(2*Q.z*Q.w+2*Q.y*Q.x, 1-2*Q.y*Q.y-2*Q.z*Q.z)
	
def clamp(v,u,d):
	return min(u,max(v,d))

def smoothStep(value, right, left, bound_r, bound_l):
	x = clamp((value-left)/(right-left),1.0,0.0)
	return x*x*(3.0-2.0*x)*(bound_r-bound_l)+bound_l

def fuzzy(value, mid, fuzzy, shift = 0.5):
	return math.atan((value-mid)/fuzzy)/3.14159+shift

def inRangeHSV(hsv, lower, upper):

	if lower[0]>upper[0]:
		lower_ = np.copy(lower)
		upper_ = np.copy(upper)
		lower_[0]=0
		upper_[0]=180
		return cv2.inRange(hsv, lower, upper_) + cv2.inRange(hsv, lower_, upper)
	else:
		return cv2.inRange(hsv, lower, upper)