#!/usr/bin/env python

#Obstacle analysis
#Author : RTU
#Version : 2.1

import rospy
import math
import cv2
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


dtr = math.pi/180.0
#draw a line with floats
def cv2LineFloat(img, x1, y1, x2, y2, color, thickness):
	cv2.line(img, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, thickness)

def cv2LineCylindarCooerd(img, r1, a1, r2, a2, color, thickness, offset):
	x1_ = np.cos(a1)*r1+offset[0]
	y1_ = np.sin(a1)*r1+offset[1]
	x2_ = np.cos(a2)*r2+offset[0]
	y2_ = np.sin(a2)*r2+offset[1]
	cv2LineFloat(img, x1_, y1_, x2_, y2_, color, thickness)

class LidarFilter:
	def __init__(self):

		self.MAX_DIST = 3.0
		self.MIN_DIST = 0.12
		self.NO_OBS = 3.5
		self.NO_GO = 1.3
		self.h = 0.27
		self.dh = 0.018

		self.rotation = 0.0
		self.ground = np.zeros((360))

		self.rot = rospy.Subscriber('/sakura/rot/command', Float64, self.rotCallBack, queue_size = 1)
		self.lidarSub = rospy.Subscriber('/scan', LaserScan, self.lidarCallback, queue_size = 1)
		self.lidarPub = rospy.Publisher('/sakura/re_scan', LaserScan, queue_size = 1)
		self.sdPub = rospy.Publisher('/sakura/safe_dir', Float64, queue_size = 1)

	def rotCallBack(self, data):
		self.rotation = data.data

		rh = self.h+self.dh*np.cos(self.rotation)
		l0 = rh/np.sin(self.rotation)

		for i in range(0,len(self.ground)):
			theta = i/180.0*np.pi
			self.ground[i] = l0/np.cos(theta)

	def middleOfRunalbe(self, ranges):

		tot = 0.0
		num = 0.0
		for i in range(-180,180):
			if ranges[i]>0.0:
				tot += i
				num += 1

		self.sdPub.publish(tot/num*dtr) if num > 0 else 0

		return int(round(tot/num)) if num > 0 else 0

	def middleOfGround(self, ranges):

		tot = 0.0
		num = 0.0
		for i in range(-180,180):
			if ranges[i]==3.5:
				tot += i
				num += 1

		self.sdPub.publish(tot/num*dtr) if num > 0 else 0

		return int(round(tot/num)) if num > 0 else 0

	def encloseDangerouse(self, ranges):

		mid = self.middleOfRunalbe(ranges)

		last_obs=0.0

		for i in range(mid,180):
			if ranges[i]<0:
				ranges[i] = ranges[i-1] if ranges[i-1]<self.NO_OBS else self.NO_GO
			elif ranges[i]<self.NO_OBS:
				last_obs=ranges[i]

		for i in range(-180,mid)[::-1]:
			if ranges[i]<0:
				ranges[i] = ranges[i+1] if ranges[i+1]<self.NO_OBS else self.NO_GO
			elif ranges[i]<self.NO_OBS:
				last_obs=ranges[i]

	#listener for lidar
	def lidarCallback(self, data):

		img = np.zeros((600,600,3),np.uint8)
		img[300,300,1]=255
		re_scan = data
		ranges = np.array(data.ranges)

		self.draw(img, ranges, (255,0,0))
		self.draw(img, self.ground, (0,0,255))

		for i in range(0,len(self.ground)):
			if ranges[i]-self.ground[i]>-0.04 and ranges[i]-self.ground[i]<0.08:
				ranges[i]=self.NO_OBS

			elif self.ground[i]>ranges[i]:
				theta = i/180.0*np.pi

				x = ranges[i]*np.sin(theta)
				l0 = ranges[i]*np.cos(theta)*np.cos(self.rotation)
				ranges[i] = np.sqrt(x*x+l0*l0)

			else:
				ranges[i] = -1.0

		#self.encloseDangerouse(ranges)

		self.draw(img, ranges, (0,150,0))
		cv2.namedWindow('ing',cv2.WINDOW_NORMAL)
		cv2.imshow('ing',img)
		cv2.waitKey(1)

		re_scan.ranges = ranges

		self.lidarPub.publish(re_scan)


	def draw(self, img, ranges, color):

		#draw the map
		for i in range(1,len(ranges)):

			if ranges[i-1]>20.0 or ranges[i]>20.0 or ranges[i-1]<0 or ranges[i]<0:
				continue

			a1=(i-1)*dtr
			a2=i*dtr
			r1 = ranges[i-1]*100.0
			r2 = ranges[i]*100.0
			cv2LineCylindarCooerd(img,r1,a1,r2,a2,color,5,(300,300))


def main():
	rospy.init_node('Lidar_Filter')

	lf = LidarFilter()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':

    try:
		main()
    except rospy.ROSInterruptException:
        pass
