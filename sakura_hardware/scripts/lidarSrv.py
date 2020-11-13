#!/usr/bin/env python

#Obstacle analysis
#Author : RTU
#Version : 2.1

import rospy
import math
import cv2
import numpy as np
#import threading

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sakura_lidar.srv import ObsDetect_srv,ObsDetect_srvResponse
from sakura_lidar.msg import obstacle_msg

from supply.performance import perfmeter
from supply.moniter import Moniter

performance = perfmeter()
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

class Obstacle:
	def __init__(self, middle, cnt, safeAng=0.1):
		self.leftAng = 0.0
		self.rightAng = 0.0
		self.leftDist = 0.0
		self.rightDist = 0.0
		self.middle = middle
		self.cnt = cnt
		self.safeAng = safeAng

	def anylize(self):

		max_l = 0.0
		max_r = 0.0

		self.rightPoint = self.cnt[0][0]
		self.leftPoint = self.cnt[0][0]
		self.leftAng = self.getAng_A(self.leftPoint)
		self.rightAng = self.getAng_A(self.rightPoint)

		for p in self.cnt[1:]:
			ang_rel = self.getAng_A(p[0])

			if ang_rel + self.safeAng > self.leftAng:
				self.leftAng = ang_rel + self.safeAng
				self.leftPoint = p[0]

			elif ang_rel - self.safeAng < self.rightAng:
				self.rightAng = ang_rel - self.safeAng
				self.rightPoint = p[0]
		
		self.leftDist = np.linalg.norm(np.subtract(self.leftPoint,self.middle))
		self.rightDist = np.linalg.norm(np.subtract(self.rightPoint,self.middle))

	def getAng_A(self, p):
		return math.atan2(p[1]-self.middle[1], p[0]-self.middle[0])

	def getAng_R(self, p1, p2):
		ang = self.getAng_A(p1) - self.getAng_A(p2)
		return ang

	def inRange(self,ang):
		return ang <= self.leftAng and ang >= self.rightAng

	def nearest(self,obs):
		if ((self.inRange(obs.leftAng) or obs.inRange(self.rightAng)) and obs.leftDist < self.rightDist) or \
			((self.inRange(obs.rightAng) or obs.inRange(self.leftAng)) and obs.rightDist < self.leftDist) or \
			(self.inRange(obs.leftAng) and self.inRange(obs.rightAng) and (self.leftDist > obs.rightDist or self.rightDist > obs.leftDist)):
			return obs
		else:
			return self

	def fixConfliction(self, obstacles):
		for obs in obstacles:
			if obs == self:
				continue
				
			if self.inRange(obs.leftAng) and obs.leftDist < self.rightDist and obs.rightAng < self.rightAng:
				self.rightAng = obs.leftAng
				self.rightDist = obs.leftDist
				self.rightPoint = obs.leftPoint
			elif self.inRange(obs.rightAng) and obs.rightDist < self.leftDist and obs.leftAng > self.leftAng:
				self.leftAng = obs.rightAng
				self.leftDist = obs.rightDist
				self.leftPoint = obs.rightPoint

	def draw(self, img, color):
		cv2LineCylindarCooerd(img, self.leftDist, self.leftAng, self.rightDist, self.rightAng, color, 1, self.middle)

	def drawBeams(self, img, color, c2=255):
		cv2LineFloat(img, self.middle[0], self.middle[1], self.leftPoint[0], self.leftPoint[1], color, 1)
		cv2LineFloat(img, self.middle[0], self.middle[1], self.rightPoint[0], self.rightPoint[1], color, 1)
		cv2.drawContours(img, self.cnt, -1 ,c2)


class LidarObsAnylizer:
	contours = []
	lidarRanges = np.zeros(360)
	obstacles = []
	frontDif = -1.0
	safeDirection = 0.0

	def __init__(self, width=300, height=300, radius=40, ratio=100.0, max_gap=0.27):
		self.width = width
		self.height = height
		self.radius = radius
		self.middle = (width/2.0, height/2.0)
		self.RATIO = ratio
		self.MAX_DIST = 3.0#height/(2.0*ratio)
		self.MIN_DIST = 0.12#height/(2.0*ratio)
		self.MAXIMUN_GAP = max_gap	#gap to distinguish objects

		self.obstacleMap =  np.zeros((self.height,self.width),np.uint8)
		self.resultImage = np.zeros((self.height,self.width),np.uint8)
		self.empty = np.zeros((self.height,self.width),np.uint8)

		self.mapPub = rospy.Publisher('/lidarObsSrv/map', Image, queue_size=1)
		self.lidarSub = rospy.Subscriber('/sakura/re_scan', LaserScan, self.lidarCallback, queue_size = 1)

		self.obs_moni = Moniter('lidar_srv')

		self.rotPub = rospy.Publisher('/sakura/rot/command', Float64, queue_size=1)

		rospy.Rate(1).sleep()
		self.rotPub.publish(0.25)

	#listener for lidar
	def lidarCallback(self, data):
		self.lidarRanges = data.ranges
		self.drawAndFindContours()

		right = [d for d in self.lidarRanges[290:360] if d < 0.35]
		left = [d for d in self.lidarRanges[0:70] if d < 0.35]
		right = np.average(right) if len(right)>0 else 0.35
		left = np.average(left) if len(left)>0 else 0.35
		self.frontDif = -right+left

	#draw the weight map of the env
	def drawAndFindContours(self):
		#print('Drawing...')
		self.obstacleMap = np.zeros((self.height,self.width),np.uint8)	#reset

		#draw the map
		for i in range(1,len(self.lidarRanges)):

			if self.lidarRanges[i-1]<0 and self.lidarRanges[i] <0:

				a1=(i-1)*dtr
				a2=i*dtr
				cv2LineCylindarCooerd(self.resultImage,1.0*self.RATIO,a1,1.0*self.RATIO,a2,255,5,self.middle)

			if (self.lidarRanges[i-1] > self.MAX_DIST or self.lidarRanges[i] > self.MAX_DIST) or\
				(self.lidarRanges[i-1] < self.MIN_DIST or self.lidarRanges[i] < self.MIN_DIST) or\
				abs(self.lidarRanges[i-1]-self.lidarRanges[i]) > self.MAXIMUN_GAP:
				continue

			a1=(i-1)*dtr
			a2=i*dtr
			r1 = self.lidarRanges[i-1]*self.RATIO
			r2 = self.lidarRanges[i]*self.RATIO
			cv2LineCylindarCooerd(self.obstacleMap,r1,a1,r2,a2,255,self.radius,self.middle)

		#cut the back side to avoid some confusing condition
		cv2LineFloat(self.obstacleMap, self.middle[0], self.middle[1], self.middle[0], self.middle[1], 0, 23)
		cv2LineFloat(self.obstacleMap, 0, self.middle[1], self.middle[0], self.middle[1], 0, 2)

		#cv2 find contours
		_,cnt,_ = cv2.findContours(self.obstacleMap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#print('Drawing Done!')
		self.anylizeContours(cnt)
		return cnt

	def anylizeContours(self, contours):
		#print('Anylizing...')
		#performance.tickStart('anylize')
		res = []
		for cnt in contours:
			new_obs = Obstacle(self.middle, cnt)
			new_obs.anylize()
			new_obs.draw(self.obstacleMap,155)
			res.append(new_obs)
		#performance.tickStop('anylize')
		self.obstacles = res

		#print('Anylizing Done!  find {0} obstacles'.format(len(res)))

	def isAnyObs(self, angle, distance):

		if distance <= 0.0:
			return True
		else:
			dx,dy = math.cos(angle), math.sin(angle)

			x,y = self.middle
			distance_cur = 0.0

			while distance>distance_cur:
				
				x+=dx
				y+=dy
				distance_cur+=1.0

				if x<0 or y<0 or x>=self.width or y>= self.height:
					break
				self.resultImage[int(y)][int(x)]=255
				if self.obstacleMap[int(y)][int(x)] > 0:
					return True

			return False

	def isDangerouse(self, angle):

		rtd = 180.0/np.pi
		deg = angle*rtd

		a_f = int(np.floor(deg))
		a_c = int(np.ceil(deg))

		if self.lidarRanges[a_f] < 0 or self.lidarRanges[a_c] < 0:
			print(a_f,a_f,'DAN!')
			return True

		return False

	def contoursFront(self, angle, distance):

		self.empty = np.zeros((self.height,self.width),np.uint8)
		self.resultImage = np.zeros((self.height,self.width),np.uint8)

		cv2LineCylindarCooerd(self.empty, 0.0, 0.0, 60.0, angle, 255, 1, self.middle)

		#if no obstacle between the robot and local goal
		if not self.isAnyObs(angle, self.RATIO*distance):
			return angle,10.0,angle,10

		obs_front = [obs for obs in self.obstacles if obs.inRange(angle)]

		#draw the beam of both sides of an obstacle
		for o in obs_front:
			o.drawBeams(self.resultImage, 100,100)

		num = len(obs_front)
		if num == 0:
			return angle,10.0,angle,10
		elif num==1:
			front = obs_front[0]
		elif num>1:
			for i in range(0,len(obs_front)-1):
				front = obs_front[i].nearest(obs_front[i+1])

		#check the confliction between obstacles
		front.fixConfliction(self.obstacles)
		front.drawBeams(self.resultImage, 255)

		'''l_isd = self.isDangerouse(front.leftAng)
		r_isd = self.isDangerouse(front.rightAng)'''

		return front.leftAng,front.leftDist/self.RATIO,front.rightAng,front.rightDist/self.RATIO


def main():
	rospy.init_node('Lidar_Srv')
	LOA = LidarObsAnylizer()
	
	def handle_obs_srv(req):
		maxa,maxd,mina,mind = LOA.contoursFront(req.angle, req.distance)
		return ObsDetect_srvResponse(obstacle_msg(maxa, maxd, mina, mind, LOA.frontDif))

	srv = rospy.Service('lidarObstacle_Srv', ObsDetect_srv, handle_obs_srv)
	print('Service Ready')

	while not rospy.is_shutdown():
		LOA.obs_moni.pubCv2(cv2.merge([LOA.obstacleMap, LOA.resultImage, LOA.empty]))
		rospy.sleep(0.1)

if __name__ == '__main__':

    try:
		main()
    except rospy.ROSInterruptException:
        pass
