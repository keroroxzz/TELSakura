#!/usr/bin/env python

#Obstacle Service
#Author : RTU
#Version : 1.0

import rospy
from lidar.srv import ObsDetect_srv

#services
obsDetect = rospy.ServiceProxy('lidarObstacle_Srv',ObsDetect_srv)
def obstacleService(ang, dis):
	global obsDetect
	try:
		res = obsDetect(ang, dis)
		return res.data
	except:
		print('Lost the connection with service. Try reconnecting...')
		rospy.wait_for_service('lidarObstacle_Srv')
		obsDetect = rospy.ServiceProxy('lidarObstacle_Srv',ObsDetect_srv)
		print('Reconnect to the service. Recalling....')
		return obstacleService(ang, dis)