#!/usr/bin/env python

#Traffic Sign Detection
#Maintain : RTU
#Version : 2.2

import sys
import cv2
import math
import time
import rospy
import rospkg
import numpy as np
import threading

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,LaserScan

from sakura_common.tool import inRangeHSV

from sakura_common.moniter import Moniter,bridge,cvShow
from sakura_common.camera_rot import camrot
from sakura_common.tool import Q2E_z,clamp,smoothStep,fuzzy
from sakura_common.xh_arm import robot_arm
from sakura_common.PID import PID, SPPID

def cv2LineFloat(img, x1, y1, x2, y2, color, thickness):
	cv2.line(img, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, thickness)

###################################
# Classes
###################################
class OrienFixer():

    arm_init = False
    arm = robot_arm()
    camera = camrot()

    dist=0.2

    pid_x = SPPID(0.6,0.0,0.02, 0.5)
    pid_y = SPPID(0.8,0.15,0.06, 0.5)
    pid_z = SPPID(1.0,0.1,0.05, 0.5)

    stage = 0
    E_state = 0

    status=0

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11)) 
    def __init__(self):

        self.m1 = Moniter('medianBlurred')
        self.m2 = Moniter('dilated')
        self.m3 = Moniter('eroded')

        self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/sakura/scan', LaserScan, self.lidarListener, queue_size = 5)

		self.status_sub = rospy.Subscriber('/sakura/status', Int32, self.status_callback, queue_size=10)

        self.dts_pub = rospy.Publisher('/sakura/detection_state', Int32, queue_size=1)
        self.E_sub = rospy.Subscriber('/sakura/E', Int32, self.E_callback, queue_size=1)

    def E_callback(self, msg):
        self.E_state = msg.data

    def initializing(self):
		self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
		self.arm_init = True

	def status_callback(self, msg):
		self.status = msg.data

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

    def anylize_wall(self, dir, lidar, sy=0.0, sx=0.0):

		point_set = self.extractside(dir, lidar, 1, [])
		point_set = self.extractside(dir, lidar, -1, point_set=point_set)

		if point_set is not None and len(point_set)>10:

			
			line = cv2.fitLine(np.array(point_set),cv2.DIST_L2,0,0.01,0.01)

			# find the right side to go
			dot = line[1][0]*line[2][0] + line[0][0]*line[3][0]
			if dot<0:
				dx = line[1][0]*self.dist+line[2][0] + sx
				dy = -line[0][0]*self.dist+line[3][0] + sy
			else:
				dx = -line[1][0]*self.dist+line[2][0] + sx
				dy = line[0][0]*self.dist+line[3][0] + sy

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
		if not self.status == 3:
			return

        # trace the right side of the track
        if self.stage == 0:

            # Initializing
            if not self.arm_init:
                self.initializing()

            self.camera.set(rot=0.6, cy=0.0)

            dx,dy,a = self.anylize_wall(0.0, msg.ranges, sy=-0.18, sx=0.15)

            cmd = Twist()

            if dx>0.15:
                cmd.linear.x = self.pid_x.resp(dx*1.0) 
                cmd.angular.z = self.pid_y.resp(dy*1.5)*1.5
            elif not dx == 0.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                print('STOP stage 0')
                self.stage = 1

            #print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
            print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
            self.vel_pub.publish(cmd)

        # trace the encountered wall
        elif self.stage == 1:

            self.camera.set(rot=0.0, cy=0.0)

            dx,dy,a = self.anylize_wall(0.0, msg.ranges)

            cmd = Twist()
            cmd.linear.x = self.pid_x.resp(dx*1.0) 
            cmd.linear.y = self.pid_y.resp(dy*1.5)
            cmd.angular.z = clamp(self.pid_z.resp(a)*1.25, 1.0, -1.0)

            if dx < 0.0 and self.pid_z.errShort() < 0.09 and self.pid_y.errShort() < 0.045:
                self.camera.set(rot=0.4, cy=0.0)
                cmd.linear.x = 0.0
                cmd.linear.y = 0.3
                cmd.angular.z = 0.0
                print('STOP stage 1')
                self.stage = 2

            #print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
            print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
            self.vel_pub.publish(cmd)

        # go left
        elif self.stage == 2:

            self.camera.set(rot=0.0, cy=0.0)

            dx,dy,a = self.anylize_wall(0.0, msg.ranges)

            cmd = Twist()
            cmd.linear.y = 0.3


            right_front = [d for d in msg.ranges[310:360] if d < 0.45 and d > 0.15]
            print('Obs index : %d'%(len(right_front)))

            if len(right_front)==0:
                self.camera.set(rot=0.55, cy=0.0)

                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)

                rospy.Rate(0.5).sleep()

                cmd.linear.x = 0.0
                cmd.linear.y = 0.3
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)


                print('STOP stage 2')
                self.stage = 3

            #print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
            print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
            self.vel_pub.publish(cmd)

        # go front
        elif self.stage == 3:

            self.camera.set(rot=0.55, cy=0.0)

            dx,dy,a = self.anylize_wall(0.0, msg.ranges)

            cmd = Twist()
            cmd.linear.x = 0.2

            if msg.ranges[0] == 0 or msg.ranges[0]>1.0:
                cmd.linear.x = 0.0
                cmd.linear.y = -0.0
                cmd.angular.z = 0.0
                print('STOP stage 3')

                # start detection
                self.dts_pub.publish(1)

                self.stage = 4

            #print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
            print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
            self.vel_pub.publish(cmd)

        # go right
        elif self.stage == 4:

            self.camera.set(rot=0.0, cy=0.0)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = -0.1
            cmd.angular.z = 0.0

            if self.E_state == 1:

                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)
                self.camera.set(rot=0.7, cy=0.0)

                #Arm working
                self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)
                self.arm.settarget(t0=0.1,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=2)
                self.arm.settarget(t0=0.1,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
                self.arm.settarget(t0=0.1,t1=0.5,t2=-0.2,t3=-1.57,t4=0,tf=0,TIME=1)
                self.arm.settarget(t0=0.1,t1=0.0,t2=-0.5,t3=-1.57,t4=0,tf=0,TIME=1)
                self.arm.settarget(t0=0.1,t1=0.0,t2=0.0,t3=-1.57,t4=0,tf=0,TIME=1)
                self.arm.settarget(t0=0.0,t1=-0.2,t2=0,t3=0.0,t4=0,tf=0,TIME=1)

                # stop detection
                self.dts_pub.publish(0)
                self.stage = 5

            self.vel_pub.publish(cmd)
            
        elif self.stage == 5:

            # Initializing
            if not self.arm_init:
                self.initializing()

            self.camera.set(rot=0.7, cy=0.0)

            dx,dy,a = self.anylize_wall(0.0, msg.ranges, sy=0.18)

            cmd = Twist()
            cmd.linear.y = 0.15

            right_front = [d for d in msg.ranges[25:60] if d < 0.8 and d > 0.16]
            print('Obs index : %d'%(len(right_front)))

            if len(right_front)==0:
                cmd.linear.x = 0.2
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)
                print('END!!')
                self.stage = 6

            #print('ed:%.2f  ea:%.2f  eb:%.2f  posx:%.2f'%(self.pid_x.errShort(), self.pid_z.errShort(), self.pid_y.errShort(), b))
            print('x:%.2f  y:%.2f  z:%.2f'%(cmd.linear.x, cmd.linear.y, cmd.angular.z))
            self.vel_pub.publish(cmd)



###################################
# MAIN
###################################

def main():
    rospy.init_node('OrienFixer')

    of = OrienFixer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
