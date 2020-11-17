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

from sakura_common.tool import inRangeHSV

from sakura_common.moniter import Moniter,bridge,cvShow
from sakura_common.camera_rot import camrot
from sakura_common.tool import Q2E_z,clamp,smoothStep,fuzzy

###################################
# Classes
###################################
class OrienFixer():

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11)) 
    def __init__(self):

        self.m1 = Moniter('medianBlurred')
        self.m2 = Moniter('dilated')
        self.m3 = Moniter('eroded')

        self.cam = camrot()
        self.vel_pub = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)
        self.img_sub = rospy.Subscriber('/sakura/camera_image/image',Image,self.imageCallback, queue_size=1)

    def imageCallback(self,data):

        self.cam.set(rot=0.35, cy=-0.2)

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        mb = cv2.medianBlur(img, 25)
        diliated = cv2.dilate(mb, self.kernel)
        eroded = cv2.erode(mb, self.kernel)
        hsv = cv2.cvtColor(eroded, cv2.COLOR_BGR2HSV)
        mask = inRangeHSV(hsv, (0, 90, 100), (30, 140, 235))
        canny = cv2.Canny(mask, 50, 100)
        blank = np.zeros(canny.shape)
        lines = cv2.HoughLinesP(canny, 1,np.pi/180,4,minLineLength=50,maxLineGap=5)

        points = np.array([])
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]

                if y2-y1 == 0:
                    ang = 90.0
                else:
                    ang = np.arctan((x2-x1)/float(y2-y1))*180/np.pi

                if ang>70 or ang<-30:
                    continue

                app=np.array(([x1,y1],[x2,y2]))
                cv2.line(blank, (x1,y1), (x2,y2), 200, 1)
                

                if len(points)==0:
                    points=app
                else:
                    points=np.append(points,app,axis=0)

		if len(points.shape)==2 and len(points)>2:
			line = cv2.fitLine(points,cv2.DIST_L2,0,0.01,0.01)

            dx,dy,x,y = np.squeeze(line)

            if dx<1.0:
                sx = y*dx/dy
                ang = np.arctan2(dy,dx)
                print(x-sx, np.arctan2(dy,dx))

                cmd = Twist()
                cmd.linear.x = 0.1
                cmd.linear.y = smoothStep((100.0 - (x-sx)),-5.0,5.0,-0.05, 0.05)
                cmd.angular.z = smoothStep(ang - 0.88, 0.5,-0.5,-1.0,1.0)
                self.vel_pub.publish(cmd)

        cvShow('ass', blank)
        self.m1.pubCv2(mb)
        self.m2.pubCv2(diliated)
        self.m3.pubCv2(hsv)


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
