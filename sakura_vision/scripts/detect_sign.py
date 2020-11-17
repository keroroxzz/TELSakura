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
from yolo import darknet

from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from sakura_common.moniter import Moniter,bridge,cvShow

###################################
# Classes
###################################
class Yolov4():

    index = 0
    state = 0
    detecting = False
    last_time = -1
    msg = ['RedBallon','BlueBallon','GreenBallon','Y','E','S']

    def __init__(self, startFrom=0):

        self.matchMoni = Moniter('Yolov4')

        self.img_sub = rospy.Subscriber('/sakura/camera_image/image',Image,self.imageCallback, queue_size=1)
        self.img_sub = rospy.Subscriber('/sakura/detection_state', Int32, self.stateCallback, queue_size=1)

    def stateCallback(self, msg):
        self.state = msg.data

    def imageCallback(self,data):
        if self.state==0 or self.detecting==True or rospy.get_time()-self.last_time<0.5:
            return

        self.data = data

        threading.Thread(target = self.detect_thread).start()

    def detect_thread(self):

        self.detecting = True
        cv_image = bridge.imgmsg_to_cv2(self.data, "bgr8")
        boxes, self.img = darknet.detect(cv_image)

        self.detecting=False

        if len(boxes)>0:

            for box in boxes:
                if box[6]==4:
                    rospy.loginfo('E is found at {0:.2f}'.format(box[0]))

        self.last_time = rospy.get_time()
        self.matchMoni.pubCv2(self.img)


###################################
# MAIN
###################################

def main():
    rospy.init_node('Yolov4')

    yv4 = Yolov4()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
