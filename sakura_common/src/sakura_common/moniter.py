#Moniter
#Author : RTU
#Version : 1.2
#date : 2020/4/8

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class Moniter:

	def __init__(self, name):
		self.image_pub = rospy.Publisher('/moniter_imgs/'+ name, Image, queue_size=1)

	def pubCv2(self, cv2_img, formate="bgr8"):
		self.image_pub.publish(bridge.cv2_to_imgmsg(cv2_img, formate))

def cvShow(name, img):
	cv2.namedWindow(name, cv2.WINDOW_NORMAL)
	cv2.imshow(name, img)
	cv2.waitKey(1)