#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np

def white_balance(img):
    imgt=img.copy()
    b,g,r=cv2.split(img)
    m,n,t=img.shape
    sum_=b+g+r
    Y=765
    hist,bins=np.histogram(sum_.flatten(),766,[0,766])
    num,key=0,0
    ratio=0.01
    while(Y>=0):
        num+=hist[Y]
        if (num>m*n*ratio/100):
            key=Y
            break
        Y=Y-1
    B=b[sum_>=key].sum()
    G=g[sum_>=key].sum()
    R=r[sum_>=key].sum()
    time=(sum_>=key).sum()

    avg_b=B/time
    avg_r=R/time
    avg_g=G/time
    maxv=float(np.max(imgt))
    b=img[:,:,0].astype(np.int32)*maxv/int(avg_b)
    g=img[:,:,1].astype(np.int32)*maxv/int(avg_g)
    r=img[:,:,2].astype(np.int32)*maxv/int(avg_r)
    b[b>255]=255
    b[b<0]=0
    g[g>255]=255
    g[g<0]=0
    r[r>255]=255
    r[r<0]=0
    imgt[:,:,0]=b
    imgt[:,:,1]=g
    imgt[:,:,2]=r
    return imgt

flip=True
def callback(data):
    global flip
    flip=data.data
pub=rospy.Publisher('/sakura/camera_image/image/compressed',CompressedImage,queue_size=10)
pub_img=rospy.Publisher('/sakura/camera_image/image',Image,queue_size=10)
sub=rospy.Subscriber('/sakura/sakura_state',Bool,callback,queue_size=1)
rospy.init_node('talker',anonymous=True)
cap=cv2.VideoCapture(0)
if(not cap.isOpened()):
  cap.open()

bridge=CvBridge()
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_FPS,30)
ret,img=cap.read()
while (not rospy.is_shutdown()):
  ret,img=cap.read()
  #print(flip)
  if(not flip):
      img=cv2.flip(img,-1)
  pub.publish(bridge.cv2_to_compressed_imgmsg(img))
  pub_img.publish(bridge.cv2_to_imgmsg(img))
  ##cv2.imshow('aaa',img)
  ##cv2.waitKey(1)


