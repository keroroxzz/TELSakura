#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64,Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist
from time import sleep
import numpy as np
import cv2
from sakura_common.xh_arm import robot_arm

state=0 #start 0
#back until find blue2 1
##small blue get 2
#big blue go 3
#small green getted 4
#big green go 5
#back 6
#escape 7 
rot_state=0 # run 0
# down 1
count=0
yaw=0
goal_yaw=1.570
start=0
pEx=0
STATUS=0
R=robot_arm()

def publish(pub,T):
    T.linear.x=T.linear.x*0.1
    T.linear.y=T.linear.y*0.1
    T.angular.z=T.angular.z*0.1
    pub.publish(T)

def status_callback(data):
    global STATUS
    STATUS=data.data

def callback(data):
  global STATUS
  if(STATUS!=2):
    return
    
  global start
  grep()
  if(start<10):
    rot.publish(0.2)
    sleep(0.1)
    start=start+1
  img=bridge.compressed_imgmsg_to_cv2(data,'bgr8')
  threshold_method(img)

def threshold_method(img):
  h,w,c=img.shape
  b,g,r=cv2.split(img)
  output1=threshold_color(b,g,r,0,1,-1,False)
  output2=threshold_color(g,b,r,3,4,2,True)
  output=cv2.addWeighted(output1,1.0,output2,1.0,0)
  if(state>=5):
    escape(output2,w,h)
  #cv2.imshow('test0',output1)
  #cv2.imshow('test1',output2)
  #cv2.imshow('ttt',img)
  #cv2.imshow('t',output)
  #cv2.waitKey(5)

def grep():
  R.settarget(t0=0.45,t1=0.6,t2=1.4,t3=0.5,t4=0,tf=0,TIME=3)
  R.settarget(t0=0.45,t1=0.9,t2=1.4,t3=0.6,t4=0,tf=0,TIME=2)
  R.grep()
  R.settarget(t0=0,t1=0,t2=0,t3=0,t4=0,tf=0.5,TIME=3)
  sleep(100)
  
def throw():
  R.settarget(t0=0.4,t1=0.5,t2=0.4,t3=0.3,t4=0,tf=0,TIME=2)
  R.ungrep()
  R.settarget(t0=0,t1=0,t2=0,t3=0,t4=0,tf=0,TIME=2)

def escape(img,w,h):
  contours,hierachy=cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  area=[]
  for i in range(len(contours)):
    area.append(cv2.contourArea(contours[i]))
  Id=np.argsort(np.array(area))
  Id=Id[::-1]
  if(state==7):
    if(len(contours)==0):
      print("END")
    else:
      gocontrol(50,50,True)
  elif(state==6):
    if(len(contours)>0):
      print(cv2.contourArea(contours[Id[0]]))
      if(cv2.contourArea(contours[Id[0]])>500):
        x2,y2=centroid(contours[Id[0]])
        gocontrol(x2,25,False)
      else:
        global state
        state=state+1
        print('next'+str(state))
  elif(state==5):
    back_until_far(contours,Id,w,h)
 
    

def threshold_color(target,no1,no2,state1=-1,state2=-1,state3=-1,isGreen=True):
  h,w=target.shape
  #var=np.var(A.ravel())
  #if(var>50):
   # ret,output=cv2.threshold(A,0,255,cv2.THRESH_OTSU+cv2.THRESH_BINARY)
  #else:
   # ret,output=cv2.threshold(A,255,255,cv2.THRESH_BINARY)
  if(isGreen):
    B=np.maximum(no1,no2)
    C=np.minimum(no1,no2)
    A=cv2.subtract(target,B)
    A=cv2.medianBlur(A,5)
    ret,output=cv2.threshold(A,10,255,cv2.THRESH_BINARY)
  else:
    B=np.maximum(no1,no2)
    C=np.minimum(no1,no2)
    A=cv2.subtract(target,B)
    A=cv2.medianBlur(A,5)
    ret,output=cv2.threshold(A,20,255,cv2.THRESH_BINARY)
  if(state!= state1 and state!=state2 and state!=state3):
   return output

  contours,hierachy=cv2.findContours(output.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  area=[]
  for i in range(len(contours)):
    area.append(cv2.contourArea(contours[i]))
  Id=np.argsort(np.array(area))
  Id=Id[::-1]
  if(state==state1):
   search_small(contours,Id,w,h)
  elif(state==state2):
   search_big(contours,Id,w,h)
  elif(state==state3):
   back_until(contours,Id,w,h)
  return output

def centroid(cnt):
  M=cv2.moments(cnt)
  cx=int(M['m10']/M['m00'])
  cy=int(M['m01']/M['m00'])
  return cx,cy

def search_small(contours,Id,w,h):
  if(len(contours)>1):
    x1,y1=centroid(contours[Id[1]])
    control(x1,y1,w,h,158)
  elif((len(contours)>0 and cv2.contourArea(contours[Id[0]])<2000)):
    x1,y1=centroid(contours[Id[0]])
    control(x1,y1,w,h,158)
  else:
    print(str(len(contours))+'objects')
    control(10,0,20,100,20)
def search_big(contours,Id,w,h):
  if(len(contours)>1 or ((len(contours)>0) and (cv2.contourArea(contours[Id[0]])>3000))):
    x2,y2=centroid(contours[Id[0]])
    print(cv2.contourArea(contours[Id[0]]))
    if(cv2.contourArea(contours[Id[0]])>23000):
      stop()
      global state
      throw()
      state=state+1
      print('next'+str(state))
    else:
      control(x2,y2,w,h,200)
  else:
    T=Twist()
    if(state==1):
       T.angular.z=0.4
    elif(state==4):
       T.angular.z=-0.4
    publish(pub,T)
    
    

def back_until(contours,Id,w,h):
  print(str(len(contours))+' objectes ')
  if(len(contours)>1):
    print(cv2.contourArea(contours[Id[1]]))
    global count
    count=count+1
    if(count>5):
      global state
      state=state+1
      print('next'+str(state))
  else:
    global count
    count=0
    T=Twist()
    k2=-0.03
    T.linear.x=k2*5
    T.angular.z=0
    publish(pub,T)

def back_until_far(contours,Id,w,h):
   if(len(contours)>0 and cv2.contourArea(contours[Id[0]])>40000):
     T=Twist()
     T.linear.x=-0.15
     publish(pub,T)
   else:
     global count
     count=count+1
     if(count>15):
       global state
       state=state+1
       print('next'+str(state))

def back_until_escape(contours,Id,w,h):
  print(str(len(contours))+' objectes')
  if(len(contours)>1 and cv2.contourArea(contours[Id[0]])>2000 and cv2.contourArea(contours[Id[1]])):
    global count
    count=count+1
    if(count>10):
      global state
      state=state+1
      print('next'+str(state))
  else:
    T=Twist()
    k2=-0.03
    T.linear.x=k2*5
    T.angular.z=-0.05
    publish(pub,T)

Ei=0
def gocontrol(x,xw,Control):
  k2=-0.005
  T=Twist()
  ex=5.0
  ey=5.0
  k=-0.035
  kd=0.003
  ki=-0.001
  Ex=x-xw
  dExdt=Ex-pEx
  PEx=Ex
  if(Control):
    T.linear.x=0.2
  else:
    if(abs(x-xw)>ex):
      T.angular.z=(-(x-xw)/abs(x-xw)*(0.7))
      print('left-right'+str((k*(x-xw)+kd*(dExdt)+ki*Ei)))
    else:
      global state
      state=state+1
      print('next'+str(state))  
  publish(pub,T)

def control(x,y,w,h,gh):
  k=-0.035
  kd=0.006
  k2=-0.005
  T=Twist()
  ex=5.0
  ey=5.0
  Ex=x-w/2.0
  global pEx
  dExdt=Ex-pEx
  PEx=Ex
  if(abs(x-w/2)>ex):
    if(abs((k*(x-w/2)+kd*(dExdt)))>0.7):
      T.angular.z=(-(x-w/2.0)/abs(x-w/2.0)*(0.7))
    else:
      T.angular.z=(k*(x-w/2)+kd*(dExdt))
    print('left-right'+str((k*(x-w/2)+kd*(dExdt))))
  if(abs((y-gh))>ey):
    if(abs(y-gh)>0.5):
      T.linear.x=(-(y-gh)/abs(y-gh)*(0.5))
    else:
      T.linear.x=(k2*(y-gh))
  print('go'+str(y-gh))
  if(abs(x-w/2.0)<=ex and abs(y-gh)<=ey):
    T.linear.x=0
    T.angular.z=0
    publish(pub,T)
    global state
    global rot_state
    if(state==0 or state==3):
      if(rot_state<100):
        rot.publish(0.4)
        rot_state=rot_state+1
        print(rot_state)
      else:
        rot.publish(0.2)
        grep()
        rot_state=0
        state=state+1
    print(rot_state)
    print('next'+str(state))
    pEx=0
  publish(pub,T)

def stop():
  T=Twist()
  publish(pub,T)

def tcontrol(x,y,w,h,gh):
  k=-0.02
  kd=0.003
  k2=-0.005
  T=Twist()
  ex=7.5
  ey=9.0
  Ex=x-w/2.0
  global pEx
  dExdt=Ex-pEx
  PEx=Ex
  if(abs(x-w/2)>ex):
    if(abs((k*(x-w/2)+kd*(dExdt)))>0.5):
      T.linear.y=(-(x-w/2.0)/abs(x-w/2.0)*(0.5))
    else:
      T.linear.y=(k*(x-w/2)+kd*(dExdt))
    print('left-right'+str((k*(x-w/2)+kd*(dExdt))))
  if(abs((y-gh))>ey):
    if(abs(y-gh)>0.5):
      T.linear.x=(-(y-gh)/abs(y-gh)*(0.5))
    else:
      T.linear.x=(k2*(y-gh))
  print('go'+str(y-gh))
  if(abs(x-w/2.0)<=ex and abs(y-gh)<=ey):
    T.linear.x=0
    T.angular.z=0
    publish(pub,T)
    global state
    global rot_state
    if(state==0 or state==3):
      if(rot_state<100):
        rot.publish(0.4)
        rot_state=rot_state+1
        print(rot_state)
      else:
        rot.publish(0.2)
        grep()
        rot_state=0
        state=state+1
    print(rot_state)
    print('next'+str(state))
    pEx=0
  publish(pub,T)


rospy.init_node('camera_getted',anonymous=True)
bridge=CvBridge()
sub=rospy.Subscriber('/sakura/camera_image/image/compressed',CompressedImage,callback,queue_size=1)
pub=rospy.Publisher('/sakura/cmd_vel',Twist,queue_size=10)
rot = rospy.Publisher('/sakura/rot/command', Float64, queue_size=5)
#Robot arm (all -pi/2 ~ +pi/2)
arm_base = rospy.Publisher('/sakura/arm/base/command', Float64, queue_size=10)
arm_j1 = rospy.Publisher('/sakura/arm/j1/command', Float64, queue_size=10)
arm_j2 = rospy.Publisher('/sakura/arm/j2/command', Float64, queue_size=10)
arm_j3 = rospy.Publisher('/sakura/arm/j3/command', Float64, queue_size=10)
arm_j4 = rospy.Publisher('/sakura/arm/j4/command', Float64, queue_size=10)

#Gripper (both 0.0 ~ 0.025)
arm_f1 = rospy.Publisher('/sakura/arm/f1/command', Float64, queue_size=10)
arm_f2 = rospy.Publisher('/sakura/arm/f2/command', Float64, queue_size=10)


status = rospy.Subscriber('/sakura/status', Int32,status_callback, queue_size=10)

rospy.spin()
