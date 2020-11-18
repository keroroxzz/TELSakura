#!/usr/bin/env python
import rospy
from time import sleep
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import numpy as np
import control_key_constant 
from sakura_common.xh_arm import robot_arm
import tf.transformations
import math
import timeit

PI=3.14215926535
rtd=1/PI*180

class control:
   def __init__(self):
        self.shovel_current=0
        self.rol_arm_current=0
        self.trans=False
        self.AR=False
        rospy.init_node('Orientation_Listener',anonymous=True)
        self.sub=rospy.Subscriber("keydown",Int32,self.callback,queue_size=1,buff_size=2**5)
        self.sub2=rospy.Subscriber("xy_axis",Float32MultiArray,self.callback2,queue_size=3)
        self.pub=rospy.Publisher("/sakura/cmd_vel",Twist,queue_size=3)
        self.shovel = rospy.Publisher('/sakura/shovel/command', Float64, queue_size=3)
        self.rol_arm = rospy.Publisher('/sakura/roller/arm/command', Float64, queue_size=3)
        self.rol_rol= rospy.Publisher('/sakura/roller/roller', Float64, queue_size=3)
        self.rol_slid= rospy.Publisher('/sakura/roller/slider/command', Float64, queue_size=3)
        self.first=True
        self.arm_c=False
        self.init_=[0,0,0]
        self.rot = rospy.Publisher('/sakura/rot/command', Float64, queue_size=1)
        self.cam_y = rospy.Publisher('/sakura/cam_y/command', Float64, queue_size=1)
        self.sub3=rospy.Subscriber("orientation",Float64MultiArray,self.callback3,queue_size=1,buff_size=2**10)
        self.ARM=robot_arm()
        self.sakura_state=rospy.Publisher('/sakura/sakura_state',Bool,queue_size=1)
        self.arm_lock=False
   def callback(self,data):
    data=data.data
    if(data==control_key_constant.START):
        self.first=True
    
    if(data==-control_key_constant.SELECT):
        if(self.arm_c==True):
            self.arm_c=False
        else:
            self.arm_c=True
        self.sakura_state.publish(self.arm_c)
    if(data==control_key_constant.B):
        self.trans=True
    if(data==-control_key_constant.B):
        self.trans=False

    if(data==-control_key_constant.R3):
        if(self.arm_lock==True):
            self.ARM.set_tohome()
            self.arm_lock=False
        else:
            self.arm_lock=True
            self.ARM.lock()

    if(self.arm_c):
        if(self.arm_lock==False):
            if(data==control_key_constant.L1):
                self.ARM.Rot(0.1) 
            if(data==control_key_constant.L2):
                self.ARM.Rot(-0.1)
            if(data==control_key_constant.R2):
                self.ARM.mgrep(True)
            if(data==-control_key_constant.R2):
                self.ARM.mgrep(False)
            if(data==control_key_constant.R1):
                self.AR=True
            if(data==-control_key_constant.R1):
                self.AR=False
            if(data==control_key_constant.A):
                self.ARM.callbackzP(0,0.1)
            if(data==control_key_constant.Y):
                self.ARM.callbackzP(0,-0.1)
            if(data==control_key_constant.X):
                self.arm_lock=True
                self.ARM.auto_grep()
                self.arm_lock=False
    else:
        if(data==control_key_constant.Y):
            if(self.rol_arm_current<-0.1):
                self.rol_arm_current=self.rol_arm_current+0.1
                self.rol_arm.publish(self.rol_arm_current)
        if(data==control_key_constant.A):
            if(self.rol_arm_current>-1.49):
                self.rol_arm_current=self.rol_arm_current-0.1
                self.rol_arm.publish(self.rol_arm_current)
        if(data==control_key_constant.R1):
            self.rol_rol.publish(4.0)
        if(data==control_key_constant.R2):
            self.rol_rol.publish(-4.0)
        if(data==-control_key_constant.R2):
            self.rol_rol.publish(0)
        if(data==control_key_constant.L1):
            if(self.shovel_current>0.1):
                self.shovel_current=self.shovel_current-0.1
                self.shovel.publish(self.shovel_current)
        if(data==control_key_constant.L2):
            if(self.shovel_current<1.49):
                self.shovel_current=self.shovel_current+0.1
                self.shovel.publish(self.shovel_current)

   def callback2(self,data):
    arr=np.array(data.data)
    n=len(arr)
    X=arr[0]
    Y=arr[1]
    Z=arr[2]
    if(abs(X)>0.15):
        X=X-0.15*X/abs(X)
    if(abs(Y)>0.15):
        Y=Y-0.15*Y/abs(Y)
    # Z=Z/1*0.15
    T=Twist()
  
    if(self.AR==True and self.arm_c==True):
        arg1=-0.5*X/abs(X)
        arg2=0.5*Y/abs(Y)
        self.ARM.callbackxy(-X/10,-Y/10)
    elif(self.trans==True):
        T.linear.x=-Y/2.0
        T.linear.y=-X
    else:
        T.linear.x=-Y/2.0
        T.angular.z=-X
    
    if(self.arm_c==True and self.arm_lock==False):
        self.ARM.callbackzP(-Z/10,0)
    else:
        self.rol_slid.publish(Z)
    self.pub.publish(T)

   def callback3(self,data):
    quaternion=(data.data[0],data.data[1],data.data[2],data.data[3])
    euler=tf.transformations.euler_from_quaternion(quaternion,'syzx') #'syzx')
    global first
    global init_
    if(self.first==True):
      self.init_[0]=euler[0]
      self.init_[1]=euler[1]
      self.init_[2]=euler[2]
      self.first=False
    else:
      E=[0,0,0]
      E[0]=(euler[0]-self.init_[0])
      E[1]=(euler[1]-self.init_[1])
      E[2]=(euler[2]-self.init_[2])
      if(not self.arm_c):
          E[0]=(-E[0]+PI)
      if(self.arm_c):
          E[1]=-E[1]
          if(E[0]>PI/2):
              E[0]=-PI
      self.rot.publish(E[0])
      self.cam_y.publish(E[1])
      
      for i in range(3):
          E[i]=E[i]/PI*180
      #print(str(E[0])+' '+str(E[1])+' '+str(E[2]))

if __name__=='__main__':
    C=control()
    rospy.spin()
