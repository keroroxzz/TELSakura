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
            self.arm_lock=False
            self.ARM.set_tohome()
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

class robot_arm():
    def __init__(self):
        self.theta0=0
        self.theta1=0
        self.theta2=0
        self.theta3=0
        self.theta4=0
        self.l1=104.3
        self.l2=91
        self.l3=71
        self.x=0.1
        self.y=50
        self.z=self.l1+50
        self.gr=0
        self.arm_base = rospy.Publisher('/sakura/arm/base/command', Float64, queue_size=10)
        self.arm_j1 = rospy.Publisher('/sakura/arm/j1/command', Float64, queue_size=10)
        self.arm_j2 = rospy.Publisher('/sakura/arm/j2/command', Float64, queue_size=10)
        self.arm_j3 = rospy.Publisher('/sakura/arm/j3/command', Float64, queue_size=10)
        self.arm_j4 = rospy.Publisher('/sakura/arm/j4/command', Float64, queue_size=10)

        #Gripper (both 0.0 ~ 0.025)
        self.arm_f1 = rospy.Publisher('/sakura/arm/f1/command', Float64, queue_size=10)
        self.arm_f2 = rospy.Publisher('/sakura/arm/f2/command', Float64, queue_size=10)
        self.f1=0
        self.f2=0
        self.bigR=self.l1+self.l2-0.1

        self.s0=0
        self.s1=0
        self.s2=0
        self.s3=0
        self.s4=0
        self.t0=0
        self.t1=0
        self.t2=0
        self.t3=0
        self.t4=0
        self.stime=0
        self.Ltime=0
        sleep(3)
    def callbackxy(self,X,Y):
        self.set(self.x+X,self.y+Y,self.z,self.theta3)
    def callbackzP(self,Z,phi):
        if(self.theta3+phi >-PI/2 and self.theta3+phi <PI/2):
            self.set(self.x,self.y,self.z+Z,self.theta3+phi)
        else:
            self.set(self.x,self.y,self.z+Z,self.theta3)

    def mgrep(self,g):
        if(g==True):
            self.seto(self.theta4,0.5)
        else:
            self.seto(self.theta4,0)
    def auto_grep(self):
        self.settarget(t0=0.45,t1=0.8,t2=1.3,t3=-0.1,t4=0,tf=0,TIME=2)
        self.settarget(t0=0.45,t1=1.2,t2=1.4,t3=-0.3,t4=0,tf=0,TIME=1)
        self.grep()
        self.set_tohome2(TF=1.5)
        

    def Rot(self,rot):
        if(self.theta4+rot >-PI/2 and self.theta4+rot< PI/2):
            self.seto(self.theta4+rot,self.gr)

    def set(self,X,Y,Z,phi):
        if(X**2+Y**2+Z**2>self.bigR**2):
            RRR=(X**2+Y**2+Z**2)
            X=X/RRR
            Y=Y/RRR
            Z=Z/RRR
            X=X*self.bigR
            Y=Y*self.bigR
            Z=Z*self.bigR
            return
        r=math.sqrt(X**2+Y**2)
        #print((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/self.l2)
        #print('y:'+str(Y)+' '+str(self.y))
        self.theta2=math.acos((r*r +Z*Z-self.l1**2-self.l2**2)/2/self.l1/self.l2)
        phix=math.acos((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/math.sqrt(r**2+Z**2))
        if(self.theta2>0):
            self.theta1=PI/2-phix-math.atan2(Z,r)
        else:
            self.theta1=PI/2+phix-math.atan2(Z,r)
        self.theta3=phi
        self.theta0=math.asin(X/r)
        self.x=X
        self.y=Y
        self.z=Z
        self.Contr()
    def set_tohome(self,X=0.1,Y=50,Z=104.3+50,TF=0.0):
        if(X**2+Y**2+Z**2>self.bigR**2):
            RRR=(X**2+Y**2+Z**2)
            X=X/RRR
            Y=Y/RRR
            Z=Z/RRR
            X=X*self.bigR
            Y=Y*self.bigR
            Z=Z*self.bigR
            return
        r=math.sqrt(X**2+Y**2)
        #print((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/self.l2)
        #print('y:'+str(Y)+' '+str(self.y))
        t2=math.acos((r*r +Z*Z-self.l1**2-self.l2**2)/2/self.l1/self.l2)
        phix=math.acos((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/math.sqrt(r**2+Z**2))
        if(t2>0):
            t1=PI/2-phix-math.atan2(Z,r)
        else:
            t1=PI/2+phix-math.atan2(Z,r)
        t0=math.asin(X/r)
        t3=0
        self.x=X
        self.y=Y
        self.z=Z
        self.settarget(t0=(1.57+t0)/2,t1=(t1+1.25)/2,t2=0,t3=(t3+1.57)/2,t4=0.0,tf=0.5,TIME=3)
        self.settarget(t0=t0,t1=t1,t2=t2,t3=t3,t4=0,tf=TF,TIME=3)
    def set_tohome2(self,X=0.1,Y=50,Z=104.3+50,TF=0.0):
        if(X**2+Y**2+Z**2>self.bigR**2):
            RRR=(X**2+Y**2+Z**2)
            X=X/RRR
            Y=Y/RRR
            Z=Z/RRR
            X=X*self.bigR
            Y=Y*self.bigR
            Z=Z*self.bigR
            return
        r=math.sqrt(X**2+Y**2)
        #print((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/self.l2)
        #print('y:'+str(Y)+' '+str(self.y))
        t2=math.acos((r*r +Z*Z-self.l1**2-self.l2**2)/2/self.l1/self.l2)
        phix=math.acos((self.l2**2-(r**2+Z**2)-self.l1**2)/-2/self.l1/math.sqrt(r**2+Z**2))
        if(t2>0):
            t1=PI/2-phix-math.atan2(Z,r)
        else:
            t1=PI/2+phix-math.atan2(Z,r)
        t0=math.asin(X/r)
        t3=0
        self.x=X
        self.y=Y
        self.z=Z
    def lock(self):
        self.settarget(t0=1.2,t1=0.95,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=2)
        self.settarget(t0=1.57,t1=1.05,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=2)
        self.settarget(t0=1.57,t1=1.25,t2=0,t3=1.57,t4=0.0,tf=0.5,TIME=3)

    def seto(self,t4,gr):
        self.theta4=t4
        self.f1=gr
        self.f2=gr
        self.Contr()
    def Contr(self):
        self.arm_base.publish(self.theta0)
        self.arm_j1.publish(-self.theta1)
        self.arm_j2.publish(-self.theta2)
        self.arm_j3.publish(self.theta3)
        self.arm_j4.publish(self.theta4)
        self.arm_f1.publish(self.f1)
        self.arm_f2.publish(self.f2)
        #print(str(self.theta1)+" "+str(self.theta2)+" "+str(self.theta3)+' '+str(self.x)+' '+str(self.y)+' '+str(self.z))
        global rtd
        #print(self.theta0*rtd,' ',self.theta1*rtd,' ',self.theta2*rtd,' ',self.theta3*rtd,' ',self.theta4*rtd,' ',self.f1*rtd)
        #print(self.theta1*rtd)
    def settarget(self,t0,t1,t2,t3,t4,tf,TIME):
        self.s0=self.theta0
        self.s1=self.theta1
        self.s2=self.theta2
        self.s3=self.theta3
        self.s4=self.theta4
        self.sf=self.f1
        self.t0=t0
        self.t1=t1
        self.t2=t2
        self.t3=t3
        self.t4=t4
        self.tf=tf
        self.Ltime=TIME
        self.stime=timeit.default_timer()

        self.running()
    def setcurrent(self,t0,t1,t2,t3,t4,tf):
        self.theta0=t0
        self.theta1=t1
        self.theta2=t2
        self.theta3=t3
        self.theta4=t4
        self.tf=tf
    def running(self):
        while True:
            if self.goto():
                break
    def grep(self):
        self.f1=1.0
        self.f2=0.5
        self.Contr()
        sleep(1)
    def ungrep(self):
        self.f1=0
        self.f2=0
        self.Contr()
        sleep(1)
    def goto(self):
        Ctime=timeit.default_timer()
        delta_t=Ctime-self.stime 

        self.middle_cal(self.s0,self.t0,delta_t,0)
        self.middle_cal(self.s1,self.t1,delta_t,1)
        self.middle_cal(self.s2,self.t2,delta_t,2)
        self.middle_cal(self.s3,self.t3,delta_t,3)
        self.middle_cal(self.s4,self.t4,delta_t,4)
        self.middle_cal(self.sf,self.tf,delta_t,5)
        self.Contr()
        
        if(delta_t>=self.Ltime):
            return True
        else:
            return False
    def middle_cal(self,start,target,dt,num):
        T=np.matrix([[1,0,0,0],[1,self.Ltime,self.Ltime**2,self.Ltime**3],[0,1,0,0],[0,1,2*self.Ltime,3*self.Ltime**2]])
        THETA=np.matrix([[start],[target],[0],[0]])
        
        T_=np.linalg.inv(T)
        A=T_*THETA
        '''
        print('inv T')
        print(T_)
        print('A:')
        print(A)
        print('THETA')
        print(THETA)
        '''
        if(num==0):
            self.theta0=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
        elif(num==1):
            self.theta1=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
        elif(num==2):
            self.theta2=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
        elif(num==3):
            self.theta3=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
        elif(num==4):
            self.theta4=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
        elif(num==5):
            self.f1=A[0,0]+A[1,0]*dt+A[2,0]*dt**2+A[3,0]*dt**3
   
if __name__=='__main__':
    C=control()
    rospy.spin()
