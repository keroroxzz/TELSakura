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
        self.settarget(t0=0.28,t1=1.5,t2=1.2,t3=-0.9,t4=0,tf=0,TIME=2)
        self.settarget(t0=0.28,t1=1.5,t2=0.7,t3=-0.1,t4=0,tf=0,TIME=1)
        self.grep()
        self.set_tohome(TF=1.5)
        

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
        self.settarget(t0=t0,t1=t1,t2=t2,t3=t3,t4=0,tf=TF,TIME=1.5)
    def lock(self):
        self.settarget(t0=1.3,t1=1.5,t2=0.5,t3=-1.4,t4=0.0,tf=0.5,TIME=1.5)

    def seto(self,t4,gr):
        self.theta4=t4
        self.f1=gr
        self.f2=gr
        self.Contr()
    def Contr(self):
        self.arm_base.publish(self.theta0)
        self.arm_j1.publish(self.theta1)
        self.arm_j2.publish(self.theta2)
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
        self.f1=0.5
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
