#!/usr/bin/env python
import rospy
from time import sleep
from std_msgs.msg import Float64

import numpy as np
import math

class camrot:
  def __init__(self):
    self.rot = rospy.Publisher('/sakura/rot/command', Float64, queue_size=1)
    self.cam_y = rospy.Publisher('/sakura/cam_y/command', Float64, queue_size=1)

  def set(self,rot=None, cy=None):
    if rot is not None:
      self.rot.publish(rot)
    if cy is not None:
      self.cam_y.publish(cy)
     
