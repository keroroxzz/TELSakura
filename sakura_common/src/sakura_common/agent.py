#!/usr/bin/env python

#Master Message Agent
#Author : RTU
#Version : 1.2

import rospy
from std_msgs.msg import String

#msgs
INIT_MASTER = 'INIT_MASTER'
INIT_CLIENT = 'INIT_CLIENT'

class Server:
	def __init__(self, callback, address, instance=None, q_size_in=10, q_size_out=10):

		self.STATE = 'INITIALIZE'

		self.instance = instance
		self.callback = callback

		self.In = rospy.Subscriber('/messages/'+address+'_c', String, self.fromClient, queue_size=q_size_in)
		self.Out = rospy.Publisher('/messages/'+address+'_s', String, queue_size=q_size_out)

		#initlaizing...
		rospy.sleep(0.01)
		self.Out.publish(INIT_MASTER)

	def fromClient(self, data):

		'''if data.data == INIT_CLIENT:
			self.post(self.STATE)'''

		self.callback(msg = data.data, server = self, instance = self.instance)

	def post(self, msg):

		self.Out.publish(msg)

		if self.Out.get_num_connections() == 0:
			self.fromClient(String('LOST'))
			return False

		return True

class Client:
	def __init__(self, callback, address, instance=None, q_size_in=10, q_size_out=10):

		self.STATE = 'INITIALIZE'

		self.instance = instance
		self.callback = callback

		self.In = rospy.Subscriber('/messages/'+address+'_s', String, self.fromServer, queue_size=q_size_in)
		self.Out = rospy.Publisher('/messages/'+address+'_c', String, queue_size=q_size_out)

		#initlaizing...
		rospy.sleep(0.01)
		self.Out.publish(INIT_CLIENT)

	def fromServer(self, data):
		self.callback(msg = data.data, client = self, instance = self.instance)

	def post(self, msg):
		self.Out.publish(msg)
		
		if self.Out.get_num_connections() == 0:
			self.fromServer(String('LOST'))

	def setInstance(self, instance):
		self.instance = instance


if __name__ == '__main__':

	import sys
	def called(msg):
		print(msg)

	rospy.init_node('MasterTest')

	Server = Server(called,sys.argv[1])

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")