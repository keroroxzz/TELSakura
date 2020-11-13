#Frame
#Author : RTU
#Version : 1.2
#date : 2020/4/8

import sys,rospy
from supply.agent import Client
from abc import ABCMeta, abstractmethod


''''''

class Frame:
	__metaclass__=ABCMeta

	callMaster = None
	initialized_ = False
	running = False
	Subscribers = []

	def initFrame(self, name):

		self.client = Client(self.masterCall, name)
		self.client.setInstance(self)

		if len(sys.argv)>1 and sys.argv[1] == 'test':
			self.activate(True)


	def masterCall(self, client, msg, instance):
		if msg == 'STATE':
			client.post('WORKING' if self.initialized_ == true else 'SLEEPING')

		elif msg == 'ACTIVATE':
			self.activate()

		elif msg == 'KILL':
			self.kill()

		elif msg == 'START':
			self.start()

		elif msg == 'STOP':
			self.stop()

		self.masterCall_custom(client, msg, instance)

	def masterCall_custom(self, client, msg, instance):
		pass

	@abstractmethod
	def setSubscribers(self):
		pass

	def kill_injector(self):
		pass

	def start_injector(self):
		pass

	def stop_injector(self):
		pass

	def start(self):
		self.running = True
		self.start_injector()
		rospy.loginfo('Start!')

	def stop(self):
		self.running = False
		self.stop_injector()
		rospy.loginfo('Stop!')

	def activate(self, run = False):
		if len(self.Subscribers) == 0:
			self.setSubscribers()
			rospy.loginfo('Activate!')

			if run == True:
				self.start()
		else:
			rospy.logwarn('The subscriber has already been activated!')

	def kill(self):
		if not len(self.Subscribers) == 0:
			for sub in self.Subscribers:
				sub.unregister()
			self.Subscribers = []
			self.kill_injector()
			rospy.loginfo('Unactivate!')
		else:
			rospy.logwarn('The subscriber has already been stopped!')

	def isActive(self):
		return not len(self.Subscribers) == 0