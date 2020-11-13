#Timer for performance test
#Author : RTU
#Version : 1.0

import time

class perfmeter:
	names = []
	start = []
	total = []
	max_ = []
	min_ = []
	count = []

	def add(self,name):
		self.names.append(name)
		self.start.append(0.0)
		self.total.append(0.0)
		self.max_.append(-1.0)
		self.min_.append(-1.0)
		self.count.append(0.0)
		return self

	def reset(self,name='',id=-1):
		id = self.getId(id,name)
		self.name[id] = name
		self.start[id] = 0.0
		self.total[id] = 0.0
		self.max_[id] = -1.0
		self.min_[id] = -1.0
		self.count[id] = 0.0
		return self

	def getId(self,id,name):
		if id>=0:
			return id  
		else:
			try:
				return self.names.index(name)
			except:
				self.add(name)
				return len(self.names)-1
			

	def tickStart(self,name='',id=-1):
		id = self.getId(id,name)
		self.start[id]=time.clock()*1000.0

	def tickStop(self,name='',id=-1):
		end = time.clock()*1000.0
		id = self.getId(id,name)
		self.duration = end-self.start[id]

		self.total[id] += self.duration
		self.max_[id] = max(self.max_[id],self.duration)
		self.min_[id] = min(self.min_[id],self.duration) if self.min_[id]>=0.0 else self.duration
		self.count[id] += 1.0

		self.showData(name,id)

	def showData(self,name='',id=-1):
		id = self.getId(id,name)
		print('{0} avg:{1:.3f} max:{2:.3f} min:{3:.3f} ---- {4:.3f}'.format(self.names[id],self.total[id]/self.count[id],self.max_[id],self.min_[id],self.duration))

pm = perfmeter()
