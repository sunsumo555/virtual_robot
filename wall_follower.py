import numpy as np
import matplotlib.pyplot as plt

class Wall_follower:
	def __init__(self,kp,ki=0,kd=0,dt=0.1):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.dt = dt

		self.z = 100
		self.ref_distance = 100

		self.track_width = 10
		self.theta = 0
		self.robot_z = 0
		self.vl = 0
		self.vr = 0
		self.ul = 0
		self.ur = 0
		self.t = 0

		self.e = 0
		self.de = 0
		self.ie = 0

		self.history_z = []
		self.history_t = []
		self.history_ref = []

	def __str__(self):
		return "robot at z = "+str(self.robot_z)+", theta = "+str(self.theta)

	def update_vl(self):
		self.vl = (self.vl+self.vr)/2.0 + 0.5*(self.kp*self.e + self.ki*self.ie + self.kd*self.de)

	def update_vr(self):
		self.vr = (self.vl+self.vr)/2.0 - 0.5*(self.kp*self.e + self.ki*self.ie + self.kd*self.de)

	def update_theta(self):
		self.theta += (self.vl-self.vr)/self.track_width*self.dt

	def update_z(self):
		self.robot_z += (self.vl+self.vr)/2.0*np.sin(self.theta)*self.dt

	def update_error(self):
		self.old_e = self.e
		self.e = self.ref_distance-self.robot_z

	def update_derror(self):
		self.de = (self.e - self.old_e)/self.dt

	def update_ierror(self):
		self.ie += self.e*self.dt

	def step(self):
		self.update_vl()
		self.update_vr()
		self.update_theta()
		self.update_z()
		self.update_error()
		self.update_derror()
		self.update_ierror()
		self.history_ref.append(self.ref_distance)
		self.history_z.append(self.robot_z)
		self.history_t.append(self.t)
		self.t += self.dt

	def run(self,n_iter = 100000):
		self.history_z = []
		self.history_t = []
		self.history_ref = []
		for i in range(n_iter):
			self.step()

	def plot(self):
		if len(self.history_z) == 0 or len(self.history_t) == 0:
			print("can't plot, history empty")
		if len(self.history_z) != len(self.history_t):
			print("lengths not equal, s is "+str(len(self.history_z))+ " t is "+str(len(self.history_t)))

		plt.figure()
		plt.plot(self.history_t,self.history_z)
		plt.plot(self.history_t,self.history_ref)
		plt.show()