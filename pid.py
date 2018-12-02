import numpy as np
import matplotlib.pyplot as plt

class Pid:
	def __init__(self,kp,ki=0,kd=0,dt=0.1):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.dt = dt

		self.wheel_i = 0.0002
		self.wheel_r = 0.05
		self.m = 0.1

		#robot start from rest
		self.t = 0
		self.s = 0
		self.v = 0
		self.a = 0
		self.u = 0
		self.s_ref = 40
		self.old_e = 0
		self.e = 0
		self.de = 0
		self.ie = 0
		self.f = 0.01

		self.history_s = []
		self.history_t = []
		self.history_u = []
		self.history_e = []

	def __str__(self):
		return "robot at s = "+str(self.s)+", v = "+str(self.v)+", a = "+str(self.a)+", t = "+str(self.t)+", u = "+str(self.u)

	def set_target(self,target_s):
		self.s_ref = target_s

	def update_a(self):
		self.a = (self.u-self.f*self.v**2)/(self.m+(self.wheel_i/self.wheel_r))

	def update_v(self):
		self.v += self.a*self.dt

	def update_s(self):
		self.s += self.v*self.dt

	def update_error(self):
		self.old_e = self.e
		self.e = self.s_ref-self.s

	def update_derror(self):
		self.de = (self.e - self.old_e)/self.dt

	def update_ierror(self):
		self.ie += self.e*self.dt

	def update_u(self):
		self.u = self.kp*self.e + self.ki*self.ie + self.kd*self.de

	def step(self):
		u = self.update_u()
		a = self.update_a()
		v = self.update_v()
		s = self.update_s()
		e = self.update_error()
		de = self.update_derror()
		ie = self.update_ierror()
		self.history_s.append(self.s)
		self.history_t.append(self.t)
		self.history_u.append(self.u)
		self.history_e.append(self.e)
		self.t += self.dt

	def run(self,n_iter = 5000):
		self.history_s = []
		self.history_t = []
		self.history_u = []
		self.history_e = []
		for i in range(n_iter):
			self.step()

	def plot(self):
		if len(self.history_s) == 0 or len(self.history_t) == 0:
			print("can't plot, history empty")
		if len(self.history_s) != len(self.history_t):
			print("lengths not equal, s is "+str(len(self.history_s))+ " t is "+str(len(self.history_t)))

		plt.figure()
		plt.plot(self.history_t,self.history_s)
		plt.show()