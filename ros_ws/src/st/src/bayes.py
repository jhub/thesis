#!/usr/bin/env python
import rospy
import sys
import tf
import pudb
import math

import numpy as np

from Particle_Filter 	import Particle_Filter
from time 				import time
from threading 			import RLock

from obj_tr_constants	import d_limits, array_size, g_func_v_p, g_res_fn,h_function, R, Q

dest_macs_lock		= RLock()

class compromized_state(object):

	current_behv 	= np.array([0,0]) #Initialize all particles to stationary TODO: if no behavior come in but sensor data does (must use random)
	stationary 		= True
	last_time 		= time()

	def __init__(self, initial_prob, init_guess = None):
		self.c_prob 			= initial_prob
		self.init_prob 			= self.c_prob 
		self.particle_filter 	= Particle_Filter(d_limits, array_size, g_func_v_p, g_res_fn,h_function, R, Q, init_guess)
		self.lock 				= RLock()
		self.last_time 			= time()

	def update_prob(self, comp_prob, uncomp_prob):
		self.c_prob = comp_prob * self.c_prob / (comp_prob * self.c_prob + uncomp_prob * (1 - self.c_prob))
		self.adjust_prob()


	def adjust_prob(self):
		if self.c_prob < 0.0001:
			self.c_prob = .0001
		elif self.c_prob > 0.9999:
			self.c_prob = 0.9999
		elif math.isnan(self.c_prob):
			print "NaN cought"
			self.c_prob = self.init_prob


	def get_c_prob(self):
		return self.c_prob

	'''
	updates the behavior to be used as control
	'''
	def upd_PF_behv(self,behv):
		with self.lock:
			self.current_behv = behv
			self.stationary = False


	'''
	Getter behv control
	'''
	def PF_behv(self):
		return self.current_behv

	'''
	Updates the particle filter based on the sensor model input
	'''
	def upd_PF_sens(self, sensor_point):
		mean = self.particle_filter.upd_points(sensor_point)
		return mean

	'''
	Updates the particle filter based on the motion model
	'''
	def get_PF_state(self):
		with self.lock:
			current_behv = self.current_behv
		#if not self.stationary:
		self.particle_filter.move_all(current_behv,self.get_duration())
		return self.particle_filter.get_pt_list()

		
	def get_duration(self):
		tmp 			= time()
		dur 			= tmp - self.last_time
		self.last_time 	= tmp
		return dur
