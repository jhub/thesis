#!/usr/bin/env python
import rospy
import sys

import numpy 	as np

from math 		import pi
from random		import gauss

x,y,th 			= 0,1,2
v_x, v_th 		= 0,1
data, pred_beh 	= 0,1

bc_interval 	= 1

d_limits				= np.array([[-50,50],[-50,50],[-np.pi,np.pi]]) 																			#x,y,theta,v_x,v_r limits
array_size				= 200																													#How many particles we want

R						= np.array([[.01,.001,.001],[.001,.01,.001],[.001,.001,.03]])															#Motion model noise
Q						= np.array([[.5,.01,.02],[.01,.5,.02],[.01,.01,.2]])																	#Measurement noise

'''
Changes state based on the behv given
'''
def g_func_v_p(tmp_state, behv, dt, R = None):
	state 		= tmp_state[:]

	if behv[v_th] != 0:
		state[x] 		= state[x] - (behv[v_x] / behv[v_th]) * np.sin(state[th]) + (behv[v_x] / behv[v_th]) * np.sin(state[th] + behv[v_th] * dt)
		state[y] 		= state[y] + (behv[v_x] / behv[v_th]) * np.cos(state[th]) - (behv[v_x] / behv[v_th]) * np.cos(state[th] + behv[v_th] * dt)
		state[th] 		= state[th] + behv[v_th] * dt

	else:
		state[x] 		= state[x] + np.cos(state[th]) * dt * behv[v_x]
		state[y] 		= state[y] + np.sin(state[th]) * dt * behv[v_x]
		state[th] 		= state[th]

	if R is not None:
		state				= np.diag(gauss(state, R)).copy()

	state[th] = rot_diff(state[th])

	return state


'''
Residual for motion model
'''
def g_res_fn(z, mean):

	residual 	= z[0:3] - mean

	residual[2] = rot_diff(residual[2])

	return residual

def rot_diff(rot_in):
	while rot_in > pi:
		rot_in -= 2 * pi
	while rot_in < -pi:
		rot_in += 2 * pi
	return rot_in

'''
Obtaineds the observation data from our input data
'''
def h_function(mean):
	return mean[0:3]