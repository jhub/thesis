#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy as np

from object_tracker.msg import behv, state, st_beh
from time 				import sleep
from math 				import pi
from obj_tr_constants	import bc_interval
from obj_tr_constants	import g_func_v_p

x,y,th,v_x, v_th = 0,1,2,3,4
a_x,a_th 		= 0,1
data, pred_beh 	= 0,1
st_ind, ctl_ind = 0,1

'''
Used to obtain the state/behavior lists and the map updated from those lists
'''
def get_full_planned(g_func,map_beh, curr_state, beh_dur_list, dt):
	state_beh_list = []
	for beh_dur in beh_dur_list:
		state_beh_list.append([curr_state[:], beh_dur[:]])
		curr_state = g_func(curr_state,beh_dur,dt)
	return state_beh_list


'''
Creates a behavior list 
'''
def get_sample_construct(sample_list, dt):
	map_beh 		= [[],[]]
	beh_list 	= []

	for bh in sample_list:
		for itr in range(int(bh[1]/dt)):
			beh_list.append(bh[0])
	
	return beh_list


def load_beh_lists():
	un_comp 				= [ [[0,0],1], [[.5,0],2],[[0,0],2],[[0,-pi/8],2],[[0,pi/4],2],[[0,-pi/8],2],[[0,0],4],[[0,pi/8],2],[[0,-pi/4],2],[[0,pi/8],2],[[0,0],2],[[-.5,0],2],[[0,0],1] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[0,0],1], [[.5,0],2],[[0,0],18],[[-.5,0],2],[[0,0],1] ]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u


def publish_state_beh(state_beh_list, bc_interval):
	for st_bh in state_beh_list:
		if not rospy.is_shutdown():
			s 		= state()
			s.x 	= st_bh[st_ind][x] 	
			s.y		= st_bh[st_ind][y] 	
			s.th 	= st_bh[st_ind][th] 	
			s.v_x	= st_bh[st_ind][v_x] 
			s.v_th	= st_bh[st_ind][v_th]
			s.MAC 	= "K1" 
			print s

			pub_state.publish(s)

			b 		= behv()
			b.a_x 	= st_bh[ctl_ind][a_x] 
			b.a_th	= st_bh[ctl_ind][a_th]
			b.MAC 	= "K1" 

			pub_behv.publish(b)

			#sb 			= st_beh()
			#sb.MAC 		= "K1"
			#sb.state 		= s 
			#sb.beh 		= b

			#pub_st_beh.publish(sb)
			
			sleep(bc_interval)

		else : break


if __name__ == '__main__':

	rospy.init_node('obj_trck', anonymous=True)

	pub_state 				= rospy.Publisher('state_k1', state, queue_size=10)
	pub_behv				= rospy.Publisher('beh_k1', behv, queue_size=10)
	pub_st_beh 				= rospy.Publisher('st_beh_k1', st_beh, queue_size=10)

	init_state 				= [0.0,0.0,0.0,0.0,0.0]
	map_beh 				= [[],[]]
	#pudb.set_trace() #For Debugging
	beh_list				= load_beh_lists()

	state_beh_list			= get_full_planned(g_func_v_p,map_beh,init_state,beh_list,bc_interval)

	for s in state_beh_list:
		print s

	publish_state_beh(state_beh_list, bc_interval)