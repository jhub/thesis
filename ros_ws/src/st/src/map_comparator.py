#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy 	as np

from visualization_msgs.msg import Marker, MarkerArray
from scipy 					import spatial, stats
from object_tracker.msg 	import behv, state, st_beh
from geometry_msgs.msg 		import Twist
from nav_msgs.msg			import Odometry
from time 					import sleep
from math 					import pi
from collections			import deque
from bayes 					import compromized_state
from random 				import gauss
from threading 				import Thread
from obj_tr_constants		import x,y,th,v_x, v_th, a_x,a_th, data, pred_beh, bc_interval
from obj_tr_constants		import g_func_v_p

MAP_MAX_NGBR 	= 100

st_var			= np.array([1,1,.4,.8,.4])
beh_var 		= np.array([.2,.2])
rand_len		= 200

COMPARE_SIZE 	= 2
state_queue		= deque([],COMPARE_SIZE) #in order to differentiate we collect last two results
twist_queue		= deque([],COMPARE_SIZE)

'''
Used to obtain the state/behavior lists and the map updated from those lists
'''
def get_full_planned(g_func,map_st_beh, curr_state, beh_list, dt):
	#pudb.set_trace() #For Debugging
	for beh_dur in beh_list:
		insert_noised(map_st_beh, curr_state, beh_dur)
		g_func(curr_state,beh_dur,dt)


def insert_noised(map_st_beh, curr_state, curr_beh):
	global st_var, beh_var#, rand_len
	if not len(map_st_beh[data]):
		map_st_beh[data] 		= np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T
		map_st_beh[pred_beh]	= np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T
	else:
		map_st_beh[data] 		= np.hstack([map_st_beh[data],np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T])
		map_st_beh[pred_beh] 	= np.hstack([map_st_beh[pred_beh],np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T])


'''
Loads the compromised and uncompromised maps
'''
def load_beh_lists():
	global bc_interval
	un_comp 				= [ [[0,0],1], [[.5,0],2],[[0,0],2],[[0,-pi/8],2],[[0,pi/4],2],[[0,-pi/8],2],[[0,0],4],[[0,pi/8],2],[[0,-pi/4],2],[[0,pi/8],2],[[0,0],2],[[-.5,0],2],[[0,0],1] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[0,0],1], [[.5,0],2],[[0,0],18],[[-.5,0],2],[[0,0],1] ]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u, beh_list_c


'''
Creates a behavior list 
'''
def get_sample_construct(sample_list, dt):
	map_st_beh 		= [[],[]]
	beh_list 	= []

	for bh in sample_list:
		for itr in range(int(bh[1]/dt)):
			beh_list.append(bh[0])
	
	return beh_list


def get_yaw(orientation):
	quaternion = (
		orientation.x,
		orientation.y,
		orientation.z,
		orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler[2]


def state_to_list(state_queue):
	sum_vx	= 0
	sum_rx 	= 0
	num_s = len(state_queue)

	for s in state_queue:
		sum_vx += s.position.x
		sum_rx += get_yaw(s.orientation)
	res_vx = sum_vx / num_s
	res_rz = sum_rx / num_s

	return [state_queue[-1].position.x, state_queue[-1].position.y, get_yaw(state_queue[-1].orientation), res_vx, res_rz]

def beh_to_list(twist_queue):
	sum_ax = 0
	sum_az = 0
	num_t = len(twist_queue)
	for t in twist_queue:
		sum_ax += t.linear.x
		sum_az += t.angular.z
	res_ax = sum_ax / num_t
	res_az = sum_az / num_t
	return [res_ax, res_az]

def get_mac(msg):
	return 12345 #TODO obtain mac.. possible with sec msgs. in simulation? CHANGE MSG TYPE


'''
Changes the control of the particle filter
'''
def beh_callback(twist):
	global k_list
	global twist_queue
	#pudb.set_trace() #For Debugging
	twist_queue.append(twist)
	if len(twist_queue) == COMPARE_SIZE:
		mac 	= get_mac(twist)
		beh 	= beh_to_list(twist_queue)
		k_list[mac].upd_PF_behv(beh)


'''
Sensor info coming in [x,y]
'''
def sns_callback(odom): #TODO if twist is already published obtain it instead
	global k_list
	global state_queue

	state_queue.append(odom.pose.pose)
	if len(state_queue) == COMPARE_SIZE:
		mac 		= get_mac(state)
		rcv_list	= state_to_list(state_queue)
		mean 		= k_list[mac].upd_PF_sens(rcv_list)
		publish_positions(mean, rcv_list)
		prob  		= get_comp_prob(mean, k_list[mac])
		if prob is not None:
			print "Likelyhood of being compromised is: " + str(prob) 


def get_comp_prob(state, bayes_obj):
	global k_list, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, MAP_MAX_NGBR

	u_results 		= kd_map_u.query(state,MAP_MAX_NGBR)
	c_results 		= kd_map_c.query(state,MAP_MAX_NGBR)

	u_prob			= compare_sb(state, bayes_obj.PF_behv(), u_results, map_st_beh_u)
	c_prob			= compare_sb(state, bayes_obj.PF_behv(), c_results, map_st_beh_c)

	try:
		bayes_obj.update_prob(c_prob, u_prob)
		return bayes_obj.get_c_prob()
		#print "compromised" if comp_prob > .9 else "Not compromised" if comp_prob < .1 else "Determining"
	except Exception:
		print "Not Found Mac"


def compare_sb(state, beh, kd_list, map_st_beh):
	if len(kd_list) > 0 and len(kd_list[1]) > len(map_st_beh[0]): #check dimensions for enough pts (non singular matrix)

		st_kernel 	= stats.gaussian_kde(map_st_beh[data]) 
		bh_kernel	= stats.gaussian_kde(map_st_beh[pred_beh]) 
		state_pdf 	= st_kernel.pdf(state)[0]			#Likelyhood of doing a recorded state
		beh_pdf 	= bh_kernel.pdf(beh)[0]				#Likelyhood of following a behavior prev done

		return state_pdf * beh_pdf
	raise Exception("Need more points")


'''
Updates the particle filter
'''
def bayes_upd(bayes_obj):
	UPD_FREQUENCY	= rospy.Rate(20)
	upd = 0
	while True:
		pointList = bayes_obj.get_PF_state()
		#pointlist can be used to display, but should not be used to determine prob
		if pointList is not None:
			publish_particles(pointList)

		UPD_FREQUENCY.sleep()


def publish_particles(pointList):

	markerArray = MarkerArray()

	for i in range(pointList.shape[0]):

		point 						= pointList[i]
		marker						= Marker()

		tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, point[2])

		marker.pose.position.x = point[x]
		marker.pose.position.y = point[y]
		marker.pose.position.z = 0

		marker.scale.x = 0.4
		marker.scale.y = 0.05
		marker.scale.z = 0.01

		marker.color.a = 1.0
		marker.color.r = .3
		marker.color.r = 1
		marker.color.b = .3

		marker.pose.orientation.x 	= tempQuaternion[0]
		marker.pose.orientation.y 	= tempQuaternion[1]
		marker.pose.orientation.z 	= tempQuaternion[2]
		marker.pose.orientation.w 	= tempQuaternion[3]

		marker.id 				= i
		marker.header.frame_id 	= "/my_frame"
		marker.header.stamp 	= rospy.Time.now()
		marker.action 			= marker.ADD

		markerArray.markers.append(marker)

	particle_pub.publish(markerArray)


def publish_positions(mean_prtcl, rcv_prtcl):
	mean_marker 	= get_marker(mean_prtcl, rand_len + 1, "/my_frame", [.3,.3,1])
	rcv_pos_marker 	= get_marker(rcv_prtcl, rand_len + 2, "/my_frame", [1,.3,.3])

	mean_pos.publish(mean_marker)
	rcv_pos.publish(rcv_pos_marker)


def get_marker(particle, ID, frame_id, color):
	marker						= Marker()

	marker.pose.position.x = particle[x]
	marker.pose.position.y = particle[y]
	marker.pose.position.z = 0

	tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, particle[th])

	marker.scale.x = 0.4
	marker.scale.y = 0.05
	marker.scale.z = 0.01

	marker.color.a = 1
	marker.color.r = color[0]
	marker.color.g = color[1]
	marker.color.b = color[2]

	marker.pose.orientation.x 	= tempQuaternion[0]
	marker.pose.orientation.y 	= tempQuaternion[1]
	marker.pose.orientation.z 	= tempQuaternion[2]
	marker.pose.orientation.w 	= tempQuaternion[3]

	marker.id 				= ID
	marker.header.frame_id 	= frame_id
	marker.header.stamp 	= rospy.Time.now()
	marker.action 			= marker.ADD

	return marker


def print_all(list_in):
	for i in list_in:
		for l in i:
			print tuple(l)


if __name__ == '__main__':
	global bc_interval, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, k_list, particle_pub
	rospy.init_node('camp_comp', anonymous=True)

	map_st_beh_u 			= [[],[]]
	map_st_beh_c 			= [[],[]]
	k_list 					= {}
	bc_interval				= 1

	init_state_u			= [0.0,0.0,0.0,0.0,0.0]
	init_state_c			= [0.0,0.0,0.0,0.0,0.0]
	#pudb.set_trace() #For Debugging
	beh_list_u, beh_list_c 	= load_beh_lists()

	get_full_planned(g_func_v_p,map_st_beh_u,init_state_u,beh_list_u,bc_interval)
	get_full_planned(g_func_v_p,map_st_beh_c,init_state_c,beh_list_c,bc_interval)


	#print_all([map_st_beh_u[0],map_st_beh_c[0]])

	kd_map_u = spatial.KDTree(map_st_beh_u[data].T)
	kd_map_c = spatial.KDTree(map_st_beh_c[data].T)

	#This is incoming conections, emulated hardcoded
	init_pos 		= np.array([0.0,0.0,0.0,0.0,0.0])
	k_list[12345] 	= compromized_state(0.05, init_pos)

	rospy.Subscriber("/turtlebot1/mobile_base/commands/velocity", Twist, beh_callback)
	rospy.Subscriber("/turtlebot1/odometry/filtered_discrete", Odometry, sns_callback)
	particle_pub 	= rospy.Publisher('pose_cloud', MarkerArray, queue_size=100)
	rcv_pos 		= rospy.Publisher('rcv_pos', Marker, queue_size=100)
	mean_pos 		= rospy.Publisher('mean_pos', Marker, queue_size=100)

	for MAC in k_list:
		listener_thread 			= Thread(target = bayes_upd, args = (k_list[MAC], ))
		listener_thread.setDaemon(True)
		listener_thread.start()

	print "Ready"

	rospy.spin()


	#publish_state_beh(state_beh_list, bc_interval)