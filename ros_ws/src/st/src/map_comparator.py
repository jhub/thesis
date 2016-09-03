#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy 	as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg 		import Twist
from nav_msgs.msg			import Odometry
from sensor_msgs.msg 		import ChannelFloat32
from scipy 					import spatial, stats
from time 					import sleep
from math 					import pi
from collections			import deque
from bayes 					import compromized_state
from random 				import gauss
from threading 				import Thread, RLock
from obj_tr_constants		import x,y,th,v_x, v_th, data, pred_beh, bc_interval
from obj_tr_constants		import g_func_v_p

curr_odom_lock 	= RLock()

MAP_MAX_NGBR 	= 50

COMPROMISED_STATE_CSV	= "uncomp_map.csv"
UNCOMPROMISED_STATE_CSV = "comp_map.csv"

st_var			= np.array([.2,.2,.3])
beh_var 		= np.array([.5,.5])
rand_len		= 200

COMPARE_SIZE 	= 3
INITIAL_PROB 	= 0.05
GAUSS_REPS 		= 10 	#1 + # of reps

UPD_FREQUENCY	= .1

last_odom 		= None

'''
Used to obtain the state/behavior lists and the map updated from those lists
'''
def get_full_planned(g_func,map_st_beh, init_state, beh_list, dt):
	#pudb.set_trace() #For Debugging
	global GAUSS_REPS
	curr_state = init_state
	for beh_dur in beh_list:
		insert_noised(map_st_beh, curr_state, beh_dur)
		curr_state = g_func(curr_state,beh_dur,dt)
	#pudb.set_trace() #For Debugging


def insert_noised(map_st_beh, curr_state, curr_beh):
	global st_var, beh_var#, rand_len
	global GAUSS_REPS
	for g in range(GAUSS_REPS - 1):
		if not len(map_st_beh[data]):
			map_st_beh[data] 		= np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T 									#[[[statex],[statey],[stateth]],[[behx][behth]]
			map_st_beh[pred_beh]	= np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T
		else:
			map_st_beh[data] 		= np.hstack([map_st_beh[data],np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T])		#[[[statex1,statex2],[statey1,statey2],[stateth1,stateth2]],
			map_st_beh[pred_beh] 	= np.hstack([map_st_beh[pred_beh],np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T])	#[[behx1, behx2][behth1,behth2]]


'''
loads a csv file of states
'''
# def load_csv_states(filename):
# 	beh_list = []
# 	with open('filename', 'rb') as csvfile:
# 		spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
# 		for row in spamreader:
# 			print ', '.join(row)

'''
Loads the compromised and uncompromised maps from planned coordinates
'''
def load_beh_lists_csv():
	beh_list_u = load_csv_states(UNCOMPROMISED_STATE_CSV)
	beh_list_c = load_csv_states(COMPROMISED_STATE_CSV)

	return beh_list_u, beh_list_c

'''
Loads the compromised and uncompromised maps from planned coordinates
'''
def load_beh_lists_planned():
	global bc_interval
	un_comp 				= [ [[.2,0],5],[[0,-pi/8],2],[[.2,0],21],[[0,pi/8],2],[[.2,0],10],[[0,pi/8],2],[[.2,0],21],[[0,-pi/8],2],[[.2,0],5] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[.2,0],40]]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u, beh_list_c


'''
Creates a behavior list 
'''
def get_sample_construct(sample_list, dt):
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


def state_to_list(odom_msg):
	st_list = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, get_yaw(odom_msg.pose.pose.orientation)]

	return st_list

def beh_to_list(twist_msg):
	bh_list = [twist_msg.linear.x , twist_msg.angular.z]
	return bh_list

def get_mac(msg):
	return 12345 #TODO obtain mac.. possible with sec msgs. in simulation? CHANGE MSG TYPE


'''
Changes the control of the particle filter
'''
def beh_callback(twist):
	global k_list
	#pudb.set_trace() #For Debugging

	mac 	= get_mac(twist)
	beh 	= beh_to_list(twist)
	k_list[mac].upd_PF_behv(beh)

def sns_callback_pool():
	global k_list, last_odom, curr_odom_lock, comp_prob
	curr_odom = None
	while True:
		if last_odom is not curr_odom and last_odom is not None:
			with curr_odom_lock:
				curr_odom 	= last_odom
				mac 		= get_mac(curr_odom)
				sensor_pt	= state_to_list(curr_odom)
				mean 		= k_list[mac].upd_PF_sens(sensor_pt)
				publish_positions(mean)
				prob  		= get_comp_prob(mean, k_list[mac])
				if prob is not None:
					comp_msg 		= ChannelFloat32()
					comp_msg.name 	= str(mac)
					comp_msg.values = [prob]
					comp_prob.publish(comp_msg)
					#print "Likelyhood of being compromised is: " + str(prob) 
		else:
			#print "Didn't receive new sensor data!"
			rospy.sleep(UPD_FREQUENCY)


'''
Sensor info coming in [x,y]
'''
def sns_callback(odom):
	global last_odom, curr_odom_lock
	with curr_odom_lock:
		last_odom = odom


def get_comp_prob(state, bayes_obj):
	global k_list, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, MAP_MAX_NGBR

	u_results 		= kd_map_u.query(state,MAP_MAX_NGBR)
	c_results 		= kd_map_c.query(state,MAP_MAX_NGBR)

	curr_beh 		= bayes_obj.PF_behv()
	u_prob			= compare_sb(state, curr_beh, u_results, map_st_beh_u)
	c_prob			= compare_sb(state, curr_beh, c_results, map_st_beh_c)

	#print "uncomp is: " + str(u_prob)
	#print "comp is: " 	+ str(c_prob)

	bayes_obj.update_prob(c_prob, u_prob)
	return bayes_obj.get_c_prob()
	#print "compromised" if comp_prob > .9 else "Not compromised" if comp_prob < .1 else "Determining"



def get_inc(data_pts, indecies):
	ind_pts = []
	for i in indecies:
		if not len(ind_pts):
			ind_pts 		= np.atleast_2d(data_pts[:,i]).T 
		else:
			ind_pts 		= np.hstack([ind_pts,np.atleast_2d(data_pts[:,i]).T])
	return ind_pts

'''
Used to obtain the probability that a state and behavior exhibited is likely based on a map
'''
def compare_sb(state, beh, kd_list, map_st_beh):
	if len(kd_list) > 0 and len(kd_list[1]) > len(map_st_beh[0]): #check dimensions for enough pts (non singular matrix)
		sliced_st	= get_inc(map_st_beh[data],kd_list[1])
		sliced_beh	= get_inc(map_st_beh[pred_beh],kd_list[1])
		st_kernel 	= stats.gaussian_kde(sliced_st)
		bh_kernel	= stats.gaussian_kde(sliced_beh)
		state_pdf 	= st_kernel.pdf(state)[0]			#Likelyhood of doing a recorded state
		beh_pdf 	= bh_kernel.pdf(beh)[0]				#Likelyhood of following a behavior prev done

		return state_pdf * beh_pdf
	raise Exception("Need more points")


'''
Updates the particle filter
'''
def bayes_upd(bayes_obj):
	
	upd = 0
	while True:
		pointList = bayes_obj.get_PF_state()
		#pointlist can be used to display, but should not be used to determine prob
		if pointList is not None:
			publish_particles(pointList, particle_pub, [1,.3,.3,1])
		rospy.sleep(UPD_FREQUENCY)


def publish_particles(pointList, pub, color, int_start = 0):

	markerArray = MarkerArray()

	for i in range(pointList.shape[0]):

		point 						= pointList[i]
		marker						= Marker()

		tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, point[2])

		marker.pose.position.x = point[x]
		marker.pose.position.y = point[y]
		marker.pose.position.z = 0

		marker.scale.x = 0.4
		marker.scale.y = 0.04
		marker.scale.z = 0.01

		marker.color.a = color[0]
		marker.color.r = color[1]
		marker.color.g = color[2]
		marker.color.b = color[3]

		marker.pose.orientation.x 	= tempQuaternion[0]
		marker.pose.orientation.y 	= tempQuaternion[1]
		marker.pose.orientation.z 	= tempQuaternion[2]
		marker.pose.orientation.w 	= tempQuaternion[3]

		marker.id 				= i + int_start
		marker.header.frame_id 	= "/map"
		marker.header.stamp 	= rospy.Time.now()
		marker.action 			= marker.ADD

		markerArray.markers.append(marker)

	pub.publish(markerArray)


def publish_positions(mean_prtcl):
	mean_marker 	= get_marker(mean_prtcl, 50002, "/map", [.3,.3,1])

	mean_pos.publish(mean_marker)


def get_marker(particle, ID, frame_id, color):
	marker						= Marker()

	marker.pose.position.x = particle[x]
	marker.pose.position.y = particle[y]
	marker.pose.position.z = 0

	tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, particle[th])

	marker.scale.x = 0.2
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
	global bc_interval, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, k_list, particle_pub, state_uncomp_pub, state_comp_pub, rcv_pos, mean_pos, comp_prob
	rospy.init_node('camp_comp', anonymous=True)

	map_st_beh_u 			= [[],[]]
	map_st_beh_c 			= [[],[]]
	k_list 					= {}
	bc_interval				= .2

	init_state_u			= [0.0,0.0,0.0]
	init_state_c			= [1.0,0.0,0.0]
	#pudb.set_trace() #For Debugging
	beh_list_u, beh_list_c 	= load_beh_lists_planned()

	state_uncomp_pub	= rospy.Publisher('state_uncomp', MarkerArray, queue_size=100)
	state_comp_pub		= rospy.Publisher('state_comp', MarkerArray, queue_size=100)
	particle_pub 		= rospy.Publisher('pose_cloud', MarkerArray, queue_size=100)
	rcv_pos 			= rospy.Publisher('rcv_pos', Marker, queue_size=100)
	mean_pos 			= rospy.Publisher('mean_pos', Marker, queue_size=100)
	comp_prob 			= rospy.Publisher('comp_prob', ChannelFloat32, queue_size=100)


	get_full_planned(g_func_v_p,map_st_beh_c,init_state_c,beh_list_c,bc_interval)
	get_full_planned(g_func_v_p,map_st_beh_u,init_state_u,beh_list_u,bc_interval)


	#print_all([map_st_beh_u[0],map_st_beh_c[0]])

	kd_map_c = spatial.KDTree(map_st_beh_c[data].T)
	kd_map_u = spatial.KDTree(map_st_beh_u[data].T)

	#This is incoming conections, emulated hardcoded
	init_pos 		= np.array([0.0,0.0,0.0])
	k_list[12345] 	= compromized_state(INITIAL_PROB, init_pos)

	sns_callback_thread 			= Thread(target = sns_callback_pool, args = ())
	sns_callback_thread.setDaemon(True)
	sns_callback_thread.start()

	rospy.Subscriber("/turtlebot1/mobile_base/commands/velocity", Twist, beh_callback)
	#rospy.Subscriber("/turtlebot1/odom_throttle", Odometry, sns_callback)
	rospy.Subscriber("/turtlebot1/odometry/filtered_discrete", Odometry, sns_callback)


	for MAC in k_list:
		print "Starting thread for robot: " + str(MAC)
		listener_thread 			= Thread(target = bayes_upd, args = (k_list[MAC], ))
		listener_thread.setDaemon(True)
		listener_thread.start()

	print "Map Comparator Ready!"

	while not rospy.is_shutdown():
		publish_particles(map_st_beh_c[0].T, state_comp_pub, [.1,.7,.7,.2], 10000)
		publish_particles(map_st_beh_u[0].T, state_uncomp_pub,  [.1,.5,.5,.5], 20000)
		rospy.sleep(UPD_FREQUENCY)

	#rospy.spin()