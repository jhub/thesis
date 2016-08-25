#!/usr/bin/env python

import rospy
import random
import math

from geometry_msgs.msg import Twist, PoseStamped
from turtlebot import TurtleBot
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from helpers import convert_quaternion_to_yaw, correct_angle, convert_yaw_to_quaternion
from math import pi


class TurtleBot2D(TurtleBot, object):

    def __init__(self, linear_speed=.2, angular_speed=.2):
        super(TurtleBot2D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values

        self.linear_speed = linear_speed  # expressed in m/s
        self.angular_speed = angular_speed  # expressed in rad/s
        rospy.loginfo("Completed TurtleBot2D initialization")

    def move(self, goal_x=0, goal_y=0, goal_yaw=0, x_lower_bound=-1, x_upper_bound=1, y_lower_bound=-1, y_upper_bound=1):
        # To prevent robot getting stuck or drifting towards just one specific area over course of experiment
        # Randomly send to middle of allowable area with 1% probability
        goal_pose = PoseStamped()


        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation = convert_yaw_to_quaternion(goal_yaw)

        try:
            rospy.logdebug('Waiting for move_base server')
            self.move_base_client.wait_for_server()
            rospy.logdebug('Sending goal to move_base server')
            self.move_base_client.send_goal(goal)
            rospy.logdebug('Waiting for goal result from move_base server')
            self.move_base_client.wait_for_result()
            success = self.move_base_client.get_result()
            if not success:
                rospy.logwarn(self.namespace + ': move_base was not able to successfully complete the request action')
            else:
                continuous_x = self.continuous_pose_wrt_map.pose.pose.position.x
                continuous_y = self.continuous_pose_wrt_map.pose.pose.position.y
                discrete_x = self.discrete_pose_wrt_map.pose.pose.position.x
                discrete_y = self.discrete_pose_wrt_map.pose.pose.position.y
                gazebo_x = self.gazebo_pose_wrt_map.pose.pose.position.x
                gazebo_y = self.gazebo_pose_wrt_map.pose.pose.position.y
                rospy.loginfo(self.namespace + ': requested move to (' + str(goal_x) + ', ' + str(goal_y) + ') ' +
                               'completed. Current gazebo odom pose is (' + str(gazebo_x) + ', ' + str(gazebo_y) + '). ' +
                               'Current continuous odom pose is (' + str(continuous_x) + ', ' + str(continuous_y) + '). ' +
                               'Current discrete odom pose is (' + str(discrete_x) + ', ' + str(discrete_y) + '). ')
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': caught exception while sending move_base a movement goal - ' + e.message)

    def stop(self):
        try:
            self.move_base_client.cancel_all_goals()
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Unable to send movement goal cancel command - ' + e.message)

    def spin(self):
        rospy.loginfo("Entering spin mode")
        cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        command = Twist()
        command.angular.z = 0.1
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(command)
            rospy.loginfo(self.namespace + ': Current yaw is ' + str(convert_quaternion_to_yaw(self.gazebo_pose_wrt_map.pose.pose.orientation)))
            rospy.sleep(0.1)

    @staticmethod
    def check_move_bounds(val, lower, upper):
        # check that the val is between lower and upper values
        return lower < val < upper

def get_full_planned(g_func,map_beh, curr_state, beh_dur_list, dt):
    state_beh_list = []
    for beh_dur in beh_dur_list:
        state_beh_list.append([curr_state[:], beh_dur[:]])
        curr_state = g_func(curr_state,beh_dur,dt)
    return state_beh_list


'''
Used to choose between open loop trajectories; each element with [x, theta] velocities and the duration. With the given dt it creates a list to call.
'''
def load_target_lists(comp_map):
    un_comp                 = [[1,4,6,9,10],
                                [0,-3,-3,0,0],
                                [-pi/4,0,pi/4,0,0]]
    comp                    = [[10],
                                [0],
                                [0]]
    return un_comp if not comp_map else comp

def main():
    # create a movable turtle bot object
    robot = TurtleBot2D(0,0)

    # cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)

    # bc_interval             = .2
    # init_state              = [0.0,0.0,0.0]
    # map_beh                 = [[],[]]
    # #pudb.set_trace() #For Debugging
    # comp_map                = False
    # beh_list                = load_beh_lists(comp_map,bc_interval)

    # for cmd in beh_list:
    #     command             = Twist()
    #     command.linear.x    = cmd[0]
    #     command.angular.z   = cmd[1]

    #     cmd_vel_pub.publish(command)

    #     rospy.sleep(bc_interval)

    x_upper = 20#rospy.get_param('/x_upper')
    x_lower = 20#rospy.get_param('/x_lower')
    y_upper = 20#rospy.get_param('/y_upper')
    y_lower = 20#rospy.get_param('/y_lower')

    comp_map                = rospy.get_param("comp")
    pos_list                = load_target_lists(comp_map)

    # move the robot back and forth randomly until process killed with ctrl-c
    for index in range(len(pos_list[0])):
        robot.move(goal_x=pos_list[0][index],
                   goal_y=pos_list[1][index],
                   goal_yaw=pos_list[1][index],
                   x_lower_bound=x_lower,
                   x_upper_bound=x_upper,
                   y_lower_bound=y_lower,
                   y_upper_bound=y_upper)
    
    while not rospy.is_shutdown():
        rospy.sleep(5)

if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
