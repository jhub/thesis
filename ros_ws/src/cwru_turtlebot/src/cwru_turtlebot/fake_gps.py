#!/usr/bin/env python

import rospy
import helpers
import numpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

initial_position = None
gazebo_odom = None
gps_publisher = None


def initial_position_cb(initial_position_msg):
    global initial_position

    if initial_position is None:
        initial_position = initial_position_msg
    else:
        rospy.logwarn('Initial position callback in fake_gps triggered twice, this should not happen!')


def gazebo_odom_cb(odom_msg):
    global gazebo_odom
    gazebo_odom = odom_msg


# Must accept event argument because of use with rospy.Timer
def publish_gps(event):
    # Use this so discrete filter can localize w/o external measurements
    # FAA 2014 research shows 95% confidence interval of 3.351 meters horizontal accuracy
    # Assuming this error is Gaussian and normally distributed, we have a mean of 0 and standard deviation of 1.71
    global initial_position
    global gazebo_odom

    x_map = initial_position.pose.pose.position.x + gazebo_odom.pose.pose.position.x
    y_map = initial_position.pose.pose.position.y + gazebo_odom.pose.pose.position.y

    horizontal_error = numpy.random.normal(scale=1.71)
    theta = numpy.random.uniform(high=2*numpy.pi)

    noisy_gps = PoseWithCovarianceStamped()
    noisy_gps.header.stamp = rospy.get_rostime()
    noisy_gps.header.frame_id = 'map'
    noisy_gps.pose.pose.position.x = x_map + horizontal_error * numpy.cos(theta)
    noisy_gps.pose.pose.position.y = y_map + horizontal_error * numpy.sin(theta)
    noisy_gps.pose.pose.orientation = gazebo_odom.pose.pose.orientation # Leave this alone because robot has no compass
    # TODO figure out setting covariance

    assert gps_publisher is not None
    gps_publisher.publish(noisy_gps)


def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('fake_gps', log_level=rospy.DEBUG)
    else:
        rospy.init_node('fake_gps')

    helpers.wait_for_services()

    initial_position_subscriber = rospy.Subscriber('initial_position', PoseWithCovarianceStamped, initial_position_cb)
    gazebo_odom_subscriber = rospy.Subscriber('odom', Odometry, gazebo_odom_cb)

    global gps_publisher
    gps_publisher = rospy.Publisher('fake_gps', PoseWithCovarianceStamped, queue_size=1)

    timer = rospy.Timer(rospy.Duration(10), publish_gps)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass