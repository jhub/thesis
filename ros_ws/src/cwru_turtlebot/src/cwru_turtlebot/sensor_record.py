#!/usr/bin/env python

import rospy
import csv
import os
import errno
import helpers

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu

# Data for recording
initial_x = None
initial_y = None
initial_yaw = None
continuous_data = None
discrete_data = None
gazebo_data = None
external_count = None
namespace = None
# TODO add protections so that a lack of starting or trailing slashes does not affect prefix and file writing
prefix = None
imu_data = None
noisy_odom_data = None
gps_data = None

behv_data = None

# csv.writer() objects
continuous_writer = None
discrete_writer = None
gazebo_writer = None
external_writer = None
imu_writer = None
noisy_odom_writer = None
gps_writer = None

behv_writer = None

def initial_position_callback(position):
    # Record the initial position of the robot so that we can convert the odometry values from the robot's odom frame
    # to the map frame
    global initial_x
    global initial_y
    global initial_yaw

    initial_x = position.pose.pose.position.x
    initial_y = position.pose.pose.position.y
    initial_yaw = helpers.convert_quaternion_to_yaw(position.pose.pose.orientation)


def continuous_odom_callback(new_odom):
    global continuous_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y, initial_yaw):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        pose_yaw = helpers.correct_angle(helpers.convert_quaternion_to_yaw(new_odom.pose.pose.orientation) + initial_yaw)
        x_variance = new_odom.pose.covariance[0]
        y_variance = new_odom.pose.covariance[7]
        yaw_variance = new_odom.pose.covariance[35]
        continuous_data = (pose_x, pose_y, pose_yaw, x_variance, y_variance, yaw_variance)
        # rospy.logdebug(namespace + ': sensor_record received new continuous data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def discrete_odom_callback(new_odom):
    global discrete_data
    global initial_x
    global initial_y
    global namespace

    # Measurement is already in map frame so no need to convert by adding initial position
    pose_x = new_odom.pose.pose.position.x
    pose_y = new_odom.pose.pose.position.y
    pose_yaw = helpers.correct_angle(helpers.convert_quaternion_to_yaw(new_odom.pose.pose.orientation))
    x_variance = new_odom.pose.covariance[0]
    y_variance = new_odom.pose.covariance[7]
    yaw_variance = new_odom.pose.covariance[35]
    discrete_data = (pose_x, pose_y, pose_yaw, x_variance, y_variance, yaw_variance)
    # rospy.logdebug(namespace + ': sensor_record received new discrete data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def gazebo_odom_callback(new_odom):
    global gazebo_data
    global initial_x
    global initial_y
    global namespace

    if None not in (initial_x, initial_y, initial_yaw):
        # Must add in the initial pose values to convert from odom frame to map frame
        pose_x = new_odom.pose.pose.position.x + initial_x
        pose_y = new_odom.pose.pose.position.y + initial_y
        pose_yaw = helpers.correct_angle(helpers.convert_quaternion_to_yaw(new_odom.pose.pose.orientation) + initial_yaw)
        x_variance = new_odom.pose.covariance[0]
        y_variance = new_odom.pose.covariance[7]
        yaw_variance = new_odom.pose.covariance[35]
        gazebo_data = (pose_x, pose_y, pose_yaw, x_variance, y_variance, yaw_variance)
        # rospy.logdebug(namespace + ': sensor_record received new gazebo data of (' + str(pose_x) + ', ' + str(pose_y) + ')')


def external_pose_count_callback(count):
    global external_count
    global namespace

    rospy.logdebug(namespace + ': sensor_record received new external_count value of ' + str(count.data))
    external_count = (count.data,)


def imu_callback(imu_msg):
    global imu_data

    yaw = helpers.convert_quaternion_to_yaw(imu_msg.orientation)
    velocity = (imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)
    acceleration = (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z)

    imu_data = (yaw, velocity[0], velocity[1], velocity[2], acceleration[0], acceleration[1], acceleration[2])


def noisy_odom_callback(odom_msg):
    global noisy_odom_data
    if odom_msg.pose.pose.position.x is not None:
        pose_x = odom_msg.pose.pose.position.x + initial_x
        pose_y = odom_msg.pose.pose.position.y + initial_y
        pose_yaw = helpers.correct_angle(helpers.convert_quaternion_to_yaw(odom_msg.pose.pose.orientation) + initial_yaw)
        x_vel = odom_msg.twist.twist.linear.x
        yaw_vel = odom_msg.twist.twist.angular.z
        x_variance = odom_msg.pose.covariance[0]
        y_variance = odom_msg.pose.covariance[7]
        yaw_variance = odom_msg.pose.covariance[35]
        noisy_odom_data = (pose_x, pose_y, pose_yaw, x_vel, yaw_vel, x_variance, y_variance, yaw_variance)
    else:
        noisy_odom_data = (0, 0, 0, 0, 0, 0, 0, 0)


def gps_callback(gps_msg):
    global gps_data

    x = gps_msg.pose.pose.position.x
    y = gps_msg.pose.pose.position.y

    gps_data = (x, y)


def behv_callback(behv_msg):
    global behv_data

    vx      = behv_msg.linear.x
    vth     = behv_msg.angular.z

    behv_data = (vx, vth)


def make_sure_path_exists(path):
    global namespace

    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            rospy.logwarn(namespace + ': Error when checking file path - ' + e.message)


def write_headers():
    global namespace
    global prefix

    # Make sure our path exists before creating our files
    make_sure_path_exists(prefix)

    if None not in (prefix, namespace):
        # rospy.logdebug(namespace + ': Writing headers to data files')
        filename = prefix + namespace + '_continuous_filter_odom.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw', 'x_variance', 'y_variance', 'yaw_variance'])

        filename = prefix + namespace + '_discrete_filter_odom.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw', 'x_variance', 'y_variance', 'yaw_variance'])

        filename = prefix + namespace + '_gazebo_odom.csv'
        with open(filename, 'w+') as pose_file:
            writer = csv.writer(pose_file)
            writer.writerow(['x_position', 'y_position', 'yaw', 'x_variance', 'y_variance', 'yaw_variance'])

        filename = prefix + namespace + '_external_pose_count.csv'
        with open(filename, 'w+') as count_file:
            writer = csv.writer(count_file)
            writer.writerow(['count'])

        filename = prefix + namespace + '_imu_data.csv'
        with open(filename, 'w+') as imu_file:
            writer = csv.writer(imu_file)
            writer.writerow(['yaw', 'x_vel', 'y_vel', 'z_vel', 'x_acc', 'y_acc', 'z_acc'])

        filename = prefix + namespace + '_noisy_odom_data.csv'
        with open(filename, 'w+') as noisy_odom_file:
            writer = csv.writer(noisy_odom_file)
            writer.writerow(['x', 'y', 'yaw', 'x_vel', 'yaw_vel', 'x_variance', 'y_variance', 'yaw_variance'])

        filename = prefix + namespace + '_gps_data.csv'
        with open(filename, 'w+') as gps_file:
            writer = csv.writer(gps_file)
            writer.writerow(['x', 'y'])

        filename = prefix + namespace + '_behv_data.csv'
        with open(filename, 'w+') as behv_file:
            writer = csv.writer(behv_file)
            writer.writerow(['vx', 'vth'])
    else:
        if namespace is None:
            rospy.logdebug('Could not write headers because namespace was not initialized')
        else:
            if prefix is None:
                rospy.logdebug(namespace + ': Could not write headers because file prefix was not initialized')


# Must accept event as argument due to use with timer
def write_to_files(event):
    global continuous_data
    global discrete_data
    global gazebo_data
    global external_count
    global imu_data
    global noisy_odom_data
    global gps_data
    global behv_data
    global namespace
    global prefix

    global continuous_writer
    global discrete_writer
    global gazebo_writer
    global external_writer
    global imu_writer
    global noisy_odom_writer
    global gps_writer

    global behv_writer

    assert None not in (continuous_writer, discrete_writer, gazebo_writer, external_writer, imu_writer,
                        noisy_odom_writer, gps_writer, behv_writer, namespace, prefix)

    # Synchronized writing for data published at 10Hz or higher
    if None not in (continuous_data, discrete_data, gazebo_data, external_count, imu_data, noisy_odom_data, behv_data):
        # rospy.logdebug(namespace + ': Writing to sensor data to files')
        continuous_writer.writerow(continuous_data)
        discrete_writer.writerow(discrete_data)
        gazebo_writer.writerow(gazebo_data)
        external_writer.writerow(external_count)
        imu_writer.writerow(imu_data)
        noisy_odom_writer.writerow(noisy_odom_data)
        behv_writer.writerow(behv_data)

        # Invalidate fields so that we don't record duplicate data multiple times in the case of sensor failure
        continuous_data = None
        discrete_data = None
        gazebo_data = None
        imu_data = None
        noisy_odom_data = None
        behv_data = None

        # GPS data publishes slower than odometry so it must be checked/written separately
        if gps_data is not None:
            gps_writer.writerow(gps_data)
            gps_data = None
    else:
        if continuous_data is None:
            rospy.logdebug(namespace + ': Could not write data because continuous data was not initialized')
        if discrete_data is None:
            rospy.logdebug(namespace + ': Could not write data because discrete data was not initialized')
        if gazebo_data is None:
            rospy.logdebug(namespace + ': Could not write data because gazebo data was not initialized')
        if external_count is None:
            rospy.logdebug(namespace + ': Could not write data because external count was not initialized')
        if prefix is None:
            rospy.logdebug(namespace + ': Could not write data because file prefix was not initialized')
        if noisy_odom_data is None:
            rospy.logdebug(namespace + ': Could not write data because noisy odom data was not initialized')
        if behv_data is None:
            rospy.logdebug(namespace + ': Could not write data because behavior data was not initialized')        
        if gps_data is None:
            rospy.logdebug(namespace + ': Could not write data because gps data was not initialized')
        if imu_data is None:
            rospy.logdebug(namespace + ': Could not write data because imu data was not initialized')


def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('sensor_record', log_level=rospy.DEBUG)
    else:
        rospy.init_node('sensor_record')

    helpers.wait_for_services()

    global namespace
    namespace = rospy.get_namespace()[1:-1]

    global prefix
    prefix = rospy.get_param('/save_file_prefix')
    if not prefix.endswith('/'):
        prefix += '/'

    write_headers()

    # Must account for single robot simulation where there are no external poses
    global external_count
    if rospy.get_param('/number_of_robots') == 1:
        external_count = [0]

    rospy.sleep(rospy.Duration(2))  # Wait 2 seconds for filters to successfully localize before recording

    initial_position_subscriber = rospy.Subscriber('initial_position', PoseWithCovarianceStamped, initial_position_callback)
    continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous', Odometry, continuous_odom_callback)
    discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete', Odometry, discrete_odom_callback)
    gazebo_odom_subscriber = rospy.Subscriber('odom_throttle', Odometry, gazebo_odom_callback)
    external_pose_count_subscriber = rospy.Subscriber('external_poses_count', UInt64, external_pose_count_callback)
    imu_subscriber = rospy.Subscriber('imu_data_remapped', Imu, imu_callback)
    noisy_odom_subscriber = rospy.Subscriber('noisy_odom_remapped', Odometry, noisy_odom_callback)
    gps_subscriber = rospy.Subscriber('fake_gps', PoseWithCovarianceStamped, gps_callback)

    behv_subcriber   = rospy.Subscriber("mobile_base/commands/velocity", Twist, behv_callback)

    global continuous_writer
    global discrete_writer
    global gazebo_writer
    global external_writer
    global imu_writer
    global noisy_odom_writer
    global gps_writer

    global behv_writer

    print "Writing data to: " + prefix + namespace

    with open(prefix + namespace + '_continuous_filter_odom.csv', 'a+') as continuous_file, \
        open(prefix + namespace + '_discrete_filter_odom.csv', 'a+') as discrete_file, \
        open(prefix + namespace + '_gazebo_odom.csv', 'a+') as gazebo_file, \
        open(prefix + namespace + '_external_pose_count.csv', 'a+') as external_file, \
        open(prefix + namespace + '_imu_data.csv', 'a+') as imu_file, \
        open(prefix + namespace + '_noisy_odom_data.csv', 'a+') as noisy_odom_file, \
        open(prefix + namespace + '_gps_data.csv', 'a+') as gps_file,\
        open(prefix + namespace + '_behv_data.csv', 'a+') as behv_file:

        continuous_writer = csv.writer(continuous_file)
        discrete_writer = csv.writer(discrete_file)
        gazebo_writer = csv.writer(gazebo_file)
        external_writer = csv.writer(external_file)
        imu_writer = csv.writer(imu_file)
        noisy_odom_writer = csv.writer(noisy_odom_file)
        gps_writer = csv.writer(gps_file)

        behv_writer = csv.writer(behv_file)

        timer = rospy.Timer(rospy.Duration(.1), write_to_files)
        
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
