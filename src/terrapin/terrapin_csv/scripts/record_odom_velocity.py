#!/usr/bin/env python

from __future__ import division

import rospy
import csv
import datetime

from math import sqrt
from nav_msgs.msg import Odometry


output_path = "../output_data/odom/"


def record_odom_velocity(msg):

    # print(msg)

    # Calculate Velocity
    x_velocity = msg.twist.twist.linear.x
    y_velocity = msg.twist.twist.linear.y
    z_velocity = msg.twist.twist.linear.z
    velocity = sqrt(x_velocity**2 + y_velocity**2 + z_velocity**2)

    # Capture Timestamp
    time = msg.header.stamp.secs + (msg.header.stamp.nsecs / 1000000000)

    # print(velocity)
    print(time)
    print(ns)

    csvwriter.writerow([time, velocity])


if __name__ == '__main__':
    # Create a csv file to write output to
    dt = datetime.datetime.now()
    dt_time = str(dt.time())[:8]
    dt_date = str(dt.date())
    file_name = dt_time + "-" + dt_date + ".csv"
    print(file_name)

    global csvwriter

    with open(output_path + file_name, "wb") as csvfile:
        csvwriter = csv.writer(csvfile, dialect='excel', delimiter=',')

        # Setup ros node and publisher
        rospy.init_node('record_odom_velocity', anonymous=True)

        rospy.Subscriber('/odom', Odometry, record_odom_velocity)

        rospy.spin()
