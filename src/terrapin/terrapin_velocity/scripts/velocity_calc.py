#!/usr/bin/env python
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

from __future__ import print_function
from __future__ import division

import rospy
import numpy as np

#new
xs1 = []
ys1 = []
#old
xs2 = []
ys2 = []


def distance_straight():
    distance = xs2 - xs1
    return distance, 0


def distance_angular():
    # x = (xs1 - xs2)/2 + xs2
    # y = (ys1 - ys2)/2 + ys2
    
    # A = ((ys2 - ys1)/(xs2 - xs1))
    # B = y + 1/A * x
    
    # y = -1/A * x + B
    # x = -(y - B)*A
    # xi = -( 0 - B) * A
    # xi = B * A
    # xi = (y + 1/A * x) * A
    # xi = A * y + x
    # xi = ((ys2 - ys1)/(xs2 - xs1)) * y + x
    # xi = ((ys2 - ys1)/(xs2 - xs1)) * ((ys1 - ys2)/2 + ys2) + (xs1 - xs2)/2 + xs2

    #camera_offset = -0.1
    x_intercept = ((ys2 - ys1)/(xs2 - xs1)) * ((ys1 - ys2)/2 + ys2) + (xs1 - xs2)/2 + xs2
    #print(x_intercept)
    dTheta = np.arctan(ys2/(xs2 - x_intercept)) - np.arctan(ys1/(xs1 - x_intercept))
    #print(dTheta)
    #print( np.average((dTheta / (np.math.pi * 2))*np.abs(x_intercept)), np.average(dTheta)*180/np.math.pi)
    distance = np.average((dTheta / (np.math.pi * 2))*np.abs(x_intercept))
    return distance, np.average(dTheta) * 180/np.math.pi


def _callback():
    pass
    if
    vel = '{}'.format(0.0)
    ang = '{}'.format(0.0)
    g_velocity_publisher.publish(vel)
    g_angular_publisher.publish(ang)


def run_velocity_calc():
    global g_velocity_publisher, g_angular_publisher
    g_velocity_publisher = rospy.Publisher('velocity', Float64, queue_size=1)
    g_angular_publisher = rospy.Publisher('angular', Float64, queue_size=1)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #vel = '{}'.format(0.0)
        #ang = '{}'.format(0.0)
        #velocity_publisher.publish(vel)
        #angular_publisher.publish(ang)

        rate.sleep()


def init_velocity_calc():
    rospy.init_node('vel_calc', anonymous=True)
    #rospy.Subscribe('', Float64MultiArray, _callback)


if __name__ = '__main__':
    init_velocity_calc()
    try:
        run_velocity_calc()
    except rospy.ROSInterruptException:
        pass


