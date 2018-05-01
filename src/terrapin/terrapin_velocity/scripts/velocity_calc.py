#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from terrapin_points_msg.msg import points
import rospy
import numpy as np

#new
xs1 = []
ys1 = []
#old
xs2 = []
ys2 = []


def distance_straight(ys1, ys2):
    distance = ys2 - ys1
    return np.average(distance), 0


def distance_angular(xs1, ys1, xs2, ys2):
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


def points_callback(points):
    #pass
    #print(points.Xs1)
    xs1 = np.array(points.Xs1)
    ys1 = np.array(points.Ys1)
    xs2 = np.array(points.Xs2)
    ys2 = np.array(points.Ys2)
    #Convert time to a float in seconds unit
    time= (points.Time2 - points.Time1).to_sec()
    vel, ang = 0.0, 0.0
    #Check if there's major deviation to determine if the bot is turning
    #Should look for better solution
    if np.average(np.abs(xs1-xs2)) < 0.1:
        d, ang = distance_straight(ys1, ys2)
    else:
        d, ang = distance_angular(xs1, ys1, xs2, ys2)
    #vel = '{}'.format(0.0)
    #ang = '{}'.format(0.0)
    print(d, ang)
    g_velocity_publisher.publish(d/time)
    g_angular_publisher.publish(ang/time)


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
    rospy.Subscriber('points_data', points, points_callback)


if __name__ == '__main__':
    init_velocity_calc()
    try:
        run_velocity_calc()
    except rospy.ROSInterruptException:
        pass


